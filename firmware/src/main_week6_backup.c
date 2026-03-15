#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>
#include <arm_math.h>
#include "guardian/audio/dma_pdm.h"
#include "guardian/resonator_df2t.h"
#include "guardian/gate/decision.h"

/* ── Test audio mode ──────────────────────────────────────────────────────────
 * Set one of these to 1 to inject synthetic audio instead of the DMA mic.
 *
 *  TEST_AUDIO_SINE  = 800 Hz sine → expect WAKE  (speech-like, periodic)
 *  TEST_AUDIO_NOISE = LFSR noise  → expect ABORT (broadband, no correlation)
 *
 * Set both to 0 to use the real PDM microphone.
 * ─────────────────────────────────────────────────────────────────────────── */
#define TEST_AUDIO_SINE  0
#define TEST_AUDIO_NOISE 0

#if (TEST_AUDIO_SINE && TEST_AUDIO_NOISE)
#error "Set only one of TEST_AUDIO_SINE or TEST_AUDIO_NOISE, not both"
#endif

/* ── Synthetic audio helpers ─────────────────────────────────────────────── */

#if TEST_AUDIO_SINE
/* 800 Hz sine at 16 kHz sample rate, 50% amplitude.
 * arm_sin_q15(x): x=0..32767 maps to 0..2π
 * phase_inc = 32768 * 800 / 16000 = 1638                                    */
#define SINE_PHASE_INC 1638
static int16_t test_buf[FRAME_SIZE];
static q15_t   sine_phase = 0;

static int16_t *get_test_frame(void)
{
    for (int i = 0; i < FRAME_SIZE; i++) {
        test_buf[i] = arm_sin_q15(sine_phase) / 2; /* /2 = 50% amplitude */
        sine_phase  = (q15_t)((sine_phase + SINE_PHASE_INC) & 0x7FFF);
    }
    return test_buf;
}
#endif /* TEST_AUDIO_SINE */

#if TEST_AUDIO_NOISE
/* 16-bit Galois LFSR — low amplitude to stay in speech energy range          */
static int16_t test_buf[FRAME_SIZE];
static uint16_t lfsr = 0xACE1u;

static int16_t *get_test_frame(void)
{
    for (int i = 0; i < FRAME_SIZE; i++) {
        lfsr = (lfsr >> 1) ^ (uint16_t)(-(lfsr & 1u) & 0xB400u);
        test_buf[i] = (int16_t)lfsr >> 4; /* scale down: ~12-bit signed range */
    }
    return test_buf;
}
#endif /* TEST_AUDIO_NOISE */

/* ── Application ─────────────────────────────────────────────────────────── */

static resonator_bank_df2t_t bank;
static gate_config_t         config = GATE_CONFIG_DEFAULT;
static noise_tracker_t       noise  = {0};

int main(void)
{
#if TEST_AUDIO_SINE
    printk("\n=== Guardian Week 6 - Gate Decision [TEST: 800Hz SINE] ===\n");
    printk("Expected: WAKE most frames (periodic speech-like signal)\n\n");
#elif TEST_AUDIO_NOISE
    printk("\n=== Guardian Week 6 - Gate Decision [TEST: LFSR NOISE] ===\n");
    printk("Expected: ABORT most frames (broadband noise)\n\n");
#else
    printk("\n=== Guardian Week 6 - Gate Decision [LIVE MIC] ===\n\n");
#endif

    timing_init();
    timing_start();

#if !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
    int ret = dma_pdm_init();
    if (ret != 0) {
        printk("DMA PDM init failed: %d\n", ret);
        return ret;
    }
#endif

    resonator_bank_df2t_init(&bank);

#if !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
    int ret2 = dma_pdm_start();
    if (ret2 != 0) {
        printk("DMA PDM start failed: %d\n", ret2);
        return ret2;
    }
#endif

    int frame_count = 0;
    int wake_count  = 0;

    while (1) {
        int16_t *frame;

#if (TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
        frame = get_test_frame();
        k_sleep(K_MSEC(20)); /* pace to ~50 frames/s (matches real DMA rate) */
#else
        int ret3 = dma_pdm_read(&frame);
        if (ret3 < 0) {
            continue;
        }

        /* TEMP DIAG: print raw DMA samples and resonator output every 50 frames */
        if (frame_count == 0) {
            int16_t raw_min = frame[0], raw_max = frame[0];
            for (int i = 1; i < FRAME_SIZE; i++) {
                if (frame[i] < raw_min) raw_min = frame[i];
                if (frame[i] > raw_max) raw_max = frame[i];
            }
            printk("RAW: min=%6d max=%6d samples[0..3]=%d %d %d %d\n",
                   raw_min, raw_max, frame[0], frame[1], frame[2], frame[3]);
        }
#endif

        resonator_bank_df2t_process(&bank, frame, FRAME_SIZE);

#if !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
        /* TEMP DIAG: print resonator ch0 output range */
        if (frame_count == 0) {
            const int16_t *r0 = resonator_bank_df2t_get_output(&bank, 0);
            int16_t r_min = r0[0], r_max = r0[0];
            for (int i = 1; i < FRAME_SIZE; i++) {
                if (r0[i] < r_min) r_min = r0[i];
                if (r0[i] > r_max) r_max = r0[i];
            }
            printk("RES0: min=%6d max=%6d\n", r_min, r_max);
        }
#endif

        gate_decision_t decision = gate_decide(&bank, &config);

        energy_features_t efeat = { .total_energy = decision.features_used.energy };
        update_noise_floor(&noise, &efeat, decision.should_wake);
        config.noise_floor = noise.noise_estimate;

        if (decision.should_wake) {
            wake_count++;
        }

        if (++frame_count >= 50) {
            frame_count = 0;
            printk("GATE: %s conf=%3u E=%5d C=%6d ZCR=%3u T=%4uus "
                   "noise=%5d wakes=%2d/50\n",
                   decision.should_wake ? "WAKE " : "ABORT",
                   decision.confidence,
                   decision.features_used.energy,
                   decision.features_used.correlation,
                   decision.features_used.zcr,
                   decision.elapsed_us,
                   config.noise_floor,
                   wake_count);
            wake_count = 0;
        }
    }

    return 0;
}
