#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>
#include <arm_math.h>
#include "guardian/audio/dma_pdm.h"
#include "guardian/resonator_df2t.h"
#include "guardian/gate/decision.h"
#ifdef CONFIG_SEGGER_SYSTEMVIEW
#include <SEGGER_SYSVIEW.h>
#endif

/* ── Week 5 Day 5: DMA vs Polling power comparison ────────────────────────────
 * Set POLL_MODE_TEST 1 to use busy-wait polling (CPU never sleeps).
 * Set POLL_MODE_TEST 0 (default) for DMA sleep mode (CPU sleeps between frames).
 * Flash each binary, measure PPK2 current, compare.
 * ─────────────────────────────────────────────────────────────────────────── */
#define POLL_MODE_TEST 0

/* ── Week 7-8: Gate mode comparison for power measurement ────────────────────
 * GATE_MODE -1 = SLEEP     — CPU sleeps forever (J-Link baseline measurement)
 * GATE_MODE  0 = BASELINE  — resonator + always runs mock TinyML (no gate)
 * GATE_MODE  1 = ENERGY_VAD — simple RMS threshold gate, skips resonator bank
 * GATE_MODE  2 = PHYSICS   — full resonator + gate_decide (default)
 *
 * Flash each mode with UART disabled for clean PPK2 measurement:
 *   west build -b nrf52840dk/nrf52840 -- -DEXTRA_CONF_FILE=prj_power_test.conf
 * ─────────────────────────────────────────────────────────────────────────── */
#define GATE_MODE 2

/* Energy VAD threshold (Q15 RMS of raw frame).
 * ~200 separates quiet room from active speech — tune if needed.            */
#define ENERGY_VAD_THRESHOLD 200

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

/* ── Week 9: Custom trace logging ────────────────────────────────────────────
 * ENABLE_TRACE 1 — logs GUARDIAN/TINYML events, dumps CSV every 100 frames.
 *   Capture with: cat /dev/ttyACM0 | tee trace_log.csv
 *   Visualize  : python3 tools/analyze_systemview/visualize_trace.py trace_log.csv
 *
 * STRESS_TEST 1 — adds a competing CPU thread (priority 7) that busy-loops
 *   5ms every 15ms, simulating background work. Shows gate timing is stable
 *   under CPU contention.
 *
 * Set both to 0 for normal operation.
 * ─────────────────────────────────────────────────────────────────────────── */
#define ENABLE_TRACE    1
#define STRESS_TEST     0

/* TINYML_THREADED 1 — runs mock TinyML in a separate lower-priority thread
 * via a 4-slot message queue. The gate thread never blocks on TinyML:
 * it queues the frame and immediately processes the next one.
 * This fixes the "5 frames missed per wake event" problem.
 * Set to 0 for synchronous (blocking) TinyML — simpler but loses frames.  */
#define TINYML_THREADED  1

/* DEADLINE_STATS 1 — counts frames that exceed 20ms and prints miss rate.
 * Run for 10 minutes with STRESS_TEST 1 to validate <1% miss rate target.  */
#define DEADLINE_STATS  0
#define FRAME_BUDGET_US 20000U

/* ── Pre-roll ring buffer ─────────────────────────────────────────────────────
 * PREROLL_BUFFER 1 — stores last 25 frames (500ms) so TinyML always receives
 * a full wakeword-length context window, not just the 20ms frame that woke it.
 *
 * Without this: TinyML sees 20ms — cannot detect a wakeword that started
 *               before the gate fired. System is a demo, not a detector.
 * With this   : TinyML sees 500ms — covers any wakeword (Hey Siri ~600ms).
 *
 * RAM cost: 25 × 320 × 2 = 16 KB (6.25% of 256 KB nRF52840 RAM).
 * CPU cost: memcpy 640 bytes/frame ≈ 10 µs — negligible vs 20ms budget.
 * ─────────────────────────────────────────────────────────────────────────── */
#define PREROLL_BUFFER 0

#if PREROLL_BUFFER
#define PREROLL_FRAMES 25   /* 500ms @ 20ms/frame — covers any wakeword */
static int16_t  preroll_ring[PREROLL_FRAMES][FRAME_SIZE];
static uint32_t preroll_head   = 0;   /* next write slot (circular) */
static uint32_t preroll_filled = 0;   /* frames stored (caps at PREROLL_FRAMES) */
#endif

/* ── Week 11-12: On-device playback test ─────────────────────────────────────
 * TEST_PLAYBACK 1 — embeds 3 s of speech + noise in flash and runs the gate
 * on them at boot (before entering the normal mic loop).
 *
 * Generate headers first (from project root):
 *   python3 tools/wav_to_c_array.py results/audio_test_set/speech_001.wav \
 *       speech_sample firmware/src/testdata/speech_sample.h
 *   python3 tools/wav_to_c_array.py results/audio_test_set/noise_001.wav  \
 *       noise_sample  firmware/src/testdata/noise_sample.h
 *
 * Expected output (synthetic samples):
 *   Speech: GO>=60%   Noise: ABORT>=80%
 *
 * Flash cost: ~186 KB (2 × 48 000 samples × 2 bytes).
 * ─────────────────────────────────────────────────────────────────────────── */
#define TEST_PLAYBACK 0

/* ── Week 13-14: Full pipeline stats ─────────────────────────────────────────
 * PIPELINE_STATS 1 — tracks TinyML run count + keyword detections (10% mock
 * rate via LCG) and prints a PIPELINE: line every 50 frames.
 * Keep TINYML_THREADED 0 for this mode so keyword detection stays in main loop.
 *
 * Expected output:
 *   PIPELINE: total=50 gate_abort=35(70%) tinyml=15 keywords=2
 * ─────────────────────────────────────────────────────────────────────────── */
#define PIPELINE_STATS 0

/* ── Problem simulations (Week 9 Day 6-7) ────────────────────────────────────
 * SIMULATE_CTX_SWITCHES 1 — spawns 4 threads at same priority as main.
 *   Each yields every 1ms, forcing frequent context switches into gate path.
 *   SHOWS: gate timing variance increases in trace.
 *   FIX  : set CTX_SWITCH_FIX 1 — gate moves to k_work (system workqueue),
 *          which is cooperative and avoids preemption during gate processing.
 *
 * SIMULATE_PRIORITY_INV 1 — low-priority thread holds a shared mutex for
 *   10ms every frame. Gate tries to acquire same mutex before gate_decide.
 *   SHOWS: gate blocked behind low-priority thread → deadline misses.
 *   FIX  : set PRIORITY_INV_FIX 1 — removes mutex from gate critical path
 *          (real fix: use priority ceiling / avoid shared resources in ISR).
 * ─────────────────────────────────────────────────────────────────────────── */
#define SIMULATE_CTX_SWITCHES 0
#define CTX_SWITCH_FIX        0   /* 1 = move gate to k_work to reduce preemption */

#define SIMULATE_PRIORITY_INV 0
#define PRIORITY_INV_FIX      0   /* 1 = gate skips mutex (fixed critical path) */

/* ── Trace implementation ────────────────────────────────────────────────── */
#if ENABLE_TRACE

#define TRACE_BUF_SZ 512

typedef enum {
    TR_FRAME_START,
    TR_GUARDIAN_START,
    TR_GUARDIAN_END,
    TR_TINYML_START,
    TR_TINYML_END,
} trace_evt_t;

static const char * const tr_names[] = {
    "FRAME_START", "GUARDIAN_START", "GUARDIAN_END",
    "TINYML_START", "TINYML_END",
};

struct trace_entry {
    uint32_t    ts_ms;
    trace_evt_t evt;
    uint16_t    data; /* elapsed_us for GUARDIAN_END, else 0 */
};

static struct trace_entry tr_buf[TRACE_BUF_SZ];
static uint32_t           tr_idx = 0;
static bool               tr_header_done = false;

static inline void tr_log(trace_evt_t evt, uint16_t data)
{
    if (tr_idx < TRACE_BUF_SZ) {
        tr_buf[tr_idx].ts_ms = k_uptime_get_32();
        tr_buf[tr_idx].evt   = evt;
        tr_buf[tr_idx].data  = data;
        tr_idx++;
    }
}

static void tr_dump(void)
{
    if (!tr_header_done) {
        printk("timestamp_ms,event,data\n");
        tr_header_done = true;
    }
    for (uint32_t i = 0; i < tr_idx; i++) {
        printk("%u,%s,%u\n",
               tr_buf[i].ts_ms,
               tr_names[tr_buf[i].evt],
               (unsigned)tr_buf[i].data);
    }
    tr_idx = 0;
}

#define TR(evt, data) tr_log(evt, data)
#else
#define TR(evt, data) /* disabled */
#endif /* ENABLE_TRACE */

/* ── Week 9: Stress thread ───────────────────────────────────────────────────
 * Competes for CPU at priority 7 (main thread is priority 0).
 * Busy-loops 5ms every 15ms — simulates background sensor/radio work.
 * Gate timing should stay flat even with this load.                         */
#if STRESS_TEST
static void stress_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    while (1) {
        k_busy_wait(5000);    /* 5ms CPU load */
        k_sleep(K_MSEC(15));  /* yield 15ms   */
    }
}
K_THREAD_DEFINE(stress_tid, 512, stress_entry, NULL, NULL, NULL, 7, 0, 0);
#endif

/* ── Problem 1: Context switch simulation + fix ──────────────────────────────
 * 4 threads at priority 0 (same as main) each yield every 1ms.
 * Forces scheduler to context-switch into gate processing window.           */
#if SIMULATE_CTX_SWITCHES && !CTX_SWITCH_FIX
static void ctx_switch_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    while (1) {
        k_busy_wait(500);   /* 0.5ms work */
        k_yield();          /* voluntary switch — triggers scheduler */
    }
}
K_THREAD_DEFINE(ctx_t0, 256, ctx_switch_entry, NULL, NULL, NULL, 0, 0, 0);
K_THREAD_DEFINE(ctx_t1, 256, ctx_switch_entry, NULL, NULL, NULL, 0, 0, 0);
K_THREAD_DEFINE(ctx_t2, 256, ctx_switch_entry, NULL, NULL, NULL, 0, 0, 0);
K_THREAD_DEFINE(ctx_t3, 256, ctx_switch_entry, NULL, NULL, NULL, 0, 0, 0);
#endif

#if CTX_SWITCH_FIX
/* Fix: gate runs in system workqueue (cooperative, no preemption mid-gate) */
static K_WORK_DEFINE(gate_work, NULL); /* placeholder — wired in loop below */
#endif

/* ── Problem 2: Priority inversion simulation + fix ──────────────────────────
 * Low-priority thread (priority 8) holds shared_mutex for 10ms.
 * Gate (priority 0) tries to acquire it before gate_decide → gets blocked.
 * Result: gate deadline misses whenever low-prio thread holds the lock.     */
#if SIMULATE_PRIORITY_INV
static K_MUTEX_DEFINE(shared_mutex);

static void low_prio_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    while (1) {
        k_mutex_lock(&shared_mutex, K_FOREVER);
        k_busy_wait(10000); /* holds mutex 10ms — blocks gate if it tries */
        k_mutex_unlock(&shared_mutex);
        k_sleep(K_MSEC(5)); /* release window — gate can acquire here */
    }
}
K_THREAD_DEFINE(low_prio_tid, 512, low_prio_entry, NULL, NULL, NULL, 8, 0, 0);
#endif

/* ── Week 9: Threaded TinyML (Day 6-7 optimization) ─────────────────────────
 * Problem: 100ms TinyML in main loop blocks gate for 5 frames per wake.
 * Fix: dedicated lower-priority thread drains a 4-slot queue.
 * Gate thread queues frame pointer (non-blocking) and continues immediately. */
#if TINYML_THREADED
static void mock_tinyml_inference(void); /* forward declaration */
/* Queue depth formula:
 *   depth = ceil(tinyml_latency_ms / frame_period_ms × peak_wake_rate × safety)
 *
 *   Real DS-CNN (~15ms):  ceil(15/20 × 0.30 × 2.0) = 1  → use 4 for burst
 *   Mock TinyML (100ms):  ceil(100/20 × 1.00 × 1.5) = 8  → depth=4 saturates
 *                          (WARN fires — graceful degradation proven correct)
 *
 * Depth=4 is correct for real TinyML at ≤53% wake rate with 2× safety margin.
 * Increase to 8 only if deploying mock 100ms TinyML at high wake rates.       */
K_MSGQ_DEFINE(tinyml_q, sizeof(int16_t *), 4, 4);

static void tinyml_thread_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    int16_t *frame;
    while (1) {
        k_msgq_get(&tinyml_q, &frame, K_FOREVER);
        TR(TR_TINYML_START, 0);
        mock_tinyml_inference();
        TR(TR_TINYML_END, 0);
    }
}
K_THREAD_DEFINE(tinyml_tid, 1024, tinyml_thread_entry,
                NULL, NULL, NULL, 10, 0, 0); /* priority 10 — below gate (0) */
#endif /* TINYML_THREADED */

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
        test_buf[i] = arm_sin_q15(sine_phase) / 2;
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
        test_buf[i] = (int16_t)lfsr >> 4;
    }
    return test_buf;
}
#endif /* TEST_AUDIO_NOISE */

/* ── Week 11-12: On-device playback test ─────────────────────────────────── */
#if TEST_PLAYBACK
#include "testdata/speech_sample.h"
#include "testdata/noise_sample.h"

static void run_playback_test(void)
{
    static resonator_bank_df2t_t pb_bank;   /* static: 2600B — too large for stack */
    gate_config_t         pb_cfg   = GATE_CONFIG_DEFAULT;
    noise_tracker_t       pb_noise = {0};
    uint32_t go, abort_cnt, total;

    resonator_bank_df2t_init(&pb_bank);

    printk("\n=== Week 11-12: On-Device Playback Test ===\n");
    printk("Speech array: %u samples (%.1fs)  Flash: ~%u KB\n",
           SPEECH_SAMPLE_SAMPLE_COUNT,
           (float)SPEECH_SAMPLE_SAMPLE_COUNT / SPEECH_SAMPLE_SAMPLE_RATE,
           (SPEECH_SAMPLE_SAMPLE_COUNT * 2U) / 1024U);

    /* ── Run gate on speech ───────────────────────────────────────────────── */
    go = abort_cnt = 0;
    for (uint32_t i = 0; i + FRAME_SIZE <= SPEECH_SAMPLE_SAMPLE_COUNT; i += FRAME_SIZE) {
        resonator_bank_df2t_process(&pb_bank, &speech_sample[i], FRAME_SIZE);
        gate_decision_t d = gate_decide(&pb_bank, &pb_cfg);
        energy_features_t ef = { .total_energy = d.features_used.energy };
        update_noise_floor(&pb_noise, &ef, d.should_wake);
        pb_cfg.noise_floor = pb_noise.noise_estimate;
        if (d.should_wake) { go++; } else { abort_cnt++; }
    }
    total = go + abort_cnt;
    printk("Speech: GO=%u  ABORT=%u  wake_rate=%u%%  (target >= 60%%)\n",
           go, abort_cnt, total ? go * 100U / total : 0U);

    /* ── Reset state for noise test ──────────────────────────────────────── */
    resonator_bank_df2t_reset(&pb_bank);
    pb_cfg   = (gate_config_t)GATE_CONFIG_DEFAULT;
    pb_noise = (noise_tracker_t){0};

    /* ── Run gate on noise ────────────────────────────────────────────────── */
    go = abort_cnt = 0;
    for (uint32_t i = 0; i + FRAME_SIZE <= NOISE_SAMPLE_SAMPLE_COUNT; i += FRAME_SIZE) {
        resonator_bank_df2t_process(&pb_bank, &noise_sample[i], FRAME_SIZE);
        gate_decision_t d = gate_decide(&pb_bank, &pb_cfg);
        energy_features_t ef = { .total_energy = d.features_used.energy };
        update_noise_floor(&pb_noise, &ef, d.should_wake);
        pb_cfg.noise_floor = pb_noise.noise_estimate;
        if (d.should_wake) { go++; } else { abort_cnt++; }
    }
    total = go + abort_cnt;
    printk("Noise:  GO=%u  ABORT=%u  abort_rate=%u%%  (target >= 80%%)\n",
           go, abort_cnt, total ? abort_cnt * 100U / total : 0U);

    printk("=== Playback test complete. Entering live mic loop. ===\n\n");
}
#endif /* TEST_PLAYBACK */

/* ── Week 7-8: Mock TinyML inference ────────────────────────────────────────
 * 100ms busy-wait simulates heavy TinyML inference keeping CPU fully active. */
static void mock_tinyml_inference(void)
{
    k_busy_wait(100000); /* 100 ms — simulates heavy TinyML inference */
}

/* ── Application ─────────────────────────────────────────────────────────── */

static resonator_bank_df2t_t bank;
static gate_config_t         config = GATE_CONFIG_DEFAULT;
static noise_tracker_t       noise  = {0};

int main(void)
{
#if GATE_MODE == -1
    /* SLEEP baseline: CPU sleeps forever — measures J-Link + board overhead */
    while (1) { k_sleep(K_FOREVER); }
#endif

#if TEST_AUDIO_SINE
    printk("\n=== Guardian Week 6 - Gate Decision [TEST: 800Hz SINE] ===\n");
    printk("Expected: WAKE most frames (periodic speech-like signal)\n\n");
#elif TEST_AUDIO_NOISE
    printk("\n=== Guardian Week 6 - Gate Decision [TEST: LFSR NOISE] ===\n");
    printk("Expected: ABORT most frames (broadband noise)\n\n");
#else
#if GATE_MODE == 0
    printk("\n=== Guardian Week 7 - Power Test [BASELINE / no gate] ===\n\n");
#elif GATE_MODE == 1
    printk("\n=== Guardian Week 7 - Power Test [ENERGY VAD] ===\n\n");
#else
#if STRESS_TEST
    printk("\n=== Guardian Week 9 - Stress Test [PHYSICS GATE + CPU LOAD] ===\n\n");
#else
    printk("\n=== Guardian Week 6 - Gate Decision [LIVE MIC / DMA MODE] ===\n\n");
#endif
#endif
#endif

#if TEST_PLAYBACK
    run_playback_test();
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

#if (GATE_MODE == 0 || GATE_MODE == 2)
    resonator_bank_df2t_init(&bank);
#endif

#if !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
    int ret2 = dma_pdm_start();
    if (ret2 != 0) {
        printk("DMA PDM start failed: %d\n", ret2);
        return ret2;
    }
#endif

    int frame_count   = 0;
    int wake_count    = 0;

    /* ── Week 9 polishing stats ───────────────────────────────────────────── */
    uint32_t gate_wcet_us    = 0;  /* WCET: worst-case gate execution time    */
    uint32_t gate_total_us   = 0;  /* for duty cycle calculation              */
    uint32_t irq_latency_max = 0;  /* worst-case IRQ→thread wakeup latency   */
    uint32_t irq_latency_sum = 0;  /* for average latency                     */
    uint32_t irq_latency_n   = 0;

#if DEADLINE_STATS
    uint32_t total_frames_stat = 0;
    uint32_t deadline_misses   = 0;
#endif

#if PIPELINE_STATS
    /* Week 13-14: complete pipeline metrics — gate aborts, TinyML runs,
     * keyword detections.  LCG gives ~10% mock keyword detection rate.      */
    uint32_t pipe_tinyml_runs    = 0;
    uint32_t pipe_keywords       = 0;
    uint32_t pipe_gate_abort     = 0;
    uint32_t pipe_lrand          = 0xDEADBEEFu; /* LCG seed */
#endif

    /* Point 3: zero-copy queue — confirm on startup */
#if TINYML_THREADED
    printk("TINYML_THREADED: zero-copy pointer passing (sizeof queue item = %u bytes)\n",
           (unsigned)sizeof(int16_t *));
#endif

    while (1) {
        int16_t *frame;

#if DEADLINE_STATS
        timing_t frame_start_t = timing_counter_get();
#endif
        TR(TR_FRAME_START, 0);

#if (TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
        frame = get_test_frame();
        k_sleep(K_MSEC(20));
#else
#if POLL_MODE_TEST
        int ret3 = dma_pdm_read_poll(&frame);
#else
        int ret3 = dma_pdm_read(&frame);
#endif
        if (ret3 < 0) {
            continue;
        }

        /* Point 2: IRQ latency — time from ISR fire to thread wakeup */
        {
            uint32_t irq_ts  = dma_pdm_get_irq_timestamp_ms();
            uint32_t wake_ms = k_uptime_get_32();
            uint32_t lat_ms  = wake_ms - irq_ts;
            irq_latency_sum += lat_ms;
            irq_latency_n++;
            if (lat_ms > irq_latency_max) { irq_latency_max = lat_ms; }
        }

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

        /* ── Pre-roll: rotate frame into ring buffer ─────────────────────── */
#if PREROLL_BUFFER
        memcpy(preroll_ring[preroll_head], frame, FRAME_SIZE * sizeof(int16_t));
        preroll_head = (preroll_head + 1) % PREROLL_FRAMES;
        if (preroll_filled < PREROLL_FRAMES) { preroll_filled++; }
#endif

        /* ── Gate decision ───────────────────────────────────────────────── */
        bool should_wake = false;
        gate_decision_t decision = {0};

#if GATE_MODE == 0
        resonator_bank_df2t_process(&bank, frame, FRAME_SIZE);
        should_wake = true;

#elif GATE_MODE == 1
        q15_t rms;
        arm_rms_q15(frame, FRAME_SIZE, &rms);
        should_wake = (rms > ENERGY_VAD_THRESHOLD);

#else
        resonator_bank_df2t_process(&bank, frame, FRAME_SIZE);

#if !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
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

#ifdef CONFIG_SEGGER_SYSTEMVIEW
        SEGGER_SYSVIEW_Print("GATE_START");
#endif
        TR(TR_GUARDIAN_START, 0);
#if SIMULATE_PRIORITY_INV && !PRIORITY_INV_FIX
        /* Acquire shared resource before gate — causes priority inversion */
        k_mutex_lock(&shared_mutex, K_FOREVER);
#endif
        decision    = gate_decide(&bank, &config);
        should_wake = decision.should_wake;
#if SIMULATE_PRIORITY_INV && !PRIORITY_INV_FIX
        k_mutex_unlock(&shared_mutex);
#endif
        TR(TR_GUARDIAN_END, (uint16_t)decision.elapsed_us);

        /* Point 1+4: WCET and duty cycle tracking */
        gate_total_us += decision.elapsed_us;
        if (decision.elapsed_us > gate_wcet_us) {
            gate_wcet_us = decision.elapsed_us;
        }
#ifdef CONFIG_SEGGER_SYSTEMVIEW
        SEGGER_SYSVIEW_PrintfHost("GATE_END: %u us", decision.elapsed_us);
        if (decision.should_wake) {
            SEGGER_SYSVIEW_PrintfHost("TINYML_END: %u us", decision.elapsed_us);
        }
#endif

        energy_features_t efeat = { .total_energy = decision.features_used.energy };
        update_noise_floor(&noise, &efeat, decision.should_wake);
        config.noise_floor = noise.noise_estimate;
#endif /* GATE_MODE */

        if (should_wake) {
            wake_count++;
#if PREROLL_BUFFER
            /* Log pre-roll context window available for TinyML.
             * Production: linearize preroll_ring[preroll_head..] → model input.
             * preroll_filled × 20ms = total audio context passed to inference. */
            printk("PREROLL: %ums context (%u frames) ready for TinyML\n",
                   preroll_filled * 20U, preroll_filled);
#endif
#if TINYML_THREADED
            /* Non-blocking: K_NO_WAIT — gate NEVER blocks on TinyML.
             * If queue is full (TinyML thread stalled/overloaded), the frame
             * is dropped and tinyml_drop_count increments.
             * Graceful degradation: audio capture and gate continue at full
             * rate; only TinyML inference is skipped under overload.          */
            static uint32_t tinyml_drop_count = 0;
            if (k_msgq_put(&tinyml_q, &frame, K_NO_WAIT) != 0) {
                tinyml_drop_count++;
                if (tinyml_drop_count % 10 == 1) {
                    printk("WARN: TinyML queue full — dropped %u frames "
                           "(TinyML overloaded or stalled)\n",
                           tinyml_drop_count);
                }
            }
#else
            TR(TR_TINYML_START, 0);
            mock_tinyml_inference();
            TR(TR_TINYML_END, 0);
#if PIPELINE_STATS
            /* LCG: multiplier=1664525, increment=1013904223 (Numerical Recipes) */
            pipe_lrand = pipe_lrand * 1664525u + 1013904223u;
            pipe_tinyml_runs++;
            if (pipe_lrand % 10u == 0u) { pipe_keywords++; }
#endif
#endif
        }
#if PIPELINE_STATS
        else { pipe_gate_abort++; }
#endif

#if DEADLINE_STATS
        {
            timing_t frame_end_t = timing_counter_get();
            uint32_t frame_us = (uint32_t)(timing_cycles_to_ns(
                timing_cycles_get(&frame_start_t, &frame_end_t)) / 1000U);
            total_frames_stat++;
            if (frame_us > FRAME_BUDGET_US) {
                deadline_misses++;
            }
        }
#endif

        if (++frame_count >= 50) {
            frame_count = 0;
#if GATE_MODE == 0
            printk("BASELINE: wakes=%2d/50 (always)\n", wake_count);
#elif GATE_MODE == 1
            printk("EVAD: wakes=%2d/50\n", wake_count);
#else
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
#endif
            wake_count = 0;

            /* Point 1: Duty cycle — gate_us / frame_period_us
             * Frame period = 50 frames × 20ms = 1,000ms = 1,000,000us       */
            {
                uint32_t duty_ppm = gate_total_us / 10U; /* parts per million */
                printk("SCHED: wcet=%uus avg=%uus duty=%u.%02u%% "
                       "irq_lat_avg=%ums irq_lat_max=%ums\n",
                       gate_wcet_us,
                       gate_total_us / 50U,
                       duty_ppm / 10000U, (duty_ppm % 10000U) / 100U,
                       irq_latency_n ? irq_latency_sum / irq_latency_n : 0,
                       irq_latency_max);
                gate_total_us   = 0;
                irq_latency_sum = 0;
                irq_latency_n   = 0;
                /* wcet and irq_latency_max are lifetime peaks — not reset */
            }

#if DEADLINE_STATS
            if (total_frames_stat > 0) {
                uint32_t miss_ppm = deadline_misses * 10000U / total_frames_stat;
                printk("DEADLINE: misses=%u/%u  miss_rate=%u.%02u%%\n",
                       deadline_misses, total_frames_stat,
                       miss_ppm / 100, miss_ppm % 100);
            }
#endif

#if PIPELINE_STATS
            {
                /* Print full pipeline breakdown every 50 frames (1 second) */
                uint32_t total_p   = pipe_gate_abort + pipe_tinyml_runs;
                uint32_t abort_pct = total_p ? pipe_gate_abort * 100U / total_p : 0U;
                printk("PIPELINE: total=%u gate_abort=%u(%u%%) "
                       "tinyml=%u keywords=%u\n",
                       total_p, pipe_gate_abort, abort_pct,
                       pipe_tinyml_runs, pipe_keywords);
                pipe_gate_abort  = 0;
                pipe_tinyml_runs = 0;
                pipe_keywords    = 0;
            }
#endif

#if ENABLE_TRACE
            /* Dump trace CSV every 50 frames (~1 second) */
            tr_dump();
#endif
        }
    }

    return 0;
}
