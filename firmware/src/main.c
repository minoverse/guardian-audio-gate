#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>
#include <zephyr/drivers/watchdog.h>
#include <hal/nrf_power.h>
#include "guardian/audio/preprocess.h"
#include "guardian/audio/mic_cal.h"
#include "guardian/features/modulation.h"
#include <arm_math.h>
#include "guardian/audio/dma_pdm.h"
#include "guardian/resonator_df2t.h"
#include "guardian/gate/decision.h"
#ifdef CONFIG_SEGGER_SYSTEMVIEW
#include <SEGGER_SYSVIEW.h>
#endif

/* ── ISR Priority Table ───────────────────────────────────────────────────────
 * | Peripheral          | IRQ                | Priority | Max Duration | Notes                  |
 * |---------------------|--------------------|----------|--------------|------------------------|
 * | PDM EasyDMA         | PDM_IRQn           | 3        | ~2µs         | Pointer swap only      |
 * | EGU0 batch wakeup   | SWI0_EGU0_IRQn     | 4        | ~5µs         | k_sem_give × BATCH_N   |
 * | BLE Radio           | RADIO_IRQn         | 1        | ~50µs        | Nordic BLE stack       |
 * | UART TX             | UARTE0_UART0_IRQn  | 5        | <5µs         | Non-blocking           |
 * | Hardware WDT feed   | (main thread)      | —        | <1µs         | Register write         |
 *
 * Rule: PDM ISR does ZERO processing — pointer update + return (<2µs).
 *       EGU0 ISR gives batch_sem × 4 (<5µs) — less time-critical than PDM.
 *       BLE at priority 1 preempts PDM (3) during radio events — measured
 *       jitter impact: ±200µs worst-case gate WCET shift (see BLE jitter test).
 *
 * PPI chain (zero CPU cycles per PDM frame within a batch):
 *   PDM EVENTS_END ──[PPI CH0]──► TIMER1 TASKS_COUNT (hardware only)
 *   TIMER1 CC[4] compare  ──[PPI CH1]──► EGU0 TRIGGER → EGU0 ISR (CPU)
 * ─────────────────────────────────────────────────────────────────────────── */

/* ── BLE jitter stress test ───────────────────────────────
 * When CONFIG_BLE_JITTER_TEST=y, starts a non-connectable BLE beacon at
 * 100ms advertising interval. nRF52840 radio IRQ fires every 100ms, stealing
 * ~300-500µs from the CPU — directly competing with gate processing.
 *
 * Build: west build ... -- -DEXTRA_CONF_FILE=prj_ble_jitter.conf
 * Compare SCHED output (wcet/avg) with and without BLE to measure jitter hit.
 * ─────────────────────────────────────────────────────────────────────────── */
#ifdef CONFIG_BT
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 0xFF, 0xFF,
                  'G','u','a','r','d','i','a','n'),
};

static void ble_beacon_start(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("BLE enable failed: %d\n", err);
        return;
    }
    /* 100ms advertising interval — maximises radio IRQ frequency */
    struct bt_le_adv_param adv_param =
        BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY,
                             BT_GAP_ADV_FAST_INT_MIN_2,   /* 100ms */
                             BT_GAP_ADV_FAST_INT_MAX_2,   /* 150ms */
                             NULL);
    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("BLE adv start failed: %d\n", err);
        return;
    }
    printk("BLE_JITTER: beacon active (100ms interval) — radio IRQ now "
           "competing with gate\n");
}
#endif /* CONFIG_BT */

/* ── DMA vs Polling power comparison ────────────────────────────
 * Set POLL_MODE_TEST 1 to use busy-wait polling (CPU never sleeps).
 * Set POLL_MODE_TEST 0 (default) for DMA sleep mode (CPU sleeps between frames).
 * Flash each binary, measure PPK2 current, compare.
 * ─────────────────────────────────────────────────────────────────────────── */
/* ── Fault injection test flags ──────────────────────────────────────────────
 * FAULT_INJECT_STALL 1 — injects a real PDM stop after frame 5 (one-shot).
 *   Expected output: "PDM_FAULT: 3 consecutive timeouts" → restart → resume.
 *
 * FAULT_INJECT_PDM_ERROR 1 — directly sets pdm_restart_requested after frame 5,
 *   simulating an NRFX_PDM_ERROR_OVERFLOW from the ISR.
 *   Expected output: "PDM_FAULT: hardware error (overflow #1)" → restart → resume.
 *
 * FAULT_INJECT_OVERRUN 1 — blocks main thread 1100ms after frame 5 (one-shot).
 *   Ring holds 50 frames × 20ms = 1000ms. A 1100ms block forces DMA to lap the
 *   consumer, triggering ~5 overruns. No crash — oldest frames silently dropped.
 *   Expected output: PDM_HEALTH: overruns=5 (approx).
 *
 * Set back to 0 after verifying each recovery path.                           */
/* Fault injection only available when -DFAULT_INJECT=ON passed to west build.
 * Production build: FAULT_INJECT_ENABLED undefined → all three are forced 0
 * and test helpers in dma_pdm.c are not compiled in.                         */
#ifndef FAULT_INJECT_ENABLED
#define FAULT_INJECT_STALL     0
#define FAULT_INJECT_PDM_ERROR 0
#define FAULT_INJECT_OVERRUN   0
#else
#define FAULT_INJECT_STALL     0   /* set to 1 to test stall watchdog  */
#define FAULT_INJECT_PDM_ERROR 0   /* set to 1 to test hw error path   */
#define FAULT_INJECT_OVERRUN   0   /* set to 1 to test overrun counter */
#endif

/* ── Fault tolerance: restart cap + safe mode ────────────────────────────────
 * After PDM_MAX_RESTARTS consecutive restarts with no clean frame in between,
 * the PDM hardware is considered unrecoverable (e.g., loose mic connector,
 * wrong pin config, or dead peripheral).  Entering an infinite restart loop
 * at full power would drain the battery in minutes.
 *
 * Safe mode: stop calling wdt_feed() → hardware WDT fires after 500ms →
 * one clean SoC reset attempt.  If the hardware is still broken after reboot,
 * the system reaches this point again quickly, resulting in a slow ~600ms
 * reset cycle (CPU idle between sleeps) instead of full-speed spinning.
 *
 * Counter resets to 0 on every successful frame — transient hardware glitches
 * that recover within MAX_RESTARTS do not trigger safe mode.                 */
#define PDM_MAX_RESTARTS 5

#define POLL_MODE_TEST 0

/* ── Gate mode configuration ────────────────────
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

/* ── Trace logging ────────────────────────────────────────────
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
#define ENABLE_TRACE    0
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

/* ── Real TinyML — Edge Impulse keyword spotting ─────────────────
 * EI_TINYML 1 — replaces LCG mock with a real Edge Impulse MFCC + NN model
 * trained on Google Speech Commands ("yes", "no", "unknown", "noise").
 *
 * Requires PREROLL_BUFFER=1 — uses last 1000ms (50 frames) as model input.
 * Pre-roll is automatically extended to 50 frames when EI_TINYML=1.
 *
 * Expected output on wake:
 *   KWS: yes      87%   ← keyword detected with confidence
 *   KWS: noise    91%   ← gate woke but model says background noise
 *
 * Set EI_TINYML 0 to revert to LCG mock (no model, no flash overhead).
 * ─────────────────────────────────────────────────────────────────────────── */
#define EI_TINYML 1

#if EI_TINYML
#include "ei_classifier_wrapper.h"
#endif

/* ── Hysteresis + hold-time ───────────────────────────────
 * Problem: gate fires KWS on every single wake frame — even a 20ms noise
 * spike triggers a full ~15ms TinyML inference call, wasting power.
 *
 * Fix — two counters work together:
 *   HYSTERESIS_FRAMES (3 × 20ms = 60ms): require N *consecutive* wake frames
 *     before calling KWS. Single-frame spikes are ignored.
 *   HOLD_FRAMES (10 × 20ms = 200ms): after KWS fires, suppress further calls
 *     for 200ms so a continuous utterance doesn't re-trigger inference 10×.
 *
 * Power benefit (measured):
 *   Without: up to ~15ms/frame inference during speech
 *   With:    one inference per utterance (1 call / ~300ms utterance)
 *            Savings ≈ (utterance_frames - 1) × inference_ms / frame_period
 *               ≈  (15 - 1) × 15ms / 20ms ≈ 10.5 ms saved per utterance
 * ─────────────────────────────────────────────────────────────────────────── */
#define HYSTERESIS_FRAMES 3   /* consecutive wake frames before KWS trigger   */
#define HOLD_FRAMES       25  /* frames to suppress re-trigger after KWS call */

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
#if EI_TINYML
#define PREROLL_BUFFER 1   /* auto-enabled: EI needs ring buffer context */
#else
#define PREROLL_BUFFER 0
#endif

#if PREROLL_BUFFER
#if EI_TINYML
#define PREROLL_FRAMES 50   /* 1000ms = EI_CLASSIFIER_RAW_SAMPLE_COUNT @ 16kHz */
#else
#define PREROLL_FRAMES 25   /* 500ms @ 20ms/frame — covers any wakeword */
#endif
static int16_t  preroll_ring[PREROLL_FRAMES][FRAME_SIZE] __aligned(4); /* EasyDMA requires 4-byte alignment */
static uint32_t preroll_head   = 0;   /* next write slot (circular) */
static uint32_t preroll_filled = 0;   /* frames stored (caps at PREROLL_FRAMES) */
#endif

/* ── On-device playback test ─────────────────────────────────────────────────
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

/* ── Pipeline statistics ─────────────────────────────────────────
 * PIPELINE_STATS 1 — tracks TinyML run count + keyword detections (10% mock
 * rate via LCG) and prints a PIPELINE: line every 50 frames.
 * Keep TINYML_THREADED 0 for this mode so keyword detection stays in main loop.
 *
 * Expected output:
 *   PIPELINE: total=50 gate_abort=35(70%) tinyml=15 keywords=2
 * ─────────────────────────────────────────────────────────────────────────── */
#define PIPELINE_STATS 0

/* ── Problem simulations ────────────────────────────────────
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

/* ── CPU stress thread ───────────────────────────────────────────────────
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
/* Fix: gate runs in system workqueue (cooperative, no preemption mid-gate).
 * When enabled, submit gate_work to k_work_submit() instead of running inline.
 * Requires implementing the work handler — left as future optimisation.       */
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

/* ── Threaded TinyML ─────────────────────────
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

/* Maximum time to wait for a TinyML job. Gate fires at most once per
 * HYSTERESIS_FRAMES × 20ms = 60ms, so 500ms is ~8× the normal inter-job
 * interval.  If the queue is silent that long, it is a stall, not normal
 * quiet operation (gate simply stopped waking).
 * On timeout: log once, then continue — main thread is unaffected and WDT
 * keeps being fed by the main loop, so the system stays alive.             */
#define TINYML_QUEUE_TIMEOUT_MS  500

static void tinyml_thread_entry(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    int16_t *frame;
    uint32_t stall_count = 0;
    while (1) {
        int rc = k_msgq_get(&tinyml_q, &frame, K_MSEC(TINYML_QUEUE_TIMEOUT_MS));
        if (rc != 0) {
            /* Timeout: gate has not fired for 500ms.  This is normal in
             * silence.  Log at 10s intervals to avoid flooding the console. */
            stall_count++;
            if (stall_count % 20 == 1) {   /* every 20 × 500ms = 10s        */
                printk("TINYML: queue idle (%u × %dms) — gate in silence\n",
                       stall_count, TINYML_QUEUE_TIMEOUT_MS);
            }
            continue;
        }
        stall_count = 0;
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

/* ── On-device playback test ─────────────────────────────────────────────── */
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

    printk("\n=== Guardian On-Device Playback Test ===\n");
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

/* ── Mock TinyML inference ────────────────────────────────────────
 * 100ms busy-wait simulates heavy TinyML inference keeping CPU fully active. */
static void mock_tinyml_inference(void)
{
    k_busy_wait(100000); /* 100 ms — simulates heavy TinyML inference */
}

/* ── Application ─────────────────────────────────────────────────────────── */

static resonator_bank_df2t_t bank;
static gate_config_t         config = GATE_CONFIG_DEFAULT;
static noise_tracker_t       noise  = {0};

/* ── Production audio front-end ──────────────────────────────────────────────
 * Initialized at startup after loading NVMC calibration gain.
 * processed_frame: DC-removed + AGC-normalized copy of each raw DMA frame.
 *   Passed to resonator bank; preroll ring stays raw for TinyML (EI SDK
 *   performs its own MFCC preprocessing on the ring buffer).
 * mod_state: cross-frame energy modulation tracker for gate rule 6.          */
static audio_frontend_t  frontend;
static int16_t           processed_frame[FRAME_SIZE];
static modulation_state_t mod_state;

int main(void)
{
#if GATE_MODE == -1
    /* SLEEP baseline: CPU sleeps forever — measures J-Link + board overhead */
    while (1) { k_sleep(K_FOREVER); }
#endif

#if TEST_AUDIO_SINE
    printk("\n=== Guardian Gate [TEST: 800Hz SINE] ===\n");
    printk("Expected: WAKE most frames (periodic speech-like signal)\n\n");
#elif TEST_AUDIO_NOISE
    printk("\n=== Guardian Gate [TEST: LFSR NOISE] ===\n");
    printk("Expected: ABORT most frames (broadband noise)\n\n");
#else
#if GATE_MODE == 0
    printk("\n=== Guardian Power Test [BASELINE / no gate] ===\n\n");
#elif GATE_MODE == 1
    printk("\n=== Guardian Power Test [ENERGY VAD] ===\n\n");
#else
#if STRESS_TEST
    printk("\n=== Guardian Stress Test [PHYSICS GATE + CPU LOAD] ===\n\n");
#else
    printk("\n=== Guardian Gate [LIVE MIC / DMA MODE] ===\n\n");
#endif
#endif
#endif

#if TEST_PLAYBACK
    run_playback_test();
#endif

    /* ── Boot diagnostics: reset reason ─────────────────────────────────────────
     * NRF_POWER->RESETREAS survives the reset that triggered it.
     * Reading it on every boot tells us whether the previous session crashed.
     * WDT bit set → previous run either hit PDM_FATAL safe mode or the main
     * thread truly hung (layer 2 fired).  Either way it is actionable.
     * Register must be cleared manually — bits are sticky until explicitly 0'd. */
    {
        uint32_t reas = nrf_power_resetreas_get(NRF_POWER);
        nrf_power_resetreas_clear(NRF_POWER, 0xFFFFFFFFU);
        if (reas & NRF_POWER_RESETREAS_DOG_MASK) {
            printk("BOOT: *** WDT reset — previous session crashed or hit "
                   "PDM_FATAL safe mode ***\n");
        } else if (reas & NRF_POWER_RESETREAS_SREQ_MASK) {
            printk("BOOT: soft reset (sys_reboot)\n");
        } else if (reas == 0) {
            printk("BOOT: power-on reset (clean start)\n");
        } else {
            printk("BOOT: reset reason = 0x%08X\n", reas);
        }
    }

    timing_init();
    timing_start();

#ifdef CONFIG_BT
    ble_beacon_start();
#endif

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

    /* ── Audio front-end: calibration + DC removal + AGC ───────────────────
     * Load mic sensitivity calibration from NVMC flash.  If no calibration
     * has been programmed (fresh board), unity gain (1.0) is used silently.
     * For factory calibration: play 1kHz @ 94 dB SPL, measure RMS, compute
     * cal_gain = NOMINAL_RMS / measured_rms, call mic_cal_save(cal_gain).   */
    {
        float cal_gain;
        mic_cal_load(&cal_gain);
        audio_frontend_init(&frontend, cal_gain);
        modulation_init(&mod_state);
        printk("FRONTEND: DC-removal HPF@20Hz + AGC + cal_gain=%.4f\n",
               (double)cal_gain);
    }

#if PREROLL_BUFFER && !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
    /* ── Zero-copy ring: hand pre-roll array directly to DMA driver ──────────
     * DMA hardware writes each frame into the next preroll_ring slot.
     * No memcpy ever touches the audio data — the PDM peripheral IS the
     * ring buffer writer.  CPU cost: 0 bytes copied per frame (was 640).     */
    dma_pdm_set_ring(&preroll_ring[0][0], PREROLL_FRAMES, FRAME_SIZE);
    printk("ZERO-COPY: DMA writes directly into preroll_ring[%d][%d] "
           "(0 bytes memcpy/frame, was %u bytes)\n",
           PREROLL_FRAMES, FRAME_SIZE,
           (unsigned)(FRAME_SIZE * sizeof(int16_t)));
#endif

#if !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
    int ret2 = dma_pdm_start();
    if (ret2 != 0) {
        printk("DMA PDM start failed: %d\n", ret2);
        return ret2;
    }
#endif

    /* ── Hardware watchdog (layer 2) ────────────────────────────────────────────
     * Layer 1 (software stall watchdog, 75ms): restarts PDM peripheral.
     * Layer 2 (hardware WDT, 500ms): full SoC reset if main thread hangs and
     *   layer 1 never fires — e.g., stuck in a spinlock or deep fault.
     *
     * wdt_feed() called once per frame (every ~20ms) — well within 500ms window.
     * WDT_OPT_PAUSE_HALTED_BY_DBG: pauses WDT during J-Link halt so the debugger
     *   doesn't reset the chip every time a breakpoint is hit.                  */
    static const struct device *wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));
    /* Zephyr device model: always call device_is_ready() before using a
     * device pointer.  DEVICE_DT_GET() returns a valid pointer at compile
     * time but the driver may not have initialised at runtime.              */
    int wdt_channel_id = -1;
    if (!device_is_ready(wdt_dev)) {
        printk("WDT: device not ready — running without hardware watchdog\n");
    } else {
        struct wdt_timeout_cfg wdt_cfg = {
            .window   = { .min = 0U, .max = 500U },
            .callback = NULL,
            .flags    = WDT_FLAG_RESET_SOC,
        };
        wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_cfg);
        if (wdt_channel_id < 0) {
            printk("WDT install failed: %d\n", wdt_channel_id);
        } else {
            wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
            printk("WDT: 500ms hardware watchdog active (channel %d)\n",
                   wdt_channel_id);
        }
    }

    int frame_count   = 0;
    int wake_count    = 0;

#if EI_TINYML
    /* Hysteresis + hold-time state */
    uint32_t consecutive_wakes = 0;
    uint32_t hold_frames_left  = 0;
#endif

    /* ── Runtime statistics ───────────────────────────────────────────── */
    uint32_t gate_wcet_us    = 0;  /* WCET: worst-case gate execution time    */
    uint32_t gate_total_us   = 0;  /* for duty cycle calculation              */
    uint32_t irq_latency_max = 0;  /* worst-case IRQ→thread wakeup latency   */
    uint32_t irq_latency_sum = 0;  /* for average latency                     */
    uint32_t irq_latency_n   = 0;

    /* ── Switching frequency power model ────────────────────
     * Duty cycle alone understates power cost — every SLEEP→WAKE transition
     * draws a current spike (HFXO startup ~0.5ms @ ~8mA = 4µJ per transition).
     *
     * Power model:
     *   P_total = P_active + P_transition
     *   P_active    = I_active × V × duty_cycle
     *                 = 6.5mA × 3.3V × duty_cycle
     *   P_transition= I_spike × V × t_startup × wake_rate_hz
     *                 = 8mA × 3.3V × 0.0005s × (wakes_per_hour / 3600)
     *
     * Printed every 3600 frames (~72 seconds) for a 1-minute power snapshot. */
    uint32_t power_wakes_total  = 0;  /* lifetime wake transitions             */
    uint32_t power_frame_start  = k_uptime_get_32(); /* ms timestamp          */
    uint32_t power_report_frames= 0;  /* frames since last power report        */

    /* ── Fault tolerance counters ───────────────────────────────────────────
     * pdm_timeout_streak      : consecutive timeouts → stall watchdog (layer 1).
     * pdm_restart_count       : lifetime PDM restarts — reported in SCHED line.
     * pdm_consecutive_restarts: restarts without a clean frame in between.
     *   Caps at PDM_MAX_RESTARTS → safe mode (stop feeding WDT → HW reset).
     *
     * ── TinyML discontinuity guard ───────────────────────────────────────────
     * When a ring buffer overrun occurs, preroll_ring contains audio from
     * before and after the gap — temporal discontinuity.  Feeding this to
     * TinyML produces spurious results.  preroll_discontinuous suppresses
     * inference until PREROLL_FRAMES consecutive clean frames refill the ring.
     * overrun_snapshot tracks pdm_overrun_count at last check to detect new
     * overruns on the current frame.                                          */
    uint32_t pdm_timeout_streak       = 0;
    uint32_t pdm_restart_count        = 0;
    uint32_t pdm_consecutive_restarts = 0;

    bool     preroll_discontinuous = false;
    uint32_t preroll_clean_frames  = 0;
    uint32_t overrun_snapshot      = 0;

#if DEADLINE_STATS
    uint32_t total_frames_stat = 0;
    uint32_t deadline_misses   = 0;
#endif

#if PIPELINE_STATS
    /* Pipeline metrics: gate aborts, TinyML runs, keyword detections.
     * LCG gives ~10% mock keyword detection rate.                            */
    uint32_t pipe_tinyml_runs    = 0;
    uint32_t pipe_keywords       = 0;
    uint32_t pipe_gate_abort     = 0;
    uint32_t pipe_lrand          = 0xDEADBEEFu; /* LCG seed */
#endif

    /* Startup: confirm zero-copy queue item size */
#if TINYML_THREADED
    printk("TINYML_THREADED: zero-copy pointer passing (sizeof queue item = %u bytes)\n",
           (unsigned)sizeof(int16_t *));
#endif

    while (1) {
        int16_t *frame;

        /* Feed hardware WDT every frame (~20ms, window=500ms).
         * If main thread ever hangs, WDT fires after 500ms → SoC reset. */
        if (wdt_channel_id >= 0) {
            wdt_feed(wdt_dev, wdt_channel_id);
        }

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
#elif PREROLL_BUFFER
        /* ── Fault: ISR-flagged hardware error ──────────────────────────────
         * PDM overflow sets pdm_restart_requested in the ISR (safe: flag only).
         * We handle it here in the main thread where stop/start is allowed.  */
        if (dma_pdm_needs_restart()) {
            pdm_restart_count++;
            pdm_consecutive_restarts++;
            printk("PDM_FAULT: hardware error (overflow #%u) — restarting "
                   "(%u consecutive)\n",
                   dma_pdm_get_error_count(), pdm_consecutive_restarts);
            if (pdm_consecutive_restarts >= PDM_MAX_RESTARTS) {
                /* Hardware unrecoverable — enter safe mode.
                 * Stop feeding WDT → HW WDT fires after 500ms → clean reboot.
                 * Avoids full-speed restart loop draining battery.            */
                printk("PDM_FATAL: %u consecutive restarts, hardware "
                       "unrecoverable — safe mode (WDT reset in 500ms)\n",
                       pdm_consecutive_restarts);
                while (1) { k_sleep(K_MSEC(100)); }
            }
            dma_pdm_restart();
            preroll_head   = 0;
            preroll_filled = 0;
            pdm_timeout_streak = 0;
            preroll_discontinuous = true;
            preroll_clean_frames  = 0;
            modulation_reset(&mod_state);   /* stale cross-frame history invalid */
            continue;
        }

        /* Zero-copy path: DMA already wrote into ring — just get pointer */
        uint32_t dma_head = 0;
        int ret3 = dma_pdm_read_ring(&frame, &dma_head);
        if (ret3 > 0) {
            /* Ring head is managed by DMA driver — sync our tracking vars */
            preroll_head   = (dma_head + 1) % PREROLL_FRAMES;
            if (preroll_filled < PREROLL_FRAMES) { preroll_filled++; }
            pdm_timeout_streak       = 0;  /* got a frame — clear stall counter */
            pdm_consecutive_restarts = 0;  /* successful frame: hardware alive  */

            /* ── TinyML discontinuity tracking ──────────────────────────────
             * Check if a new overrun occurred since the last frame.
             * If yes: mark ring as discontinuous, reset clean-frame counter.
             * If no and ring was discontinuous: count clean frames until the
             *   ring has been fully refreshed (PREROLL_FRAMES clean frames)
             *   — then clear the flag and allow TinyML to fire again.        */
            uint32_t cur_overruns = dma_pdm_get_overrun_count();
            if (cur_overruns != overrun_snapshot) {
                overrun_snapshot      = cur_overruns;
                preroll_discontinuous = true;
                preroll_clean_frames  = 0;
            } else if (preroll_discontinuous) {
                preroll_clean_frames++;
                if (preroll_clean_frames >= PREROLL_FRAMES) {
                    preroll_discontinuous = false;
                    preroll_clean_frames  = 0;
                    printk("PREROLL: ring clean after overrun — TinyML re-enabled\n");
                }
            }

#if FAULT_INJECT_STALL
            /* Inject a real PDM stall after frame 5 (one-shot).
             * Stops the PDM peripheral and drains the semaphore so
             * dma_pdm_read_ring() actually times out — a CPU k_sleep alone
             * does NOT work because DMA keeps running during CPU sleep and
             * fills the semaphore, preventing timeouts.
             * Expected: 3 × 25ms = 75ms → "PDM_FAULT: stall" → restart.   */
            {
                static bool stall_injected = false;
                if (frame_count == 5 && !stall_injected) {
                    stall_injected = true;
                    printk("FAULT_INJECT: stopping PDM to simulate hardware stall\n");
                    dma_pdm_stop_for_test();
                }
            }
#endif

#if FAULT_INJECT_PDM_ERROR
            /* Inject a fake PDM hardware error after frame 5 (one-shot).
             * Sets the same flag the ISR would set on NRFX_PDM_ERROR_OVERFLOW.
             * Expected: next frame sees dma_pdm_needs_restart()==true → restart. */
            {
                static bool error_injected = false;
                if (frame_count == 5 && !error_injected) {
                    error_injected = true;
                    printk("FAULT_INJECT: injecting fake PDM hardware error\n");
                    dma_pdm_inject_error();
                }
            }
#endif

#if FAULT_INJECT_OVERRUN
            /* Block main thread 1100ms after frame 5 (one-shot).
             * Ring = 50 frames × 20ms = 1000ms capacity.
             * 1100ms block → DMA fills 55 frames → laps consumer ~5 times.
             * Expected: PDM_HEALTH overruns≈5, no crash, audio resumes.     */
            {
                static bool overrun_injected = false;
                if (frame_count == 5 && !overrun_injected) {
                    overrun_injected = true;
                    printk("FAULT_INJECT: blocking 1100ms to force ring overrun\n");
                    k_busy_wait(1100000U);  /* 1100ms — longer than 50-frame ring */
                }
            }
#endif
        }
#else
        int ret3 = dma_pdm_read(&frame);
#endif
        if (ret3 < 0) {
#if PREROLL_BUFFER
            /* ── Fault: DMA timeout / stall watchdog ────────────────────────
             * dma_pdm_read_ring() times out after 25ms (> 20ms frame period).
             * One timeout is survivable (scheduler jitter, BLE radio).
             * Three consecutive timeouts (60ms) = hardware stall — restart.  */
            pdm_timeout_streak++;
            if (pdm_timeout_streak >= 3) {
                pdm_restart_count++;
                pdm_consecutive_restarts++;
                printk("PDM_FAULT: %u consecutive timeouts (stall) — restarting "
                       "(%u consecutive)\n",
                       pdm_timeout_streak, pdm_consecutive_restarts);
                if (pdm_consecutive_restarts >= PDM_MAX_RESTARTS) {
                    printk("PDM_FATAL: %u consecutive restarts, hardware "
                           "unrecoverable — safe mode (WDT reset in 500ms)\n",
                           pdm_consecutive_restarts);
                    while (1) { k_sleep(K_MSEC(100)); }
                }
                dma_pdm_restart();
                preroll_head   = 0;
                preroll_filled = 0;
                pdm_timeout_streak = 0;
                preroll_discontinuous = true;
                preroll_clean_frames  = 0;
                modulation_reset(&mod_state);
            }
#endif
            continue;
        }

        /* IRQ latency — time from ISR fire to thread wakeup */
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

        /* ── Pre-roll: zero-copy — DMA already wrote into ring slot ─────────
         * Zero-copy path: when PREROLL_BUFFER=1, the
         * DMA driver writes directly into preroll_ring[] via dma_pdm_set_ring().
         * preroll_head / preroll_filled are updated in the dma_pdm_read_ring()
         * block above.  For TEST_AUDIO_SINE/NOISE, copy is still needed.      */
#if PREROLL_BUFFER && (TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
        memcpy(preroll_ring[preroll_head], frame, FRAME_SIZE * sizeof(int16_t));
        preroll_head = (preroll_head + 1) % PREROLL_FRAMES;
        if (preroll_filled < PREROLL_FRAMES) { preroll_filled++; }
#endif

        /* ── Gate decision ───────────────────────────────────────────────── */
        bool should_wake = false;
        gate_decision_t decision = {0};

#if !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
        /* ── Audio front-end preprocessing ─────────────────────────────────
         * Apply to raw DMA frame → processed_frame before resonator.
         * Preroll ring stays raw: EI SDK handles its own MFCC preprocessing.
         * Steps: (1) mic calibration gain  (2) DC HPF @ 20Hz  (3) AGC
         *
         * Return value: -EINVAL only if pointers are NULL or len=0.
         * All three args are statically allocated — this fires only on a
         * programmer error.  __ASSERT panics in DEBUG, compiles out in
         * RELEASE (CONFIG_ASSERT=n).                                         */
        {
            int _fe_err = audio_frontend_process(
                              &frontend, frame, processed_frame, FRAME_SIZE);
            __ASSERT(_fe_err == 0, "audio_frontend_process: %d", _fe_err);
            (void)_fe_err;
        }
#else
        /* Test audio path: synthetic signals are already clean — copy as-is */
        arm_copy_q15(frame, processed_frame, FRAME_SIZE);
#endif

#if GATE_MODE == 0
        resonator_bank_df2t_process(&bank, processed_frame, FRAME_SIZE);
        should_wake = true;

#elif GATE_MODE == 1
        q15_t rms;
        arm_rms_q15(processed_frame, FRAME_SIZE, &rms);
        should_wake = (rms > ENERGY_VAD_THRESHOLD);

#else
        resonator_bank_df2t_process(&bank, processed_frame, FRAME_SIZE);

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
        decision    = gate_decide(&bank, &config, &mod_state);
        should_wake = decision.should_wake;
#if SIMULATE_PRIORITY_INV && !PRIORITY_INV_FIX
        k_mutex_unlock(&shared_mutex);
#endif
        TR(TR_GUARDIAN_END, (uint16_t)decision.elapsed_us);

        /* WCET and duty cycle tracking */
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
            power_wakes_total++;
            power_report_frames++;

#if EI_TINYML
            /* ── Hysteresis + hold-time ────────────────────────────────────────
             * Only call KWS when HYSTERESIS_FRAMES consecutive wakes arrive
             * and we are not already in a hold window.                        */
            if (hold_frames_left > 0) {
                hold_frames_left--;
                /* suppress: still in hold window after last KWS call */
            } else {
                consecutive_wakes++;
                if (consecutive_wakes >= HYSTERESIS_FRAMES) {
                    /* Confident speech onset — run KWS once */
                    consecutive_wakes = 0;
                    hold_frames_left  = HOLD_FRAMES;
#if PREROLL_BUFFER
                    if (preroll_discontinuous) {
                        /* Ring contains audio with a temporal gap (overrun
                         * occurred — DMA lapped consumer).  Feeding this to
                         * TinyML produces unreliable results.  Skip until
                         * PREROLL_FRAMES clean frames refill the ring.       */
                        printk("KWS: suppressed — preroll discontinuous "
                               "(%u/%u clean frames)\n",
                               preroll_clean_frames, PREROLL_FRAMES);
                    } else {
                    printk("PREROLL: %ums context (%u frames) ready for TinyML\n",
                           preroll_filled * 20U, preroll_filled);
#endif
                    ei_kws_result_t kws = {0};
                    int rc = ei_classify((const int16_t *)preroll_ring,
                                        preroll_head, preroll_filled,
                                        PREROLL_FRAMES, FRAME_SIZE, &kws);
                    if (rc == 0) {
                        printk("KWS: %-8s %d%%\n",
                               kws.label, (int)(kws.score * 100.0f));
                    } else {
                        printk("KWS: inference error %d\n", rc);
                    }
#if PREROLL_BUFFER
                    } /* end: !preroll_discontinuous */
#endif
                }
            }
#elif TINYML_THREADED
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
        } else {
#if EI_TINYML
            /* Gate aborted — reset hysteresis counter, let hold window drain */
            consecutive_wakes = 0;
            if (hold_frames_left > 0) { hold_frames_left--; }
#endif
#if PIPELINE_STATS
            pipe_gate_abort++;
#endif
        }

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
            printk("GATE: %s conf=%3u ACH=%u E=%5d C=%6d ZCR=%3u "
                   "SFM=%.2f CV=%.2f T=%4uus noise=%5d wakes=%2d/50\n",
                   decision.should_wake ? "WAKE " : "ABORT",
                   decision.confidence,
                   decision.features_used.active_ch,
                   decision.features_used.energy,
                   decision.features_used.correlation,
                   decision.features_used.zcr,
                   (double)decision.features_used.spectral_flatness,
                   (double)decision.features_used.modulation_cv,
                   decision.elapsed_us,
                   config.noise_floor,
                   wake_count);
            /* FRONTEND line: AGC gain and calibration — printed every 50 frames.
             * AGC=1.00 at startup; rises toward 4.0 in silence, drops on loud
             * speech. CAL shows loaded mic calibration gain (1.00 = unity).    */
            printk("FRONTEND: agc_gain=%.2f agc_env=%.4f cal=%.4f\n",
                   (double)frontend.agc_gain,
                   (double)frontend.agc_envelope,
                   (double)frontend.cal_gain);
#endif
            wake_count = 0;

            /* Duty cycle — gate_us / frame_period_us
             * Frame period = 50 frames × 20ms = 1,000ms = 1,000,000us       */
            {
                /* duty% = gate_total_us / 1,000,000µs (50 frames × 20ms)
                 * display as X.XX%: integer = /10000, decimal = %10000/100 */
                uint32_t duty_ppm = gate_total_us; /* µs in 1s window = ppm */
                printk("SCHED: wcet=%uus avg=%uus duty=%u.%02u%% "
                       "irq_lat_avg=%ums irq_lat_max=%ums"
#ifdef CONFIG_BT
                       " [BLE_ACTIVE]"
#endif /* CONFIG_BT */
                       "\n",
                       gate_wcet_us,
                       gate_total_us / 50U,
                       duty_ppm / 10000U, (duty_ppm % 10000U) / 100U,
                       irq_latency_n ? irq_latency_sum / irq_latency_n : 0,
                       irq_latency_max);
                uint32_t gate_total_snapshot = gate_total_us; /* save before reset */
                gate_total_us   = 0;
                irq_latency_sum = 0;
                irq_latency_n   = 0;
                /* wcet and irq_latency_max are lifetime peaks — not reset */

                /* ── Fault tolerance health log ─────────────────────────────
                 * Only prints when something abnormal has occurred — silent
                 * in normal operation so it doesn't clutter the output.      */
#if PREROLL_BUFFER && !(TEST_AUDIO_SINE || TEST_AUDIO_NOISE)
                {
                    uint32_t errs = dma_pdm_get_error_count();
                    uint32_t ovrs = dma_pdm_get_overrun_count();
                    if (errs > 0 || ovrs > 0 || pdm_restart_count > 0) {
                        printk("PDM_HEALTH: hw_errors=%u overruns=%u "
                               "restarts=%u\n",
                               errs, ovrs, pdm_restart_count);
                    }
                }
#endif

                /* ── Switching frequency power model (corrected) ────────────
                 * Three components (nRF52840 PS v1.8 Table 55 + 56):
                 *
                 *  P_base   = always-on peripherals (regardless of gate):
                 *               PDM EasyDMA running:  ~500 µA
                 *               HFXO (required for PDM 1.28 MHz): ~300 µA
                 *               CPU sleep (WFI between frames):     ~1 µA
                 *             Total baseline = 801 µA × 3.3V = 2643 µW
                 *
                 *  P_gate   = incremental CPU active cost during gate decision:
                 *               (I_active - I_base) × VDD × duty_cycle
                 *               (6500 - 801)µA × 3300µV × duty  ≈ 18,630 µW/duty
                 *             Duty = gate_total_us / 1,000,000
                 *             → P_gate = gate_total_us × 18630 / 1,000,000
                 *
                 *             Note: previous model used 6500µA (not incremental)
                 *             → overstated gate cost, understated baseline.
                 *
                 *  P_trans  = HFXO restart per WAKE event (batch path eliminates
                 *             this — HFXO never stops with PPI batch wakeup).
                 *             Kept for comparison: 8mA × 3.3V × 0.5ms = 13.2µJ/wake
                 *
                 * Report every 50 frames (1 second window).                 */
/* Baseline power constants (µW) — see comment above for derivation       */
#define POWER_BASE_UW    2643U   /* PDM+HFXO+sleep, always on             */
#define POWER_GATE_COEFF 18630U  /* incremental CPU active µW per duty-1  */
                {
                    uint32_t now_ms      = k_uptime_get_32();
                    uint32_t elapsed_ms  = now_ms - power_frame_start;
                    if (elapsed_ms == 0) { elapsed_ms = 1; }

                    /* wakes per second (×100 for 2 decimal places) */
                    uint32_t wakes_per_sec_x100 =
                        power_report_frames * 100000U / elapsed_ms;

                    /* P_gate (µW): incremental CPU cost above baseline.
                     * duty_fraction = gate_total_us / 1,000,000
                     * P_gate = POWER_GATE_COEFF × gate_total_us / 1,000,000 */
                    uint32_t p_gate_uw = (uint32_t)(
                        (uint64_t)gate_total_snapshot * POWER_GATE_COEFF / 1000000U);

                    /* P_transition (µW): HFXO restart cost per WAKE.
                     * With PPI batch wakeup, HFXO stays on → P_trans ≈ 0.
                     * Retained for reference: 13.2µJ × wakes/s.
                     * ×100 fixed point: 1320 × wakes_x100 / 10000          */
                    uint32_t p_trans_uw  = 1320U * wakes_per_sec_x100 / 10000U;

                    uint32_t p_total_uw  = POWER_BASE_UW + p_gate_uw + p_trans_uw;

                    printk("POWER: wakes=%u/s(x100=%u) "
                           "P_base=%uuW P_gate=%uuW P_trans=%uuW P_total=%uuW "
                           "lifetime_wakes=%u\n",
                           wakes_per_sec_x100 / 100U,
                           wakes_per_sec_x100,
                           POWER_BASE_UW, p_gate_uw, p_trans_uw, p_total_uw,
                           power_wakes_total);

                    power_report_frames = 0;
                    power_frame_start   = now_ms;
                }
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
