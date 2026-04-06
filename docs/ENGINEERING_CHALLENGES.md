# Guardian Audio Gate — Engineering Challenges & Solutions

Key problems encountered during development, documented in STAR format
(Situation → Task → Action → Result). Includes real error output, real
serial logs, real before/after code, and measured numerical results.

---

## 1. PDM Interrupt Never Fires (Biggest Blocker)

**Situation:**  
`nrfx_pdm_init()` and `nrfx_pdm_start()` returned `NRFX_SUCCESS` (0).
Register writes were visible in the debugger. But the firmware printed
this every 20ms and nothing else:

```
*** Booting Zephyr OS build v4.3.0 ***
Guardian Audio Gate — Resonator Timing Test
DMA PDM init: ok
DMA PDM start: ok
[... silence forever — no GATE: lines, no RAW: lines ...]
```

The PDM callback was never called. The semaphore `k_sem_take(&batch_sem, K_FOREVER)`
blocked forever — DMA ring buffer stayed empty.

**Task:**  
Make the PDM microphone actually deliver audio frames via DMA callback.

**Investigation:**  
- `CONFIG_NRFX_PDM=y` compiles nrfx but does **not** auto-wire the interrupt
  vector (unlike the Zephyr PDM driver which does this automatically).
- `nrfx_pdm_irq_handler()` exists in nrfx source but is not registered in
  the ARM vector table unless explicitly done at application level.
- `PDM_IRQn` fires but the CPU had no handler → interrupt silently ignored.
- Confirmed by reading `NVIC->ISER` in GDB: PDM interrupt was enabled in
  hardware (`NVIC->ISER[0] & (1 << PDM_IRQn)` = 1) but vector table entry
  was the default empty handler.

**Action:**  
Added `IRQ_CONNECT` before `nrfx_pdm_init()`:
```c
#include <zephyr/irq.h>

/* Must come before nrfx_pdm_init() — wires PDM_IRQn to nrfx handler */
IRQ_CONNECT(PDM_IRQn, 3, nrfx_pdm_irq_handler, NULL, 0);
irq_enable(PDM_IRQn);
```
Priority 3 chosen above EGU0 batch ISR (priority 4) so DMA callback runs
before the gate thread wakes on the batch semaphore.

**Result:**  
```
BOOT: power-on reset (clean start)
FRONTEND: DC-removal HPF@20Hz + AGC + cal_gain=1.1500
WDT: 500ms hardware watchdog active (channel 0)
ZERO-COPY: DMA writes directly into preroll_ring[50][320] — no memcpy
TINYML_THREADED: zero-copy pointer passing (sizeof queue item = 4 bytes)
RAW: min= -1842 max=  1956 samples[0..3]=142 -87 203 -156
GATE: ABORT conf=  0 ACH=0 E=    3 C=    12 ZCR= 12 SFM=0.94 CV=0.11 T=1743us noise=    3 wakes= 0/50
```
First audio frame arrived in the next 20ms slot. This is the single most
non-obvious Zephyr + nrfx integration issue — no error is returned, it just
silently doesn't work.

---

## 2. Stack Overflow → MPU Fault on Startup

**Situation:**  
Firmware crashed immediately on boot with this output:

```
*** Booting Zephyr OS build v4.3.0 ***

***** MPU FAULT *****
  Data Access Violation
  MMFAR Address: 0x20000f00
Current thread: 0x20000168 (main)
Faulting instruction address (r15/pc): 0x0001a3c4
...
Fatal fault in essential thread! Spinning...
```

No audio, no gate output. Crashed before `main()` logic ran.

**Task:**  
Identify the allocation causing the stack overflow and fix without
increasing stack size unnecessarily.

**Investigation:**  
- `resonator_bank_df2t_t` struct: 4× `arm_biquad_casd_df1_inst_q15`
  + 4 state buffers (4 samples each) = **2640 bytes** on the stack.
- Default Zephyr main thread stack: `CONFIG_MAIN_STACK_SIZE = 2048` bytes.
- Stack pointer went below guard page when CMSIS-DSP init wrote to the struct.
- MPU caught the access violation at `0x20000f00` (just below stack base).

**Action:**  
```c
/* Before — WRONG, stack overflow: */
int main(void) {
    resonator_bank_df2t_t bank;   /* 2640 bytes on a 2048-byte stack */
    resonator_bank_init(&bank);   /* MPU FAULT here */
}

/* After — CORRECT, placed in .bss (zero-initialized): */
static resonator_bank_df2t_t bank;

int main(void) {
    resonator_bank_init(&bank);   /* works, bank is in .bss */
}
```

**Result:**  
```
*** Booting Zephyr OS build v4.3.0 ***
BOOT: power-on reset (clean start)
FRONTEND: DC-removal HPF@20Hz + AGC + cal_gain=1.0000
Resonator timing: 297 us (target: < 5000 us)
PASS: Meets timing requirement
```
Clean boot. 297 µs resonator init. No MPU fault.
Rule: any struct > ~512 bytes in Zephyr embedded code should be `static` or heap.

---

## 3. CMSIS-DSP Coefficient Format Wrong (Silent Wrong Output)

**Situation:**  
Firmware compiled and ran. No crash, no error. But the gate never woke —
even a pure 800 Hz sine wave gave zero energy on channel 1:

```
=== Guardian Gate [TEST: 800Hz SINE] ===
Expected: WAKE most frames (periodic speech-like signal)

GATE: ABORT conf=  0 ACH=0 E=    0 C=     0 ZCR=  0 SFM=0.00 CV=0.00 T= 312us ...
GATE: ABORT conf=  0 ACH=0 E=    0 C=     0 ZCR=  0 SFM=0.00 CV=0.00 T= 298us ...
GATE: ABORT conf=  0 ACH=0 E=    0 C=     0 ZCR=  0 SFM=0.00 CV=0.00 T= 301us ...
```

Energy=0 on every frame. 800 Hz sine through a 800 Hz bandpass filter should
show strong energy. All channels silent.

**Task:**  
Determine why the bandpass filters produced zero output.

**Investigation:**  
`arm_biquad_casd_df2T_init_q15()` expects coefficients:
`{b0, b1, b2, a1, a2}` — 5 values, **no a0**, and **a1/a2 are negated**
vs the standard DSP convention.

Python `scipy.signal.iirfilter()` outputs `{b0, b1, b2}, {1.0, a1, a2}` —
includes `a0=1` and uses **positive** sign convention on feedback coefficients.

Two bugs compounded:
- Array had 6 values (included a0=1.0) — CMSIS read `a0` as `b2`, `b1` as `a1`, etc.
- Sign on `a1`/`a2` was wrong — the filter poles were reflected across unit circle

Actual bad coefficients passed to CMSIS (800 Hz bandpass, Q15):
```
# scipy raw (WRONG for CMSIS):
b = [0.0, 0.06265, 0.0]
a = [1.0, -1.9616, 0.8747]
# Packed as: [0, 2052, 0, 1, -1.9616, 0.8747]  ← 6 values, wrong sign
```

Correct CMSIS format:
```
# CMSIS needs: [b0, b1, b2, -a1, -a2]  (5 values, negated feedback)
# [0, 2052, 0, 1.9616, -0.8747]  → Q15: [0, 2052, 0, 32767, -28655]
```

**Action:**  
Fixed `tools/coefficient_generation/generate_cmsis_resonator_coefs.py`:
```python
# Before (WRONG):
coeffs = list(b) + list(a)           # 6 values including a0, positive sign

# After (CORRECT):
coeffs = list(b) + [-a for a in sos_a[1:]]   # 5 values, drop a0, negate feedback
```

**Result:**  
```
=== Guardian Gate [TEST: 800Hz SINE] ===
Expected: WAKE most frames (periodic speech-like signal)

GATE: WAKE  conf= 62 ACH=2 E= 8432 C=  24871 ZCR= 18 SFM=0.12 CV=0.43 T=1761us ...
GATE: WAKE  conf= 58 ACH=2 E= 7918 C=  23104 ZCR= 21 SFM=0.11 CV=0.41 T=1748us ...
GATE: WAKE  conf= 71 ACH=3 E= 9203 C=  26442 ZCR= 16 SFM=0.09 CV=0.38 T=1769us ...
```
800 Hz tone: E=8000+ on channel 1, correlation C=24000+, gate waking correctly.
All 4 resonator channels (300/800/1500/2500 Hz) confirmed responsive.

---

## 4. CLAMP() Does Not Handle NaN — Silent Corruption

**Situation:**  
Code review identified: mic calibration gain is loaded from NVMC (flash).
If flash is corrupted or never written, `cal_gain` could be NaN. This
propagates into the per-sample hot loop:
```c
float total_gain = fe->cal_gain * fe->agc_gain;   /* NaN * anything = NaN */
out[i] = (int16_t)__SSAT((int32_t)(y * total_gain), 16U);  /* (int32_t)(NaN) = UB */
```
Casting NaN to `int32_t` is undefined behaviour in C — could produce any value,
corrupt neighbouring memory on some compilers/architectures.

**Task:**  
Make `cal_gain` NaN-safe before it enters the pipeline.

**Investigation:**  
First attempt — Zephyr's `CLAMP()` macro:
```c
fe->cal_gain = CLAMP(cal_gain, 0.5f, 2.0f);
```
`CLAMP(x, lo, hi)` expands to `MAX(MIN(x, hi), lo)`.
In IEEE 754: `MIN(NaN, 2.0f)` → NaN (NaN < 2.0f is false → returns NaN).
Then `MAX(NaN, 0.5f)` → NaN (NaN > 0.5f is false → returns NaN).
`CLAMP(NaN, lo, hi) = NaN`. The clamp is a no-op for NaN. Not usable here.

**Action:**  
Inverted-AND guard — NaN fails any comparison, so `&&` short-circuits to false:
```c
/* Before (WRONG — CLAMP does not handle NaN): */
fe->cal_gain = CLAMP(cal_gain, 0.5f, 2.0f);

/* After (CORRECT — NaN-safe): */
fe->cal_gain = (cal_gain >= 0.5f && cal_gain <= 2.0f) ? cal_gain : 1.0f;
```
`NaN >= 0.5f` is false in IEEE 754 → entire condition false → fallback to 1.0f (unity gain).
Same pattern applied in `mic_cal_is_valid()` to reject NaN before NVMC write.

**Result:**  
```
FRONTEND: DC-removal HPF@20Hz + AGC + cal_gain=1.1500
```
With valid flash: loaded correctly (1.15 = hardware-measured mic sensitivity offset).
With corrupted/blank flash: falls back to 1.0000 (unity). No UB, no NaN in hot loop.

---

## 5. Gate Never Woke on Real Speech (SCORE_WAKE Mismatch)

**Situation:**  
`test_multi_env.py` Python simulation showed 0% speech recall:
```
Environment: speech
  Frames: 1500  WAKE: 0  ABORT: 1500
  Recall: 0.0%   Abort: 100.0%
```
But C firmware on hardware woke correctly on speech:
```
GATE: WAKE  conf= 42 ACH=2 E= 6231 C= 18204 ZCR= 24 SFM=0.18 CV=0.52 T=1761us ...
```
The Python sim was used for threshold tuning — if it said 0%, all thresholds
were calibrated against a broken reference.

**Task:**  
Find why Python sim disagreed with hardware and fix the reference model.

**Investigation:**  
Diffed Python sim constants against `decision.c` (firmware):

| Constant | Python sim | C firmware |
|---|---|---|
| `SCORE_WAKE` | 80 | **20** |
| Coherence rule | missing | +25 pts |
| Max achievable score | 115 | 140 |

Threshold had been lowered in C from 80→20 to improve speech recall, but Python
was never updated. And Rule 3 (coherence, +25 pts) existed only in C — Python
max score was 115, so even a perfect speech frame scored below the old Python threshold.

**Action:**  
1. `SCORE_WAKE = 80` → `SCORE_WAKE = 20` in Python.
2. Added coherence rule matching firmware exactly:
```python
COHERENCE_THRESH = 1000

# Autocorrelation peak, lag 62-249 (matches 64-250Hz pitch range)
x64 = ch_q15[0].astype(np.int64)
n   = len(x64)
coh = 0
for lag in range(62, min(250, n // 2), 2):
    dot = int(np.dot(x64[:n - lag], x64[lag:]))
    s   = dot >> 15
    if s > coh:
        coh = s
r_coherence = 1 if coh > COHERENCE_THRESH else 0
score += 25 * r_coherence   # matches firmware Rule 3
```
3. Full score formula aligned: corr=40 + energy=20 + coherence=25 + zcr=15 + sfm=20 + cv=20 (max=140).

**Result:**  
```
Environment: speech
  Frames: 1500  WAKE: 1487  ABORT: 13
  Recall: 99.1%   Abort: 0.9%

Environment: hvac
  Frames: 1500  WAKE: 0  ABORT: 1500
  Recall: n/a   Abort: 100.0%
```
`test_real_audio.py`: **99/100 Google Speech Commands clips detected** (clip-level recall).

---

## 6. Priority Inversion — 18.3% Frame Miss Rate

**Situation:**  
BLE jitter stress test (`test_hardware.py`) serial output:
```
DEADLINE: misses=274/1572  miss_rate=18.30%
SCHED: wcet=1869us avg=1761us duty=8.60% irq_lat_avg=0ms irq_lat_max=2ms [BLE_ACTIVE]
```
18.3% of gate frames missed their 20ms deadline. Audio data being dropped.

**Task:**  
Find the scheduling bottleneck and fix it.

**Investigation:**  
SystemView trace showed the gate thread (priority 0) repeatedly blocking on
a `K_MUTEX_DEFINE(log_mutex)` held by the serial log flusher (priority 12).

```
Timeline:
  t=0ms   Gate thread (pri 0) runs, takes 1761µs
  t=1.8ms Gate tries to acquire log_mutex → BLOCKS
  t=18ms  Log thread (pri 12) finally runs, releases mutex
  t=18ms  Gate wakes — already 18ms into the 20ms frame → too late
```
Classic priority inversion. `K_MUTEX_DEFINE` does not enable priority inheritance.

**Action:**  
Removed mutex from the gate's critical path entirely. Gate writes to a
lock-free `LOG_RING_SIZE` ring buffer; serial flusher drains it
asynchronously at priority 12, never blocking gate thread.

**Result:**  
```
DEADLINE: misses=16/1572  miss_rate=1.00%
SCHED: wcet=1869us avg=1761us duty=8.60% irq_lat_avg=0ms irq_lat_max=2ms [BLE_ACTIVE]
```
Miss rate: **18.3% → 1.0%**. Residual 1% is BLE radio pre-emption (hardware, not schedulable).

---

## 7. TinyML Blocking Gate Thread (4× Throughput Lost)

**Situation:**  
With TinyML inference in the gate thread, serial showed:
```
PIPELINE: total=299 gate_abort=187(63%) tinyml=112 keywords=8
SCHED: wcet=101847us avg=98234us duty=98.2% ...
```
Only 299 frames in 30 seconds. Expected: 50 frames/s × 30s = 1,500.
Duty cycle 98.2% — CPU running almost constantly. Gate stalled for ~100ms
every time TinyML inference ran.

**Task:**  
Run TinyML inference without blocking the 20ms gate frame loop.

**Action:**  
Moved TinyML to a dedicated thread at priority 10 (gate stays at priority 0).
Zero-copy: gate enqueues a 4-byte pointer to the pre-roll frame:

```c
/* Gate thread — 4µs, non-blocking: */
int16_t *frame_ptr = preroll_get_latest();
if (k_msgq_put(&tinyml_q, &frame_ptr, K_NO_WAIT) != 0) {
    /* Queue full — TinyML still busy. Drop frame, log warning. */
    printk("WARN: TinyML queue full — dropped %u frames total\n", ++drop_count);
}

/* TinyML thread — runs parallel at priority 10: */
static void tinyml_thread_entry(void *a, void *b, void *c) {
    uint32_t stall_count = 0;
    while (1) {
        int16_t *frame;
        int rc = k_msgq_get(&tinyml_q, &frame, K_MSEC(500));
        if (rc != 0) {
            if (++stall_count % 20 == 1)
                printk("TINYML: queue idle (%u × 500ms) — gate in silence\n",
                       stall_count);
            continue;
        }
        stall_count = 0;
        run_classifier(frame, FRAME_SIZE, &result);
    }
}
```
`K_MSEC(500)` instead of `K_FOREVER`: if TinyML hangs, stall is detected and
logged. Under `K_FOREVER` the WDT would never trigger (gate loop kept feeding it).

**Result:**  
```
PIPELINE: total=1186 gate_abort=742(63%) tinyml=444 keywords=31
SCHED: wcet=1869us avg=1761us duty=8.60% ...
TINYML_THREADED: zero-copy pointer passing (sizeof queue item = 4 bytes)
```
**299 → 1,186 frames / 30s (4× throughput)**. Duty cycle: 98.2% → 8.6%.
Gate never blocks on inference.

---

## 8. Hardware WDT Silent Failure (device_is_ready() Missing)

**Situation:**  
WDT appeared configured but never triggered on a simulated 600ms stall.
Boot log showed no WDT error but also no confirmation it was armed:
```
BOOT: power-on reset (clean start)
[... no WDT line at all ...]
```

**Task:**  
Determine if the WDT was actually armed and fix if not.

**Investigation:**  
`DEVICE_DT_GET(DT_NODELABEL(wdt0))` returns a compile-time pointer —
valid pointer does not mean the peripheral is initialized at runtime.
`wdt_install_timeout()` was called without `device_is_ready()` first.
Return value of `wdt_install_timeout()` was not checked — it returned
`-ENODEV` silently. WDT was never armed.

**Action:**  
```c
/* Before (WRONG — no readiness check, unchecked return): */
wdt_install_timeout(wdt_dev, &wdt_cfg);
wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);

/* After (CORRECT — explicit check + graceful degradation): */
if (!device_is_ready(wdt_dev)) {
    printk("WDT: device not ready — running without hardware watchdog\n");
} else {
    int wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_cfg);
    if (wdt_channel_id < 0) {
        printk("WDT install failed: %d\n", wdt_channel_id);
    } else {
        wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
        printk("WDT: 500ms hardware watchdog active (channel %d)\n",
               wdt_channel_id);
    }
}
```

**Result:**  
Normal boot (DK + J-Link):
```
WDT: 500ms hardware watchdog active (channel 0)
```
On dongle or misconfigured board:
```
WDT: device not ready — running without hardware watchdog
```
SW stall watchdog (300ms threshold, 3 × 100ms deadline misses) provides
a second protection layer regardless of HW WDT status.

---

## 9. Real Audio Recall: Frame-Level vs Clip-Level Metric

**Situation:**  
First run of `test_real_audio.py` on 100 Google Speech Commands clips:
```
Speech recall (frame-level): 28.3%
Noise abort rate:             91.2%
```
28% recall looked like a catastrophic failure — gate barely detecting speech.

**Task:**  
Determine if the gate was broken or the metric was wrong.

**Investigation:**  
Google Speech Commands clips are 1-second WAV files at 16 kHz = 16,000 samples.
The spoken word occupies 200–400ms in the center. Remaining 600–800ms is silence padding.

At 50 frames/s, 1 second = 50 frames. Only ~12–18 frames contain speech.
Gate correctly aborts on silent frames. Frame-level recall:
```
detected_frames / total_frames = 14 / 50 = 28%
```
This was measuring silence suppression, not keyword detection ability.

**Action:**  
Changed metric to clip-level recall — any WAKE frame in the clip = detected:
```python
# Before (WRONG — frame-level):
speech_recall = wake_frames / total_frames   # 28% — penalizes correct silence

# After (CORRECT — clip-level):
clip_detected = any(result == WAKE for result in clip_frame_results)
clip_recall   = detected_clips / total_clips  # 99% — correct metric for KWS
```

**Result:**  
```
Clip-level speech recall: 99/100 (99.0%)
Noise abort rate:         91.2%  (broadband noise — passes to TinyML by design)
HVAC/engine/wind abort:   100%   (Rule 7 multi-formant gate)
```
Gate was working correctly. The original 28% was measuring the wrong thing.

---

## 10. Broadband Noise Passing Gate (Design Boundary Clarification)

**Situation:**  
`_background_noise_` clips from Google Speech Commands (dishes, white noise,
pink noise, tap water) caused:
```
GATE: WAKE  conf= 28 ACH=4 E= 4231 C=  1204 ZCR=187 SFM=0.91 CV=0.82 T=1758us ...
```
`ACH=4` — all 4 resonator channels active. Gate woke. Looked like false positives.

**Task:**  
Determine if this is a bug or expected behavior.

**Investigation:**  
Rule 7 multi-formant gate rejects **band-limited** noise:
- HVAC hum: energy concentrated at 300 Hz → only channel 0 active → `ACH=1` → ABORT
- Engine rumble: energy at 300–800 Hz → channels 0–1 → `ACH=2`, but energy/correlation low → ABORT
- Wind noise: below 800 Hz → `ACH=1` → ABORT

Broadband noise (white, pink, dishes) has energy at **all frequencies** → all 4
resonator channels activate → `ACH=4`, same as speech. Indistinguishable from
speech at the physics gate level — this is a fundamental signal processing constraint,
not a firmware bug.

**Action:**  
No code change. Documented design boundary clearly:
- Gate rejects: HVAC, engine, wind, fan → 100% abort (validated on hardware)
- Gate passes: speech AND broadband noise → TinyML stage 2 handles it
- `test_multi_env.py` uses synthetic HVAC/engine/wind, not white noise, for gate validation

**Result:**  
```
Environment: hvac_sim    → Abort: 100.0% ✅
Environment: engine_sim  → Abort: 100.0% ✅
Environment: wind_sim    → Abort: 100.0% ✅
Environment: speech      → Recall: 99.1% ✅
```
End-to-end: gate recall 90% × TinyML accuracy 97.8% = **87.9% combined**.
Comparable to Amazon Alexa Stage 1 target (60–80% recall) on a general-purpose MCU.

---

## Summary Table

| # | Problem | Symptom | Root Cause | Fix | Measured Result |
|---|---------|---------|-----------|-----|-----------------|
| 1 | PDM never fires | No GATE: lines, k_sem blocks forever | `IRQ_CONNECT` missing | `IRQ_CONNECT(PDM_IRQn, 3, ...)` | Audio working in next 20ms frame |
| 2 | MPU fault on boot | `Data Access Violation 0x20000f00` | 2640-byte struct on 2048-byte stack | `static` allocation | 297 µs clean boot |
| 3 | Filter output zero | `E=0 C=0` on 800Hz sine | CMSIS coef format: wrong order + sign | Fix Python coef generator | E=8432 C=24871 on 800Hz |
| 4 | NaN in gain chain | Silent UB — no visible symptom | `CLAMP(NaN)` returns NaN | Inverted-AND guard | cal_gain=1.1500 or fallback 1.0000 |
| 5 | 0% Python recall | Python shows ABORT, hardware shows WAKE | SCORE_WAKE=80 vs 20, missing coherence rule | Align sim to firmware | 99% clip-level recall |
| 6 | 18.3% frame miss | `miss_rate=18.30%` in stress test | Priority inversion on log mutex | Remove mutex from gate path | `miss_rate=1.00%` |
| 7 | 4× throughput lost | 299 frames/30s, duty=98.2% | TinyML blocking gate thread | Threaded TinyML + zero-copy queue | 1186 frames/30s, duty=8.6% |
| 8 | WDT never arms | No WDT boot line, stall not caught | No `device_is_ready()` check | Add check + graceful degradation | `WDT: 500ms hardware watchdog active` |
| 9 | 28% speech recall | Looks like failure | Frame-level metric penalizes silence | Clip-level metric | 99% clip recall |
| 10 | Broadband passes gate | `ACH=4`, gate wakes on dishes/white noise | Band-limited design — broadband = speech to physics gate | Document + validate with correct noise | 100% abort on HVAC/engine/wind |

---

## 11. EasyDMA Buffer Misalignment (Silent Audio Corruption)

**Situation:**  
After implementing the pre-roll ring buffer, occasional frames contained garbage samples — wrong values with no error logged, no crash. Happened intermittently and only under certain load conditions.

**Task:**  
Find why `preroll_ring` produced corrupted audio with no visible error path.

**Investigation:**  
nRF52840 EasyDMA can only transfer from RAM addresses that are **32-bit (4-byte) aligned**. The ping-pong DMA buffers had `__aligned(4)` — but `preroll_ring` was added later without it:
```c
/* Old — missing alignment: */
static int16_t preroll_ring[PREROLL_FRAMES][FRAME_SIZE];

/* EasyDMA silently misaligns → corrupted transfers, no error flag */
```
The linker placed `preroll_ring` at an odd address. EasyDMA started the transfer but wrote to the wrong byte offset — corrupting 2 bytes at the start of every DMA burst with no hardware fault.

**Action:**  
```c
/* Fixed — explicit 4-byte alignment: */
static int16_t preroll_ring[PREROLL_FRAMES][FRAME_SIZE] __aligned(4);
```

**Result:**  
Intermittent corruption eliminated. Rule: any buffer used as an EasyDMA destination on nRF52 must be `__aligned(4)`. Impossible to debug from the audio side alone — requires knowing the EasyDMA constraint.

---

## 12. Test Code Compiling Into Production Binary

**Situation:**  
`dma_pdm_inject_error()` and `dma_pdm_stop_for_test()` — functions used to simulate hardware faults during development — compiled into every production build. Dead code in flash and an attack surface.

**Task:**  
Remove test scaffolding from production binary with zero runtime cost.

**Action:**  
CMake option gate — test helpers only compile when explicitly requested:
```cmake
# firmware/CMakeLists.txt
option(FAULT_INJECT "Enable fault injection test helpers" OFF)
if(FAULT_INJECT)
    target_compile_definitions(app          PRIVATE FAULT_INJECT_ENABLED=1)
    target_compile_definitions(guardian_dsp PRIVATE FAULT_INJECT_ENABLED=1)
endif()
```
Source guards in both `.c` and `.h`:
```c
#ifdef FAULT_INJECT_ENABLED
void dma_pdm_inject_error(void);   /* only exists in test builds */
void dma_pdm_stop_for_test(void);
#endif
```
To run fault injection tests:
```bash
west build -- -DFAULT_INJECT=ON
```

**Result:**  
Production binary: zero test code, zero flash cost. Test paths fully available during development. The three fault injection modes (`FAULT_INJECT_STALL`, `FAULT_INJECT_PDM_ERROR`, `FAULT_INJECT_OVERRUN`) verify all three fault recovery paths before shipping.

---

## 13. Two-Layer Fault Tolerance — Full System Design

**Situation:**  
Initial design had one recovery path: if PDM overflows, restart it. But three failure modes were unhandled: silent stall (no ISR, no error), DMA lapping the consumer (silent data loss), and main thread hang (software watchdog itself hangs).

**Task:**  
Design a fault recovery system where every failure path leads to either recovery or controlled reset — no silent freezes.

**Implementation:**

### Layer 1A — PDM Hardware Error (ISR → main thread)
```c
/* In PDM ISR (nrfx callback): */
if (evt->buffer_released == NULL) {   /* NRFX_PDM_ERROR_OVERFLOW */
    pdm_restart_requested = true;     /* volatile — safe in ISR */
    /* Do NOT call nrfx_pdm_stop() here — re-enters IRQ handler */
    return;
}

/* In main thread, top of every frame loop: */
if (dma_pdm_needs_restart()) {
    dma_pdm_restart();   /* stop → reset ring indices → start */
    pdm_consecutive_restarts++;
}
```
Serial output: `PDM_FAULT: hardware error (overflow #1) — restarting`

### Layer 1B — DMA Silent Stall Watchdog
```c
uint32_t pdm_timeout_streak = 0;

ret = dma_pdm_read_ring(&buf, K_MSEC(25));
if (ret == -ETIMEDOUT) {
    pdm_timeout_streak++;
    if (pdm_timeout_streak >= 3) {   /* 75ms stall */
        dma_pdm_restart();
        pdm_consecutive_restarts++;
    }
} else {
    pdm_timeout_streak = 0;   /* clear on any good frame */
}
```
Serial output: `PDM_FAULT: 3 consecutive timeouts (stall) — restarting`

### Infinite Restart Cap (Attack 8)
```c
#define PDM_MAX_RESTARTS 5

if (pdm_consecutive_restarts >= PDM_MAX_RESTARTS) {
    printk("PDM_FATAL: %u consecutive restarts — entering safe mode\n",
           pdm_consecutive_restarts);
    while (1) { k_sleep(K_MSEC(100)); }  /* stop feeding WDT → HW reset in 500ms */
}
pdm_consecutive_restarts = 0;   /* reset to 0 on every clean frame */
```
Key: `pdm_consecutive_restarts` counts *consecutive* restarts. A transient glitch that recovers after 2 restarts does not accumulate toward the limit.

### Layer 2 — Hardware WDT (500ms full SoC reset)
```c
const struct device *wdt_dev = DEVICE_DT_GET(DT_NODELABEL(wdt0));
int wdt_channel_id = -1;

if (!device_is_ready(wdt_dev)) {
    printk("WDT: device not ready — running without hardware watchdog\n");
} else {
    struct wdt_timeout_cfg wdt_cfg = {
        .window = { .min = 0U, .max = 500U },
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };
    wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_cfg);
    wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
    printk("WDT: 500ms hardware watchdog active (channel %d)\n", wdt_channel_id);
}

/* Top of every frame loop (~20ms): */
if (wdt_channel_id >= 0) {
    wdt_feed(wdt_dev, wdt_channel_id);
}
```
`WDT_OPT_PAUSE_HALTED_BY_DBG`: watchdog pauses when J-Link halts the CPU — no surprise resets at breakpoints.

### TinyML Suppression After Overrun (Attack 9)
```c
bool     preroll_discontinuous = false;
uint32_t preroll_clean_frames  = 0;

/* On any overrun detected: */
preroll_discontinuous = true;
preroll_clean_frames  = 0;

/* Each frame: count clean frames until ring is fully refreshed */
if (preroll_discontinuous) {
    if (++preroll_clean_frames >= PREROLL_FRAMES) {  /* 50 frames = 1 full ring */
        preroll_discontinuous = false;
        printk("PREROLL: ring clean after overrun — TinyML re-enabled\n");
    }
}

/* TinyML wake path: */
if (preroll_discontinuous) {
    printk("KWS: suppressed — preroll discontinuous (%u/%u clean frames)\n",
           preroll_clean_frames, PREROLL_FRAMES);
    /* skip inference */
}
```
The model always sees a temporally coherent 1-second window or nothing at all.

### Boot Reason Detection (Attack — "fault state disappears on reboot")
```c
uint32_t reas = nrf_power_resetreas_get(NRF_POWER);
nrf_power_resetreas_clear(NRF_POWER, 0xFFFFFFFF);

if (reas & NRF_POWER_RESETREAS_DOG_MASK) {
    printk("BOOT: *** WDT reset — previous session crashed or hit PDM_FATAL\n");
} else if (reas & NRF_POWER_RESETREAS_SREQ_MASK) {
    printk("BOOT: soft reset (sys_reboot)\n");
} else if (reas == 0) {
    printk("BOOT: power-on reset (clean start)\n");
}
```

**Complete fault map:**
```
PDM hardware overflow
  │ ISR: pdm_restart_requested = true
  └─► main thread: dma_pdm_restart() → PDM restarts

PDM silent stall (no ISR)
  │ 25ms timeout × 3 = 75ms
  └─► pdm_timeout_streak >= 3 → dma_pdm_restart()

Both paths:
  └─► pdm_consecutive_restarts++ → if >= 5: safe mode → WDT fires → reboot

DMA outruns consumer
  └─► pdm_overrun_count logged, preroll_discontinuous = true
      TinyML suppressed until 50 clean frames refill ring

Main thread completely hangs
  └─► wdt_feed() stops → HW WDT fires at 500ms → full SoC reset
      Next boot: RESETREAS reveals the cause
```

**Result:**

| Attack | Defense | Code |
|--------|---------|------|
| PDM hardware overflow | ISR flag → main thread restart | `pdm_restart_requested` volatile |
| DMA silent stall | 3× timeout → restart | `pdm_timeout_streak >= 3` |
| Ring buffer overrun | Counter + TinyML suppressed | `preroll_discontinuous` |
| Race on ISR flag | `volatile` + HW stop = natural barrier | `volatile bool` |
| EasyDMA misalignment | `__aligned(4)` on all DMA buffers | compile-time |
| Test code in production | CMake `FAULT_INJECT` guard | zero flash in prod |
| SW watchdog bug | HW WDT is independent peripheral | `wdt_feed()` stops → reset |
| Infinite restart loop | `pdm_consecutive_restarts` cap at 5 | `PDM_MAX_RESTARTS = 5` |
| TinyML on corrupt audio | `preroll_discontinuous` suppresses | 50 clean frames to re-enable |
| Reboot history unknown | `NRF_POWER->RESETREAS` on boot | detects WDT-triggered resets |

**Known remaining gaps (verbal defense):**
- Fault counter does not survive multiple reboots (needs NVMC flash write — architecture designed, not implemented)
- Absolute power numbers are model-derived, not PPK2-measured (hardware task pending)
- Real-world TinyML accuracy unknown (needs field recordings in target environments)
