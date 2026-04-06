# Guardian Audio Gate — Engineering Challenges & Solutions

Key problems encountered during development, documented in STAR format
(Situation → Task → Action → Result). Each entry covers a real failure,
what was tried, and what finally worked.

---

## 1. PDM Interrupt Never Fires (Biggest Blocker)

**Situation:**  
`nrfx_pdm_init()` and `nrfx_pdm_start()` returned success. Register writes
were visible in the debugger. But the PDM callback was never called — the DMA
ring buffer stayed empty forever. No audio data ever arrived.

**Task:**  
Make the PDM microphone actually deliver audio frames via DMA callback.

**Investigation:**  
- Confirmed `CONFIG_NRFX_PDM=y` was set — compiles nrfx but does **not**
  auto-wire the interrupt vector (unlike the Zephyr PDM driver which does).
- Checked nrfx source: `nrfx_pdm_irq_handler()` exists but is never connected
  to the ARM interrupt table unless explicitly done at application level.
- Without a connected ISR, `PDM_IRQn` fires into the void — CPU ignores it.

**Action:**  
Added `IRQ_CONNECT` before `nrfx_pdm_init()`:
```c
#include <zephyr/irq.h>
IRQ_CONNECT(PDM_IRQn, 3, nrfx_pdm_irq_handler, NULL, 0);
irq_enable(PDM_IRQn);
```
Priority set to 3 (higher than EGU0 batch ISR at priority 4) to ensure
the DMA callback runs before the gate thread wakes.

**Result:**  
First audio frame arrived immediately. DMA ring buffer filled correctly.
This is the single most non-obvious Zephyr + nrfx integration issue in the project.

---

## 2. Stack Overflow → MPU Fault on Startup

**Situation:**  
Firmware crashed immediately on boot with:
```
***** MPU FAULT *****
Data Access Violation
MMFAR Address: 0x20000f00
```
No useful stack trace — the fault occurred before `main()` logic ran.

**Task:**  
Identify the allocation causing the stack overflow and fix it without
increasing stack size unnecessarily.

**Investigation:**  
- `resonator_bank_df2t_t` struct holds 4 CMSIS-DSP `arm_biquad_casd_df1_inst_q15`
  instances + state buffers = **2640 bytes** on the stack.
- Default Zephyr main thread stack: 2048 bytes.
- Stack pointer went out of bounds when the CMSIS-DSP init function wrote
  to the struct — MPU caught the access violation immediately.

**Action:**  
Changed from stack to static allocation:
```c
// Before (WRONG — stack overflow):
int main(void) {
    resonator_bank_df2t_t bank;
    ...
}

// After (CORRECT — .bss, zero-init):
static resonator_bank_df2t_t bank;
int main(void) {
    ...
}
```

**Result:**  
Clean boot. Resonator initialized in 297 µs. No MPU fault.
Lesson: any struct > ~512 bytes in embedded code should be static or heap.

---

## 3. CMSIS-DSP Coefficient Format Wrong (Silent Wrong Output)

**Situation:**  
Firmware compiled and ran. Resonator bank produced output but gate never
woke on speech — even a pure 800 Hz tone gave zero energy on channel 1.
No error, no crash.

**Task:**  
Determine why the bandpass filters produced wrong output.

**Investigation:**  
- `arm_biquad_casd_df2T_init_q15()` expects coefficients in the order:
  `{b0, b1, b2, a1, a2}` — note: **no a0** (normalized), and **a coefficients
  are negated** compared to standard DSP convention.
- Python `scipy.signal.iirfilter()` outputs `{b0, b1, b2, 1, a1, a2}` —
  includes `a0=1` and uses **positive** a coefficients.
- Two bugs compounded: extra coefficient in array (6 values, expected 5),
  and wrong signs on a1/a2.

**Action:**  
Fixed coefficient generation script to output CMSIS format:
```python
# scipy output: [b0, b1, b2], [1.0, a1, a2]
# CMSIS needs: [b0, b1, b2, -a1, -a2]  (drop a0, negate feedback)
cmsis_coeffs = list(b) + [-a for a in sos_a[1:]]
```
Converted to Q15 fixed-point with appropriate scaling.

**Result:**  
800 Hz tone produced strong energy on channel 1 (expected). All 4 resonator
channels responded correctly to their target frequencies.

---

## 4. CLAMP() Does Not Handle NaN — Silent Corruption

**Situation:**  
During code review, discovered that mic calibration gain loaded from NVMC
could theoretically be NaN (corrupted flash). This would propagate into
`total_gain = cal_gain × agc_gain` in the per-sample hot loop, corrupting
every output sample silently — `(int32_t)(NaN)` is undefined behaviour in C.

**Task:**  
Make cal_gain NaN-safe at the point it enters the audio pipeline.

**Investigation:**  
Attempted fix using Zephyr's `CLAMP()` macro:
```c
fe->cal_gain = CLAMP(cal_gain, 0.5f, 2.0f);
```
But `CLAMP(NaN, lo, hi)` expands to `MAX(MIN(NaN, 2.0f), 0.5f)`.
`MIN(NaN, 2.0f)` → NaN (NaN comparison is always false → conditional returns NaN).
`MAX(NaN, 0.5f)` → NaN. The clamp is a no-op for NaN.

**Action:**  
Replaced with inverted-AND guard:
```c
fe->cal_gain = (cal_gain >= 0.5f && cal_gain <= 0.2f) ? cal_gain : 1.0f;
```
`NaN >= 0.5f` is false in IEEE 754 → condition fails → fallback to 1.0f (unity gain).
Applied same pattern in `mic_cal_is_valid()` to reject NaN before NVMC write.

**Result:**  
NaN can never reach `total_gain`. No UB in the per-sample loop regardless
of flash contents. Pattern also documented in code comments explaining why
`CLAMP()` was not used.

---

## 5. Gate Never Woke on Real Speech (SCORE_WAKE Mismatch)

**Situation:**  
Python simulation (`test_multi_env.py`) showed 0% speech recall. C firmware
on hardware woke correctly on speech. The simulation was trusted for threshold
tuning — but it was producing the wrong answer.

**Task:**  
Find why the Python sim disagreed with hardware.

**Investigation:**  
- Python had `SCORE_WAKE = 80`.
- C firmware `decision.c` had `SCORE_WAKE = 20`.
- Threshold had been lowered in C (from 80 to 20 to improve recall) but
  the Python sim was never updated.
- Additionally, the Python sim was missing the coherence rule entirely
  (Rule 3, +25 pts). C firmware had it; Python didn't — so maximum
  achievable score in Python was 115 instead of 140.

**Action:**  
1. Aligned `SCORE_WAKE = 20` in Python.
2. Added coherence computation matching firmware exactly:
   - Autocorrelation via dot product over lag range 62–249 samples
   - Threshold: 1000 (Q15 dot-product >> 15 units)
   - Score contribution: +25 pts
3. Verified score formula: corr=40 + energy=20 + coherence=25 + zcr=15 + sfm=20 + cv=20.

**Result:**  
Python sim now matches firmware: 100% abort on band-limited noise,
100% speech recall. `test_real_audio.py` achieved 99% clip-level recall
on Google Speech Commands dataset.

---

## 6. Priority Inversion — 18% Frame Miss Rate

**Situation:**  
Stress test (`test_hardware.py`) showed 18.3% gate frame miss rate during
a 30-second BLE jitter run. Gate frames were being dropped — audio data lost.

**Task:**  
Find the scheduling bottleneck causing the gate thread to miss its 20ms deadline.

**Investigation:**  
- SystemView trace showed the gate thread (priority 0) blocking on a mutex.
- The mutex was held by a low-priority logging thread (priority 12) during
  serial output flush.
- Classic priority inversion: low-priority thread owns resource needed by
  high-priority thread.
- Zephyr's priority inheritance was not configured for this mutex
  (`K_MUTEX_DEFINE` does not enable PI by default in this config).

**Action:**  
Removed the mutex from the gate's critical path entirely. Serial logging
moved to a lock-free approach — gate thread writes to a ring buffer,
logger thread drains it asynchronously.

**Result:**  
Frame miss rate dropped from 18.3% to ~1.0% (residual from BLE radio
pre-emption, acceptable). Gate runs uncontested at priority 0.

---

## 7. TinyML Blocking Gate Thread (4× Throughput Lost)

**Situation:**  
With TinyML inference (100ms) in the gate thread, only 299 frames were
processed in 30 seconds instead of the expected 1,500 (50 frames/s × 30s).
The gate was stalling for 100ms every time it woke — missing all audio
during inference.

**Task:**  
Run TinyML inference without blocking the gate thread.

**Action:**  
Moved TinyML to a separate thread at priority 10 (below gate at priority 0).
Gate enqueues a 4-byte pointer to the pre-roll frame — zero memcpy:
```c
// Gate thread — 4µs, non-blocking:
k_msgq_put(&tinyml_q, &frame_ptr, K_NO_WAIT);

// TinyML thread — 100ms, parallel:
k_msgq_get(&tinyml_q, &frame_ptr, K_MSEC(500));
run_classifier(frame_ptr, ...);
```
Also changed `K_FOREVER` wait to `K_MSEC(500)` — logs stall if inference
hangs (WDT would never reset a hung TinyML thread under `K_FOREVER` since
the gate loop would keep feeding the watchdog).

**Result:**  
1,186 frames processed in 30 seconds — **4× throughput improvement**.
Gate never blocks. TinyML runs in background. Stall detection prevents
silent hangs.

---

## 8. Hardware WDT Silent Failure (device_is_ready() Missing)

**Situation:**  
WDT was configured with `DEVICE_DT_GET(DT_NODELABEL(wdt0))` and
`wdt_install_timeout()` called directly. In a Zephyr hardware test,
the WDT device pointer was valid at compile time but the peripheral
was not yet initialized at the point `wdt_install_timeout()` was called.
The call silently returned an error that was not checked — WDT was never
armed.

**Task:**  
Ensure HW WDT is actually armed, or fail clearly if it cannot be.

**Action:**  
Added `device_is_ready()` check with graceful degradation:
```c
if (!device_is_ready(wdt_dev)) {
    printk("WDT: device not ready — running without hardware watchdog\n");
} else {
    wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_cfg);
    wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
}
```
System continues without HW WDT rather than silently running with an unarmed watchdog.
SW stall watchdog (300ms) provides a second layer of protection regardless.

**Result:**  
Boot log clearly states WDT status. On nRF52840-DK with J-Link attached,
WDT arms correctly. On dongle (different config), degradation path prevents crash.

---

## 9. Real Audio Recall: Frame-Level vs Clip-Level Metric

**Situation:**  
`test_real_audio.py` reported 28% speech recall on Google Speech Commands
clips. This looked like a catastrophic failure — almost nothing waking.

**Task:**  
Determine if the gate was actually broken or the metric was wrong.

**Investigation:**  
Google Speech Commands clips are 1-second WAV files. The spoken word
occupies ~200–400ms in the center; the rest is silence (padding).
A 1-second clip at 50 frames/s = 50 frames, of which only ~15 contain
speech. The gate correctly sleeps on silent frames — penalizing it for
not waking during silence is measuring the wrong thing.

**Action:**  
Changed from frame-level recall (% of all frames that woke) to
**clip-level recall** (did ANY frame in this clip wake = detected?):
```python
# Clip-level: any WAKE in clip = detected
clip_detected = any(frame_result == WAKE for frame_result in clip_frames)
```

**Result:**  
Clip-level recall: **99%** (99/100 Google Speech Commands clips detected).
Frame-level was measuring silence suppression, not keyword detection ability.
The gate was working correctly all along.

---

## 10. Broadband Noise Passing Gate (Design Clarification)

**Situation:**  
`_background_noise_` folder in Google Speech Commands contains clips of
dishes clattering, white noise, pink noise, and tap water. All of them
caused `ACH=4` (all 4 resonator channels active) → gate woke → looked
like false positives.

**Task:**  
Determine if this was a bug or expected behavior.

**Investigation:**  
The multi-formant Rule 7 gate (`active_ch ≥ 2`) is designed to reject
**band-limited** noise: HVAC hum (300 Hz), engine rumble (300–800 Hz),
wind noise (below 800 Hz). These only activate channel 0–1.
Broadband noise (white, pink, dishes) has energy across all 4 channels
(300/800/1500/2500 Hz) — indistinguishable from speech at the physics gate level.

**Action:**  
No code change. Documented design boundary:
- Gate rejects: HVAC, engine, wind, fan (band-limited, Rule 7 fails)
- Gate passes to TinyML: speech, broadband noise (TinyML stage 2 rejects noise)
- Test 5 uses synthetic HVAC/engine/wind for gate-specific validation

**Result:**  
100% abort on HVAC/engine/wind (validated). Broadband noise passes gate
by design — TinyML stage 2 is the correct rejection point for it.
End-to-end system: gate recall 90% × TinyML accuracy 97.8% = **87.9%**.

---

## Summary Table

| # | Problem | Root Cause | Fix | Impact |
|---|---------|-----------|-----|--------|
| 1 | PDM never fires | `IRQ_CONNECT` missing for nrfx | Add `IRQ_CONNECT(PDM_IRQn, 3, ...)` | Unblocked entire project |
| 2 | MPU fault on boot | 2640-byte struct on 2048-byte stack | `static` allocation | Clean boot |
| 3 | Filter output wrong | CMSIS coefficient format/sign mismatch | Fix Python coef generator | Correct resonator output |
| 4 | NaN in gain chain | `CLAMP(NaN)` returns NaN | Inverted-AND guard | UB-free hot loop |
| 5 | Python sim disagrees | SCORE_WAKE=80 vs 20, missing coherence rule | Align to firmware | 99% clip recall |
| 6 | 18% frame miss rate | Priority inversion (mutex in gate path) | Remove mutex from critical path | ~1% miss rate |
| 7 | 4× throughput lost | TinyML blocking gate thread | Threaded TinyML + zero-copy queue | 4× frames/s |
| 8 | WDT silently unarmed | No `device_is_ready()` check | Add check + graceful degradation | Reliable WDT |
| 9 | 28% recall (wrong) | Frame-level metric penalizes silence | Clip-level metric | 99% true recall |
| 10 | Broadband passes gate | Band-limited design, not a bug | Document + test with correct noise | Correct system model |
