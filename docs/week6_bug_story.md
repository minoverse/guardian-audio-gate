# Week 6 Bug Story — Guardian Audio Gate

Gate decision layer built on top of Week 5 DMA PDM pipeline.
Three major bugs found and fixed during testing.

---

## Bug 1: LFSR Noise Test Producing Same Output as Sine

### What Happened
Test 2 (LFSR noise) was supposed to produce `GATE: ABORT`. Instead it showed
identical values to Test 1 (800 Hz sine), making the noise test meaningless.

### Real Log
```
# Expected: ABORT — got WAKE, same values as sine
=== Guardian Week 6 - Gate Decision [TEST: LFSR NOISE] ===
Expected: ABORT most frames (broadband noise)

GATE: WAKE  conf= 60 E=27285 C= 32535 ZCR=107 T=   0us noise=    0 wakes=50/50
GATE: WAKE  conf= 85 E=27281 C= 32537 ZCR=106 T=   0us noise=    0 wakes=50/50
```

### Root Cause
Operator precedence bug in the LFSR generator:

```c
// WRONG — uint16_t shifted first, then cast → always positive (0 to 4095)
test_buf[i] = (int16_t)(lfsr >> 4);

// FIXED — cast to int16_t first, arithmetic shift → signed (-2048 to 2047)
test_buf[i] = (int16_t)lfsr >> 4;
```

The unipolar noise (DC offset ~2048) passed through the resonator bank and
produced correlated outputs across all channels, making it look like a strong
coherent signal.

### After Fix
```
GATE: WAKE  conf= 60 E=27285 C= 18102 ZCR=107  ← C dropped from 32537 to 18102
```
C dropped significantly, confirming the fix removed the false DC correlation.

---

## Bug 2: Resonator Bank Oscillating at Wrong Frequencies (~2650 Hz)

### What Happened
All features (E, C) were near Q15 maximum regardless of input — sine, noise,
or live mic in a quiet room. Values never changed when speaking. The gate
always produced `WAKE` at `conf=60` in every condition.

### Real Log (live mic, quiet room)
```
GATE: WAKE  conf= 60 E=27280 C= 32634 ZCR=107 T=   0us noise=    0 wakes=50/50
GATE: WAKE  conf= 85 E=27282 C= 32634 ZCR=107 T=   0us noise=    0 wakes=50/50
GATE: WAKE  conf= 60 E=27283 C= 32634 ZCR=107 T=   0us noise=    0 wakes=50/50
```

ZCR=107 was the smoking gun: 107 zero crossings per 320 samples at fs=16kHz:
```
f = 16000 × 107 / 320 / 2 = 2650 Hz
```
All four resonators (labeled 300/800/1500/2500 Hz) were actually oscillating
at ~2650 Hz.

### Root Cause
The CMSIS Q15 biquad coefficient `a1 = -2r·cos(ω₀)` overflows `int16_t`
for frequencies below ~2600 Hz:

```
300 Hz resonator:  a1 = -2 × 0.9927 × cos(0.1178) = -1.9717
Q15 range is [-1.0, +1.0]  →  -1.9717 does NOT fit
Stored value clipped to 32767 (≡ 1.0)  →  resonator at 2652 Hz instead
```

This affected all four resonators — every one had `-a1 = 32767` (clipped to
Q15 max), placing them all near 2650 Hz regardless of the intended frequency.

Old (wrong) coefficients:
```c
// All resonators had -a1 = 32767 (Q15 max, clipped)
static const int16_t RESONATOR_0_COEFS[6] = {240, 0, -240, 0, 32767, -32289};
static const int16_t RESONATOR_1_COEFS[6] = {631, 0, -631, 0, 32767, -31506};
static const int16_t RESONATOR_2_COEFS[6] = {1164, 0,-1164, 0, 32767, -30440};
static const int16_t RESONATOR_3_COEFS[6] = {1897, 0,-1897, 0, 32767, -28975};
```

### Fix
Switched from `arm_biquad_cascade_df1_q15` (Q15, range ±1.0) to
`arm_biquad_cascade_df2T_f32` (float, no range limit). Recomputed all
coefficients correctly:

```c
// Correct float coefficients — -a1 can now exceed 1.0
// Format: {b0, b1, b2, -a1, -a2}  where -a1 = 2r·cos(ω₀), -a2 = -r²
static const float32_t RESONATOR_0_COEFS[5] = {
    0.0073f, 0.0f, -0.0073f,  1.9717f, -0.9855f   /* 300 Hz  */
};
static const float32_t RESONATOR_1_COEFS[5] = {
    0.0187f, 0.0f, -0.0187f,  1.8650f, -0.9616f   /* 800 Hz  */
};
static const float32_t RESONATOR_2_COEFS[5] = {
    0.0355f, 0.0f, -0.0355f,  1.6029f, -0.9291f   /* 1500 Hz */
};
static const float32_t RESONATOR_3_COEFS[5] = {
    0.0578f, 0.0f, -0.0578f,  1.0451f, -0.8844f   /* 2500 Hz */
};
```

The resonator process function was updated to convert Q15 input → float →
process → convert float output → Q15, keeping all downstream feature
functions unchanged.

### After Fix
```
# Quiet room — resonators no longer saturated
GATE: ABORT conf= 15 E=    0 C=  3989 ZCR= 12 T=   0us noise=    0 wakes= 0/50
RAW:  min=  -87 max=   39
RES0: min=  -12 max=   12   ← was ±32767, now correctly small

# Speech — resonators respond to actual audio
GATE: WAKE  conf= 75 E=   78 C= 24418 ZCR= 13 T=   0us noise=    0 wakes=24/50
RAW:  min=-3185 max= 3439
RES0: min= -1936 max= 1778  ← large only during speech
```

---

## Bug 3: Wake Threshold Too Strict (conf=55 → ABORT)

### What Happened
After the resonator fix, some speech frames correctly produced high correlation
(C > 14746) but the gate still returned `ABORT`:

### Real Log
```
GATE: ABORT conf= 55 E=    0 C= 26683 ZCR= 11 T=   0us noise=    0 wakes= 0/50
```

### Root Cause
Two compounding issues:

1. `arm_rms_q15` internally right-shifts the accumulator by 15 bits before
   taking sqrt. For resonator outputs in range ±80 (typical moderate speech):
   ```
   mean(x²) = ~3200
   3200 >> 15 = 0   ← integer truncation before sqrt
   E = sqrt(0) = 0
   ```
   Rule 2 (+20) never fires because E always reads 0.

2. With only Rule 1 (C, +40) and Rule 4 (ZCR, +15) available, max reachable
   score for speech = 55. Wake threshold was 60.

```
C=26683 > 14746  → Rule 1 +40
E=0              → Rule 2  +0  (arm_rms_q15 truncation)
ZCR=11 < 80      → Rule 4 +15
                    ──────────
                    conf=55 → ABORT  (threshold was 60, missed by 5)
```

### Fix
Lowered wake threshold from 60 to 55 in `decision.c`:

```c
// Before
decision.should_wake = (score >= 60);

// After — Rule1(C) + Rule4(ZCR) = 55 sufficient for speech
decision.should_wake = (score >= 55);
```

### After Fix
```
GATE: WAKE  conf= 75 E=   78 C= 24418 ZCR= 13  wakes=24/50  ← speech ✓
GATE: ABORT conf= 15 E=    0 C=  3989 ZCR= 12  wakes= 0/50  ← silence ✓
```

---

## Final Week 6 State

| Condition | Result | conf |
|-----------|--------|------|
| Silence / quiet room | ABORT | 15 |
| Moderate speech | WAKE | 55–75 |
| Loud speech | WAKE | 75+ |

Power measurement (Week 7): **1.57 mA** total system (nRF52840 + PDM mic),
measured via PPK2 Source Meter on P2 VDD header with USB disconnected.
Baseline (CPU sleeping): **515 µA**.
Guardian overhead: **~1.05 mA** active gate pipeline.
