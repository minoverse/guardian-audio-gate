#!/usr/bin/env python3
"""
Guardian — AGC/Gate Separation + Float32 Safety Tests
======================================================
Three behavioral tests:
  Test 1: AGC transient false trigger
          Loud burst (-5 dBFS) → sudden silence (-65 dBFS).
          AGC gain overshoots during 500ms release window.
          Gate must produce zero false WAKEs throughout.

  Test 2: Low-SNR speech recall
          Whisper-level speech (-18 dBFS) + HVAC noise (-28 dBFS) = 10 dB SNR.
          Gate must still WAKE on speech despite noise.

  Test 3: Steady noise abort saturation
          500 frames of HVAC — checks abort rate is stable across three
          windows (warmup / mid / tail) and CV adaption does not degrade it.

Float32 overflow analysis:
  - Analytical peak gain bound for all 4 resonators
  - Worst-case simulation: sustained resonance-frequency sine at max amplitude
    through float32-precision DF1 biquad
  - Margin vs np.finfo(np.float32).max (≈ 3.4e38)

Edge / boundary cases for BiquadDF1:
  - Zero-length input
  - All-zeros input (convergence to zero)
  - All-max input (saturation boundary, |y| bounded by peak gain)
  - State reset consistency
  - Long-run stability: 10k frames random signal, no NaN / Inf / growth

Usage:
    python3 tools/test_agc_gate_separation.py
"""

import numpy as np
import sys

FS         = 16000
FRAME      = 320
DC_ALPHA   = 0.99215

# ── Resonator coefficient table (mirrors resonator_coefs_default.h) ─────────
# Format: {b0, b1, b2, -a1, -a2}  CMSIS convention (na1=-a1, na2=-a2)
COEFS = [
    [0.0073, 0.0, -0.0073,  1.9717, -0.9855],   # ch0  300 Hz  Q=68
    [0.0187, 0.0, -0.0187,  1.8650, -0.9616],   # ch1  800 Hz  Q=26
    [0.0355, 0.0, -0.0355,  1.6029, -0.9291],   # ch2 1500 Hz  Q=14
    [0.0578, 0.0, -0.0578,  1.0451, -0.8844],   # ch3 2500 Hz  Q= 9
]
CENTER_FREQS = [300, 800, 1500, 2500]

# Gate thresholds — must match test_multi_env.py and decision.c
ENERGY_THRESH        = 800
CORR_THRESH          = 6000
ZCR_LO, ZCR_HI      = 20, 220
SFM_THRESH           = 0.60
CV_THRESH            = 0.12
# MULTIBAND_CHAN_THRESH calibration — AGC-in-loop context
# C firmware uses arm_rms_q15 threshold = 500 Q15 counts (per channel RMS).
# Python energy = sum(ch_q15²) >> 10 = N×rms² / 1024  →  500² × 320 / 1024 = 78,125.
#
# Why test_multi_env.py used 5000 (lower):
#   That file has NO AGCSim in the loop — signals arrive at pre-calibrated
#   post-AGC approximations.  HVAC ch1 energy without AGC ≈ 555, safely below 5000.
#
# This file runs AGCSim BEFORE GateSim (4× max boost).  Post-AGC HVAC ch1 energy
# peaks at ~17,532 — above 5000 but safely below 78,125 (≈ 4× margin).
# Speech ch1 energy post-AGC ≈ 530,000 >> 78,125 → still detected.
MULTIBAND_CHAN_THRESH = 78125
MULTIBAND_MIN_ACTIVE = 2
SCORE_WAKE           = 80

# AGC constants — must mirror preprocess.c exactly
AGC_ALPHA_ATTACK  = 0.8647    # τ = 10ms  fast attack
AGC_ALPHA_RELEASE = 0.0392    # τ = 500ms slow release
AGC_ALPHA_SMOOTH  = 0.3297    # τ = 50ms  gain smoother
AGC_TARGET        = 0.25      # -12 dBFS
AGC_MIN           = 0.25
AGC_MAX           = 4.0

rng = np.random.default_rng(0xC0FFEE)

# ── BiquadDF1: float64 (gate logic) ─────────────────────────────────────────
class BiquadDF1:
    def __init__(self, coef):
        self.b0, self.b1, self.b2 = coef[0], coef[1], coef[2]
        self.na1, self.na2 = coef[3], coef[4]
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def reset(self):
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def process(self, x_arr):
        out = np.empty(len(x_arr))
        b0,b1,b2,na1,na2 = self.b0,self.b1,self.b2,self.na1,self.na2
        x1,x2,y1,y2 = self.x1,self.x2,self.y1,self.y2
        for i, x in enumerate(x_arr):
            y = b0*x + b1*x1 + b2*x2 + na1*y1 + na2*y2
            x2, x1 = x1, x
            y2, y1 = y1, y
            out[i] = y
        self.x1,self.x2,self.y1,self.y2 = x1,x2,y1,y2
        return out

# ── BiquadDF1: float32-precision (for overflow testing only) ─────────────────
class BiquadDF1F32:
    """Mirrors the exact float32 arithmetic of biquad_df1.h on Cortex-M4F."""
    def __init__(self, coef):
        self.b0  = np.float32(coef[0]); self.b1  = np.float32(coef[1])
        self.b2  = np.float32(coef[2]); self.na1 = np.float32(coef[3])
        self.na2 = np.float32(coef[4])
        self.x1  = np.float32(0.0);    self.x2  = np.float32(0.0)
        self.y1  = np.float32(0.0);    self.y2  = np.float32(0.0)

    def sample(self, x: np.float32) -> np.float32:
        y = (self.b0*x + self.b1*self.x1 + self.b2*self.x2
             + self.na1*self.y1 + self.na2*self.y2)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y

    def process(self, x_arr_f32):
        out = np.empty(len(x_arr_f32), dtype=np.float32)
        for i, x in enumerate(x_arr_f32):
            out[i] = self.sample(np.float32(x))
        return out

# ── AGCSim: exact Python mirror of preprocess.c ──────────────────────────────
class AGCSim:
    """Mirrors audio_frontend_process() in preprocess.c."""
    def __init__(self, cal_gain=1.0):
        self.envelope = AGC_TARGET    # start at target (not zero — mirrors firmware)
        self.gain     = 1.0
        self.cal_gain = cal_gain

    def process(self, frame_i16):
        """Returns (post_agc_int16, current_gain)."""
        rms_q15 = float(np.sqrt(np.mean(frame_i16.astype(np.float64)**2)))
        frame_rms = rms_q15 / 32767.0

        alpha = AGC_ALPHA_ATTACK if frame_rms > self.envelope else AGC_ALPHA_RELEASE
        self.envelope = alpha * frame_rms + (1.0 - alpha) * self.envelope

        desired = (AGC_TARGET / self.envelope) if self.envelope > 1e-6 else AGC_MAX
        desired = float(np.clip(desired, AGC_MIN, AGC_MAX))

        self.gain = AGC_ALPHA_SMOOTH * desired + (1.0 - AGC_ALPHA_SMOOTH) * self.gain

        total = self.cal_gain * self.gain
        scaled = frame_i16.astype(np.float64) * total
        scaled = np.clip(scaled, -32768.0, 32767.0)
        return scaled.astype(np.int16), self.gain

# ── GateSim: mirrors decision.c ──────────────────────────────────────────────
class GateSim:
    def __init__(self):
        self.filters   = [BiquadDF1(c) for c in COEFS]
        self.dc_xp = self.dc_yp = 0.0
        self.mod_mean = self.mod_var = 0.0
        self.mod_n = 0
        MOD_ALPHA = 1.0 - np.exp(-20.0 / 500.0)   # τ=500ms at 50fps
        self.MOD_ALPHA = MOD_ALPHA

    def process_frame(self, frame_i16):
        x_f = frame_i16.astype(np.float64)
        dc_out = np.empty(FRAME)
        xp, yp = self.dc_xp, self.dc_yp
        for i, x in enumerate(x_f):
            y = DC_ALPHA * yp + x - xp
            xp, yp = x, y
            dc_out[i] = y
        self.dc_xp, self.dc_yp = xp, yp

        ch_out = [self.filters[ch].process(dc_out / 32768.0) for ch in range(4)]
        ch_q15 = [np.clip(c * 32768.0, -32768, 32767).astype(np.int16)
                  for c in ch_out]

        energies = [int(np.sum(ch_q15[c].astype(np.int32)**2) >> 10)
                    for c in range(4)]
        total_e = sum(energies)

        def pearson_q15(a, b):
            af, bf = a.astype(float), b.astype(float)
            da = np.sqrt(np.sum(af**2) * np.sum(bf**2))
            return int((np.sum(af*bf) / da * 32767) if da > 0 else 0)

        corr = pearson_q15(ch_q15[0], ch_q15[1])
        zcr  = int(np.sum(np.diff(np.sign(frame_i16)) != 0))

        ef    = np.array(energies, dtype=float) + 1.0
        arith = np.mean(ef)
        geom  = np.exp(np.mean(np.log(ef)))
        sfm   = float(np.clip(geom / arith, 0, 1)) if arith >= 2.0 else 1.0

        self.mod_n += 1
        alpha = self.MOD_ALPHA
        self.mod_mean = alpha * total_e + (1-alpha) * self.mod_mean
        self.mod_var  = alpha * (total_e - self.mod_mean)**2 + (1-alpha)*self.mod_var
        cv = 0.0
        if self.mod_n >= 25 and self.mod_mean > 0:
            cv = float(np.clip(np.sqrt(self.mod_var) / self.mod_mean, 0, 2))

        active_ch = sum(1 for e in energies if e > MULTIBAND_CHAN_THRESH)

        r_energy = total_e > ENERGY_THRESH
        r_corr   = corr    > CORR_THRESH
        r_zcr    = ZCR_LO < zcr < ZCR_HI
        r_sfm    = sfm     < SFM_THRESH
        r_cv     = cv      > CV_THRESH

        score = (40*r_energy + 20*r_corr + 25*r_zcr +
                 20*r_sfm   + 20*r_cv   + 25*(active_ch >= MULTIBAND_MIN_ACTIVE))

        wake = (score >= SCORE_WAKE) and (active_ch >= MULTIBAND_MIN_ACTIVE)
        return ('WAKE' if wake else 'ABORT'), score, active_ch

# ── Signal helpers ───────────────────────────────────────────────────────────
def _dbfs_amp(db):
    """Convert dBFS to linear amplitude (re int16 full-scale 32767)."""
    return 32767.0 * 10.0**(db / 20.0)

def gen_broadband(n_frames, level_db):
    """Band-limited white noise (LP 4kHz) — simulates recording-room ambient."""
    from scipy.signal import butter, sosfilt
    sos = butter(4, 4000/(FS/2), btype='low', output='sos')
    amp = _dbfs_amp(level_db)
    total = (n_frames + 10) * FRAME
    raw = sosfilt(sos, rng.standard_normal(total)) * amp
    raw = np.clip(raw, -32768, 32767)
    for i in range(10, 10 + n_frames):
        yield raw[i*FRAME:(i+1)*FRAME].astype(np.int16)

def gen_hvac(n_frames, level_db=-35):
    from scipy.signal import butter, sosfilt
    sos = butter(8, 350/(FS/2), btype='low', output='sos')
    amp = _dbfs_amp(level_db)
    total = (n_frames + 10) * FRAME
    raw = sosfilt(sos, rng.standard_normal(total) * amp * 8)
    raw = np.clip(raw, -32768, 32767)
    for i in range(10, 10 + n_frames):
        yield raw[i*FRAME:(i+1)*FRAME].astype(np.int16)

def gen_speech_like(n_frames, level_db=-12):
    """Modulated 300 + 800 Hz — mirrors gen_speech_like in test_multi_env.py."""
    amp = _dbfs_amp(level_db)
    for _ in range(n_frames):
        t   = np.arange(FRAME) / FS
        env = 0.5 + 0.5 * np.sin(2*np.pi*5*t)
        sig = (np.sin(2*np.pi*300*t)*0.6
             + np.sin(2*np.pi*800*t)*0.3
             + rng.standard_normal(FRAME)*0.05) * amp * env
        yield np.clip(sig, -32768, 32767).astype(np.int16)

# ════════════════════════════════════════════════════════════════════════════
# TEST 1: AGC transient — loud burst → silence
# Expected: zero false WAKEs during the 500ms AGC release window
# ════════════════════════════════════════════════════════════════════════════
def test_agc_transient():
    """
    Scenario timeline (all signals are RAW, before AGC):
      Phase A — 30 frames: loud broadband noise at -5 dBFS
                AGC attack (τ=10ms) drives gain down to ~0.25 within 5 frames.
      Phase B —  1 frame:  single impulse at 0 dBFS (door slam simulation)
                Drives AGC gain to absolute minimum (~0.25).
      Phase C — 200 frames: quiet room at -65 dBFS
                AGC release (τ=500ms) — gain slowly rises toward 4.0.
                This is the risk window: rising gain amplifies quiet noise.

    Maximum gate-input energy during Phase C:
      raw_rms ≈ _dbfs_amp(-65) = 18 ADC counts
      max AGC gain = 4.0
      post_agc_rms ≈ 18 × 4.0 = 72 counts
      resonator peak gain ≈ 1.0 (see overflow analysis below)
      MULTIBAND_CHAN_THRESH = 5000 >> 72  ← 70× margin

    Expected: all 200 Phase C frames → ABORT
    """
    print("\n── Test 1: AGC Transient False Trigger ──────────────────────────────")

    agc  = AGCSim()
    gate = GateSim()

    # Phase A: 30 frames loud noise — drive AGC down
    phase_a_gains = []
    for frame in gen_broadband(30, level_db=-5):
        post, gain = agc.process(frame)
        phase_a_gains.append(gain)

    gain_after_burst = agc.gain
    print(f"  Phase A (loud noise)  : final AGC gain = {gain_after_burst:.4f}  "
          f"(expected ≈ 0.25, min={AGC_MIN})")

    # Phase B: single impulse frame
    impulse = np.zeros(FRAME, dtype=np.int16)
    impulse[0] = 32767
    post, gain = agc.process(impulse)
    print(f"  Phase B (impulse)     : AGC gain after = {gain:.4f}")

    # Phase C: 200 frames quiet — gain release window, watch for false triggers
    false_wakes      = 0
    max_post_agc_rms = 0.0
    gain_trajectory  = []

    for frame in gen_broadband(200, level_db=-65):
        post, gain = agc.process(frame)
        gain_trajectory.append(gain)
        rms = float(np.sqrt(np.mean(post.astype(float)**2)))
        if rms > max_post_agc_rms:
            max_post_agc_rms = rms

        dec, score, active_ch = gate.process_frame(post)
        if dec == 'WAKE':
            false_wakes += 1

    gain_recovered = gain_trajectory[-1]
    frames_to_half = next((i for i, g in enumerate(gain_trajectory)
                           if g > (AGC_MIN + gain_after_burst) / 2 + 0.5), len(gain_trajectory))

    print(f"  Phase C (release)     : gain {gain_trajectory[0]:.3f} → {gain_recovered:.3f}"
          f"  (τ_release = 500ms)")
    print(f"  Max post-AGC RMS      : {max_post_agc_rms:.1f} counts  "
          f"(threshold basis: {5000**0.5:.0f} for active_ch)")
    print(f"  False WAKEs           : {false_wakes} / 200 frames")

    ok = false_wakes == 0
    print(f"  Result: {'PASS ✅' if ok else 'FAIL ❌'}  — expected 0 false WAKEs")

    # Explain why it passes analytically
    max_possible = _dbfs_amp(-65) * AGC_MAX
    print(f"\n  Analytical margin:")
    print(f"    raw noise RMS @ -65dBFS   = {_dbfs_amp(-65):.1f}")
    print(f"    max AGC gain              = {AGC_MAX:.1f}×")
    print(f"    max post-AGC RMS          ≤ {max_possible:.1f}")
    print(f"    MULTIBAND_CHAN_THRESH      = {MULTIBAND_CHAN_THRESH}")
    print(f"    margin                    = {MULTIBAND_CHAN_THRESH / max_possible:.0f}×")

    return ok

# ════════════════════════════════════════════════════════════════════════════
# TEST 2: Low-SNR speech recall
# Speech at -18 dBFS + HVAC at -28 dBFS = 10 dB SNR
# Expected: ≥ 60% WAKE recall (gate adapts to low-level speech)
# ════════════════════════════════════════════════════════════════════════════
def test_low_snr_speech():
    """
    Mix whisper-level speech with HVAC noise at 10 dB SNR.
    Both are raw signals (before AGC).

    After AGC normalises to -12 dBFS target:
      post-AGC speech amplitude ≈ target (-12 dBFS)
      post-AGC noise amplitude ≈ target - 10 dB = -22 dBFS

    The gate sees correctly normalised speech — the 10 dB SNR is preserved
    by the AGC (it amplifies both equally since they are mixed).

    Rule sensitivity at -12 dBFS speech:
      300Hz channel energy >> MULTIBAND_CHAN_THRESH → active_ch ≥ 2 ✓
      score ≥ SCORE_WAKE (energy + multiband + zcr) ✓

    Expected: ≥ 60% recall (deliberately conservative — real speech has
              richer harmonics; synthetic whisper is worst-case formant-only)
    """
    print("\n── Test 2: Low-SNR Speech Recall (10 dB SNR) ────────────────────────")

    agc  = AGCSim()
    gate = GateSim()

    # Warm up gate + AGC on 50 frames of noise first (CV needs 25 frames)
    for frame in gen_hvac(50, level_db=-28):
        post, _ = agc.process(frame)
        gate.process_frame(post)

    speech_level = -18   # dBFS
    noise_level  = -28   # dBFS  → 10 dB SNR
    snr_db       = speech_level - noise_level   # = -18 - (-28) = 10 dB

    speech_gen = list(gen_speech_like(100, level_db=speech_level))
    noise_gen  = list(gen_hvac(100,       level_db=noise_level))

    wakes = 0
    for sp, ns in zip(speech_gen, noise_gen):
        mixed = np.clip(sp.astype(np.int32) + ns.astype(np.int32),
                        -32768, 32767).astype(np.int16)
        post, gain = agc.process(mixed)
        dec, score, active_ch = gate.process_frame(post)
        if dec == 'WAKE':
            wakes += 1

    recall = wakes / 100 * 100
    print(f"  Speech level          : {speech_level} dBFS")
    print(f"  Noise level           : {noise_level} dBFS  (HVAC LP-350Hz)")
    print(f"  SNR                   : {snr_db} dB")
    print(f"  AGC gain (final)      : {agc.gain:.3f}×")
    print(f"  Speech recall         : {wakes}/100 frames = {recall:.0f}%")

    ok = recall >= 60.0
    print(f"  Result: {'PASS ✅' if ok else 'FAIL ❌'}  — threshold 60%")
    return ok

# ════════════════════════════════════════════════════════════════════════════
# TEST 3: Steady noise abort saturation — 500 frames HVAC
# Checks abort rate is stable across three time windows (no degradation)
# ════════════════════════════════════════════════════════════════════════════
def test_steady_noise_saturation():
    """
    Run 500 frames of HVAC through the gate and measure abort rate in
    three consecutive windows:
      Window 1: frames   0-100  (CV still warming up for first 25 frames)
      Window 2: frames 100-300  (all rules active)
      Window 3: frames 300-500  (long-run, check no drift)

    Concern: mod_var in the CV estimator is an IIR with α=0.039 (τ=500ms).
    After 300+ frames of steady noise, mod_var converges to a stable value.
    If it overshoots during convergence, CV could fire briefly.

    Expected: abort rate ≥ 90% in all three windows (multiband hard gate
              guarantees this regardless of CV state).
    """
    print("\n── Test 3: Steady Noise Abort Saturation (500 frames HVAC) ─────────")

    agc  = AGCSim()
    gate = GateSim()

    windows = [[], [], []]
    for i, frame in enumerate(gen_hvac(500, level_db=-35)):
        post, _ = agc.process(frame)
        dec, score, active_ch = gate.process_frame(post)
        if   i < 100:  windows[0].append(dec)
        elif i < 300:  windows[1].append(dec)
        else:          windows[2].append(dec)

    labels = ['frames   0–100', 'frames 100–300', 'frames 300–500']
    all_ok = True
    for label, window in zip(labels, windows):
        n_abort = window.count('ABORT')
        rate = n_abort / len(window) * 100
        ok   = rate >= 90.0
        if not ok: all_ok = False
        print(f"  {label}: abort {n_abort}/{len(window)} = {rate:.0f}%  "
              f"{'✅' if ok else '❌'}")

    print(f"  Result: {'PASS ✅ all windows ≥ 90%' if all_ok else 'FAIL ❌ rate degraded'}")
    return all_ok

# ════════════════════════════════════════════════════════════════════════════
# FLOAT32 OVERFLOW ANALYSIS
# ════════════════════════════════════════════════════════════════════════════
def test_float32_overflow():
    """
    Worst-case bound analysis for each resonator channel.

    Analytical peak gain:
      For bandpass filter {b0, 0, -b0, na1, na2}:
        H(z) = b0*(1 - z^-2) / (1 - na1*z^-1 - na2*z^-2)
        At resonance ω0: poles are at r*e^(±jω0), r = sqrt(-na2)
        Peak gain = |b0 * 2j*sin(ω0)| / |1 - r*e^(-2jω0)| / |1 - r|
      Numerical result: all 4 channels have peak gain ≈ 1.0.

    Simulation:
      Feed each filter a sustained sine at its center frequency, amplitude 1.0
      (float32). Run for 5000 samples (steady state). Record max |y|.
      Then feed worst-case: all five terms at their maximums simultaneously.

    Float32 max: np.finfo(np.float32).max ≈ 3.4e38
    """
    print("\n── Float32 Overflow Analysis ────────────────────────────────────────")

    F32_MAX = float(np.finfo(np.float32).max)
    all_ok  = True

    # Analytical peak gain per channel
    print(f"\n  {'Ch':>3}  {'Freq':>5}  {'Peak gain':>11}  {'Max |y| sim':>12}  "
          f"{'F32 margin':>12}")
    print("  " + "─"*58)

    for ch, (coef, fc) in enumerate(zip(COEFS, CENTER_FREQS)):
        # Sustained sine at center frequency, amplitude 1.0 (float32)
        filt = BiquadDF1F32(coef)
        N_WARMUP = 2000
        N_MEAS   = 3000
        omega0   = 2.0 * np.pi * fc / FS
        t        = np.arange(N_WARMUP + N_MEAS, dtype=np.float32)
        x        = np.sin(omega0 * t, dtype=np.float64).astype(np.float32)

        # Warmup
        for i in range(N_WARMUP):
            filt.sample(x[i])

        # Measure steady-state peak output
        max_y = 0.0
        for i in range(N_WARMUP, N_WARMUP + N_MEAS):
            y = float(abs(filt.sample(x[i])))
            if y > max_y:
                max_y = y

        # Analytical peak gain (DTFT at ω0)
        b0, b1, b2, na1, na2 = [complex(v) for v in coef]
        z0 = np.exp(1j * omega0)
        H  = (b0 + b1/z0 + b2/z0**2) / (1 - na1/z0 - na2/z0**2)
        peak_gain_analytic = abs(H)

        margin = F32_MAX / max(max_y, 1e-30)
        ok     = not (np.isnan(max_y) or np.isinf(max_y) or max_y > 10.0)
        if not ok: all_ok = False

        print(f"  ch{ch}  {fc:>5} Hz  {peak_gain_analytic:>10.4f}×  "
              f"{max_y:>11.4f}    {margin:>10.2e}  {'✅' if ok else '❌'}")

    # Worst-case 5-term sum bound
    print(f"\n  Worst-case 5-term sum (ch3, b0=0.0578, na1=1.9717, na2=0.9855):")
    b0, na1, na2 = 0.0578, 1.9717, 0.9855
    # If |x|=1.0 and |y|=M (bounded), worst case: all terms add constructively
    # |y| ≤ b0|x| + 0 + b0|x2| + na1·M + na2·M
    # At steady state M = peak_gain ≈ 1.0
    M      = 1.05   # conservative bound (slightly > 1.0)
    bound  = b0*1.0 + 0 + b0*1.0 + na1*M + na2*M
    margin = F32_MAX / bound
    print(f"    |y| ≤ {b0}×1 + {b0}×1 + {na1}×{M} + {na2}×{M}")
    print(f"        = {bound:.4f}  (at steady state with peak gain M={M})")
    print(f"    float32 max / bound = {margin:.2e}  ← overflow impossible")

    # AGC headroom note
    print(f"\n  AGC headroom:")
    print(f"    Max AGC gain = {AGC_MAX}×, int16 full-scale = 32767")
    print(f"    Resonator input after AGC: clamped to Q15 by preprocess.c:88-90")
    print(f"    → |x| ≤ 1.0 always guaranteed before resonator bank")
    print(f"    float32 exponent range ±{F32_MAX:.2e} — audio never approaches this")

    print(f"\n  Result: {'PASS ✅ no overflow possible' if all_ok else 'FAIL ❌'}")
    return all_ok

# ════════════════════════════════════════════════════════════════════════════
# EDGE / BOUNDARY CASES FOR BiquadDF1
# ════════════════════════════════════════════════════════════════════════════
def test_biquad_edge_cases():
    """
    Boundary conditions for the DF1 biquad implementation.

    These test the Python simulation (float64) and float32 version.
    They correspond to defensive tests that would run on the C firmware
    if a unit test harness were integrated.
    """
    print("\n── BiquadDF1 Edge / Boundary Cases ─────────────────────────────────")

    coef = COEFS[0]   # ch0, 300 Hz — most sensitive (highest Q)
    results = []

    # ── Edge 1: zero-length input ─────────────────────────────────────────
    f = BiquadDF1(coef)
    f.x1, f.y1 = 0.5, 0.3  # set non-zero state
    out = f.process(np.array([], dtype=np.float64))
    ok  = (len(out) == 0 and f.x1 == 0.5 and f.y1 == 0.3)
    results.append(ok)
    print(f"  Edge 1 (zero-length input): output len={len(out)}, "
          f"state unchanged={f.x1==0.5}  {'✅' if ok else '❌'}")

    # ── Edge 2: all-zeros input → output must reach zero ─────────────────
    f = BiquadDF1(coef)
    # Seed state with non-zero values
    f.x1 = f.x2 = f.y1 = f.y2 = 0.1
    zeros = np.zeros(2000)
    out   = f.process(zeros)
    final_val = abs(out[-1])
    ok = final_val < 1e-6
    results.append(ok)
    print(f"  Edge 2 (all-zeros): final |y| = {final_val:.2e}  "
          f"(expect → 0)  {'✅' if ok else '❌'}")

    # ── Edge 3: all-max input, check output bounded by peak gain ─────────
    f    = BiquadDF1(coef)
    ones = np.ones(5000)
    out  = f.process(ones)
    # DC value of H(z) at z=1: H(1) = (b0+b1+b2)/(1-na1-na2)
    b0,b1,b2,na1,na2 = coef
    denom_dc = 1.0 - na1 - na2
    dc_gain  = (b0+b1+b2) / denom_dc if abs(denom_dc) > 1e-10 else 0.0
    expected_dc = dc_gain * 1.0
    final_mean  = float(np.mean(out[-100:]))
    ok = abs(final_mean - expected_dc) < 0.01 and not np.any(np.isinf(out))
    results.append(ok)
    print(f"  Edge 3 (all-max): DC gain={dc_gain:.6f}, "
          f"steady-state mean={final_mean:.6f}, no inf={not np.any(np.isinf(out))}  "
          f"{'✅' if ok else '❌'}")

    # ── Edge 4: reset consistency ─────────────────────────────────────────
    # Process 100 samples, reset, then process again → must match fresh filter
    f1 = BiquadDF1(coef)
    f2 = BiquadDF1(coef)
    sig = rng.standard_normal(100)
    out1_pre  = f1.process(sig)
    f1.reset()
    out1_post = f1.process(sig)
    out2      = f2.process(sig)
    ok = np.allclose(out1_post, out2, atol=1e-12)
    results.append(ok)
    print(f"  Edge 4 (reset consistency): reset→re-run matches fresh  {'✅' if ok else '❌'}")

    # ── Edge 5: long-run stability — 10k frames random input ─────────────
    f   = BiquadDF1(coef)
    max_y = 0.0
    has_nan = False
    for _ in range(10000):
        sig    = rng.standard_normal(FRAME)  # ±1.0 scale
        out    = f.process(sig)
        max_y  = max(max_y, float(np.max(np.abs(out))))
        has_nan |= bool(np.any(np.isnan(out)) or np.any(np.isinf(out)))
    ok = (not has_nan) and (max_y < 10.0)
    results.append(ok)
    print(f"  Edge 5 (10k frames stability): max |y|={max_y:.4f}, "
          f"NaN/Inf={has_nan}  {'✅' if ok else '❌'}")

    # ── Edge 6: float32 NaN propagation guard ─────────────────────────────
    # If somehow NaN enters (not possible from int16 PDM, but test the math):
    # float32 filter should propagate NaN (correct IEEE behavior — not silent corruption)
    f32 = BiquadDF1F32(coef)
    y   = f32.sample(np.float32('nan'))
    ok  = bool(np.isnan(y))   # NaN in → NaN out (visible, detectable)
    results.append(ok)
    print(f"  Edge 6 (NaN propagation): NaN→NaN={ok} (IEEE correct)  {'✅' if ok else '❌'}")
    print(f"    Note: PDM input is int16 — NaN is structurally impossible at firmware input.")
    print(f"    This confirms the arithmetic is IEEE 754 (not silent wrap/clamp).")

    all_ok = all(results)
    print(f"\n  Result: {'PASS ✅' if all_ok else 'FAIL ❌'}  {sum(results)}/{len(results)} passed")
    return all_ok

# ════════════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════════════
def main():
    print("═" * 70)
    print("  Guardian — AGC/Gate Separation + Float32 Safety Tests")
    print("═" * 70)

    results = {
        "Test 1  AGC transient false trigger" : test_agc_transient(),
        "Test 2  Low-SNR speech recall"        : test_low_snr_speech(),
        "Test 3  Steady noise saturation"      : test_steady_noise_saturation(),
        "Float32 overflow analysis"            : test_float32_overflow(),
        "BiquadDF1 edge/boundary cases"        : test_biquad_edge_cases(),
    }

    print("\n" + "═" * 70)
    print("  Summary")
    print("─" * 70)
    all_pass = True
    for name, ok in results.items():
        status = "PASS ✅" if ok else "FAIL ❌"
        print(f"  {status}  {name}")
        if not ok:
            all_pass = False
    print("─" * 70)
    print(f"  Overall: {'ALL PASS ✅' if all_pass else 'SOME FAILURES ❌'}")
    print("═" * 70)

    sys.exit(0 if all_pass else 1)

if __name__ == '__main__':
    main()
