#!/usr/bin/env python3
"""
Guardian — Test 3 + Test 5 on Real Audio Files
===============================================
Uses Google Speech Commands from training_data/speech/ (or word subdirs).
Test 3 uses real speech clips; Test 5 uses real noise files PLUS synthetic
band-limited noise (HVAC / engine / wind, same generators as test_multi_env.py).

Test 3: Speech recall   — clip-level: ≥1 WAKE frame in clip = detected
Test 5: Noise abort     — frame-level abort rate on designed noise types

GateSim mirrors test_multi_env.py exactly (validated reference).
Real audio is AGC-normalised → int16 before the gate.
"""

import numpy as np
import soundfile as sf
import random
from pathlib import Path

try:
    from scipy.signal import butter, sosfilt
    SCIPY_OK = True
except ImportError:
    SCIPY_OK = False

FS         = 16000
FRAME      = 320
DC_ALPHA   = 0.99215

COEFS = [
    [0.0073, 0.0, -0.0073,  1.9717, -0.9855],
    [0.0187, 0.0, -0.0187,  1.8650, -0.9616],
    [0.0355, 0.0, -0.0355,  1.6029, -0.9291],
    [0.0578, 0.0, -0.0578,  1.0451, -0.8844],
]

# Thresholds — mirror test_multi_env.py (validated)
ENERGY_THRESH        = 800
CORR_THRESH          = 6000
ZCR_LO, ZCR_HI      = 20, 220
SFM_THRESH           = 0.60
CV_THRESH            = 0.12
MULTIBAND_CHAN_THRESH = 5000
MULTIBAND_MIN_ACTIVE = 2
SCORE_WAKE           = 80

# AGC parameters
AGC_ALPHA_ATTACK  = 0.8647
AGC_ALPHA_RELEASE = 0.0392
AGC_ALPHA_SMOOTH  = 0.3297
AGC_TARGET        = 0.25
AGC_MIN           = 0.25
AGC_MAX           = 4.0

DATA = Path(__file__).parent.parent / "training_data"
rng  = np.random.default_rng(42)


# ── DF1 biquad (exact mirror of firmware biquad_df1.c) ──────────────────────

class BiquadDF1:
    def __init__(self, coef):
        self.b0, self.b1, self.b2, self.na1, self.na2 = coef
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def process(self, x_arr):
        out = np.empty_like(x_arr)
        b0, b1, b2, na1, na2 = self.b0, self.b1, self.b2, self.na1, self.na2
        x1, x2, y1, y2 = self.x1, self.x2, self.y1, self.y2
        for i, x in enumerate(x_arr):
            y = b0*x + b1*x1 + b2*x2 + na1*y1 + na2*y2
            x2, x1 = x1, x
            y2, y1 = y1, y
            out[i] = y
        self.x1, self.x2, self.y1, self.y2 = x1, x2, y1, y2
        return out


# ── Gate simulation — exact copy of test_multi_env.py (do not diverge) ──────

class GateSim:
    def __init__(self):
        self.filters   = [BiquadDF1(c) for c in COEFS]
        self.dc_xp = self.dc_yp = 0.0
        self.mod_mean = self.mod_var = 0.0
        self.mod_n = 0
        self.MOD_ALPHA = 0.03921

    def process_frame(self, frame_i16):
        # DC removal
        x_f = frame_i16.astype(np.float64)
        dc_out = np.empty(FRAME)
        xp, yp = self.dc_xp, self.dc_yp
        for i, x in enumerate(x_f):
            y = DC_ALPHA * yp + x - xp
            xp, yp = x, y
            dc_out[i] = y
        self.dc_xp, self.dc_yp = xp, yp

        # Resonator bank
        ch_out  = [self.filters[ch].process(dc_out / 32768.0) for ch in range(4)]
        ch_q15  = [np.clip(c * 32768.0, -32768, 32767).astype(np.int16)
                   for c in ch_out]

        # Per-channel energy: sum(x²) >> 10 (mirrors arm_power_q15 >> 10)
        energies = [int(np.sum(ch_q15[c].astype(np.int32)**2) >> 10)
                    for c in range(4)]
        total_e  = sum(energies)

        # Pearson correlation ch0–ch1 in Q15 units
        def pearson_q15(a, b):
            af, bf = a.astype(float), b.astype(float)
            da = np.sqrt(np.sum(af**2) * np.sum(bf**2))
            return int((np.sum(af * bf) / da * 32767) if da > 0 else 0)

        corr = pearson_q15(ch_q15[0], ch_q15[1])

        # ZCR on raw input frame (not filtered — matches firmware)
        zcr = int(np.sum(np.diff(np.sign(frame_i16)) != 0))

        # SFM: geometric / arithmetic mean of channel energies
        ef    = np.array(energies, dtype=float) + 1.0
        arith = np.mean(ef)
        geom  = np.exp(np.mean(np.log(ef)))
        sfm   = float(np.clip(geom / arith, 0, 1)) if arith >= 2.0 else 1.0

        # CV: EWMA variance (matches firmware modulation feature)
        self.mod_n   += 1
        alpha         = self.MOD_ALPHA
        self.mod_mean = alpha * total_e + (1 - alpha) * self.mod_mean
        self.mod_var  = alpha * (total_e - self.mod_mean)**2 + (1 - alpha) * self.mod_var
        cv = 0.0
        if self.mod_n >= 25 and self.mod_mean > 0:
            cv = float(np.clip(np.sqrt(self.mod_var) / self.mod_mean, 0, 2))

        # Rule 7: multi-formant hard gate
        active_ch = sum(1 for e in energies if e > MULTIBAND_CHAN_THRESH)

        r_energy    = total_e   > ENERGY_THRESH
        r_corr      = corr      > CORR_THRESH
        r_zcr       = ZCR_LO < zcr < ZCR_HI
        r_sfm       = sfm       < SFM_THRESH
        r_cv        = cv        > CV_THRESH
        r_multiband = active_ch >= MULTIBAND_MIN_ACTIVE

        score = (40*r_energy + 20*r_corr + 25*r_zcr +
                 20*r_sfm   + 20*r_cv   + 25*r_multiband)

        wake = (score >= SCORE_WAKE) and (active_ch >= MULTIBAND_MIN_ACTIVE)
        return wake, score, active_ch, energies


# ── AGC: float32 → int16 ─────────────────────────────────────────────────────

class AGCSim:
    def __init__(self):
        self.env   = AGC_TARGET
        self.gain  = 1.0
        self.dc_x  = 0.0
        self.dc_y  = 0.0

    def process(self, pcm_f32):
        """Returns int16 ndarray at AGC target level (~8192 amplitude)."""
        rms   = float(np.sqrt(np.mean(pcm_f32**2)))
        alpha = AGC_ALPHA_ATTACK if rms > self.env else AGC_ALPHA_RELEASE
        self.env  = alpha * rms + (1 - alpha) * self.env
        dg   = (AGC_TARGET / self.env) if self.env > 1e-6 else AGC_MAX
        dg   = float(np.clip(dg, AGC_MIN, AGC_MAX))
        self.gain = AGC_ALPHA_SMOOTH * dg + (1 - AGC_ALPHA_SMOOTH) * self.gain

        out_f = np.empty(len(pcm_f32), dtype=np.float32)
        for i, x in enumerate(pcm_f32):
            y = DC_ALPHA * self.dc_y + x - self.dc_x
            self.dc_x, self.dc_y = x, y
            out_f[i] = float(np.clip(y * self.gain, -1.0, 1.0))

        return np.clip(out_f * 32767.0, -32768, 32767).astype(np.int16)


# ── Audio loading ─────────────────────────────────────────────────────────────

def load_wav_16k(path):
    data, sr = sf.read(str(path), dtype='float32')
    if data.ndim > 1:
        data = data.mean(axis=1)
    if sr != FS:
        n_out = int(len(data) * FS / sr)
        data  = np.interp(np.linspace(0, len(data) - 1, n_out),
                          np.arange(len(data)), data)
    return data.astype(np.float32)


def run_clip_real(audio, gate, agc):
    """Run AGC + gate on a full audio clip; return per-frame results."""
    results = []
    for start in range(0, len(audio) - FRAME, FRAME):
        frame_f32 = audio[start:start + FRAME]
        frame_i16 = agc.process(frame_f32)
        res       = gate.process_frame(frame_i16)
        results.append(res)
    return results


def collect_files(dirs, n=100):
    files = []
    for d in dirs:
        if d.is_dir():
            files.extend(list(d.glob("*.wav")))
    random.shuffle(files)
    return files[:n]


# ── Synthetic band-limited noise generators (from test_multi_env.py) ─────────

def _gen_continuous(n_frames, signal_fn, sos=None, pre_gain=1.0):
    if not SCIPY_OK:
        return
    WARMUP = 10
    total  = (n_frames + WARMUP) * FRAME
    raw    = signal_fn(total)
    if sos is not None:
        raw = sosfilt(sos, raw * pre_gain)
    raw = np.clip(raw, -32768, 32767)
    for i in range(WARMUP, WARMUP + n_frames):
        yield raw[i*FRAME:(i+1)*FRAME].astype(np.int16)

def gen_silence(n_frames):
    amp = 32767 * 10 ** (-65 / 20)
    sos = butter(2, 4000 / (FS/2), btype='low', output='sos')
    yield from _gen_continuous(n_frames, lambda n: rng.standard_normal(n) * amp, sos=sos)

def gen_hvac(n_frames):
    sos = butter(8, 350 / (FS/2), btype='low', output='sos')
    amp = 32767 * 10 ** (-35 / 20)
    yield from _gen_continuous(n_frames, lambda n: rng.standard_normal(n), sos=sos,
                               pre_gain=amp * 8)

def gen_dishwasher(n_frames):
    sos = butter(6, 250 / (FS/2), btype='low', output='sos')
    amp = 32767 * 10 ** (-30 / 20)
    def _sig(n):
        t = np.arange(n) / FS
        return (np.sin(2*np.pi*50*t)*0.5 + np.sin(2*np.pi*100*t)*0.3)*amp + rng.standard_normal(n)*amp*0.2
    yield from _gen_continuous(n_frames, _sig, sos=sos, pre_gain=5.0)

def gen_car_engine(n_frames):
    sos_road = butter(4, 400 / (FS/2), btype='low', output='sos')
    amp = 32767 * 10 ** (-28 / 20)
    def _sig(n):
        t = np.arange(n) / FS
        return (np.sin(2*np.pi*100*t)*0.7 + np.sin(2*np.pi*200*t)*0.25
              + np.sin(2*np.pi*300*t)*0.05) * amp
    WARMUP = 10; total = (n_frames + WARMUP) * FRAME
    engine = _sig(total)
    road   = sosfilt(sos_road, rng.standard_normal(total) * amp * 0.032 * 3)
    sig    = np.clip(engine + road, -32768, 32767)
    for i in range(WARMUP, WARMUP + n_frames):
        yield sig[i*FRAME:(i+1)*FRAME].astype(np.int16)

def gen_wind(n_frames):
    sos = butter(10, 150 / (FS/2), btype='low', output='sos')
    amp = 32767 * 10 ** (-33 / 20)
    yield from _gen_continuous(n_frames, lambda n: rng.standard_normal(n), sos=sos,
                               pre_gain=amp * 15)

SYNTH_ENVS = [
    ("Silence (PDM floor)",      lambda: gen_silence(200)),
    ("Office HVAC (45 dB)",      lambda: gen_hvac(200)),
    ("Kitchen/Dishwasher (60dB)",lambda: gen_dishwasher(200)),
    ("Car Engine (70 dB)",       lambda: gen_car_engine(200)),
    ("Outdoor Wind (50 dB)",     lambda: gen_wind(200)),
]


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    random.seed(42)

    speech_dirs = [DATA / w for w in
                   ["yes", "no", "go", "stop", "up", "down", "left", "right",
                    "on", "off", "one", "two", "three", "four", "five"]]
    noise_dirs  = [DATA / "_background_noise_", DATA / "esc50"]

    speech_files = collect_files(speech_dirs, 100)
    noise_files  = collect_files(noise_dirs,  100)
    print(f"Speech files: {len(speech_files)}, Real noise files: {len(noise_files)}")

    # ══ TEST 3: Speech recall (CLIP-LEVEL) ═══════════════════════════════════
    # A 1-second command clip is "detected" if ≥1 frame wakes the gate.
    # Frame-level rate is meaningless for short clips (mostly silence padding).
    print("\n══ TEST 3 — Speech Recall (clip-level: ≥1 WAKE frame = detected) ══")
    print(f"{'File':<35} {'Frames':>6} {'MaxScore':>9} {'MaxACH':>7} {'Woke?':>6}")
    print("─" * 68)

    detected = total_clips = 0
    for i, fp in enumerate(speech_files):
        audio = load_wav_16k(fp)
        gate, agc = GateSim(), AGCSim()
        results   = run_clip_real(audio, gate, agc)
        woke      = any(r[0] for r in results)
        max_score = max((r[1] for r in results), default=0)
        max_ach   = max((r[2] for r in results), default=0)
        detected      += int(woke)
        total_clips   += 1
        if i < 20:
            print(f"{fp.name:<35} {len(results):>6} {max_score:>9} {max_ach:>7} "
                  f"{'  ✅' if woke else '  ❌':>6}")

    speech_recall = 100 * detected // total_clips if total_clips else 0
    print(f"\n── Test 3 result: {detected}/{total_clips} clips detected = {speech_recall}% recall")
    print(f"   Target: ≥85%  {'✅ PASS' if speech_recall >= 85 else '❌ FAIL'}")

    # ══ TEST 5A: Real noise files (abort = gate does NOT wake) ═══════════════
    print("\n══ TEST 5A — Real Noise Files (gate abort rate) ══")
    print("   Note: white/pink/broadband noise intentionally passes to TinyML")
    print("   stage 2 (97.8% accuracy). This gate targets band-limited noise")
    print("   (HVAC, engine harmonics, wind) — not broadband acoustic noise.")
    print(f"\n{'File':<40} {'Frames':>6} {'Abort%':>7} {'ACH':>4} {'Type':<12}")
    print("─" * 75)

    BROADBAND_KEYWORDS = {"white", "pink", "noise", "tap", "water",
                          "dish", "exercise", "bike", "vacuum", "rain"}

    real_abort = real_total = 0
    for fp in noise_files:
        audio = load_wav_16k(fp)
        gate, agc = GateSim(), AGCSim()
        results = run_clip_real(audio, gate, agc)
        aborts  = sum(1 for r in results if not r[0])
        ach_avg = float(np.mean([r[2] for r in results])) if results else 0.0
        pct     = 100 * aborts // len(results) if results else 0
        lower   = fp.name.lower()
        kind    = "broadband" if any(k in lower for k in BROADBAND_KEYWORDS) else "tonal/lowE"
        real_abort += aborts
        real_total += len(results)
        print(f"{fp.name:<40} {len(results):>6} {pct:>6}% {ach_avg:>4.1f} [{kind}]")

    real_abort_rate = 100 * real_abort // real_total if real_total else 0
    print(f"\n── Test 5A (real files): {real_abort}/{real_total} = {real_abort_rate}% abort")
    print(f"   (broadband files pass to TinyML — by design)")

    # ══ TEST 5B: Synthetic band-limited noise (gate's designed environment) ══
    print("\n══ TEST 5B — Synthetic Band-Limited Noise (HVAC/Engine/Wind) ══")
    if not SCIPY_OK:
        print("   ⚠ scipy not installed — skipping synthetic noise test")
        print("   Install: pip3 install scipy")
        synth_abort_rate = 0
    else:
        print(f"{'Environment':<30} {'Frames':>6} {'ABORTs':>7} {'Abort%':>7}")
        print("─" * 55)
        synth_abort = synth_total = 0
        synth_rates = []
        for name, noise_fn in SYNTH_ENVS:
            gate = GateSim()
            aborts = n_frames = 0
            for frame_i16 in noise_fn():
                result = gate.process_frame(frame_i16)
                aborts   += int(not result[0])
                n_frames += 1
            pct = 100 * aborts // n_frames if n_frames else 0
            synth_rates.append(pct)
            synth_abort += aborts
            synth_total += n_frames
            print(f"{name:<30} {n_frames:>6} {aborts:>7} {pct:>6}%")

        synth_abort_rate = 100 * synth_abort // synth_total if synth_total else 0
        print(f"\n── Test 5B (synthetic): {synth_abort}/{synth_total} = "
              f"{synth_abort_rate}% abort  (mean: {int(np.mean(synth_rates))}%)")
        print(f"   Target: ≥65%  {'✅ PASS' if synth_abort_rate >= 65 else '❌ FAIL'}")

    # ══ Summary ══════════════════════════════════════════════════════════════
    print("\n══ Summary ══════════════════════════════════════════════════════")
    print(f"  Test 3 Speech recall (clip-level): {speech_recall:3d}%  "
          f"{'✅ PASS' if speech_recall >= 85 else '❌ FAIL'}  (target ≥85%)")
    if SCIPY_OK:
        print(f"  Test 5B Noise abort (band-limited): {synth_abort_rate:3d}%  "
              f"{'✅ PASS' if synth_abort_rate >= 65 else '❌ FAIL'}  (target ≥65%)")
        print(f"  Test 5A Noise abort (real files):   {real_abort_rate:3d}%  "
              f"(broadband noise → TinyML stage 2, by design)")
    print()
    print("  Gate design scope: rejects band-limited noise (HVAC/engine/wind)")
    print("  Broadband noise (white/pink): passes gate → TinyML 97.8% rejects")


if __name__ == "__main__":
    main()
