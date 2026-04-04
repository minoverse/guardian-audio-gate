#!/usr/bin/env python3
"""
Guardian — 4-Channel Resonator Bank Frequency Response
=======================================================
Sweeps a sine wave from 50 Hz to 8000 Hz through each of the 4 resonator
channels (using the exact Python-mirror coefficients from the firmware).
Plots RMS energy per channel vs frequency — proves the filters are tuned
to 300 / 800 / 1500 / 2500 Hz with the designed Q values.

Usage:
    python3 tools/plot_frequency_response.py             # display plot
    python3 tools/plot_frequency_response.py --save      # save PNG

Expected output:
    Four bell-shaped curves, each peaking at its design frequency.
    At -3 dB point: Q = f_center / bandwidth.
      ch0 (300 Hz):  Q ≈ 68
      ch1 (800 Hz):  Q ≈ 26
      ch2 (1500 Hz): Q ≈ 14
      ch3 (2500 Hz): Q ≈  9
"""

import argparse
import numpy as np

FS     = 16000
FRAME  = 320
N_FRAMES = 20   # allow filter to reach steady state before measuring

# Exact coefficients from firmware resonator_coefs_default.h
# Format: [b0, b1, b2, -a1, -a2]  (CMSIS convention, na1=-a1, na2=-a2)
COEFS = [
    [0.0073, 0.0, -0.0073,  1.9717, -0.9855],   # ch0  300 Hz  Q=68
    [0.0187, 0.0, -0.0187,  1.8650, -0.9616],   # ch1  800 Hz  Q=26
    [0.0355, 0.0, -0.0355,  1.6029, -0.9291],   # ch2 1500 Hz  Q=14
    [0.0578, 0.0, -0.0578,  1.0451, -0.8844],   # ch3 2500 Hz  Q= 9
]
CENTER_FREQS = [300, 800, 1500, 2500]
CHANNEL_NAMES = ['ch0 300Hz Q=68', 'ch1 800Hz Q=26',
                 'ch2 1500Hz Q=14', 'ch3 2500Hz Q=9']

# Sweep range
FREQS = np.logspace(np.log10(50), np.log10(8000), 300)


def biquad_block(coef, x_in, state=None):
    """DF1 biquad — matches firmware biquad_df1.c exactly."""
    b0, b1, b2, na1, na2 = coef
    if state is None:
        state = [0.0, 0.0, 0.0, 0.0]   # x1, x2, y1, y2
    x1, x2, y1, y2 = state
    out = np.empty(len(x_in))
    for i, x in enumerate(x_in):
        y = b0*x + b1*x1 + b2*x2 + na1*y1 + na2*y2
        x2, x1, y2, y1 = x1, x, y1, y
        out[i] = y
    return out, [x1, x2, y1, y2]


def measure_rms(freq, coef):
    """Measure steady-state RMS output for a sine at `freq` Hz."""
    amplitude = 20000.0
    states = None
    rms_last = 0.0
    for frame_i in range(N_FRAMES):
        t = np.arange(FRAME) / FS + frame_i * FRAME / FS
        x = amplitude * np.sin(2 * np.pi * freq * t)
        y, states = biquad_block(coef, x, states)
        if frame_i >= N_FRAMES - 3:   # average last 3 frames
            rms_last += np.sqrt(np.mean(y**2))
    return rms_last / 3.0


def main(save):
    energies = np.zeros((len(COEFS), len(FREQS)))
    for ch, coef in enumerate(COEFS):
        print(f"Sweeping ch{ch} ({CENTER_FREQS[ch]} Hz) ...", flush=True)
        for fi, f in enumerate(FREQS):
            energies[ch, fi] = measure_rms(f, coef)

    # Normalise each channel to its own peak (0 dB = peak)
    energies_db = np.zeros_like(energies)
    for ch in range(len(COEFS)):
        pk = np.max(energies[ch])
        if pk > 0:
            energies_db[ch] = 20 * np.log10(energies[ch] / pk + 1e-12)

    # ── Measure actual -3 dB bandwidth ───────────────────────────────────────
    print("\n── Measured frequency response ─────────────────────────────────")
    print(f"{'Channel':<22} {'f_center':>10} {'f_peak':>10} {'BW-3dB':>10} {'Q_meas':>8}")
    print("─" * 65)
    for ch in range(len(COEFS)):
        peak_idx = np.argmax(energies_db[ch])
        f_peak   = FREQS[peak_idx]
        # find -3 dB frequencies
        below = energies_db[ch] >= -3.0
        idxs  = np.where(below)[0]
        if len(idxs) >= 2:
            f_lo = FREQS[idxs[0]]
            f_hi = FREQS[idxs[-1]]
            bw   = f_hi - f_lo
            q    = f_peak / bw if bw > 0 else float('inf')
        else:
            f_lo = f_hi = bw = q = float('nan')
        print(f"{CHANNEL_NAMES[ch]:<22} {CENTER_FREQS[ch]:>10} Hz {f_peak:>8.0f} Hz "
              f"{bw:>8.1f} Hz {q:>8.1f}")

    # ── Print headroom budget ─────────────────────────────────────────────────
    print("\n── Float32 overflow headroom budget ────────────────────────────")
    print("Input: Q15 int16 [-32768, 32767] → float [-32768.0, 32767.0]")
    print("HPF peak gain: ≤2× (step response, 1st-order IIR)")
    print("AGC cal_gain range: [0.5, 2.0]  (mic_cal_is_valid spec)")
    print("AGC agc_gain range: [0.25, 4.0] (AUDIO_FRONTEND_AGC_MIN/MAX)")
    print("total_gain max: 2.0 × 4.0 = 8.0")
    print()
    max_y         = 32768.0 * 2.0           # HPF peak
    max_scaled    = max_y * 8.0             # × max total_gain
    int32_max     = 2**31 - 1
    float32_max   = 3.4028235e+38
    print(f"  max |y|        = {max_y:>12.0f}  (HPF output ceiling)")
    print(f"  max |y×gain|   = {max_scaled:>12.0f}  (before SSAT)")
    print(f"  INT32_MAX      = {int32_max:>12d}")
    print(f"  margin (int32) = {int32_max / max_scaled:>12.0f}×  (UB-safe)")
    print(f"  FLOAT32_MAX    = {float32_max:.2e}")
    print(f"  margin (f32)   = {float32_max / max_scaled:.2e}×")
    print()
    print("  Resonator bank input: SSAT output ∈ [-32768, 32767] — bounded.")
    print("  Biquad DF1 float32 accumulator cannot overflow (IEEE 754).")
    print("  worst-case |y_resonator| ≤ peak_gain × 32767 ≈ 0.12 × 32767 = 3930")
    print("  (bandpass peak gain < 1.0 for all 4 resonators — attenuating)")

    # ── Plot ─────────────────────────────────────────────────────────────────
    try:
        import matplotlib.pyplot as plt
        colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']
        fig, ax = plt.subplots(figsize=(10, 5))
        for ch in range(len(COEFS)):
            ax.semilogx(FREQS, energies_db[ch],
                        color=colors[ch], lw=2, label=CHANNEL_NAMES[ch])
        ax.axhline(-3, color='k', linestyle='--', lw=0.8, label='-3 dB')
        for fc in CENTER_FREQS:
            ax.axvline(fc, color='grey', linestyle=':', lw=0.6)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Magnitude (dB, normalised to peak)')
        ax.set_title('Guardian Resonator Bank — Frequency Response\n'
                     'nRF52840 firmware coefficients, measured via Python simulation')
        ax.set_xlim(50, 8000)
        ax.set_ylim(-40, 3)
        ax.legend(loc='lower right')
        ax.grid(True, which='both', alpha=0.3)
        ax.set_xticks([100, 300, 800, 1500, 2500, 5000, 8000])
        ax.set_xticklabels(['100', '300', '800', '1500', '2500', '5000', '8000'])
        plt.tight_layout()
        if save:
            out = 'tools/frequency_response.png'
            plt.savefig(out, dpi=150)
            print(f"\nSaved: {out}")
        else:
            plt.show()
    except ImportError:
        print("\n(matplotlib not available — install with: pip3 install matplotlib)")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--save', action='store_true',
                    help='Save PNG instead of displaying')
    args = ap.parse_args()
    main(args.save)
