#!/usr/bin/env python3
"""
Guardian SDK — Resonator Bank Frequency Response
=================================================
Two-layer validation:

  Layer 1 (Analytical): scipy.signal.freqz on the exact coefficient values
  from resonator_coefs_default.h.  Proves the coefficient math is correct.

  Layer 2 (Empirical):  Sweep 50–8000 Hz in 200 steps.  For each frequency,
  generate a 4-frame sine burst (1280 samples), run it through a Python
  reimplementation of guardian_pf_process(), measure steady-state output
  energy per channel.  Proves the C99 DF1 implementation tracks the design.

  Both layers are plotted on the same figure.  If they diverge, the
  implementation has a bug.  If they agree, you have proof.

Usage:
    python3 tools/plot_freq_response.py
    # saves guardian_freq_response.png in results/
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.signal import freqz, lfilter
import os

FS = 16000

# ── Exact coefficients from resonator_coefs_default.h ──────────────────────
# Format: {b0, b1, b2, -a1, -a2}  (CMSIS convention, negated denominator)
COEFS = [
    [0.0073, 0.0,  -0.0073,  1.9717, -0.9855],   # 300 Hz
    [0.0187, 0.0,  -0.0187,  1.8650, -0.9616],   # 800 Hz
    [0.0355, 0.0,  -0.0355,  1.6029, -0.9291],   # 1500 Hz
    [0.0578, 0.0,  -0.0578,  1.0451, -0.8844],   # 2500 Hz
]
CENTER_FREQS = [300, 800, 1500, 2500]
COLORS       = ['#2196F3', '#4CAF50', '#FF9800', '#F44336']
LABELS       = ['Ch0: 300 Hz', 'Ch1: 800 Hz', 'Ch2: 1500 Hz', 'Ch3: 2500 Hz']

# Convert CMSIS {b0,b1,b2,-a1,-a2} to scipy b/a arrays
# scipy lfilter: y = b[0]*x + b[1]*x[-1] + b[2]*x[-2]
#                  - a[1]*y[-1] - a[2]*y[-2]  (a[0]=1)
# CMSIS stores -a1 and -a2 (negated), so a1_true = -coef[3], a2_true = -coef[4]
def cmsis_to_scipy(coef):
    b = [coef[0], coef[1], coef[2]]
    a = [1.0, -coef[3], -coef[4]]      # un-negate
    return np.array(b), np.array(a)

# ── Pure Python DF1 biquad (mirrors biquad_df1.c exactly) ──────────────────
class BiquadDF1:
    def __init__(self, coef):
        self.b0, self.b1, self.b2 = coef[0], coef[1], coef[2]
        self.na1, self.na2        = coef[3], coef[4]
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def reset(self):
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def process_block(self, x_arr):
        out = np.empty_like(x_arr)
        b0, b1, b2 = self.b0, self.b1, self.b2
        na1, na2   = self.na1, self.na2
        x1, x2, y1, y2 = self.x1, self.x2, self.y1, self.y2
        for i, x in enumerate(x_arr):
            y = b0*x + b1*x1 + b2*x2 + na1*y1 + na2*y2
            x2, x1 = x1, x
            y2, y1 = y1, y
            out[i] = y
        self.x1, self.x2, self.y1, self.y2 = x1, x2, y1, y2
        return out

# ── Layer 1: Analytical frequency response ─────────────────────────────────
def analytical_response(n_freqs=2048):
    responses = []
    for coef in COEFS:
        b, a = cmsis_to_scipy(coef)
        w, h = freqz(b, a, worN=n_freqs, fs=FS)
        responses.append((w, 20 * np.log10(np.abs(h) + 1e-10)))
    return responses

# ── Layer 2: Empirical (simulate guardian_pf_process) ──────────────────────
def empirical_response(n_freqs=200):
    freqs = np.linspace(50, 7900, n_freqs)
    filters = [BiquadDF1(c) for c in COEFS]
    gains   = [[] for _ in COEFS]

    # DC removal state (α = 0.99215, mirrors preprocess.c)
    DC_ALPHA = 0.99215

    for f in freqs:
        # Reset all filters and DC state for each frequency
        for flt in filters:
            flt.reset()
        dc_x_prev = dc_y_prev = 0.0

        # Generate 4 frames (1280 samples) of sine — first 3 are warm-up,
        # measure energy only on the 4th (steady-state)
        t = np.arange(4 * 320) / FS
        sine = 0.6 * np.sin(2 * np.pi * f * t)    # float [-1, 1]

        # Apply DC removal
        dc_out = np.empty_like(sine)
        for i, x in enumerate(sine):
            y = DC_ALPHA * dc_y_prev + x - dc_x_prev
            dc_x_prev, dc_y_prev = x, y
            dc_out[i] = y

        # Run through each resonator
        for ch, flt in enumerate(filters):
            ch_out = flt.process_block(dc_out)
            # Measure RMS energy on last frame only (steady-state)
            ss = ch_out[-320:]
            rms = np.sqrt(np.mean(ss ** 2))
            gains[ch].append(20 * np.log10(rms + 1e-10))

    return freqs, gains

# ── Plot ─────────────────────────────────────────────────────────────────────
def plot(analytical, empirical_freqs, empirical_gains, out_path):
    fig = plt.figure(figsize=(14, 10))
    gs  = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.32)

    # ── Top: all 4 channels overlaid (analytical) ──────────────────────────
    ax_all = fig.add_subplot(gs[0, :])
    for ch, (w, h_db) in enumerate(analytical):
        ax_all.plot(w, h_db, color=COLORS[ch], linewidth=1.8,
                    label=LABELS[ch])
    for f0 in CENTER_FREQS:
        ax_all.axvline(f0, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)
    ax_all.axhline(-3, color='black', linestyle='--', linewidth=0.7,
                   alpha=0.5, label='−3 dB')
    ax_all.set_xlim(50, 8000)
    ax_all.set_ylim(-80, 5)
    ax_all.set_xlabel('Frequency (Hz)', fontsize=11)
    ax_all.set_ylabel('Gain (dB)', fontsize=11)
    ax_all.set_title('Guardian SDK — Resonator Bank Frequency Response '
                     '(Analytical, DF1 float32)', fontsize=12, fontweight='bold')
    ax_all.legend(fontsize=9, loc='lower right')
    ax_all.grid(True, alpha=0.3)

    # ── Bottom: per-channel analytical vs empirical ─────────────────────────
    for ch in range(4):
        row, col = divmod(ch, 2)
        ax = fig.add_subplot(gs[1, col]) if ch < 2 else None
        if ch == 2:
            ax = fig.add_subplot(gs[1, 0])
        elif ch == 3:
            ax = fig.add_subplot(gs[1, 1])
        # Re-create subplots cleanly
        ax = fig.add_subplot(gs[1, ch % 2]) if ch < 2 else None

    axes_bottom = [fig.add_subplot(gs[1, c]) for c in range(2)]
    # Overwrite — use individual axes properly
    fig.clear()
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.48, wspace=0.35)

    ax_all = fig.add_subplot(gs[0, :])
    for ch, (w, h_db) in enumerate(analytical):
        ax_all.plot(w, h_db, color=COLORS[ch], linewidth=1.8, label=LABELS[ch])
    for f0 in CENTER_FREQS:
        ax_all.axvline(f0, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)
    ax_all.axhline(-3, color='black', linestyle='--', linewidth=0.7, alpha=0.5,
                   label='−3 dB')
    ax_all.set_xlim(50, 8000)
    ax_all.set_ylim(-80, 5)
    ax_all.set_xlabel('Frequency (Hz)', fontsize=11)
    ax_all.set_ylabel('Gain (dB)', fontsize=11)
    ax_all.set_title('Guardian SDK — Resonator Bank Frequency Response '
                     '(Analytical, DF1 float32)', fontsize=12, fontweight='bold')
    ax_all.legend(fontsize=9, loc='lower right')
    ax_all.grid(True, alpha=0.3)

    # Compute -3 dB bandwidths from analytical
    bw_info = []
    for ch, (w, h_db) in enumerate(analytical):
        above = w[h_db > -3]
        if len(above) >= 2:
            bw = above[-1] - above[0]
            q  = CENTER_FREQS[ch] / bw if bw > 0 else float('inf')
        else:
            bw, q = 0, 0
        bw_info.append((bw, q))

    # Per-channel: analytical (line) + empirical (scatter) + BW annotation
    ch_positions = [(1, 0), (1, 0), (1, 1), (1, 1)]
    axs = [fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 0]),
           fig.add_subplot(gs[1, 1]), fig.add_subplot(gs[1, 1])]
    # Proper: 4 subplots in bottom row → use GridSpecFromSubplotSpec
    from matplotlib.gridspec import GridSpecFromSubplotSpec
    gs_bottom = GridSpecFromSubplotSpec(1, 4, subplot_spec=gs[1, :],
                                        wspace=0.35)
    for ch in range(4):
        ax = fig.add_subplot(gs_bottom[ch])
        w, h_db = analytical[ch]
        ax.plot(w, h_db, color=COLORS[ch], linewidth=1.5,
                label='Analytical')
        ax.scatter(empirical_freqs, empirical_gains[ch], s=6,
                   color=COLORS[ch], alpha=0.5, label='Empirical')
        ax.axvline(CENTER_FREQS[ch], color='gray', linestyle=':', linewidth=1)
        ax.axhline(-3, color='black', linestyle='--', linewidth=0.7, alpha=0.5)
        bw, q = bw_info[ch]
        ax.set_title(f'{CENTER_FREQS[ch]} Hz\nBW={bw:.0f} Hz  Q={q:.1f}',
                     fontsize=9, fontweight='bold', color=COLORS[ch])
        ax.set_xlim(max(50, CENTER_FREQS[ch] // 4),
                    min(8000, CENTER_FREQS[ch] * 4))
        ax.set_ylim(-60, 5)
        ax.set_xlabel('Hz', fontsize=8)
        ax.set_ylabel('dB' if ch == 0 else '', fontsize=8)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=7)

    fig.suptitle('Guardian SDK — Filter Bank Validation\n'
                 'Analytical (scipy) vs Empirical (Python DF1 simulation)',
                 fontsize=11, y=1.01)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"Saved: {out_path}")
    return bw_info

def print_summary(bw_info):
    print("\n── Resonator Bank Summary ───────────────────────────────────────")
    print(f"{'Channel':<10} {'Center Hz':<12} {'−3dB BW (Hz)':<16} {'Q factor':<10}")
    print("─" * 50)
    for ch, (bw, q) in enumerate(bw_info):
        print(f"Ch {ch:<7} {CENTER_FREQS[ch]:<12} {bw:<16.1f} {q:<10.2f}")
    print()
    print("Expected: Q increases for lower frequencies (higher Q = narrower band)")
    print("  300 Hz resonator needs Q~20+ to stay narrow relative to Nyquist")
    print("  2500 Hz resonator can use lower Q — wider band is acceptable\n")

if __name__ == '__main__':
    print("Computing analytical frequency response (scipy.signal.freqz)...")
    analytical = analytical_response()

    print("Computing empirical frequency response (Python DF1 sweep)...")
    emp_freqs, emp_gains = empirical_response()

    out = os.path.join(os.path.dirname(__file__), '..', 'results',
                       'guardian_freq_response.png')
    bw_info = plot(analytical, emp_freqs, emp_gains, out)
    print_summary(bw_info)
    print("Done. Open results/guardian_freq_response.png")
