#!/usr/bin/env python3
"""
Guardian — SQNR (Signal-to-Quantization Noise Ratio) Analysis

Proves that the nRF52840 resonator bank (float32 internal, Q15 I/O) introduces
negligible quantization distortion versus the ideal float64 reference.

Method:
  1. Generate a test signal (speech-like harmonic mix)
  2. Run through float64 reference pipeline (ideal)
  3. Run through float32/Q15 firmware-replica pipeline (as implemented in C)
  4. Compute per-channel SQNR = 10*log10(signal_power / error_power)

Big-tech target: SQNR > 40 dB for audio gating ("distortion < 1%")
                 SQNR > 60 dB is "transparent" (equivalent to 10-bit precision)

Usage:
    python3 sqnr_analysis.py
"""
import os
import math
import datetime
import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False

SAMPLE_RATE = 16000
FRAME_SIZE  = 320
NUM_FRAMES  = 150     # 3 seconds
NUM_RESONATORS = 4

# ── Resonator coefficients (from resonator_coefs_cmsis.h) ─────────────────────
# Format: [b0, b1, b2, -a1, -a2]  (CMSIS DF2T storage)
RESONATOR_COEFS = [
    [0.0073,  0.0, -0.0073,  1.9717, -0.9855],   # 300 Hz
    [0.0187,  0.0, -0.0187,  1.8650, -0.9616],   # 800 Hz
    [0.0355,  0.0, -0.0355,  1.6029, -0.9291],   # 1500 Hz
    [0.0578,  0.0, -0.0578,  1.0451, -0.8844],   # 2500 Hz
]
FREQ_LABELS = ['300 Hz', '800 Hz', '1500 Hz', '2500 Hz']


# ── DF2T biquad implementations ───────────────────────────────────────────────

def biquad_df2t_f64(coefs, signal_f64):
    """
    Reference: float64 arithmetic (ideal — no quantization).
    Signal is normalized float64 in [-1.0, 1.0].
    """
    b0, b1, b2, A1, A2 = [np.float64(c) for c in coefs]
    d0 = np.float64(0); d1 = np.float64(0)
    out = np.empty_like(signal_f64, dtype=np.float64)
    for n, xn in enumerate(signal_f64):
        yn     = b0 * xn + d0
        d0_new = b1 * xn + A1 * yn + d1
        d1     = b2 * xn + A2 * yn
        d0     = d0_new
        out[n] = yn
    return out


def biquad_df2t_f32_q15(coefs, signal_int16):
    """
    Firmware replica:
      1. int16 Q15 input → float32 (divide by 32768)   [matches C firmware]
      2. float32 DF2T biquad arithmetic                 [arm_biquad_cascade_df2T_f32]
      3. float32 output * 32768 → clip → int16 Q15      [firmware output format]
    Returns int16 Q15 output (exactly as C firmware bank->outputs[ch]).
    """
    b0, b1, b2, A1, A2 = [np.float32(c) for c in coefs]
    # Q15 → float32
    x = signal_int16.astype(np.float32) / np.float32(32768.0)
    d0 = np.float32(0); d1 = np.float32(0)
    out_f32 = np.empty(len(x), dtype=np.float32)
    for n, xn in enumerate(x):
        yn     = b0 * xn + d0
        d0_new = b1 * xn + A1 * yn + d1
        d1     = b2 * xn + A2 * yn
        d0     = d0_new
        out_f32[n] = yn
    # float32 → int16 Q15 with clipping
    scaled = out_f32 * np.float32(32768.0)
    scaled = np.clip(scaled, -32768.0, 32767.0)
    return scaled.astype(np.int16)


# ── Signal generation ─────────────────────────────────────────────────────────

def gen_speech_signal(n_samples):
    """
    Speech-like harmonic signal: f0=150 Hz + 6 harmonics, AM envelope.
    Amplitude 40% full scale (Q15 ~13107) — representative speech level.
    """
    t   = np.arange(n_samples, dtype=np.float64) / SAMPLE_RATE
    f0  = 150.0
    sig = np.zeros(n_samples, dtype=np.float64)
    for k in range(1, 7):
        sig += (0.4 / k) * np.sin(2 * math.pi * f0 * k * t)
    # AM modulation (syllable rate 4 Hz)
    env = 0.5 + 0.5 * np.sin(2 * math.pi * 4.0 * t)
    sig *= env
    sig  = np.clip(sig, -1.0, 1.0)
    return sig


def gen_noise_signal(n_samples):
    """Broadband LFSR noise (same generator as firmware TEST_AUDIO_NOISE)."""
    lfsr = 0xACE1
    out  = np.zeros(n_samples, dtype=np.float64)
    for i in range(n_samples):
        lfsr = (lfsr >> 1) ^ (int(-(lfsr & 1)) & 0xB400)
        lfsr &= 0xFFFF
        out[i] = ((lfsr ^ 0x8000) - 32768) / 32768.0 / 8.0
    return out


# ── SQNR computation ──────────────────────────────────────────────────────────

def compute_sqnr(reference_f64, quantized_int16):
    """
    SQNR = 10 * log10(signal_power / error_power)

    reference_f64  : float64 output from ideal pipeline ([-1, 1] normalized)
    quantized_int16: int16 Q15 output from firmware-replica pipeline

    Both are normalized to [-1, 1] for comparison.
    """
    ref_norm  = reference_f64                             # already [-1, 1]
    quant_norm = quantized_int16.astype(np.float64) / 32768.0

    # Align lengths
    n = min(len(ref_norm), len(quant_norm))
    ref_norm   = ref_norm[:n]
    quant_norm = quant_norm[:n]

    signal_power = np.mean(ref_norm ** 2)
    error        = ref_norm - quant_norm
    error_power  = np.mean(error ** 2)

    if error_power < 1e-20:
        return float('inf')

    sqnr_db = 10.0 * math.log10(signal_power / error_power)
    return sqnr_db


# ── Run analysis ──────────────────────────────────────────────────────────────

def run_analysis(signal_f64, signal_label):
    """Run all 4 resonator channels for the given signal, return SQNR table."""
    n = len(signal_f64)
    signal_int16 = np.clip(signal_f64 * 32768.0, -32768, 32767).astype(np.int16)

    results = []
    ref_outputs   = []
    quant_outputs = []

    for ch in range(NUM_RESONATORS):
        ref_out   = biquad_df2t_f64(RESONATOR_COEFS[ch], signal_f64)
        quant_out = biquad_df2t_f32_q15(RESONATOR_COEFS[ch], signal_int16)
        sqnr      = compute_sqnr(ref_out, quant_out)

        results.append({
            'channel':   ch,
            'freq':      FREQ_LABELS[ch],
            'sqnr_db':   sqnr,
            'pass':      sqnr > 40.0,
        })
        ref_outputs.append(ref_out)
        quant_outputs.append(quant_out)

    print(f"\n  Signal: {signal_label}")
    print(f"  {'Channel':<12} {'Freq':>8}  {'SQNR (dB)':>10}  {'Result':>8}")
    print(f"  {'─'*44}")
    for r in results:
        verdict = 'PASS ✓' if r['pass'] else 'FAIL ✗'
        print(f"  {'Ch'+str(r['channel']):<12} {r['freq']:>8}  {r['sqnr_db']:>10.1f}  {verdict:>8}")

    avg_sqnr = np.mean([r['sqnr_db'] for r in results])
    print(f"  {'─'*44}")
    print(f"  {'Average':<12} {'':>8}  {avg_sqnr:>10.1f}  "
          f"{'PASS ✓' if avg_sqnr > 40.0 else 'FAIL ✗':>8}")

    return results, ref_outputs, quant_outputs


def main():
    print("=" * 60)
    print("GUARDIAN — SQNR ANALYSIS")
    print("Quantization distortion: float32/Q15 vs float64 reference")
    print("=" * 60)
    print(f"\nTarget: SQNR > 40 dB  (distortion < 1% power)")
    print(f"        SQNR > 60 dB  (transparent — no audible artefacts)")
    print(f"\nNote: Firmware uses float32 arithmetic internally.")
    print(f"      Q15 only at I/O boundaries (ADC input → resonator output).")

    n_samples = FRAME_SIZE * NUM_FRAMES
    speech_f64 = gen_speech_signal(n_samples)
    noise_f64  = gen_noise_signal(n_samples)

    speech_results, sp_refs, sp_quants = run_analysis(speech_f64, "Speech (150Hz f0, 6 harmonics, AM)")
    noise_results,  ns_refs, ns_quants = run_analysis(noise_f64,  "Noise (LFSR broadband)")

    all_pass = all(r['pass'] for r in speech_results + noise_results)
    all_sqnr = [r['sqnr_db'] for r in speech_results + noise_results]

    print(f"\n{'='*60}")
    print(f"OVERALL VERDICT: {'PASS — All channels exceed 40 dB SQNR' if all_pass else 'FAIL'}")
    print(f"Min SQNR : {min(all_sqnr):.1f} dB")
    print(f"Max SQNR : {max(all_sqnr):.1f} dB")
    print(f"Avg SQNR : {np.mean(all_sqnr):.1f} dB")
    print(f"\nInterpretation:")
    avg = np.mean(all_sqnr)
    if avg > 80:
        print(f"  {avg:.0f} dB >> 60 dB target — float32 arithmetic introduces")
        print(f"  negligible distortion. Equivalent to >13-bit precision.")
        print(f"  Q15 I/O rounding is the dominant (and acceptable) error source.")
    elif avg > 60:
        print(f"  {avg:.0f} dB > 60 dB — transparent quality.")
    elif avg > 40:
        print(f"  {avg:.0f} dB > 40 dB — meets big-tech audio gate target.")
    else:
        print(f"  {avg:.0f} dB < 40 dB — review filter implementation.")
    print(f"{'='*60}")

    # ── Save results ──────────────────────────────────────────────────────────
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '../results')
    os.makedirs(out_dir, exist_ok=True)
    ts  = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f'sqnr_analysis_{ts}.csv')
    png_path = os.path.join(out_dir, f'sqnr_analysis_{ts}.png')

    import csv
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['signal', 'channel', 'freq', 'sqnr_db', 'pass'])
        w.writeheader()
        for r in speech_results:
            w.writerow({'signal': 'speech', **r})
        for r in noise_results:
            w.writerow({'signal': 'noise', **r})
    print(f"\nCSV: {csv_path}")

    if not HAS_PLOT:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle('Guardian SQNR Analysis — float32/Q15 vs float64 Reference',
                 fontsize=13, fontweight='bold')

    colors = ['#2ca02c', '#1f77b4', '#ff7f0e', '#9467bd']

    # ── Top-left: SQNR bar chart ──────────────────────────────────────────────
    ax = axes[0][0]
    x     = np.arange(NUM_RESONATORS)
    width = 0.35
    bars_s = ax.bar(x - width/2, [r['sqnr_db'] for r in speech_results],
                    width, label='Speech', color='#2ca02c', alpha=0.82, edgecolor='black')
    bars_n = ax.bar(x + width/2, [r['sqnr_db'] for r in noise_results],
                    width, label='Noise',  color='#d62728', alpha=0.82, edgecolor='black')
    ax.axhline(40, color='orange', linestyle='--', linewidth=1.5, label='40 dB target')
    ax.axhline(60, color='green',  linestyle='--', linewidth=1.2, label='60 dB transparent')
    ax.set_xticks(x); ax.set_xticklabels(FREQ_LABELS)
    ax.set_ylabel('SQNR (dB)'); ax.set_title('SQNR per Resonator Channel')
    ax.legend(); ax.grid(axis='y', alpha=0.3)
    for bar in list(bars_s) + list(bars_n):
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, h + 0.5,
                f'{h:.0f}', ha='center', fontsize=8)

    # ── Top-right: Error signal (channel 0, speech) ───────────────────────────
    ax   = axes[0][1]
    ref  = sp_refs[0]
    qout = sp_quants[0].astype(np.float64) / 32768.0
    t_ms = np.arange(len(ref)) / SAMPLE_RATE * 1000
    show = min(len(t_ms), FRAME_SIZE * 5)   # show 5 frames = 100ms
    ax.plot(t_ms[:show], (ref[:show] - qout[:show]) * 1000,
            linewidth=0.6, color='#d62728', label='Error ×1000')
    ax.set_xlabel('Time (ms)'); ax.set_ylabel('Quantization error (×10⁻³)')
    ax.set_title('Quantization Error — Ch0 300 Hz (Speech, first 100ms)')
    ax.legend(); ax.grid(alpha=0.3)

    # ── Bottom-left: Frequency response overlay (channel 1, 800 Hz) ──────────
    ax   = axes[1][0]
    ref1 = sp_refs[1]
    qout1= sp_quants[1].astype(np.float64) / 32768.0
    t_ms = np.arange(len(ref1)) / SAMPLE_RATE * 1000
    show = FRAME_SIZE * 3
    ax.plot(t_ms[:show], ref1[:show],  linewidth=0.8, color='#1f77b4', label='Float64 ref', alpha=0.8)
    ax.plot(t_ms[:show], qout1[:show], linewidth=0.8, color='#ff7f0e', linestyle='--',
            label='Float32/Q15 firmware', alpha=0.8)
    ax.set_xlabel('Time (ms)'); ax.set_ylabel('Amplitude (normalized)')
    ax.set_title('Float64 vs Float32/Q15 — Ch1 800 Hz (Speech, first 60ms)')
    ax.legend(); ax.grid(alpha=0.3)

    # ── Bottom-right: Summary text ─────────────────────────────────────────────
    ax = axes[1][1]
    ax.axis('off')
    summary = (
        f"SQNR Summary\n"
        f"{'─'*35}\n"
        f"{'Channel':<12} {'Speech':>8}  {'Noise':>8}\n"
        f"{'─'*35}\n"
    )
    for s, n in zip(speech_results, noise_results):
        summary += f"{s['freq']:<12} {s['sqnr_db']:>7.1f}dB  {n['sqnr_db']:>7.1f}dB\n"
    summary += f"{'─'*35}\n"
    summary += f"{'Average':<12} {np.mean([r['sqnr_db'] for r in speech_results]):>7.1f}dB  "
    summary += f"{np.mean([r['sqnr_db'] for r in noise_results]):>7.1f}dB\n\n"
    summary += f"Target: > 40 dB\n"
    summary += f"Result: {'PASS' if all_pass else 'FAIL'}\n\n"
    summary += f"Firmware uses float32 internally;\n"
    summary += f"Q15 only at ADC input boundary.\n"
    summary += f"Dominant error: 1-LSB Q15 rounding\n"
    summary += f"= 1/32768 = 0.003% amplitude."
    ax.text(0.05, 0.95, summary, transform=ax.transAxes,
            fontfamily='monospace', fontsize=9, verticalalignment='top')

    plt.tight_layout()
    plt.savefig(png_path, dpi=300)
    print(f"Plot: {png_path}")
    plt.show()


if __name__ == '__main__':
    main()
