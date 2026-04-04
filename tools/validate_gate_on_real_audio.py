#!/usr/bin/env python3
"""
Guardian Week 10 — Gate Validation on Real (or Synthetic) Audio

Runs a Python-exact simulation of the nRF52840 gate logic on WAV files and
computes Precision / Recall / F1.  All feature extraction replicates the C
implementation in firmware/lib/guardian_dsp exactly:

  • Resonator bank  : 4× DF2T biquad (300/800/1500/2500 Hz) — same coefficients
  • Energy          : arm_rms_q15 equivalent (sqrt of mean squared Q15 samples)
  • Correlation     : Pearson r × 32767 (Q15), adjacent channel pairs
  • Coherence       : dot-product autocorrelation lag 80-250 step-by-2, >> 15
  • ZCR             : sign-change count per frame
  • Gate scoring    : Rule1 +40 | Rule2 +20 | Rule3 +25 | Rule4 +15; wake ≥ 55

Usage:
    # Generate synthetic set first (no microphone needed):
    python3 record_test_set.py --synthetic

    # Then validate:
    python3 validate_gate_on_real_audio.py results/audio_test_set/labels.csv

    # Or with a custom labels CSV:
    python3 validate_gate_on_real_audio.py /path/to/labels.csv
"""
import sys
import os
import csv
import wave
import struct
import math
import datetime
import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False
    print("matplotlib not available — skipping plots (pip3 install matplotlib)")

# ── Gate configuration (mirrors GATE_CONFIG_DEFAULT in decision.h) ────────────
CORRELATION_THRESHOLD = 14746   # 0.45 × 32767 (Q15)
COHERENCE_THRESHOLD   = 1000    # autocorr peak units
ZCR_MAX               = 80      # max zero crossings per frame
NOISE_FLOOR_INIT      = 500     # Q15 initial noise estimate
WAKE_SCORE            = 55      # minimum score to wake TinyML
FRAME_SIZE            = 320     # samples per frame (20 ms @ 16 kHz)

# EMA coefficients (Q15): 0.9 → 29491,  0.1 → 3277
EMA_ALPHA_Q15 = 29491
EMA_BETA_Q15  = 3277

# ── Resonator bank coefficients (from resonator_coefs_cmsis.h) ────────────────
# Format: [b0, b1, b2, -a1, -a2]  (CMSIS DF2T storage)
RESONATOR_COEFS = [
    [0.0073,  0.0, -0.0073,  1.9717, -0.9855],   # 300 Hz
    [0.0187,  0.0, -0.0187,  1.8650, -0.9616],   # 800 Hz
    [0.0355,  0.0, -0.0355,  1.6029, -0.9291],   # 1500 Hz
    [0.0578,  0.0, -0.0578,  1.0451, -0.8844],   # 2500 Hz
]
NUM_RESONATORS = 4


# ── Resonator bank (CMSIS DF2T biquad — exact match to C firmware) ────────────

class BiquadDF2T:
    """Single-stage DF2T biquad.  Matches arm_biquad_cascade_df2T_f32."""
    def __init__(self, coefs):
        self.b0, self.b1, self.b2, self.A1, self.A2 = coefs
        self.d0 = 0.0
        self.d1 = 0.0

    def process_frame(self, x_float):
        """Process a numpy float array in-place; returns float output array."""
        out  = np.empty_like(x_float)
        d0, d1 = self.d0, self.d1
        b0, b1, b2, A1, A2 = self.b0, self.b1, self.b2, self.A1, self.A2
        for n, xn in enumerate(x_float):
            yn      = b0 * xn + d0
            d0_new  = b1 * xn + A1 * yn + d1
            d1      = b2 * xn + A2 * yn
            d0      = d0_new
            out[n]  = yn
        self.d0, self.d1 = d0, d1
        return out


class ResonatorBank:
    def __init__(self):
        self.filters = [BiquadDF2T(c) for c in RESONATOR_COEFS]

    def process(self, frame_q15):
        """
        Input : int16 numpy array (Q15 samples from PDM mic / WAV)
        Output: (4, FRAME_SIZE) int16 array — Q15 resonator outputs
        """
        x_float = frame_q15.astype(np.float64) / 32768.0
        outputs = np.zeros((NUM_RESONATORS, FRAME_SIZE), dtype=np.int16)
        for ch, filt in enumerate(self.filters):
            y = filt.process_frame(x_float)
            # float → Q15 with clipping (matches C firmware)
            y_scaled = y * 32768.0
            y_scaled = np.clip(y_scaled, -32768, 32767)
            outputs[ch] = y_scaled.astype(np.int16)
        return outputs


# ── Feature extraction (exact Python replicas of C functions) ─────────────────

def extract_energy(outputs_q15):
    """
    arm_rms_q15 per channel: sqrt(mean(x^2)) in Q15 units, then average.
    Returns int16 (total_energy).
    """
    total = 0
    for ch in range(NUM_RESONATORS):
        rms = int(math.sqrt(np.mean(outputs_q15[ch].astype(np.int64) ** 2)))
        total += rms
    return total // NUM_RESONATORS   # int16 average


def _pearson_q15(x, y):
    """Pearson r × 32767 in Q15.  Matches compute_correlation_q15 in C."""
    n      = len(x)
    mx     = int(np.sum(x.astype(np.int64))) // n
    my     = int(np.sum(y.astype(np.int64))) // n
    dx     = x.astype(np.int64) - mx
    dy     = y.astype(np.int64) - my
    cov    = int(np.sum(dx * dy))
    var_x  = int(np.sum(dx * dx))
    var_y  = int(np.sum(dy * dy))
    if var_x <= 0 or var_y <= 0:
        return 0
    denom = math.sqrt(float(var_x) * float(var_y))
    if denom < 1.0:
        return 0
    r = cov / denom
    r = max(-1.0, min(1.0, r))
    return int(r * 32767.0)


def extract_correlation(outputs_q15):
    """Max Pearson r (Q15) across adjacent channel pairs 0-1, 1-2, 2-3."""
    c01 = _pearson_q15(outputs_q15[0], outputs_q15[1])
    c12 = _pearson_q15(outputs_q15[1], outputs_q15[2])
    c23 = _pearson_q15(outputs_q15[2], outputs_q15[3])
    return max(c01, c12, c23)


def estimate_lag_min(signal_q15, default=62):
    """
    Dynamic lag window: estimate fundamental frequency from raw signal
    autocorrelation, return lag_min = floor(16000 / (f0 * 1.15)).

    Why 1.15× safety margin: we want lag_min slightly BELOW the true pitch
    period so the coherence search always includes the first harmonic peak.

    Falls back to default=62 (258 Hz) if no clear pitch found.
    Coverage mapping:
        lag_min=40 → f0 up to ~400 Hz  (child / falsetto)
        lag_min=62 → f0 up to ~258 Hz  (female, fixed value)
        lag_min=80 → f0 up to ~200 Hz  (original male-only value)
    """
    sig = signal_q15.astype(np.float64)
    sig -= sig.mean()
    energy = np.sqrt(np.mean(sig ** 2))
    if energy < 50:               # silent frame — no pitch to estimate
        return default
    # Normalised autocorrelation over lag range 32-300 (53-500 Hz)
    corr = np.correlate(sig, sig, mode='full')[len(sig)-1:]
    if corr[0] < 1e-6:
        return default
    corr_norm = corr / corr[0]
    # Find first peak after lag 32 that exceeds 0.25 normalised correlation
    for i in range(32, min(300, len(corr_norm) - 1)):
        if (corr_norm[i] > corr_norm[i-1] and
                corr_norm[i] > corr_norm[i+1] and
                corr_norm[i] > 0.25):
            f0 = 16000.0 / i
            lag_min = max(32, int(16000.0 / (f0 * 1.15)))
            return lag_min
    return default


def extract_coherence(signal_q15, lag_min=None):
    """
    Autocorrelation via arm_dot_prod_q15 at lags lag_min-250 step-2.
    result = sum(signal[i] * signal[i+lag]) >> 15  → stored as int16.

    lag_min is dynamic when called from extract_coherence_dynamic().
    Default fixed lag_min=62 preserves exact C firmware behaviour.
    Returns int16 autocorr_peak (matches C exactly, including int16 cast).
    """
    if lag_min is None:
        lag_min = 62              # firmware default: covers 64-258 Hz
    sig  = signal_q15.astype(np.int64)
    n    = len(sig)
    max_corr = np.int16(0)
    for lag in range(lag_min, min(250, n // 2), 2):
        result_q63 = int(np.dot(sig[:n - lag], sig[lag:]))
        sum_val    = result_q63 >> 15
        sum_i16    = np.int16(np.clip(sum_val, -32768, 32767))
        if sum_i16 > max_corr:
            max_corr = sum_i16
    return int(max_corr)


def compute_zcr(signal_q15):
    """Count sign changes between adjacent samples (matches C compute_zcr)."""
    s = signal_q15
    crossings = int(np.sum(
        ((s[:-1] >= 0) & (s[1:] < 0)) | ((s[:-1] < 0) & (s[1:] >= 0))
    ))
    return crossings


# ── Gate scoring (mirrors gate_decide in decision.c) ─────────────────────────

def gate_decide(outputs_q15, noise_floor):
    """
    Returns (should_wake, score, features_dict).
    noise_floor is a Q15 int16 value (EMA estimate).
    """
    energy    = extract_energy(outputs_q15)
    max_corr  = extract_correlation(outputs_q15)
    coherence = extract_coherence(outputs_q15[0])
    zcr       = compute_zcr(outputs_q15[0])

    score = 0
    if max_corr  > CORRELATION_THRESHOLD:    score += 40  # Rule 1
    if energy    > noise_floor * 2:          score += 20  # Rule 2
    if coherence > COHERENCE_THRESHOLD:      score += 25  # Rule 3
    if zcr       < ZCR_MAX:                  score += 15  # Rule 4

    return (score >= WAKE_SCORE), score, {
        'correlation': max_corr,
        'energy':      energy,
        'coherence':   coherence,
        'zcr':         zcr,
    }


def update_noise_floor(noise_floor, energy, speech_detected):
    """EMA noise floor update (matches update_noise_floor in decision.c)."""
    if not speech_detected:
        noise_floor = (EMA_ALPHA_Q15 * noise_floor + EMA_BETA_Q15 * energy) >> 15
        noise_floor = max(-32768, min(32767, noise_floor))
    return noise_floor


# ── WAV loader ────────────────────────────────────────────────────────────────

def load_wav_mono16(path):
    """Load a WAV file and return int16 numpy array at native sample rate."""
    with wave.open(path, 'r') as wf:
        assert wf.getsampwidth() == 2, "Need 16-bit WAV"
        assert wf.getnchannels() == 1, "Need mono WAV"
        raw = wf.readframes(wf.getnframes())
    n = len(raw) // 2
    return np.array(struct.unpack(f'<{n}h', raw), dtype=np.int16)


# ── Per-file evaluation ───────────────────────────────────────────────────────

def evaluate_file(wav_path, true_label):
    """
    Run gate on every FRAME_SIZE-sample frame in the WAV file.
    Returns dict with per-file stats.
    """
    try:
        samples = load_wav_mono16(wav_path)
    except Exception as e:
        print(f"  WARN: cannot load {wav_path}: {e}")
        return None

    bank        = ResonatorBank()
    noise_floor = NOISE_FLOOR_INIT
    total = wake = 0
    scores_list = []
    features_log = []

    n_frames = len(samples) // FRAME_SIZE
    for i in range(n_frames):
        frame   = samples[i * FRAME_SIZE:(i + 1) * FRAME_SIZE]
        if len(frame) < FRAME_SIZE:
            break
        outputs = bank.process(frame)
        wake_flag, score, feats = gate_decide(outputs, noise_floor)

        # Extract energy for noise floor update
        energy = feats['energy']
        noise_floor = update_noise_floor(noise_floor, energy, wake_flag)

        total += 1
        if wake_flag:
            wake += 1
        scores_list.append(score)
        features_log.append(feats)

    if total == 0:
        return None

    wake_rate = wake / total

    # For a 3-second clip:
    #   speech label → predicted positive if wake_rate ≥ 30% of frames
    #   noise  label → predicted positive if wake_rate ≥ 30% of frames
    # Gate verdict per clip: use majority-frame decision
    predicted_speech = (wake_rate >= 0.30)

    return {
        'file':           os.path.basename(wav_path),
        'true_label':     true_label,
        'total_frames':   total,
        'wake_frames':    wake,
        'wake_rate':      wake_rate,
        'predicted':      'speech' if predicted_speech else 'noise',
        'correct':        (predicted_speech == (true_label == 'speech')),
        'avg_score':      float(np.mean(scores_list)),
        'avg_corr':       float(np.mean([f['correlation'] for f in features_log])),
        'avg_energy':     float(np.mean([f['energy']      for f in features_log])),
        'avg_coherence':  float(np.mean([f['coherence']   for f in features_log])),
        'avg_zcr':        float(np.mean([f['zcr']         for f in features_log])),
    }


# ── Results summary ───────────────────────────────────────────────────────────

def print_results(results, out_csv):
    speech_results = [r for r in results if r['true_label'] == 'speech']
    noise_results  = [r for r in results if r['true_label'] == 'noise']

    # Confusion matrix
    TP = sum(1 for r in speech_results if r['predicted'] == 'speech')
    FN = sum(1 for r in speech_results if r['predicted'] == 'noise')
    FP = sum(1 for r in noise_results  if r['predicted'] == 'speech')
    TN = sum(1 for r in noise_results  if r['predicted'] == 'noise')

    precision = TP / (TP + FP) if (TP + FP) > 0 else 0.0
    recall    = TP / (TP + FN) if (TP + FN) > 0 else 0.0
    f1        = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
    accuracy  = (TP + TN) / len(results) if results else 0.0

    print(f"\n{'='*65}")
    print("GUARDIAN GATE — REAL AUDIO VALIDATION RESULTS")
    print(f"{'='*65}")
    print(f"  Files evaluated : {len(results)}")
    print(f"  Speech clips    : {len(speech_results)}  |  Noise clips: {len(noise_results)}")
    print(f"\n  Confusion Matrix:")
    print(f"                    Predicted Speech   Predicted Noise")
    print(f"    True Speech     {TP:>8}            {FN:>8}")
    print(f"    True Noise      {FP:>8}            {TN:>8}")
    print(f"\n{'─'*65}")
    print(f"  Precision   : {precision:.3f}  (of all gate wakes, {precision*100:.1f}% are real speech)")
    print(f"  Recall      : {recall:.3f}  (gate caught {recall*100:.1f}% of speech clips)")
    print(f"  F1 Score    : {f1:.3f}  (harmonic mean of precision & recall)")
    print(f"  Accuracy    : {accuracy:.3f}")
    print(f"{'─'*65}")

    if speech_results:
        avg_wake_speech = np.mean([r['wake_rate'] for r in speech_results])
        print(f"\n  Feature averages (speech clips):")
        print(f"    Wake rate  : {avg_wake_speech*100:.1f}%")
        print(f"    Correlation: {np.mean([r['avg_corr']      for r in speech_results]):.0f}  (Q15, threshold {CORRELATION_THRESHOLD})")
        print(f"    Energy     : {np.mean([r['avg_energy']    for r in speech_results]):.0f}  (Q15)")
        print(f"    Coherence  : {np.mean([r['avg_coherence'] for r in speech_results]):.0f}  (threshold {COHERENCE_THRESHOLD})")
        print(f"    ZCR        : {np.mean([r['avg_zcr']       for r in speech_results]):.0f}  (max {ZCR_MAX})")

    if noise_results:
        avg_wake_noise  = np.mean([r['wake_rate']  for r in noise_results])
        print(f"\n  Feature averages (noise clips):")
        print(f"    Wake rate  : {avg_wake_noise*100:.1f}%")
        print(f"    Correlation: {np.mean([r['avg_corr']      for r in noise_results]):.0f}")
        print(f"    Energy     : {np.mean([r['avg_energy']    for r in noise_results]):.0f}")
        print(f"    Coherence  : {np.mean([r['avg_coherence'] for r in noise_results]):.0f}")
        print(f"    ZCR        : {np.mean([r['avg_zcr']       for r in noise_results]):.0f}")

    print(f"\n{'='*65}")
    verdict = ("PASS" if f1 >= 0.60
               else "PARTIAL" if f1 >= 0.40
               else "FAIL")
    print(f"  Verdict : {verdict}  (F1={f1:.3f}, target ≥ 0.60)")
    print(f"{'='*65}")
    print(f"\n  Results CSV: {out_csv}")

    return {'TP': TP, 'FN': FN, 'FP': FP, 'TN': TN,
            'precision': precision, 'recall': recall, 'f1': f1,
            'accuracy': accuracy}


def save_csv(results, path):
    if not results:
        return
    fieldnames = list(results[0].keys())
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()})


def plot_results(results, metrics, out_png):
    if not HAS_PLOT:
        return

    speech_r = [r for r in results if r['true_label'] == 'speech']
    noise_r  = [r for r in results if r['true_label'] == 'noise']

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle("Guardian Gate — Real Audio Validation (Week 10)", fontsize=13, fontweight='bold')

    # ── Plot 1: Wake rate distribution ──────────────────────────────────────
    ax = axes[0][0]
    s_rates = [r['wake_rate'] * 100 for r in speech_r]
    n_rates = [r['wake_rate'] * 100 for r in noise_r]
    bins    = np.linspace(0, 100, 21)
    ax.hist(s_rates, bins=bins, alpha=0.7, color='#2ca02c', label=f'Speech (n={len(speech_r)})', edgecolor='darkgreen')
    ax.hist(n_rates, bins=bins, alpha=0.7, color='#d62728', label=f'Noise  (n={len(noise_r)})',  edgecolor='darkred')
    ax.axvline(30, color='blue', linestyle='--', linewidth=1.5, label='Decision boundary 30%')
    ax.set_xlabel('Wake rate per clip (%)')
    ax.set_ylabel('Clip count')
    ax.set_title('Gate Wake Rate Distribution')
    ax.legend()
    ax.grid(alpha=0.3)

    # ── Plot 2: Feature scatter — correlation vs ZCR ─────────────────────────
    ax = axes[0][1]
    for r in speech_r:
        ax.scatter(r['avg_corr'], r['avg_zcr'], c='#2ca02c', alpha=0.7, s=60, marker='o')
    for r in noise_r:
        ax.scatter(r['avg_corr'], r['avg_zcr'], c='#d62728', alpha=0.7, s=60, marker='x')
    ax.axvline(CORRELATION_THRESHOLD, color='blue', linestyle='--', linewidth=1,
               label=f'Corr threshold ({CORRELATION_THRESHOLD})')
    ax.axhline(ZCR_MAX, color='orange', linestyle='--', linewidth=1,
               label=f'ZCR max ({ZCR_MAX})')
    ax.set_xlabel('Avg Correlation (Q15)')
    ax.set_ylabel('Avg ZCR per frame')
    ax.set_title('Feature Space: Correlation vs ZCR')
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0],[0], marker='o', color='w', markerfacecolor='#2ca02c', label='Speech', markersize=8),
        Line2D([0],[0], marker='x', color='#d62728', label='Noise', markersize=8),
        Line2D([0],[0], color='blue',   linestyle='--', label=f'Corr >{CORRELATION_THRESHOLD}'),
        Line2D([0],[0], color='orange', linestyle='--', label=f'ZCR <{ZCR_MAX}'),
    ]
    ax.legend(handles=legend_elements)
    ax.grid(alpha=0.3)

    # ── Plot 3: Confusion matrix heatmap ────────────────────────────────────
    ax = axes[1][0]
    cm = np.array([[metrics['TP'], metrics['FN']],
                   [metrics['FP'], metrics['TN']]])
    im = ax.imshow(cm, interpolation='nearest', cmap='Blues')
    ax.set_xticks([0, 1]); ax.set_yticks([0, 1])
    ax.set_xticklabels(['Pred: Speech', 'Pred: Noise'])
    ax.set_yticklabels(['True: Speech', 'True: Noise'])
    ax.set_title('Confusion Matrix')
    for i in range(2):
        for j in range(2):
            ax.text(j, i, str(cm[i, j]), ha='center', va='center',
                    fontsize=14, fontweight='bold',
                    color='white' if cm[i, j] > cm.max() / 2 else 'black')

    # ── Plot 4: Precision / Recall / F1 bar chart ───────────────────────────
    ax = axes[1][1]
    metric_names = ['Precision', 'Recall', 'F1 Score', 'Accuracy']
    metric_vals  = [metrics['precision'], metrics['recall'],
                    metrics['f1'], metrics['accuracy']]
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#9467bd']
    bars = ax.bar(metric_names, metric_vals, color=colors, alpha=0.82, edgecolor='black')
    ax.axhline(0.60, color='green', linestyle='--', linewidth=1.2,
               label='Target F1 ≥ 0.60')
    ax.set_ylim(0, 1.1)
    ax.set_ylabel('Score')
    ax.set_title(f"Gate Performance Metrics  (F1={metrics['f1']:.3f})")
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    for bar, val in zip(bars, metric_vals):
        ax.text(bar.get_x() + bar.get_width() / 2, val + 0.02,
                f'{val:.2f}', ha='center', fontweight='bold')

    plt.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
    plt.savefig(out_png, dpi=300)
    print(f"\nPlot saved: {out_png}")
    plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               '../results/audio_test_set/labels.csv')
        if os.path.exists(default):
            labels_csv = default
            print(f"Using default labels: {labels_csv}")
        else:
            print("Usage: python3 validate_gate_on_real_audio.py <labels.csv>")
            print("       python3 record_test_set.py --synthetic   # to generate test set")
            sys.exit(1)
    else:
        labels_csv = sys.argv[1]

    if not os.path.exists(labels_csv):
        print(f"ERROR: labels CSV not found: {labels_csv}")
        sys.exit(1)

    audio_dir = os.path.dirname(os.path.abspath(labels_csv))
    results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               '../results')
    ts  = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_csv = os.path.join(results_dir, f'week10_validation_{ts}.csv')
    out_png = os.path.join(results_dir, f'week10_validation_{ts}.png')

    # Load labels
    rows = []
    with open(labels_csv) as f:
        for row in csv.DictReader(f):
            rows.append(row)

    print(f"\n=== Guardian Week 10 — Gate Validation ===")
    print(f"Labels : {labels_csv}  ({len(rows)} clips)")
    print(f"Running gate simulation on each clip...\n")

    results = []
    for row in rows:
        wav_path    = os.path.join(audio_dir, row['file'])
        true_label  = row['label'].strip().lower()
        r = evaluate_file(wav_path, true_label)
        if r is None:
            continue
        indicator = 'OK' if r['correct'] else 'MISS'
        print(f"  [{indicator}] {r['file']:30s} label={true_label:6s} "
              f"wake={r['wake_rate']*100:5.1f}%  score={r['avg_score']:.0f}  "
              f"corr={r['avg_corr']:.0f}")
        results.append(r)

    if not results:
        print("ERROR: no results — check WAV paths in labels.csv")
        sys.exit(1)

    os.makedirs(results_dir, exist_ok=True)
    save_csv(results, out_csv)
    metrics = print_results(results, out_csv)
    plot_results(results, metrics, out_png)


if __name__ == '__main__':
    main()
