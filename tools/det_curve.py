#!/usr/bin/env python3
"""
Guardian — DET Curve (Detection Error Tradeoff)

Sweeps the gate wake threshold from 0 to 100 and plots:
  - FAR  (False Acceptance Rate)  = FP / (FP + TN)  — noise that wakes gate
  - FRR  (False Rejection Rate)   = FN / (FN + TP)  — speech that gets aborted
  - F1 score at each threshold
  - Operating points: power-saver, balanced, high-recall

Also shows per-rule contribution so product teams can tune individual features.

Usage:
    python3 det_curve.py results/audio_test_set/labels.csv

    # Or generate synthetic set first:
    python3 record_test_set.py --synthetic
    python3 det_curve.py results/audio_test_set/labels.csv
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
    print("matplotlib not installed — skipping plots")

# ── Import gate simulation from validate_gate_on_real_audio.py ───────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_gate_on_real_audio import (
    ResonatorBank, extract_energy, extract_correlation,
    extract_coherence, compute_zcr, load_wav_mono16,
    update_noise_floor, FRAME_SIZE, NOISE_FLOOR_INIT,
    CORRELATION_THRESHOLD, COHERENCE_THRESHOLD, ZCR_MAX,
)

RULE_WEIGHTS = {
    'correlation': 40,   # Rule 1
    'energy':      20,   # Rule 2
    'coherence':   25,   # Rule 3
    'zcr':         15,   # Rule 4
}
MAX_SCORE = sum(RULE_WEIGHTS.values())   # 100


# ── Per-clip score extraction ─────────────────────────────────────────────────

def extract_scores(wav_path, true_label):
    """
    Run gate on every frame and return (list_of_raw_scores, true_label).
    Raw score = sum of all rules that fired (0–100).
    """
    try:
        samples = load_wav_mono16(wav_path)
    except Exception as e:
        print(f"  WARN: {wav_path}: {e}")
        return None

    bank        = ResonatorBank()
    noise_floor = NOISE_FLOOR_INIT
    scores      = []

    n_frames = len(samples) // FRAME_SIZE
    for i in range(n_frames):
        frame   = samples[i * FRAME_SIZE:(i + 1) * FRAME_SIZE]
        if len(frame) < FRAME_SIZE:
            break
        outputs  = bank.process(frame)
        energy   = extract_energy(outputs)
        max_corr = extract_correlation(outputs)
        coherence= extract_coherence(outputs[0])
        zcr      = compute_zcr(outputs[0])

        score = 0
        if max_corr  > CORRELATION_THRESHOLD:  score += RULE_WEIGHTS['correlation']
        if energy    > noise_floor * 2:        score += RULE_WEIGHTS['energy']
        if coherence > COHERENCE_THRESHOLD:    score += RULE_WEIGHTS['coherence']
        if zcr       < ZCR_MAX:               score += RULE_WEIGHTS['zcr']

        scores.append(score)
        noise_floor = update_noise_floor(noise_floor, energy,
                                         score >= 55)  # use default threshold

    if not scores:
        return None
    return np.array(scores), true_label


# ── DET curve computation ─────────────────────────────────────────────────────

def compute_det_curve(clip_scores):
    """
    For each threshold t in 0..MAX_SCORE:
      A clip is 'predicted speech' if its wake_rate >= 0.30 at threshold t.
      Compute FAR and FRR.

    Returns arrays: thresholds, FAR, FRR, F1, precision, recall.
    """
    thresholds = np.arange(0, MAX_SCORE + 2, 1)
    FAR = np.zeros(len(thresholds))
    FRR = np.zeros(len(thresholds))
    F1  = np.zeros(len(thresholds))
    PRE = np.zeros(len(thresholds))
    REC = np.zeros(len(thresholds))

    speech_clips = [(s, l) for (s, l) in clip_scores if l == 'speech']
    noise_clips  = [(s, l) for (s, l) in clip_scores if l == 'noise']

    for idx, t in enumerate(thresholds):
        TP = FN = FP = TN = 0
        for scores, _ in speech_clips:
            wake_rate = np.mean(scores >= t)
            if wake_rate >= 0.30: TP += 1
            else:                 FN += 1
        for scores, _ in noise_clips:
            wake_rate = np.mean(scores >= t)
            if wake_rate >= 0.30: FP += 1
            else:                 TN += 1

        FAR[idx] = FP / (FP + TN) if (FP + TN) > 0 else 0.0
        FRR[idx] = FN / (FN + TP) if (FN + TP) > 0 else 0.0
        prec      = TP / (TP + FP) if (TP + FP) > 0 else 0.0
        rec       = TP / (TP + FN) if (TP + FN) > 0 else 0.0
        F1[idx]   = 2*prec*rec/(prec+rec) if (prec+rec) > 0 else 0.0
        PRE[idx]  = prec
        REC[idx]  = rec

    return thresholds, FAR, FRR, F1, PRE, REC


# ── Operating point analysis ──────────────────────────────────────────────────

OPERATING_POINTS = {
    'Power-saver (>90% abort)': 80,   # only very strong speech wakes
    'Balanced (current design)': 55,  # GATE_CONFIG_DEFAULT
    'High-recall (>95% speech)': 40,  # accept more false alarms
}


def find_op_metrics(thresholds, FAR, FRR, F1, t):
    """Return metrics at threshold t."""
    idx = np.argmin(np.abs(thresholds - t))
    return {
        'threshold': int(thresholds[idx]),
        'FAR':       float(FAR[idx]),
        'FRR':       float(FRR[idx]),
        'F1':        float(F1[idx]),
    }


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               '../results/audio_test_set/labels.csv')
        if os.path.exists(default):
            labels_csv = default
        else:
            print("Usage: python3 det_curve.py <labels.csv>")
            sys.exit(1)
    else:
        labels_csv = sys.argv[1]

    audio_dir = os.path.dirname(os.path.abspath(labels_csv))
    rows = []
    with open(labels_csv) as f:
        for row in csv.DictReader(f):
            rows.append(row)

    print(f"\n=== Guardian DET Curve Analysis ===")
    print(f"Labels: {labels_csv}  ({len(rows)} clips)")

    clip_scores = []
    for row in rows:
        wav_path   = os.path.join(audio_dir, row['file'])
        true_label = row['label'].strip().lower()
        result     = extract_scores(wav_path, true_label)
        if result:
            clip_scores.append(result)

    print(f"Processed: {len(clip_scores)} clips")

    thresholds, FAR, FRR, F1, PRE, REC = compute_det_curve(clip_scores)

    # Best F1 threshold
    best_idx = np.argmax(F1)
    best_t   = int(thresholds[best_idx])

    print(f"\n{'='*60}")
    print("DET CURVE — OPERATING POINTS")
    print(f"{'='*60}")
    print(f"{'Operating Point':<30} {'Threshold':>10}  {'FAR':>7}  {'FRR':>7}  {'F1':>7}")
    print(f"{'─'*60}")
    for name, t in OPERATING_POINTS.items():
        m = find_op_metrics(thresholds, FAR, FRR, F1, t)
        current = ' ← current' if t == 55 else ''
        print(f"{name:<30} {m['threshold']:>10}  "
              f"{m['FAR']*100:>6.1f}%  {m['FRR']*100:>6.1f}%  {m['F1']:>6.3f}{current}")

    m_best = find_op_metrics(thresholds, FAR, FRR, F1, best_t)
    print(f"{'─'*60}")
    print(f"{'Best F1 point':<30} {m_best['threshold']:>10}  "
          f"{m_best['FAR']*100:>6.1f}%  {m_best['FRR']*100:>6.1f}%  {m_best['F1']:>6.3f}")
    print(f"{'='*60}")

    print(f"\nInterpretation:")
    print(f"  Threshold=55 (current): FAR={find_op_metrics(thresholds, FAR, FRR, F1, 55)['FAR']*100:.1f}%  "
          f"FRR={find_op_metrics(thresholds, FAR, FRR, F1, 55)['FRR']*100:.1f}%")
    print(f"  → Battery saved by aborting FRR% of speech is acceptable for")
    print(f"    always-on wakeword (user repeats if missed once).")
    print(f"  → FAR% is false-alarm rate — drives unnecessary TinyML runs.")
    print(f"  Product decision: tune threshold up (more saving) or down (more recall).")

    # ── Save ──────────────────────────────────────────────────────────────────
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../results')
    os.makedirs(out_dir, exist_ok=True)
    ts      = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f'det_curve_{ts}.csv')
    png_path = os.path.join(out_dir, f'det_curve_{ts}.png')

    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['threshold', 'FAR', 'FRR', 'F1', 'precision', 'recall'])
        for t, far, frr, f1, pre, rec in zip(thresholds, FAR, FRR, F1, PRE, REC):
            w.writerow([int(t), f'{far:.4f}', f'{frr:.4f}', f'{f1:.4f}',
                        f'{pre:.4f}', f'{rec:.4f}'])
    print(f"\nCSV: {csv_path}")

    if not HAS_PLOT:
        return

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('Guardian Gate — DET Curve & Operating Point Analysis',
                 fontsize=13, fontweight='bold')

    op_colors = {'Power-saver (>90% abort)': '#d62728',
                 'Balanced (current design)': '#2ca02c',
                 'High-recall (>95% speech)': '#1f77b4'}

    # ── DET curve ─────────────────────────────────────────────────────────────
    ax = axes[0]
    ax.plot(FAR * 100, FRR * 100, 'b-', linewidth=2, label='DET curve')
    for name, t in OPERATING_POINTS.items():
        m = find_op_metrics(thresholds, FAR, FRR, F1, t)
        ax.scatter(m['FAR']*100, m['FRR']*100, s=100, zorder=5,
                   color=op_colors[name], label=f"t={t} ({name.split(' ')[0]})")
        ax.annotate(f"t={t}", (m['FAR']*100, m['FRR']*100),
                    textcoords='offset points', xytext=(5, 5), fontsize=8)
    ax.set_xlabel('FAR — False Acceptance Rate (%)\n(noise waking gate)')
    ax.set_ylabel('FRR — False Rejection Rate (%)\n(speech aborted by gate)')
    ax.set_title('DET Curve\n(lower-left = better)')
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    ax.set_xlim(-1, 101); ax.set_ylim(-1, 101)

    # ── F1 vs threshold ───────────────────────────────────────────────────────
    ax = axes[1]
    ax.plot(thresholds, F1, 'g-', linewidth=2, label='F1')
    ax.plot(thresholds, PRE, 'b--', linewidth=1.2, alpha=0.7, label='Precision')
    ax.plot(thresholds, REC, 'r--', linewidth=1.2, alpha=0.7, label='Recall')
    for name, t in OPERATING_POINTS.items():
        ax.axvline(t, color=op_colors[name], linestyle=':', linewidth=1.5,
                   label=f"t={t}")
    ax.set_xlabel('Wake threshold (score)')
    ax.set_ylabel('Score')
    ax.set_title('F1 / Precision / Recall vs Threshold')
    ax.legend(fontsize=7); ax.grid(alpha=0.3)
    ax.set_xlim(0, MAX_SCORE)

    # ── Power saving vs recall ─────────────────────────────────────────────────
    ax = axes[2]
    recall_pct = (1 - FRR) * 100
    # Power saving ≈ abort_rate when gate fires (FRR = missed speech → still aborted)
    # Abort rate ≈ 1 - wake_fraction; wake_fraction ≈ (1-FRR) × speech_fraction + FAR × noise_fraction
    speech_frac = sum(1 for _, l in clip_scores if l == 'speech') / len(clip_scores)
    noise_frac  = 1 - speech_frac
    wake_frac   = (1 - FRR) * speech_frac + FAR * noise_frac
    abort_pct   = (1 - wake_frac) * 100
    ax.plot(recall_pct, abort_pct, 'purple', linewidth=2)
    for name, t in OPERATING_POINTS.items():
        m = find_op_metrics(thresholds, FAR, FRR, F1, t)
        rec_pt = (1 - m['FRR']) * 100
        wake_pt = ((1-m['FRR'])*speech_frac + m['FAR']*noise_frac)
        ab_pt = (1 - wake_pt) * 100
        ax.scatter(rec_pt, ab_pt, s=100, color=op_colors[name],
                   zorder=5, label=f"t={t} F1={m['F1']:.2f}")
        ax.annotate(f"t={t}", (rec_pt, ab_pt),
                    textcoords='offset points', xytext=(4, -10), fontsize=8)
    ax.set_xlabel('Speech Recall (%)\n(fraction of speech frames that wake gate)')
    ax.set_ylabel('TinyML Abort Rate (%)\n(battery saving proxy)')
    ax.set_title('Recall vs Power Saving Trade-off\n(upper-right = ideal)')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(png_path, dpi=300)
    print(f"Plot: {png_path}")
    plt.show()


if __name__ == '__main__':
    main()
