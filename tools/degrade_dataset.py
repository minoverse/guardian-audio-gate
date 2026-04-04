#!/usr/bin/env python3
"""
Guardian — Degraded Dataset Generator + DET Curve on Noisy Audio

Mixes speech WAV files with noise at controlled SNR levels to stress-test
the physics gate beyond the clean synthetic dataset (F1=1.000 is too easy).

SNR levels tested:
  +5 dB  — speech slightly louder than noise  (moderate noise)
   0 dB  — equal power                        (heavy noise)
  -5 dB  — noise louder than speech           (very hard — interviewer's challenge)

For each SNR level, creates a degraded test set and runs DET analysis.
Pass/fail criteria (from Gemini critique):
  - 90% abort rate maintained on noise at threshold=55
  - 90% recall maintained on speech at threshold=55

Usage:
    cd ~/projects/guardian-audio-gate
    python3 tools/degrade_dataset.py
"""
import os
import sys
import csv
import wave
import struct
import numpy as np
import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_gate_on_real_audio import load_wav_mono16

try:
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False

# ── Paths ─────────────────────────────────────────────────────────────────────
TOOLS_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(TOOLS_DIR)
CLEAN_DIR   = os.path.join(PROJECT_DIR, 'results', 'audio_test_set')
OUT_BASE    = os.path.join(PROJECT_DIR, 'results', 'degraded_test_set')
SAMPLE_RATE = 16000

SNR_LEVELS  = [5, 0, -5]   # dB — +5 moderate, 0 equal power, -5 noise dominant


# ── Audio helpers ──────────────────────────────────────────────────────────────

def rms(samples):
    """Root-mean-square of int16 or float array."""
    s = samples.astype(np.float64)
    return np.sqrt(np.mean(s ** 2)) if len(s) > 0 else 1.0


def mix_at_snr(speech, noise, snr_db):
    """
    Mix speech + noise at the given SNR.
    SNR = 10*log10(P_speech / P_noise)
    At -5dB: noise power is 3.16× speech power — noise dominates.
    Returns int16 array clipped to [-32768, 32767].
    """
    speech_f = speech.astype(np.float64)
    noise_f  = noise.astype(np.float64)

    speech_rms = rms(speech_f)
    noise_rms  = rms(noise_f)

    if noise_rms < 1e-6:
        return speech.copy()

    # Desired noise RMS = speech_rms / 10^(SNR/20)
    desired_noise_rms = speech_rms / (10 ** (snr_db / 20.0))
    scale = desired_noise_rms / noise_rms

    # Loop noise to match speech length
    if len(noise_f) < len(speech_f):
        repeats = int(np.ceil(len(speech_f) / len(noise_f)))
        noise_f = np.tile(noise_f, repeats)
    noise_f = noise_f[:len(speech_f)]

    mixed = speech_f + scale * noise_f
    mixed = np.clip(mixed, -32768, 32767).astype(np.int16)
    return mixed


def save_wav(path, samples, sample_rate=SAMPLE_RATE):
    """Save int16 numpy array as mono 16-bit WAV."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with wave.open(path, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(samples.tobytes())


# ── Degraded set generation ────────────────────────────────────────────────────

def build_degraded_set(snr_db):
    """
    Create degraded speech clips by mixing with noise at snr_db.
    Also copies noise/silence clips unchanged (they're already non-speech).
    Returns path to labels.csv.
    """
    tag     = f'snr{"p" if snr_db >= 0 else "m"}{abs(snr_db)}dB'
    out_dir = os.path.join(OUT_BASE, tag)
    os.makedirs(out_dir, exist_ok=True)

    # Load source clips
    clean_csv = os.path.join(CLEAN_DIR, 'labels.csv')
    rows = []
    with open(clean_csv) as f:
        rows = list(csv.DictReader(f))

    # Load all noise clips for mixing pool
    noise_pool = []
    for fn in sorted(os.listdir(CLEAN_DIR)):
        if fn.startswith('noise_') and fn.endswith('.wav'):
            s = load_wav_mono16(os.path.join(CLEAN_DIR, fn))
            noise_pool.append(s)
    if not noise_pool:
        print("ERROR: no noise WAV files found in", CLEAN_DIR)
        sys.exit(1)
    # Concatenate all noise into one long track for mixing
    noise_track = np.concatenate(noise_pool)

    out_rows = []
    noise_offset = 0

    for row in rows:
        src_path = os.path.join(CLEAN_DIR, row['file'])
        label    = row['label'].strip().lower()
        out_fn   = row['file']
        out_path = os.path.join(out_dir, out_fn)

        if label == 'speech':
            # Mix with noise at target SNR
            speech = load_wav_mono16(src_path)
            # Get noise segment starting at current offset
            seg_len = len(speech)
            if noise_offset + seg_len > len(noise_track):
                noise_offset = 0
            noise_seg = noise_track[noise_offset:noise_offset + seg_len]
            noise_offset += seg_len

            mixed = mix_at_snr(speech, noise_seg, snr_db)
            save_wav(out_path, mixed)
        else:
            # Copy noise/silence unchanged
            src_samples = load_wav_mono16(src_path)
            save_wav(out_path, src_samples)

        out_rows.append({'file': out_fn, 'label': label})

    csv_path = os.path.join(out_dir, 'labels.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['file', 'label'])
        w.writeheader()
        w.writerows(out_rows)

    return csv_path, out_dir, tag


# ── Gate simulation (inline — reuses validate_gate logic) ─────────────────────

from validate_gate_on_real_audio import (
    ResonatorBank, extract_energy, extract_correlation,
    extract_coherence, compute_zcr, update_noise_floor,
    FRAME_SIZE, NOISE_FLOOR_INIT,
    CORRELATION_THRESHOLD, COHERENCE_THRESHOLD, ZCR_MAX,
)

RULE_WEIGHTS = {'correlation': 40, 'energy': 20, 'coherence': 25, 'zcr': 15}
MAX_SCORE    = 100
WAKE_THRESHOLD = 55


def run_gate_on_clip(wav_path, threshold=WAKE_THRESHOLD):
    """Returns (wake_rate, mean_score) for a clip."""
    try:
        samples = load_wav_mono16(wav_path)
    except Exception:
        return None, None

    bank        = ResonatorBank()
    noise_floor = NOISE_FLOOR_INIT
    scores      = []

    for i in range(len(samples) // FRAME_SIZE):
        frame    = samples[i * FRAME_SIZE:(i + 1) * FRAME_SIZE]
        outputs  = bank.process(frame)
        energy   = extract_energy(outputs)
        max_corr = extract_correlation(outputs)
        coherence= extract_coherence(outputs[0])
        zcr      = compute_zcr(outputs[0])

        score = 0
        if max_corr  > CORRELATION_THRESHOLD: score += RULE_WEIGHTS['correlation']
        if energy    > noise_floor * 2:       score += RULE_WEIGHTS['energy']
        if coherence > COHERENCE_THRESHOLD:   score += RULE_WEIGHTS['coherence']
        if zcr       < ZCR_MAX:              score += RULE_WEIGHTS['zcr']
        scores.append(score)
        noise_floor = update_noise_floor(noise_floor, energy, score >= threshold)

    if not scores:
        return None, None
    scores = np.array(scores)
    wake_rate = float(np.mean(scores >= threshold))
    return wake_rate, float(np.mean(scores))


def evaluate_set(csv_path, audio_dir, threshold=WAKE_THRESHOLD):
    """Run gate on all clips. Returns dict with TP/FP/TN/FN and per-clip results."""
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))

    TP = FP = TN = FN = 0
    details = []

    for row in rows:
        path  = os.path.join(audio_dir, row['file'])
        label = row['label'].strip().lower()
        wake_rate, mean_score = run_gate_on_clip(path, threshold)
        if wake_rate is None:
            continue

        predicted_speech = (wake_rate >= 0.30)
        true_speech      = (label == 'speech')

        if true_speech  and predicted_speech: TP += 1
        elif true_speech and not predicted_speech: FN += 1
        elif not true_speech and predicted_speech: FP += 1
        else: TN += 1

        details.append({
            'file': row['file'], 'label': label,
            'wake_rate': wake_rate, 'mean_score': mean_score,
            'predicted': 'speech' if predicted_speech else 'non-speech',
        })

    total       = TP + FP + TN + FN
    noise_total = FP + TN
    speech_total= TP + FN
    abort_rate  = TN / noise_total if noise_total > 0 else 0
    recall      = TP / speech_total if speech_total > 0 else 0
    precision   = TP / (TP + FP) if (TP + FP) > 0 else 0
    f1          = 2*precision*recall/(precision+recall) if (precision+recall) > 0 else 0
    far         = FP / noise_total if noise_total > 0 else 0
    frr         = FN / speech_total if speech_total > 0 else 0

    return {
        'TP': TP, 'FP': FP, 'TN': TN, 'FN': FN,
        'abort_rate': abort_rate, 'recall': recall,
        'precision': precision, 'f1': f1,
        'FAR': far, 'FRR': frr,
        'details': details,
    }


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("\n" + "="*65)
    print("GUARDIAN — DEGRADED DATASET DET ANALYSIS")
    print("="*65)
    print(f"Speech clips mixed with LFSR noise at: {SNR_LEVELS} dB SNR")
    print(f"Challenge: maintain ≥90% abort rate AND ≥90% recall at SNR=-5dB")
    print("="*65)

    # Also evaluate clean set as baseline
    clean_csv = os.path.join(CLEAN_DIR, 'labels.csv')
    print("\n[Baseline — Clean synthetic set]")
    baseline = evaluate_set(clean_csv, CLEAN_DIR)
    print(f"  F1={baseline['f1']:.3f}  Recall={baseline['recall']*100:.1f}%  "
          f"Abort={baseline['abort_rate']*100:.1f}%  "
          f"FAR={baseline['FAR']*100:.1f}%  FRR={baseline['FRR']*100:.1f}%")

    results = {'clean': baseline}
    snr_labels = ['clean'] + [f'{s:+d}dB' for s in SNR_LEVELS]
    f1_vals     = [baseline['f1']]
    recall_vals = [baseline['recall']]
    abort_vals  = [baseline['abort_rate']]

    for snr in SNR_LEVELS:
        tag_str = f'SNR={snr:+d}dB'
        print(f"\n[Generating degraded set at {tag_str}]")
        csv_path, out_dir, tag = build_degraded_set(snr)
        print(f"  Saved to: {out_dir}")

        print(f"  Running gate validation...")
        res = evaluate_set(csv_path, out_dir)
        results[tag_str] = res

        abort_ok  = '✓' if res['abort_rate']  >= 0.90 else '✗'
        recall_ok = '✓' if res['recall']       >= 0.90 else '✗'

        print(f"  F1={res['f1']:.3f}  "
              f"Recall={res['recall']*100:.1f}% {recall_ok}  "
              f"Abort={res['abort_rate']*100:.1f}% {abort_ok}  "
              f"FAR={res['FAR']*100:.1f}%  FRR={res['FRR']*100:.1f}%")

        f1_vals.append(res['f1'])
        recall_vals.append(res['recall'])
        abort_vals.append(res['abort_rate'])

    # ── Summary table ──────────────────────────────────────────────────────────
    print(f"\n{'='*65}")
    print("SUMMARY TABLE")
    print(f"{'='*65}")
    print(f"{'Condition':<14} {'F1':>6} {'Recall':>8} {'Abort':>8} {'FAR':>7} {'FRR':>7}  {'Pass?':>6}")
    print(f"{'─'*65}")
    for label, res in zip(snr_labels, [results['clean']] +
                          [results[f'SNR={s:+d}dB'] for s in SNR_LEVELS]):
        abort_ok  = '✓' if res['abort_rate'] >= 0.90 else '✗'
        recall_ok = '✓' if res['recall']     >= 0.90 else '✗'
        both_ok   = 'PASS' if (res['abort_rate'] >= 0.90 and res['recall'] >= 0.90) else 'FAIL'
        print(f"{label:<14} {res['f1']:>6.3f} {res['recall']*100:>7.1f}% "
              f"{res['abort_rate']*100:>7.1f}% {recall_ok} "
              f"{res['FAR']*100:>6.1f}% {res['FRR']*100:>6.1f}% {abort_ok} "
              f"  {both_ok}")
    print(f"{'─'*65}")
    print("Criteria: Abort ≥ 90% AND Recall ≥ 90%")
    print(f"{'='*65}")

    # ── Save results CSV ───────────────────────────────────────────────────────
    ts       = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir  = os.path.join(PROJECT_DIR, 'results')
    csv_out  = os.path.join(out_dir, f'degraded_det_{ts}.csv')
    with open(csv_out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['condition', 'F1', 'recall', 'abort_rate', 'FAR', 'FRR'])
        for label, res in zip(snr_labels, [results['clean']] +
                              [results[f'SNR={s:+d}dB'] for s in SNR_LEVELS]):
            w.writerow([label, f"{res['f1']:.4f}", f"{res['recall']:.4f}",
                        f"{res['abort_rate']:.4f}", f"{res['FAR']:.4f}",
                        f"{res['FRR']:.4f}"])
    print(f"\nCSV saved: {csv_out}")

    # ── Plot ───────────────────────────────────────────────────────────────────
    if not HAS_PLOT:
        return

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Guardian Gate — DET Curve on Degraded Audio\n'
                 '(Noise mixed at +5/0/−5 dB SNR)',
                 fontsize=13, fontweight='bold')

    x     = np.arange(len(snr_labels))
    colors= ['#2ca02c', '#1f77b4', '#ff7f0e', '#d62728']

    ax = axes[0]
    bars = ax.bar(x, [v*100 for v in f1_vals], color=colors)
    ax.axhline(90, color='red', linestyle='--', linewidth=1, label='90% target')
    ax.set_xticks(x); ax.set_xticklabels(snr_labels)
    ax.set_ylabel('F1 Score (%)'); ax.set_title('F1 Score vs SNR')
    ax.set_ylim(0, 105); ax.legend(); ax.grid(axis='y', alpha=0.3)
    for bar, val in zip(bars, f1_vals):
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1,
                f'{val*100:.1f}%', ha='center', va='bottom', fontsize=9)

    ax = axes[1]
    bars = ax.bar(x, [v*100 for v in recall_vals], color=colors)
    ax.axhline(90, color='red', linestyle='--', linewidth=1, label='90% target')
    ax.set_xticks(x); ax.set_xticklabels(snr_labels)
    ax.set_ylabel('Recall (%)'); ax.set_title('Speech Recall vs SNR')
    ax.set_ylim(0, 105); ax.legend(); ax.grid(axis='y', alpha=0.3)
    for bar, val in zip(bars, recall_vals):
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1,
                f'{val*100:.1f}%', ha='center', va='bottom', fontsize=9)

    ax = axes[2]
    bars = ax.bar(x, [v*100 for v in abort_vals], color=colors)
    ax.axhline(90, color='red', linestyle='--', linewidth=1, label='90% target')
    ax.set_xticks(x); ax.set_xticklabels(snr_labels)
    ax.set_ylabel('Noise Abort Rate (%)'); ax.set_title('Noise Rejection vs SNR')
    ax.set_ylim(0, 105); ax.legend(); ax.grid(axis='y', alpha=0.3)
    for bar, val in zip(bars, abort_vals):
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1,
                f'{val*100:.1f}%', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    png_out = os.path.join(out_dir, f'degraded_det_{ts}.png')
    plt.savefig(png_out, dpi=300)
    print(f"Plot saved: {png_out}")
    plt.show()


if __name__ == '__main__':
    main()
