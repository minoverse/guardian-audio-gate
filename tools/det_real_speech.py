#!/usr/bin/env python3
"""
Guardian — DET Curve on Real Speech + Real Background Noise

Uses Google Speech Commands (yes/no) as real speech,
mixed with real background noise files (_background_noise_/) at
+5 dB, 0 dB, -5 dB SNR.

This is the honest test: can the physics gate detect real human speech
when noise is louder than speech (-5 dB SNR)?

Pass criteria (Gemini critique):
  - Abort rate >= 90% on noise-only clips
  - Recall     >= 90% on speech clips

Usage:
    cd ~/projects/guardian-audio-gate
    python3 tools/det_real_speech.py
"""
import os, sys, csv, wave, random, datetime
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_gate_on_real_audio import (
    ResonatorBank, extract_energy, extract_correlation,
    extract_coherence, compute_zcr, update_noise_floor,
    FRAME_SIZE, NOISE_FLOOR_INIT,
    CORRELATION_THRESHOLD, COHERENCE_THRESHOLD, ZCR_MAX,
)

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False

# ── Paths ─────────────────────────────────────────────────────────────────────
PROJECT_DIR  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRAIN_DIR    = os.path.join(PROJECT_DIR, 'training_data')
NOISE_DIR    = os.path.join(TRAIN_DIR, '_background_noise_')
RESULTS_DIR  = os.path.join(PROJECT_DIR, 'results')
SAMPLE_RATE  = 16000
CLIP_LEN     = 16000   # 1 second (Google Speech Commands standard)
N_SPEECH     = 50      # clips per word (yes + no = 100 total speech)
N_NOISE      = 50      # noise-only clips from background files
SNR_LEVELS   = [5, 0, -5]
RANDOM_SEED  = 42


# ── Audio helpers ──────────────────────────────────────────────────────────────

def load_wav_mono16(path):
    with wave.open(path, 'r') as wf:
        raw = wf.readframes(wf.getnframes())
        samples = np.frombuffer(raw, dtype=np.int16).copy()
        # Resample if needed (Google Speech Commands is already 16kHz)
        if wf.getframerate() != SAMPLE_RATE:
            raise ValueError(f"{path}: expected {SAMPLE_RATE} Hz, got {wf.getframerate()}")
    return samples

def pad_or_trim(samples, length=CLIP_LEN):
    if len(samples) >= length:
        return samples[:length]
    return np.pad(samples, (0, length - len(samples)))

def rms(samples):
    s = samples.astype(np.float64)
    r = np.sqrt(np.mean(s ** 2))
    return r if r > 1e-6 else 1e-6

def mix_at_snr(speech, noise, snr_db):
    speech_f = speech.astype(np.float64)
    noise_f  = noise.astype(np.float64)
    desired_noise_rms = rms(speech_f) / (10 ** (snr_db / 20.0))
    scale = desired_noise_rms / rms(noise_f)
    mixed = speech_f + scale * noise_f
    return np.clip(mixed, -32768, 32767).astype(np.int16)


# ── Build test set ─────────────────────────────────────────────────────────────

def collect_speech_clips():
    """Pick N_SPEECH clips each from yes/ and no/."""
    rng = random.Random(RANDOM_SEED)
    clips = []
    for word in ('yes', 'no'):
        folder = os.path.join(TRAIN_DIR, word)
        wavs = sorted(f for f in os.listdir(folder) if f.endswith('.wav'))
        rng.shuffle(wavs)
        for fn in wavs[:N_SPEECH]:
            clips.append(os.path.join(folder, fn))
    return clips

def build_noise_track():
    """Concatenate all background noise WAVs into one long track."""
    parts = []
    for fn in sorted(os.listdir(NOISE_DIR)):
        if fn.endswith('.wav'):
            try:
                parts.append(load_wav_mono16(os.path.join(NOISE_DIR, fn)))
            except Exception:
                pass
    return np.concatenate(parts) if parts else np.zeros(SAMPLE_RATE * 60, dtype=np.int16)

def extract_noise_clips(noise_track, n):
    """Cut n non-overlapping 1-second clips from the noise track."""
    rng = random.Random(RANDOM_SEED + 1)
    max_start = len(noise_track) - CLIP_LEN
    starts = random.sample(range(0, max_start, CLIP_LEN), min(n, max_start // CLIP_LEN))
    clips = [noise_track[s:s+CLIP_LEN] for s in starts[:n]]
    return clips


# ── Gate simulation ────────────────────────────────────────────────────────────

RULE_WEIGHTS   = {'correlation': 40, 'energy': 20, 'coherence': 25, 'zcr': 15}
WAKE_THRESHOLD = 55

def run_gate(samples, threshold=WAKE_THRESHOLD):
    """Returns wake_rate (fraction of frames >= threshold)."""
    bank        = ResonatorBank()
    noise_floor = NOISE_FLOOR_INIT
    scores      = []
    for i in range(len(samples) // FRAME_SIZE):
        frame     = samples[i*FRAME_SIZE:(i+1)*FRAME_SIZE]
        outputs   = bank.process(frame)
        energy    = extract_energy(outputs)
        max_corr  = extract_correlation(outputs)
        coherence = extract_coherence(outputs[0])
        zcr       = compute_zcr(outputs[0])
        score = 0
        if max_corr  > CORRELATION_THRESHOLD: score += RULE_WEIGHTS['correlation']
        if energy    > noise_floor * 2:       score += RULE_WEIGHTS['energy']
        if coherence > COHERENCE_THRESHOLD:   score += RULE_WEIGHTS['coherence']
        if zcr       < ZCR_MAX:               score += RULE_WEIGHTS['zcr']
        scores.append(score)
        noise_floor = update_noise_floor(noise_floor, energy, score >= threshold)
    if not scores:
        return 0.0
    return float(np.mean(np.array(scores) >= threshold))


def evaluate(speech_clips, noise_clips, snr_db=None):
    """
    Run gate on all clips. If snr_db is given, mix speech with noise.
    Returns dict with recall, abort_rate, FAR, FRR, F1.
    """
    noise_track = build_noise_track()
    noise_offset = 0

    TP = FP = TN = FN = 0

    for path in speech_clips:
        speech = pad_or_trim(load_wav_mono16(path))
        if snr_db is not None:
            seg = noise_track[noise_offset:noise_offset+CLIP_LEN]
            if len(seg) < CLIP_LEN:
                noise_offset = 0
                seg = noise_track[0:CLIP_LEN]
            noise_offset = (noise_offset + CLIP_LEN) % (len(noise_track) - CLIP_LEN)
            audio = mix_at_snr(speech, seg, snr_db)
        else:
            audio = speech
        wake_rate = run_gate(audio)
        if wake_rate >= 0.30:
            TP += 1
        else:
            FN += 1

    for clip in noise_clips:
        audio = pad_or_trim(clip)
        wake_rate = run_gate(audio)
        if wake_rate >= 0.30:
            FP += 1
        else:
            TN += 1

    noise_total  = FP + TN
    speech_total = TP + FN
    recall       = TP / speech_total if speech_total > 0 else 0
    abort_rate   = TN / noise_total  if noise_total  > 0 else 0
    precision    = TP / (TP + FP)    if (TP + FP) > 0   else 0
    f1           = 2*precision*recall/(precision+recall) if (precision+recall) > 0 else 0
    far          = FP / noise_total  if noise_total > 0  else 0
    frr          = FN / speech_total if speech_total > 0 else 0
    return dict(recall=recall, abort_rate=abort_rate, precision=precision,
                f1=f1, FAR=far, FRR=frr, TP=TP, FP=FP, TN=TN, FN=FN)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("\n" + "="*65)
    print("GUARDIAN — DET CURVE ON REAL SPEECH + REAL BACKGROUND NOISE")
    print("="*65)
    print(f"Speech: Google Speech Commands  yes×{N_SPEECH} + no×{N_SPEECH} = {N_SPEECH*2} clips")
    print(f"Noise : _background_noise_/ (pink, white, dishes, fan, tap)")
    print(f"SNR levels tested: clean, +5 dB, 0 dB, -5 dB")
    print(f"Pass criteria: Recall ≥ 90% AND Abort ≥ 90%")
    print("="*65)

    speech_clips = collect_speech_clips()
    noise_track  = build_noise_track()
    noise_clips  = extract_noise_clips(noise_track, N_NOISE)

    print(f"\nLoaded {len(speech_clips)} speech clips, {len(noise_clips)} noise clips")
    print(f"Background noise track: {len(noise_track)/SAMPLE_RATE:.1f} seconds\n")

    conditions = [('Clean (no noise)', None)] + [(f'SNR={s:+d}dB', s) for s in SNR_LEVELS]
    results = []

    for label, snr in conditions:
        print(f"[{label}] ", end='', flush=True)
        res = evaluate(speech_clips, noise_clips, snr_db=snr)
        results.append((label, res))
        abort_ok  = '✓' if res['abort_rate'] >= 0.90 else '✗'
        recall_ok = '✓' if res['recall']     >= 0.90 else '✗'
        passed    = 'PASS' if (res['abort_rate'] >= 0.90 and res['recall'] >= 0.90) else 'FAIL'
        print(f"F1={res['f1']:.3f}  Recall={res['recall']*100:.1f}%{recall_ok}  "
              f"Abort={res['abort_rate']*100:.1f}%{abort_ok}  "
              f"FAR={res['FAR']*100:.1f}%  FRR={res['FRR']*100:.1f}%  → {passed}")

    # ── Summary table ──────────────────────────────────────────────────────────
    print(f"\n{'='*65}")
    print("SUMMARY")
    print(f"{'─'*65}")
    print(f"{'Condition':<22} {'F1':>6} {'Recall':>8} {'Abort':>8} {'FAR':>7} {'FRR':>7}  {'Pass?'}")
    print(f"{'─'*65}")
    for label, res in results:
        passed = 'PASS' if (res['abort_rate'] >= 0.90 and res['recall'] >= 0.90) else 'FAIL'
        print(f"{label:<22} {res['f1']:>6.3f} {res['recall']*100:>7.1f}% "
              f"{res['abort_rate']*100:>7.1f}% "
              f"{res['FAR']*100:>6.1f}% {res['FRR']*100:>6.1f}%   {passed}")
    print(f"{'─'*65}")
    print("Criteria: Recall ≥ 90% AND Abort ≥ 90%")
    print(f"{'='*65}")

    # ── CSV ────────────────────────────────────────────────────────────────────
    ts      = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_out = os.path.join(RESULTS_DIR, f'det_real_{ts}.csv')
    with open(csv_out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['condition','F1','recall','abort_rate','FAR','FRR','TP','FP','TN','FN'])
        for label, res in results:
            w.writerow([label, f"{res['f1']:.4f}", f"{res['recall']:.4f}",
                        f"{res['abort_rate']:.4f}", f"{res['FAR']:.4f}", f"{res['FRR']:.4f}",
                        res['TP'], res['FP'], res['TN'], res['FN']])
    print(f"\nCSV: {csv_out}")

    # ── Plot ───────────────────────────────────────────────────────────────────
    if not HAS_PLOT:
        return

    labels_plot = [l for l, _ in results]
    recall_vals = [r['recall']     for _, r in results]
    abort_vals  = [r['abort_rate'] for _, r in results]
    f1_vals     = [r['f1']         for _, r in results]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Guardian Physics Gate — DET on Real Speech + Real Noise\n'
                 f'(Google Speech Commands yes/no × {N_SPEECH*2}, '
                 f'background noise at +5/0/−5 dB SNR)',
                 fontsize=12, fontweight='bold')

    colors = ['#2ca02c', '#1f77b4', '#ff7f0e', '#d62728']
    x = np.arange(len(labels_plot))

    for ax, vals, title, ylabel in zip(
            axes,
            [recall_vals, abort_vals, f1_vals],
            ['Speech Recall vs SNR', 'Noise Abort Rate vs SNR', 'F1 Score vs SNR'],
            ['Recall (%)', 'Abort Rate (%)', 'F1 (%)']):
        bars = ax.bar(x, [v*100 for v in vals], color=colors[:len(x)])
        ax.axhline(90, color='red', linestyle='--', linewidth=1.2, label='90% target')
        ax.set_xticks(x)
        ax.set_xticklabels(labels_plot, fontsize=8)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.set_ylim(0, 108)
        ax.legend(fontsize=8)
        ax.grid(axis='y', alpha=0.3)
        for bar, val in zip(bars, vals):
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1,
                    f'{val*100:.1f}%', ha='center', va='bottom', fontsize=8)

    plt.tight_layout()
    png_out = os.path.join(RESULTS_DIR, f'det_real_{ts}.png')
    plt.savefig(png_out, dpi=150)
    print(f"Plot: {png_out}")


if __name__ == '__main__':
    main()
