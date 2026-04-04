#!/usr/bin/env python3
"""
Guardian — DET Curve on LibriSpeech + ESC-50

Real speech  : LibriSpeech test-clean (2620 FLAC clips, multiple speakers)
Real noise   : ESC-50 (2000 environmental sound clips at 44.1kHz)
               Categories used: cafe/restaurant, street noise, rain,
               vacuum cleaner, air conditioner (non-speech backgrounds)

SNR levels   : clean, +5dB, 0dB, -5dB
Pass criteria: Recall >= 90% AND Abort >= 90%

Usage:
    cd ~/projects/guardian-audio-gate
    python3 tools/det_librispeech_esc50.py
"""
import os, sys, csv, random, datetime
import numpy as np
import soundfile as sf
from scipy.signal import resample_poly
from math import gcd

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
PROJECT_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIBRI_DIR     = os.path.join(PROJECT_DIR, 'training_data', 'librispeech', 'test-clean')
ESC50_DIR     = os.path.join(PROJECT_DIR, 'training_data', 'esc50')
ESC50_AUDIO   = os.path.join(ESC50_DIR, 'audio')
ESC50_META    = os.path.join(ESC50_DIR, 'meta', 'esc50.csv')
RESULTS_DIR   = os.path.join(PROJECT_DIR, 'results')

TARGET_SR  = 16000
CLIP_LEN   = TARGET_SR        # 1 second
N_SPEECH   = 50               # LibriSpeech clips
N_NOISE    = 50               # ESC-50 clips (noise-only test)
SNR_LEVELS = [5, 0, -5]
RANDOM_SEED= 42

# ESC-50 categories suitable as background noise (non-speech)
NOISE_CATEGORIES = {
    'cafe_restaurant', 'street_music', 'rain', 'vacuum_cleaner',
    'air_conditioner', 'engine', 'wind', 'crickets', 'thunderstorm',
    'water_drops', 'clock_tick', 'train', 'car_horn', 'chainsaw',
}


# ── Audio helpers ──────────────────────────────────────────────────────────────

def load_resample(path, target_sr=TARGET_SR):
    """Load any audio file and resample to target_sr, return int16 mono."""
    data, sr = sf.read(path, dtype='float32', always_2d=False)
    if data.ndim > 1:
        data = data.mean(axis=1)          # stereo → mono
    if sr != target_sr:
        g = gcd(sr, target_sr)
        data = resample_poly(data, target_sr // g, sr // g).astype(np.float32)
    # float32 [-1,1] → int16
    data = np.clip(data * 32768.0, -32768, 32767).astype(np.int16)
    return data

def pad_or_trim(samples, length=CLIP_LEN):
    if len(samples) >= length:
        return samples[:length]
    return np.pad(samples, (0, length - len(samples)))

def rms(s):
    f = s.astype(np.float64)
    r = np.sqrt(np.mean(f**2))
    return r if r > 1e-6 else 1e-6

def mix_at_snr(speech, noise, snr_db):
    sf_ = speech.astype(np.float64)
    nf_ = noise.astype(np.float64)
    scale = rms(sf_) / rms(nf_) / (10 ** (snr_db / 20.0))
    mixed = sf_ + scale * nf_
    return np.clip(mixed, -32768, 32767).astype(np.int16)


# ── Dataset loaders ────────────────────────────────────────────────────────────

def collect_librispeech(n):
    """Pick n FLAC clips from LibriSpeech test-clean."""
    rng = random.Random(RANDOM_SEED)
    all_flac = []
    for root, _, files in os.walk(LIBRI_DIR):
        for f in files:
            if f.endswith('.flac'):
                all_flac.append(os.path.join(root, f))
    rng.shuffle(all_flac)
    return all_flac[:n]

def collect_esc50_noise(n):
    """Pick n ESC-50 clips from non-speech noise categories."""
    rng = random.Random(RANDOM_SEED + 1)
    rows = []
    with open(ESC50_META) as f:
        for row in csv.DictReader(f):
            if row['category'] in NOISE_CATEGORIES:
                rows.append(os.path.join(ESC50_AUDIO, row['filename']))
    rng.shuffle(rows)
    return rows[:n]

def collect_esc50_all(n):
    """Pick n ESC-50 clips of any category for noise-only abort test."""
    rng = random.Random(RANDOM_SEED + 2)
    files = [os.path.join(ESC50_AUDIO, f)
             for f in os.listdir(ESC50_AUDIO) if f.endswith('.wav')]
    rng.shuffle(files)
    return files[:n]


# ── Gate simulation ────────────────────────────────────────────────────────────

WAKE_THRESHOLD = 55
RULE_WEIGHTS   = {'correlation': 40, 'energy': 20, 'coherence': 25, 'zcr': 15}

def run_gate(samples):
    bank = ResonatorBank()
    nf   = NOISE_FLOOR_INIT
    scores = []
    for i in range(len(samples) // FRAME_SIZE):
        frame = samples[i*FRAME_SIZE:(i+1)*FRAME_SIZE]
        out   = bank.process(frame)
        e     = extract_energy(out)
        c     = extract_correlation(out)
        co    = extract_coherence(out[0])
        z     = compute_zcr(out[0])
        s = 0
        if c > CORRELATION_THRESHOLD: s += 40
        if e > nf * 2:               s += 20
        if co > COHERENCE_THRESHOLD: s += 25
        if z < ZCR_MAX:              s += 15
        scores.append(s)
        nf = update_noise_floor(nf, e, s >= WAKE_THRESHOLD)
    return float(np.mean(np.array(scores) >= WAKE_THRESHOLD)) if scores else 0.0

def evaluate(speech_paths, noise_paths, snr_db=None):
    """Evaluate gate on speech + noise clips. Mix speech with noise if snr_db given."""
    # Build noise pool for mixing
    noise_pool = []
    for p in noise_paths:
        try:
            noise_pool.append(pad_or_trim(load_resample(p)))
        except Exception:
            pass
    noise_idx = 0

    TP = FP = TN = FN = 0

    for path in speech_paths:
        try:
            speech = pad_or_trim(load_resample(path))
        except Exception:
            continue
        if snr_db is not None and noise_pool:
            noise  = noise_pool[noise_idx % len(noise_pool)]
            noise_idx += 1
            audio  = mix_at_snr(speech, noise, snr_db)
        else:
            audio  = speech
        wake_rate = run_gate(audio)
        if wake_rate >= 0.30: TP += 1
        else:                 FN += 1

    for path in noise_paths:
        try:
            audio = pad_or_trim(load_resample(path))
        except Exception:
            continue
        wake_rate = run_gate(audio)
        if wake_rate >= 0.30: FP += 1
        else:                 TN += 1

    ns = FP + TN; ss = TP + FN
    recall     = TP / ss if ss > 0 else 0
    abort_rate = TN / ns if ns > 0 else 0
    precision  = TP / (TP+FP) if (TP+FP) > 0 else 0
    f1         = 2*precision*recall/(precision+recall) if (precision+recall) > 0 else 0
    return dict(recall=recall, abort_rate=abort_rate, f1=f1,
                FAR=FP/ns if ns>0 else 0, FRR=FN/ss if ss>0 else 0,
                TP=TP, FP=FP, TN=TN, FN=FN)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("\n" + "="*68)
    print("GUARDIAN — DET CURVE: LibriSpeech test-clean + ESC-50 noise")
    print("="*68)
    print(f"Speech : LibriSpeech test-clean FLAC  ({N_SPEECH} clips, real speakers)")
    print(f"Noise  : ESC-50 environmental sounds  ({N_NOISE} clips, 44.1kHz→16kHz)")
    print(f"SNR    : clean, +5dB, 0dB, -5dB")
    print(f"Pass   : Recall ≥ 90% AND Abort ≥ 90%")
    print("="*68)

    print("\nLoading clips...", flush=True)
    speech_paths = collect_librispeech(N_SPEECH)
    noise_mix    = collect_esc50_noise(N_NOISE)    # for mixing with speech
    noise_only   = collect_esc50_all(N_NOISE)      # for abort rate test
    print(f"  Speech: {len(speech_paths)} LibriSpeech clips")
    print(f"  Noise (mix): {len(noise_mix)} ESC-50 clips")
    print(f"  Noise (abort test): {len(noise_only)} ESC-50 clips\n")

    conditions = [('Clean (no noise)', None)] + [(f'SNR={s:+d}dB', s) for s in SNR_LEVELS]
    results = []

    for label, snr in conditions:
        print(f"[{label}] running...", end=' ', flush=True)
        res = evaluate(speech_paths, noise_only, snr_db=snr)
        results.append((label, res))
        abort_ok  = '✓' if res['abort_rate'] >= 0.90 else '✗'
        recall_ok = '✓' if res['recall']     >= 0.90 else '✗'
        passed    = 'PASS' if (res['abort_rate'] >= 0.90 and res['recall'] >= 0.90) else 'FAIL'
        print(f"F1={res['f1']:.3f} Recall={res['recall']*100:.1f}%{recall_ok} "
              f"Abort={res['abort_rate']*100:.1f}%{abort_ok} → {passed}")

    print(f"\n{'='*68}")
    print("SUMMARY")
    print(f"{'─'*68}")
    print(f"{'Condition':<22} {'F1':>6} {'Recall':>8} {'Abort':>8} {'FAR':>7} {'FRR':>7}  Pass?")
    print(f"{'─'*68}")
    for label, res in results:
        passed = 'PASS' if (res['abort_rate'] >= 0.90 and res['recall'] >= 0.90) else 'FAIL'
        print(f"{label:<22} {res['f1']:>6.3f} {res['recall']*100:>7.1f}% "
              f"{res['abort_rate']*100:>7.1f}% "
              f"{res['FAR']*100:>6.1f}% {res['FRR']*100:>6.1f}%   {passed}")
    print(f"{'─'*68}")
    print("Criteria: Recall ≥ 90% AND Abort ≥ 90%")
    print(f"{'='*68}")

    # Save CSV
    ts      = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_out = os.path.join(RESULTS_DIR, f'det_librispeech_esc50_{ts}.csv')
    with open(csv_out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['condition','F1','recall','abort_rate','FAR','FRR','TP','FP','TN','FN'])
        for label, res in results:
            w.writerow([label, f"{res['f1']:.4f}", f"{res['recall']:.4f}",
                        f"{res['abort_rate']:.4f}", f"{res['FAR']:.4f}", f"{res['FRR']:.4f}",
                        res['TP'], res['FP'], res['TN'], res['FN']])
    print(f"\nCSV: {csv_out}")

    if not HAS_PLOT:
        return

    labels_p    = [l for l, _ in results]
    recall_vals = [r['recall']     for _, r in results]
    abort_vals  = [r['abort_rate'] for _, r in results]
    f1_vals     = [r['f1']         for _, r in results]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Guardian Physics Gate — DET on LibriSpeech + ESC-50\n'
                 f'(LibriSpeech test-clean ×{N_SPEECH}, ESC-50 noise at +5/0/−5 dB SNR)',
                 fontsize=12, fontweight='bold')
    colors = ['#2ca02c','#1f77b4','#ff7f0e','#d62728']
    x = np.arange(len(labels_p))

    for ax, vals, title, ylabel in zip(
            axes,
            [recall_vals, abort_vals, f1_vals],
            ['Speech Recall vs SNR','Noise Abort Rate vs SNR','F1 Score vs SNR'],
            ['Recall (%)','Abort Rate (%)','F1 (%)']):
        bars = ax.bar(x, [v*100 for v in vals], color=colors[:len(x)])
        ax.axhline(90, color='red', linestyle='--', linewidth=1.2, label='90% target')
        ax.set_xticks(x); ax.set_xticklabels(labels_p, fontsize=8)
        ax.set_ylabel(ylabel); ax.set_title(title)
        ax.set_ylim(0, 108); ax.legend(fontsize=8); ax.grid(axis='y', alpha=0.3)
        for bar, val in zip(bars, vals):
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1,
                    f'{val*100:.1f}%', ha='center', va='bottom', fontsize=8)

    plt.tight_layout()
    png_out = os.path.join(RESULTS_DIR, f'det_librispeech_esc50_{ts}.png')
    plt.savefig(png_out, dpi=150)
    print(f"Plot: {png_out}")


if __name__ == '__main__':
    main()
