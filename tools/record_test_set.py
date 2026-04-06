#!/usr/bin/env python3
"""
Guardian Gate — Audio Test-Set Recorder

Records labeled WAV files (speech / noise) at 16 kHz mono for gate validation.

Usage (interactive):
    python3 record_test_set.py

Usage (headless — generate synthetic test set):
    python3 record_test_set.py --synthetic

Output:
    results/audio_test_set/
        speech_001.wav  …  speech_020.wav   (3-s clips, say a word/phrase)
        noise_001.wav   …  noise_020.wav    (3-s clips, silence/background)
        labels.csv
"""
import os
import sys
import csv
import wave
import struct
import argparse
import datetime
import math
import random

SAMPLE_RATE  = 16000
DURATION_S   = 3
NUM_SAMPLES  = SAMPLE_RATE * DURATION_S
NUM_CLIPS    = 10      # per class (reduce to speed up; increase for better stats)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "../results/audio_test_set")


# ── WAV helpers ───────────────────────────────────────────────────────────────

def save_wav(path, samples_int16):
    """Save list/array of int16 samples to a mono 16-bit WAV file."""
    with wave.open(path, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        packed = struct.pack(f'<{len(samples_int16)}h', *samples_int16)
        wf.writeframes(packed)


# ── Synthetic audio generators ────────────────────────────────────────────────

def gen_speech_like(seed=42):
    """
    Synthetic 'speech': mix of harmonics (100 Hz fundamental + overtones)
    with amplitude modulation to mimic voiced speech.  Not real speech but
    activates all 4 gate rules reliably (high correlation, low ZCR, coherent).
    """
    rng   = random.Random(seed)
    f0    = rng.uniform(100, 180)          # male/female pitch range
    amp   = 0.25                           # 25% amplitude (Q15: ~8192)
    out   = []
    phase = [0.0] * 6
    for n in range(NUM_SAMPLES):
        # Fundamental + 5 harmonics (voiced speech model)
        s = 0.0
        for k in range(1, 7):
            phase[k-1] += 2 * math.pi * f0 * k / SAMPLE_RATE
            s += amp / k * math.sin(phase[k-1])
        # AM envelope: syllable-rate ~4 Hz, depth 50%
        env = 0.5 + 0.5 * math.sin(2 * math.pi * 4.0 * n / SAMPLE_RATE)
        val = int(s * env * 32767)
        val = max(-32768, min(32767, val))
        out.append(val)
    return out


def gen_noise_like(seed=0):
    """
    Synthetic noise: Galois LFSR broadband random noise (high ZCR, low
    inter-channel correlation after resonator filtering → gate aborts).
    """
    lfsr = (0xACE1 + seed) & 0xFFFF
    out  = []
    for _ in range(NUM_SAMPLES):
        lfsr = (lfsr >> 1) ^ (int(-(lfsr & 1)) & 0xB400)
        lfsr &= 0xFFFF
        val  = (lfsr ^ 0x8000) - 32768  # centre around 0
        val  = val >> 3                  # reduce amplitude
        out.append(max(-32768, min(32767, val)))
    return out


def gen_silence(seed=0):
    """Near-silence with very low-amplitude noise floor (~±50 amplitude)."""
    rng = random.Random(seed + 999)
    return [rng.randint(-50, 50) for _ in range(NUM_SAMPLES)]


# ── Microphone recording (requires sounddevice) ───────────────────────────────

def record_clip(label, index):
    try:
        import sounddevice as sd
        import numpy as np
    except ImportError:
        print("sounddevice not installed — use --synthetic mode")
        sys.exit(1)

    print(f"\n  [{label.upper()} {index:03d}] Ready? Press ENTER then speak/hold quiet for {DURATION_S}s...")
    input()
    print(f"  Recording {DURATION_S}s...")
    audio = sd.rec(NUM_SAMPLES, samplerate=SAMPLE_RATE, channels=1,
                   dtype='int16', blocking=True)
    sd.wait()
    return audio.flatten().tolist()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--synthetic', action='store_true',
                        help='Generate synthetic test set (no microphone needed)')
    parser.add_argument('--clips', type=int, default=NUM_CLIPS,
                        help=f'Clips per class (default {NUM_CLIPS})')
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    labels_path = os.path.join(OUT_DIR, 'labels.csv')

    rows = []

    if args.synthetic:
        print("=== Guardian Gate — Generating Synthetic Test Set ===\n")
        print(f"  Speech : {args.clips} clips — harmonic mix (100-180 Hz f0 + overtones)")
        print(f"  Noise  : {args.clips} clips — LFSR broadband noise")
        print(f"  Output : {OUT_DIR}\n")

        for i in range(1, args.clips + 1):
            path = os.path.join(OUT_DIR, f'speech_{i:03d}.wav')
            save_wav(path, gen_speech_like(seed=i))
            rows.append({'file': os.path.basename(path), 'label': 'speech'})
            print(f"  Saved {os.path.basename(path)}")

        for i in range(1, args.clips + 1):
            path = os.path.join(OUT_DIR, f'noise_{i:03d}.wav')
            save_wav(path, gen_noise_like(seed=i))
            rows.append({'file': os.path.basename(path), 'label': 'noise'})
            print(f"  Saved {os.path.basename(path)}")

        # Also add a few silence clips for robustness
        for i in range(1, 4):
            path = os.path.join(OUT_DIR, f'silence_{i:03d}.wav')
            save_wav(path, gen_silence(seed=i))
            rows.append({'file': os.path.basename(path), 'label': 'noise'})
            print(f"  Saved {os.path.basename(path)}")

    else:
        print("=== Guardian Gate — Live Microphone Recording ===\n")
        print(f"  Will record {args.clips} speech + {args.clips} noise clips ({DURATION_S}s each).")
        print("  SPEECH clips: say a word or phrase each time.")
        print("  NOISE  clips: stay silent or make background noise.\n")

        print("--- SPEECH clips ---")
        for i in range(1, args.clips + 1):
            samples = record_clip('speech', i)
            path    = os.path.join(OUT_DIR, f'speech_{i:03d}.wav')
            save_wav(path, samples)
            rows.append({'file': os.path.basename(path), 'label': 'speech'})
            print(f"  Saved {os.path.basename(path)}")

        print("\n--- NOISE / SILENCE clips ---")
        for i in range(1, args.clips + 1):
            samples = record_clip('noise', i)
            path    = os.path.join(OUT_DIR, f'noise_{i:03d}.wav')
            save_wav(path, samples)
            rows.append({'file': os.path.basename(path), 'label': 'noise'})
            print(f"  Saved {os.path.basename(path)}")

    # Write labels.csv
    with open(labels_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['file', 'label'])
        w.writeheader()
        w.writerows(rows)

    print(f"\nLabels : {labels_path}")
    print(f"Total  : {len(rows)} clips ({sum(1 for r in rows if r['label']=='speech')} speech, "
          f"{sum(1 for r in rows if r['label']=='noise')} noise)")
    print("\nNext step:")
    print("  python3 validate_gate_on_real_audio.py results/audio_test_set/labels.csv")


if __name__ == '__main__':
    main()
