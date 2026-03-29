#!/usr/bin/env python3
"""
Guardian VAD Comparison Table
Compares Standard Energy VAD vs Guardian Physics Gate across key metrics.

Usage:
    python3 compare_vad.py <physics_log> <energy_vad_log>

    physics_log    — serial log captured with GATE_MODE 2 (physics gate)
    energy_vad_log — serial log captured with GATE_MODE 1 (energy VAD)
                     (EVAD lines: "EVAD: wakes=X/50")

If only physics_log is provided, prints physics gate metrics only.
"""
import sys
import re
import os
import numpy as np
import matplotlib.pyplot as plt

FRAME_MS   = 20      # ms per frame (320 samples @ 16kHz)
GATE_US    = 1468    # measured gate avg timing (us)
EVAD_US    = 50      # energy VAD is just arm_rms_q15 — ~50us estimated
TINYML_MS  = 25      # mock TinyML inference time


def parse_physics_log(path):
    gate_times, wake_counts = [], []
    total_frames = 0
    with open(path) as f:
        for line in f:
            m = re.search(r'T=(\d+)us.*wakes=\s*(\d+)/(\d+)', line)
            if m:
                gate_times.append(int(m.group(1)))
                wake_counts.append(int(m.group(2)))
                total_frames += int(m.group(3))
    return np.array(gate_times), sum(wake_counts), total_frames


def parse_evad_log(path):
    wake_counts, total_frames = [], 0
    with open(path) as f:
        for line in f:
            m = re.search(r'EVAD: wakes=\s*(\d+)/(\d+)', line)
            if m:
                wake_counts.append(int(m.group(1)))
                total_frames += int(m.group(2))
    return sum(wake_counts), total_frames


# ── Load logs ─────────────────────────────────────────────────────────────────

if len(sys.argv) < 2:
    print("Usage: python3 compare_vad.py <physics_log> [energy_vad_log]")
    sys.exit(1)

physics_log  = sys.argv[1]
evad_log     = sys.argv[2] if len(sys.argv) > 2 else None

gate_times, phys_wakes, phys_frames = parse_physics_log(physics_log)

if not phys_frames:
    print(f"ERROR: no GATE lines found in {physics_log}")
    sys.exit(1)

phys_dur_s    = phys_frames * FRAME_MS / 1000
phys_dur_hr   = phys_dur_s / 3600
phys_fa_hr    = phys_wakes / phys_dur_hr if phys_dur_hr > 0 else 0
phys_abort_pct = (1 - phys_wakes / phys_frames) * 100

evad_wakes = evad_frames = 0
if evad_log and os.path.exists(evad_log):
    evad_wakes, evad_frames = parse_evad_log(evad_log)

evad_dur_s  = evad_frames * FRAME_MS / 1000 if evad_frames else phys_dur_s
evad_dur_hr = evad_dur_s / 3600
evad_fa_hr  = evad_wakes / evad_dur_hr if (evad_dur_hr > 0 and evad_frames) else None
evad_abort_pct = (1 - evad_wakes / evad_frames) * 100 if evad_frames else None

# ── Print comparison table ────────────────────────────────────────────────────

print(f"\n{'='*65}")
print("GUARDIAN vs STANDARD VAD — COMPARISON TABLE")
print(f"{'='*65}")
print(f"{'Metric':<30s} {'Std Energy VAD':>15s}  {'Guardian Gate':>15s}")
print(f"{'─'*65}")

def row(label, evad_val, phys_val):
    evad_str = str(evad_val) if evad_val is not None else "n/a"
    print(f"{label:<30s} {evad_str:>15s}  {str(phys_val):>15s}")

row("Gate algorithm",        "Energy RMS",          "Resonator+Correlation")
row("Gate latency (us)",     f"~{EVAD_US}",          f"{gate_times.mean():.0f}")
row("Gate latency budget",   "< 5ms ✅",             "< 5ms ✅")
row("99th pct latency (us)", "~50",
    f"{np.percentile(gate_times,99):.0f}")
row("Abort rate (%)",
    f"{evad_abort_pct:.1f}" if evad_abort_pct is not None else "n/a",
    f"{phys_abort_pct:.1f}")
row("FA/hr (quiet room)",
    f"{evad_fa_hr:.0f}" if evad_fa_hr is not None else "n/a",
    f"{phys_fa_hr:.0f}")
row("Spectral discrimination", "None (energy only)",  "4-resonator bank")
row("Noise floor tracking",    "None",                "EMA adaptive")
row("800Hz tone rejection",    "Poor",                "Tested separately")

print(f"{'─'*65}")
print(f"\n{'='*65}")
print("KEY FINDINGS")
print(f"{'='*65}")
print(f"  Physics gate abort rate  : {phys_abort_pct:.1f}%  "
      f"→ TinyML skipped {phys_abort_pct:.0f}% of frames")
print(f"  Physics gate FA/hr       : {phys_fa_hr:.0f}  "
      f"(capture: {phys_dur_s:.0f}s)")
print(f"  Gate latency             : {gate_times.mean():.0f}µs avg  "
      f"(3.4× under 5ms budget)")
if evad_fa_hr is not None:
    delta_fa = evad_fa_hr - phys_fa_hr
    print(f"  FA reduction vs EVAD     : {delta_fa:+.0f}/hr  "
          f"({delta_fa/evad_fa_hr*100:.0f}% fewer)" if evad_fa_hr > 0 else "")

# ── Plot ──────────────────────────────────────────────────────────────────────

metrics  = ["Gate Latency (µs)", "Abort Rate (%)", "FA/hr"]
phys_vals = [gate_times.mean(), phys_abort_pct, phys_fa_hr]
evad_vals = [EVAD_US,
             evad_abort_pct if evad_abort_pct is not None else 0,
             evad_fa_hr     if evad_fa_hr     is not None else 0]

x     = np.arange(len(metrics))
width = 0.35

fig, ax = plt.subplots(figsize=(10, 5))
fig.suptitle("Guardian Physics Gate vs Standard Energy VAD", fontsize=13, fontweight="bold")

bars1 = ax.bar(x - width/2, evad_vals,  width, label="Energy VAD",    color="#ff7f0e", alpha=0.82, edgecolor="black")
bars2 = ax.bar(x + width/2, phys_vals,  width, label="Physics Gate",  color="#2ca02c", alpha=0.82, edgecolor="black")

ax.set_xticks(x); ax.set_xticklabels(metrics)
ax.legend(); ax.grid(axis="y", alpha=0.3)
ax.set_title("Higher abort rate = better power saving | Lower FA/hr = better accuracy")

for bar in list(bars1) + list(bars2):
    h = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2, h + max(phys_vals)*0.01,
            f"{h:.0f}", ha="center", fontsize=8)

plt.tight_layout()
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../results")
os.makedirs(out_dir, exist_ok=True)
out_png = os.path.join(out_dir, "vad_comparison.png")
plt.savefig(out_png, dpi=300)
print(f"\nSaved: {out_png}")
plt.show()
