#!/usr/bin/env python3
"""
Parse guardian serial log and produce gate timing stats + plot.

Usage:
    python analyze_serial_log.py gate_log.txt

Log line format (every 50 frames):
    GATE: WAKE  conf= 82 E=  512 C=  4231 ZCR= 31 T=1546us noise=  200 wakes= 3/50
"""
import sys
import re
import numpy as np
import matplotlib.pyplot as plt
import os

LOG_PATH   = sys.argv[1] if len(sys.argv) > 1 else "gate_log.txt"
BUDGET_US  = 5000
OUTPUT_PNG = os.path.join(os.path.dirname(__file__),
                          "../../results/gate_timing_serial.png")

gate_times   = []
wake_counts  = []
total_frames = 0

with open(LOG_PATH) as f:
    for line in f:
        m = re.search(r'T=(\d+)us.*wakes=\s*(\d+)/(\d+)', line)
        if m:
            gate_times.append(int(m.group(1)))
            wake_counts.append(int(m.group(2)))
            total_frames += int(m.group(3))

if not gate_times:
    print("ERROR: no GATE lines found. Check log format.")
    sys.exit(1)

gate_times  = np.array(gate_times)
total_wakes = sum(wake_counts)
abort_rate  = (1 - total_wakes / total_frames) * 100
p99         = np.percentile(gate_times, 99)

# ── Jitter / sigma statistics ─────────────────────────────────────────────────
mean_us     = gate_times.mean()
sigma_us    = gate_times.std()
p3sigma_us  = mean_us + 3 * sigma_us    # 99.73% of frames below this
p6sigma_us  = mean_us + 6 * sigma_us    # Six-sigma bound (3.4 PPM above)
p9999_us    = np.percentile(gate_times, 99.99) if len(gate_times) >= 10000 \
              else None                  # meaningful only with ≥10k samples

verdict     = "PASS — Gate meets <5ms budget" if p3sigma_us < BUDGET_US \
              else "FAIL — Gate exceeds budget at 3-sigma"

print(f"\n{'='*60}")
print("GATE TIMING ANALYSIS")
print(f"{'='*60}")
print(f"  Frames analysed : {total_frames:,}  ({len(gate_times)} log lines)")
print(f"  Min             : {gate_times.min():,} us")
print(f"  Avg (mean)      : {mean_us:,.0f} us")
print(f"  Std dev (sigma) : {sigma_us:,.1f} us")
print(f"  Max (observed)  : {gate_times.max():,} us")
print(f"  95th percentile : {np.percentile(gate_times, 95):,.0f} us")
print(f"  99th percentile : {p99:,.0f} us")
if p9999_us is not None:
    print(f"  99.99th pct     : {p9999_us:,.0f} us")
print(f"  3-sigma bound   : {p3sigma_us:,.0f} us  (99.73% of frames)")
print(f"  6-sigma bound   : {p6sigma_us:,.0f} us  (theoretical worst case)")
print(f"  Budget          : {BUDGET_US:,} us")
print(f"\n  Verdict: {verdict}")
print(f"  Jitter (sigma)  : {sigma_us:.1f} us  "
      f"({'deterministic <50us' if sigma_us < 50 else 'variable — check contention'})")
print(f"\n{'='*60}")
print("GATE DECISION ANALYSIS")
print(f"{'='*60}")
print(f"  Total wakes  : {total_wakes:,} / {total_frames:,} frames")
print(f"  Abort rate   : {abort_rate:.1f}%")
print(f"  Power saving : gate aborted {abort_rate:.0f}% of TinyML runs")

# FA/hr — each frame is 20ms (FRAME_SIZE=320 @ 16kHz)
duration_s  = total_frames * 0.020
duration_hr = duration_s / 3600
fa_hr       = total_wakes / duration_hr if duration_hr > 0 else 0
print(f"\n{'='*60}")
print("FALSE ALARM ANALYSIS")
print(f"{'='*60}")
print(f"  Capture duration : {duration_s:.0f}s  ({duration_hr*60:.1f} min)")
print(f"  Wake events      : {total_wakes:,}")
print(f"  FA/hr            : {fa_hr:,.0f}  (false alarms per hour)")
fa_verdict = ("✅ Low FA rate (<100/hr)" if fa_hr < 100
              else "⚠  High FA rate — consider raising thresholds")
print(f"  FA verdict       : {fa_verdict}")

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(14, 7))
fig.suptitle("Guardian Gate Timing (serial log)", fontsize=13, fontweight="bold")

ax = axes[0]
ax.plot(gate_times, linewidth=0.8, color="steelblue", label="Gate time")
ax.axhline(BUDGET_US,             color="red",    linestyle="--", linewidth=1.5,
           label=f"5ms budget")
ax.axhline(gate_times.mean(),     color="green",  linestyle="--", linewidth=1.2,
           label=f"Avg {gate_times.mean():.0f} µs")
ax.axhline(p99,                   color="orange", linestyle=":",  linewidth=1.2,
           label=f"99th {p99:.0f} µs")
ax.set_xlabel("Log line"); ax.set_ylabel("Gate time (µs)")
ax.set_title("Gate Execution Time"); ax.legend(); ax.grid(alpha=0.3)

ax = axes[1]
ax.hist(gate_times, bins=40, color="steelblue", alpha=0.75, edgecolor="white")
ax.axvline(BUDGET_US, color="red",    linestyle="--", linewidth=1.5, label="5ms budget")
ax.axvline(np.percentile(gate_times, 95), color="orange", linestyle="--",
           label=f"95th {np.percentile(gate_times,95):.0f} µs")
ax.set_xlabel("Gate time (µs)"); ax.set_ylabel("Count")
ax.set_title(f"Distribution — {verdict}"); ax.legend(); ax.grid(alpha=0.3)

plt.tight_layout()
os.makedirs(os.path.dirname(os.path.abspath(OUTPUT_PNG)), exist_ok=True)
plt.savefig(OUTPUT_PNG, dpi=300)
print(f"\nSaved: {OUTPUT_PNG}")
plt.show()
