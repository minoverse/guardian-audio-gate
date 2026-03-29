#!/usr/bin/env python3
"""
Guardian SystemView Trace Analyzer
Parses SEGGER SystemView CSV export and produces gate timing statistics + plots.

Export from SystemView:
  File → Export → Timeline Events → CSV  → save as timeline_export.csv
"""
import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

BUDGET_US  = 5000   # gate must complete within 5ms
CSV_PATH   = "timeline_export.csv"
OUTPUT_PNG = os.path.join(os.path.dirname(__file__),
                          "../../results/gate_timing_analysis.png")

# ── Load CSV ─────────────────────────────────────────────────────────────────

if not os.path.exists(CSV_PATH):
    print(f"ERROR: {CSV_PATH} not found.")
    print("Export from SystemView: File → Export → Timeline Events → CSV")
    sys.exit(1)

df = pd.read_csv(CSV_PATH)
print(f"Loaded {len(df)} events from {CSV_PATH}")

# ── Parse gate and TinyML timings ────────────────────────────────────────────

gate_times   = []
tinyml_times = []

for _, row in df.iterrows():
    desc = str(row.get("Description", ""))
    if "GATE_END:" in desc:
        try:
            us = int(desc.split("GATE_END:")[1].strip().split()[0])
            gate_times.append(us)
        except (IndexError, ValueError):
            pass
    elif "TINYML_END:" in desc:
        try:
            us = int(desc.split("TINYML_END:")[1].strip().split()[0])
            tinyml_times.append(us)
        except (IndexError, ValueError):
            pass

if not gate_times:
    print("ERROR: no GATE_END events found. Check CSV column name for 'Description'.")
    print("Available columns:", list(df.columns))
    sys.exit(1)

gate_times   = np.array(gate_times)
tinyml_times = np.array(tinyml_times) if tinyml_times else np.array([0])

total_frames = len(gate_times)
abort_count  = total_frames - len(tinyml_times)
abort_rate   = abort_count / total_frames * 100

# ── Statistics ────────────────────────────────────────────────────────────────

print(f"\n{'='*60}")
print("GATE TIMING ANALYSIS")
print(f"{'='*60}")
print(f"  Frames analysed : {total_frames:,}")
print(f"  Min             : {gate_times.min():,} us")
print(f"  Average         : {gate_times.mean():,.0f} us")
print(f"  Max             : {gate_times.max():,} us")
print(f"  95th percentile : {np.percentile(gate_times, 95):,.0f} us")
print(f"  99th percentile : {np.percentile(gate_times, 99):,.0f} us")
print(f"  Budget          : {BUDGET_US:,} us")

p99 = np.percentile(gate_times, 99)
verdict = "✅ PASS — Gate meets <5ms budget" if p99 < BUDGET_US \
          else "❌ FAIL — Gate exceeds budget"
print(f"\n  Verdict: {verdict}")

print(f"\n{'='*60}")
print("GATE DECISION ANALYSIS")
print(f"{'='*60}")
print(f"  TinyML triggered : {len(tinyml_times):,} / {total_frames:,} frames")
print(f"  Abort rate       : {abort_rate:.1f}%")
if tinyml_times[0] != 0:
    print(f"  TinyML avg time  : {tinyml_times.mean():,.0f} us")
print(f"\n  Power saving: gate aborted {abort_rate:.0f}% of TinyML runs")
print(f"  {'='*40}")

# ── Plot ──────────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(2, 1, figsize=(15, 8))
fig.suptitle("Guardian Gate Timing Analysis", fontsize=14, fontweight="bold")

# Plot 1: gate time over frames
ax = axes[0]
ax.plot(gate_times, linewidth=0.6, alpha=0.8, color="steelblue", label="Gate time")
ax.axhline(BUDGET_US, color="red",   linestyle="--", linewidth=1.5,
           label=f"5ms budget ({BUDGET_US:,} µs)")
ax.axhline(gate_times.mean(), color="green", linestyle="--", linewidth=1.2,
           label=f"Average ({gate_times.mean():.0f} µs)")
ax.axhline(np.percentile(gate_times, 99), color="orange", linestyle=":",
           linewidth=1.2, label=f"99th pct ({np.percentile(gate_times,99):.0f} µs)")
ax.set_xlabel("Frame number")
ax.set_ylabel("Gate execution time (µs)")
ax.set_title("Gate Execution Time Per Frame")
ax.legend(loc="upper right")
ax.grid(alpha=0.3)
ax.set_ylim(bottom=0)

# Plot 2: histogram
ax = axes[1]
ax.hist(gate_times, bins=60, alpha=0.75, color="steelblue", edgecolor="white")
ax.axvline(BUDGET_US, color="red",   linestyle="--", linewidth=1.5, label="5ms budget")
ax.axvline(np.percentile(gate_times, 95), color="orange", linestyle="--",
           linewidth=1.2, label=f"95th pct ({np.percentile(gate_times,95):.0f} µs)")
ax.axvline(np.percentile(gate_times, 99), color="purple", linestyle=":",
           linewidth=1.2, label=f"99th pct ({np.percentile(gate_times,99):.0f} µs)")
ax.set_xlabel("Gate execution time (µs)")
ax.set_ylabel("Frame count")
ax.set_title(f"Gate Timing Distribution  —  {verdict}")
ax.legend()
ax.grid(alpha=0.3)

plt.tight_layout()
os.makedirs(os.path.dirname(os.path.abspath(OUTPUT_PNG)), exist_ok=True)
plt.savefig(OUTPUT_PNG, dpi=300)
print(f"\nSaved: {OUTPUT_PNG}")
plt.show()
