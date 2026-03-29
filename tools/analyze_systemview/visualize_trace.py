#!/usr/bin/env python3
"""
Guardian Week 9 — Execution Timeline Visualizer

Parses CSV trace dumped from firmware (ENABLE_TRACE 1) and produces:
  - Execution timeline (Guardian gate + TinyML spans)
  - Gate timing statistics + deadline miss count

Usage:
    python3 visualize_trace.py trace_log.csv

Capture trace:
    cat /dev/ttyACM0 | tee trace_log.csv    # Ctrl+C after ~30s
"""
import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

DEADLINE_MS = 20    # 20ms frame budget
CSV_PATH    = sys.argv[1] if len(sys.argv) > 1 else "trace_log.csv"
OUTPUT_PNG  = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "../../results/guardian_timeline.png")

# ── Load CSV ──────────────────────────────────────────────────────────────────

valid_events = {"FRAME_START","GUARDIAN_START","GUARDIAN_END","TINYML_START","TINYML_END"}

rows = []
with open(CSV_PATH) as f:
    for line in f:
        parts = line.strip().split(",")
        if len(parts) == 3 and parts[1].strip() in valid_events:
            try:
                rows.append({
                    "timestamp_ms": int(parts[0]),
                    "event":        parts[1].strip(),
                    "data":         int(parts[2]),
                })
            except ValueError:
                pass

if not rows:
    print("ERROR: no trace events found in file.")
    print("Make sure ENABLE_TRACE 1 and firmware was rebuilt + reflashed.")
    sys.exit(1)

df = pd.DataFrame(rows)

print(f"Loaded {len(df)} trace events")

# ── Parse event spans ─────────────────────────────────────────────────────────

events     = df.to_dict("records")
guardian   = []   # list of (start_ms, end_ms, elapsed_us)
tinyml     = []   # list of (start_ms, end_ms)
frame_starts = []

i = 0
while i < len(events):
    ev = events[i]
    if ev["event"] == "FRAME_START":
        frame_starts.append(ev["timestamp_ms"])
    elif ev["event"] == "GUARDIAN_START":
        for j in range(i + 1, len(events)):
            if events[j]["event"] == "GUARDIAN_END":
                guardian.append((
                    ev["timestamp_ms"],
                    events[j]["timestamp_ms"],
                    events[j]["data"],
                ))
                break
    elif ev["event"] == "TINYML_START":
        for j in range(i + 1, len(events)):
            if events[j]["event"] == "TINYML_END":
                tinyml.append((
                    ev["timestamp_ms"],
                    events[j]["timestamp_ms"],
                ))
                break
    i += 1

if not guardian:
    print("ERROR: no GUARDIAN_START/END pairs found.")
    print("Make sure ENABLE_TRACE 1 and GATE_MODE 2 are set.")
    sys.exit(1)

gate_dur_us = np.array([g[2] for g in guardian])
gate_dur_ms = (np.array([g[1] - g[0] for g in guardian]))

# ── Statistics ────────────────────────────────────────────────────────────────

deadline_misses = int(np.sum(gate_dur_us > DEADLINE_MS * 1000))
miss_rate       = deadline_misses / len(guardian) * 100

print(f"\n{'='*55}")
print("SCHEDULER ANALYSIS")
print(f"{'='*55}")
print(f"  Frames captured   : {len(frame_starts):,}")
print(f"  Gate events       : {len(guardian):,}")
print(f"  TinyML events     : {len(tinyml):,}  ({len(tinyml)/len(guardian)*100:.1f}% of frames)")
print(f"\n  Gate timing (µs):")
print(f"    Min  : {gate_dur_us.min():,}")
print(f"    Avg  : {gate_dur_us.mean():,.0f}")
print(f"    Max  : {gate_dur_us.max():,}")
print(f"    99th : {np.percentile(gate_dur_us, 99):,.0f}")
print(f"\n  Deadline ({DEADLINE_MS}ms budget):")
print(f"    Misses    : {deadline_misses} / {len(guardian)}")
print(f"    Miss rate : {miss_rate:.2f}%")
verdict = "✅ Zero deadline misses" if deadline_misses == 0 else f"❌ {deadline_misses} deadline misses"
print(f"    Verdict   : {verdict}")
print(f"{'='*55}")

# ── Timeline plot ─────────────────────────────────────────────────────────────

# Show first 3 seconds of trace
t0       = frame_starts[0] if frame_starts else guardian[0][0]
t_max    = t0 + 3000  # 3 seconds
GUARDIAN_Y = 0.0
TINYML_Y   = 1.3

fig, axes = plt.subplots(2, 1, figsize=(16, 8))
fig.suptitle("Guardian Execution Timeline (Week 9)", fontsize=14, fontweight="bold")

# ── Top: timeline ──────────────────────────────────────────────────────────
ax = axes[0]

for (start, end, us) in guardian:
    if start > t_max:
        break
    dur = end - start
    ax.add_patch(mpatches.Rectangle(
        (start - t0, GUARDIAN_Y), max(dur, 0.5), 1.0,
        facecolor="#2ca02c", alpha=0.75, edgecolor="darkgreen", linewidth=0.5
    ))

for (start, end) in tinyml:
    if start > t_max:
        break
    dur = end - start
    ax.add_patch(mpatches.Rectangle(
        (start - t0, TINYML_Y), max(dur, 0.5), 1.0,
        facecolor="#ff7f0e", alpha=0.75, edgecolor="darkorange", linewidth=0.5
    ))

# Frame deadline lines
for fs in frame_starts:
    if fs - t0 > t_max - t0:
        break
    ax.axvline(fs - t0, color="blue", alpha=0.2, linewidth=0.8)

ax.set_xlim(0, t_max - t0)
ax.set_ylim(-0.2, 2.6)
ax.set_xlabel("Time (ms)")
ax.set_yticks([0.5, 1.8])
ax.set_yticklabels(["Guardian Gate", "TinyML"], fontsize=11)
ax.set_title(f"Execution Timeline (first 3s)  —  {verdict}", fontweight="bold")
ax.grid(axis="x", alpha=0.3)

legend = [
    mpatches.Patch(color="#2ca02c", alpha=0.75, label=f"Guardian gate (~{gate_dur_us.mean():.0f}µs)"),
    mpatches.Patch(color="#ff7f0e", alpha=0.75, label="Mock TinyML (100ms)"),
    mpatches.Patch(color="blue",    alpha=0.2,  label="Frame boundary (20ms)"),
]
ax.legend(handles=legend, loc="upper right")

# ── Bottom: gate timing distribution ──────────────────────────────────────
ax = axes[1]
ax.hist(gate_dur_us, bins=40, color="#2ca02c", alpha=0.75, edgecolor="white")
ax.axvline(DEADLINE_MS * 1000, color="red",    linestyle="--", linewidth=1.5,
           label=f"20ms budget ({DEADLINE_MS*1000:,}µs)")
ax.axvline(gate_dur_us.mean(), color="green",  linestyle="--", linewidth=1.2,
           label=f"Avg {gate_dur_us.mean():.0f}µs")
ax.axvline(np.percentile(gate_dur_us, 99), color="orange", linestyle=":",
           linewidth=1.2, label=f"99th {np.percentile(gate_dur_us,99):.0f}µs")
ax.set_xlabel("Gate execution time (µs)")
ax.set_ylabel("Frame count")
ax.set_title(f"Gate Timing Distribution  —  miss rate {miss_rate:.2f}%")
ax.legend()
ax.grid(alpha=0.3)

plt.tight_layout()
os.makedirs(os.path.dirname(OUTPUT_PNG), exist_ok=True)
plt.savefig(OUTPUT_PNG, dpi=300)
print(f"\nSaved: {OUTPUT_PNG}")
plt.show()
