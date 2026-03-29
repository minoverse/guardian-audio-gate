#!/usr/bin/env python3
"""
Guardian Week 7-8 — Gate Power Comparison (3 modes)

Measures average current for:
  Mode 0 BASELINE   — resonator + always runs mock TinyML (no gate)
  Mode 1 ENERGY_VAD — simple RMS threshold, skips resonator bank
  Mode 2 PHYSICS    — full resonator + gate_decide (default)

Usage:
  1. Build with GATE_MODE 0 + prj_power_test.conf → flash → press ENTER
  2. Build with GATE_MODE 1 + prj_power_test.conf → flash → press ENTER
  3. Build with GATE_MODE 2 + prj_power_test.conf → flash → press ENTER

  Build command for each mode (edit GATE_MODE in main.c first):
    west build -b nrf52840dk/nrf52840 -- -DEXTRA_CONF_FILE=prj_power_test.conf
    west flash
"""
import time
import sys
import os
import datetime

try:
    from ppk2_api.ppk2_api import PPK2_API
    import numpy as np
    import matplotlib.pyplot as plt
except ImportError:
    print("Install: pip3 install ppk2-api numpy matplotlib --break-system-packages")
    sys.exit(1)

DURATION_S  = 30       # seconds per phase
VOLTAGE_MV  = 3300     # match prj.conf
BATTERY_MAH = 200      # CR2032 for battery life projection
MODES       = ["BASELINE (no gate)", "ENERGY_VAD", "PHYSICS_GATE"]


def find_ppk2_port():
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if 'PPK2' in (p.description or '') or 'JLink' in (p.description or ''):
            print(f"Found: {p.device} — {p.description}")
            if input(f"Use {p.device}? (y/n): ").strip().lower() == 'y':
                return p.device
    print("PPK2 not found. Enter port manually (e.g., /dev/ttyACM0 or COM3):")
    return input("Port: ").strip()


def measure_phase(ppk2, label, duration):
    print(f"\n{'='*60}")
    print(f"Phase: {label}  [{duration}s]")
    print(f"{'='*60}")
    print("Steps:")
    print("  1. Edit main.c: set GATE_MODE to the correct value")
    print("  2. west build -b nrf52840dk/nrf52840 -- -DEXTRA_CONF_FILE=prj_power_test.conf")
    print("  3. west flash")
    print("  4. Reconnect PPK2 VDD wire to P22")
    input("Press ENTER when ready...")

    ppk2.ser.flushInput()
    time.sleep(0.2)
    ppk2.ser.flushInput()

    ppk2.get_modifiers()
    ppk2.use_source_meter()
    ppk2.set_source_voltage(VOLTAGE_MV)
    ppk2.toggle_DUT_power("ON")
    time.sleep(2)  # boot settle

    ppk2.ser.timeout = 0.1
    ppk2.ser.flushInput()

    samples = []
    ppk2.start_measuring()
    time.sleep(0.1)
    t0 = time.time()
    while time.time() - t0 < duration:
        raw = ppk2.ser.read(4096)
        if raw:
            s, _ = ppk2.get_samples(raw)
            samples.extend(s)

    ppk2.stop_measuring()
    ppk2.toggle_DUT_power("OFF")

    print(f"  Collected {len(samples):,} samples")
    if not samples:
        print(f"ERROR: no samples for {label}")
        sys.exit(1)

    arr = np.array(samples) / 1000.0  # µA → mA
    result = {
        "label":    label,
        "n":        len(arr),
        "mean_ma":  float(np.mean(arr)),
        "min_ma":   float(np.min(arr)),
        "max_ma":   float(np.max(arr)),
        "p99_ma":   float(np.percentile(arr, 99)),
        "samples":  arr,
    }
    print(f"  Avg: {result['mean_ma']:.3f} mA")
    return result


def print_results(results):
    baseline_ma = results[0]["mean_ma"]

    print(f"\n{'='*65}")
    print("GUARDIAN GATE POWER COMPARISON")
    print(f"{'='*65}")
    print(f"{'':30s} {'BASELINE':>10s}  {'ENERGY_VAD':>10s}  {'PHYSICS':>10s}")
    print(f"{'─'*65}")
    print(f"{'Samples':30s}", end="")
    for r in results:
        print(f"  {r['n']:>10,}", end="")
    print()
    print(f"{'Average (mA)':30s}", end="")
    for r in results:
        print(f"  {r['mean_ma']:>10.3f}", end="")
    print()
    print(f"{'Min (mA)':30s}", end="")
    for r in results:
        print(f"  {r['min_ma']:>10.3f}", end="")
    print()
    print(f"{'Max (mA)':30s}", end="")
    for r in results:
        print(f"  {r['max_ma']:>10.3f}", end="")
    print()
    print(f"{'99th pct (mA)':30s}", end="")
    for r in results:
        print(f"  {r['p99_ma']:>10.3f}", end="")
    print()
    print(f"{'─'*65}")

    for r in results[1:]:
        saving_ma  = baseline_ma - r["mean_ma"]
        saving_pct = saving_ma / baseline_ma * 100
        batt_baseline = BATTERY_MAH / baseline_ma
        batt_mode     = BATTERY_MAH / r["mean_ma"]
        print(f"\n  {r['label']} vs BASELINE:")
        print(f"    Saving    : {saving_ma:+.3f} mA  ({saving_pct:+.1f}%)")
        print(f"    Battery   : {batt_baseline:.0f}h → {batt_mode:.0f}h  "
              f"(+{batt_mode - batt_baseline:.0f}h)")

    print(f"\n{'='*65}")

    physics = results[2]
    verdict = ("✅ Physics gate saves power vs baseline"
               if physics["mean_ma"] < baseline_ma * 0.9
               else "⚠  Difference < 10% — UART may still be active (use prj_power_test.conf)")
    print(f"Verdict: {verdict}")


def plot_results(results):
    colors  = ["#d62728", "#ff7f0e", "#2ca02c"]
    modes   = [r["label"] for r in results]
    means   = [r["mean_ma"] for r in results]
    baseline = means[0]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Guardian Gate Power Comparison (Week 7-8)", fontsize=13, fontweight="bold")

    # Plot 1: average current bar chart with % savings
    ax = axes[0]
    bars = ax.bar(modes, means, color=colors, alpha=0.82, edgecolor="black")
    ax.set_ylabel("Average Current (mA)", fontweight="bold")
    ax.set_title("Power per Gate Mode")
    ax.grid(axis="y", alpha=0.3)
    for i, (bar, val) in enumerate(zip(bars, means)):
        label = f"{val:.2f} mA"
        if i > 0:
            pct = (1 - val / baseline) * 100
            label += f"\n({pct:+.0f}%)"
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.05,
                label, ha="center", fontsize=9, fontweight="bold")

    # Plot 2: projected battery life
    ax = axes[1]
    batt_hours = [BATTERY_MAH / m for m in means]
    bars = ax.bar(modes, batt_hours, color=colors, alpha=0.82, edgecolor="black")
    ax.set_ylabel("Battery Life (hours)", fontweight="bold")
    ax.set_title(f"Projected Battery Life ({BATTERY_MAH} mAh)")
    ax.grid(axis="y", alpha=0.3)
    for bar, val in zip(bars, batt_hours):
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 1,
                f"{val:.0f}h", ha="center", fontsize=10, fontweight="bold")

    plt.tight_layout()
    results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "../../results")
    os.makedirs(results_dir, exist_ok=True)
    ts      = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    png     = os.path.join(results_dir, f"week7_power_comparison_{ts}.png")
    csv     = os.path.join(results_dir, f"week7_power_comparison_{ts}.csv")
    plt.savefig(png, dpi=300)
    print(f"\nSaved plot : {png}")

    with open(csv, "w") as f:
        f.write("mode,mean_ma,min_ma,max_ma,p99_ma,samples\n")
        for r in results:
            f.write(f"{r['label']},{r['mean_ma']:.4f},{r['min_ma']:.4f},"
                    f"{r['max_ma']:.4f},{r['p99_ma']:.4f},{r['n']}\n")
    print(f"Saved CSV  : {csv}")
    plt.show()


def main():
    print("=== Guardian Week 7-8 — Gate Power Comparison ===\n")
    print(f"Supply voltage : {VOLTAGE_MV} mV")
    print(f"Duration/phase : {DURATION_S} s")
    print(f"Battery model  : {BATTERY_MAH} mAh (CR2032)\n")
    print("You will be prompted to flash 3 firmware binaries sequentially.\n")

    port = find_ppk2_port()
    ppk2 = PPK2_API(port, timeout=1)

    results = []
    gate_modes = [
        ("GATE_MODE 0 — BASELINE (set #define GATE_MODE 0 in main.c)", 0),
        ("GATE_MODE 1 — ENERGY_VAD (set #define GATE_MODE 1 in main.c)", 1),
        ("GATE_MODE 2 — PHYSICS_GATE (set #define GATE_MODE 2 in main.c)", 2),
    ]

    for label, _ in gate_modes:
        results.append(measure_phase(ppk2, label, DURATION_S))

    print_results(results)
    plot_results(results)


if __name__ == "__main__":
    main()
