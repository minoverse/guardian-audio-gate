#!/usr/bin/env python3
"""
Guardian Week 5 Day 5 — DMA vs Polling Power Comparison
Measures both modes sequentially using PPK2 Source Meter.

Usage:
  1. Flash DMA binary   (POLL_MODE_TEST 0, default)
  2. Run this script — Phase 1 auto-measures DMA mode
  3. Script prompts you to flash POLL binary (POLL_MODE_TEST 1)
  4. Script auto-measures polling mode and prints comparison table
"""
import time
import sys

try:
    from ppk2_api.ppk2_api import PPK2_API
    import numpy as np
except ImportError:
    print("Install: pip3 install ppk2-api numpy --break-system-packages")
    sys.exit(1)

DURATION_S   = 20      # seconds per phase
VOLTAGE_MV   = 3300    # supply voltage (match prj.conf: 3.3 V)
BASELINE_MA  = 0.516   # nRF52840 sleep baseline measured Week 7 (515 µA)


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
    print(f"\n{'='*55}")
    print(f"Phase: {label}  [{duration}s]")
    print(f"{'='*55}")
    input("Press ENTER when ready to start measurement...")

    # Flush stale binary data before re-init (prevents UnicodeDecodeError)
    ppk2.ser.flushInput()
    time.sleep(0.2)
    ppk2.ser.flushInput()

    # Re-init each phase: reload calibration, reconfigure mode+voltage
    ppk2.get_modifiers()
    ppk2.use_source_meter()
    ppk2.set_source_voltage(VOLTAGE_MV)
    ppk2.toggle_DUT_power("ON")
    time.sleep(2)  # boot settle

    # Use blocking serial reads (usbipd: in_waiting is unreliable)
    ppk2.ser.timeout = 0.1   # 100 ms read timeout per call
    ppk2.ser.flushInput()

    samples = []
    ppk2.start_measuring()
    time.sleep(0.1)  # give PPK2 time to start streaming before first read
    t0 = time.time()
    while time.time() - t0 < duration:
        raw = ppk2.ser.read(4096)   # blocks up to 100 ms for data
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
    return {
        "label":   label,
        "n":       len(arr),
        "mean_ma": float(np.mean(arr)),
        "min_ma":  float(np.min(arr)),
        "max_ma":  float(np.max(arr)),
        "p99_ma":  float(np.percentile(arr, 99)),
    }


def print_results(dma, poll):
    overhead_ma  = poll["mean_ma"] - dma["mean_ma"]
    saving_pct   = overhead_ma / poll["mean_ma"] * 100
    dma_gate_ma  = dma["mean_ma"]  - BASELINE_MA
    poll_gate_ma = poll["mean_ma"] - BASELINE_MA

    print(f"\n{'='*55}")
    print("GUARDIAN DMA vs POLLING POWER COMPARISON")
    print(f"{'='*55}")
    print(f"{'':30s} {'DMA':>8s}  {'POLL':>8s}")
    print(f"{'─'*55}")
    print(f"{'Samples':30s} {dma['n']:>8,}  {poll['n']:>8,}")
    print(f"{'Average (mA)':30s} {dma['mean_ma']:>8.3f}  {poll['mean_ma']:>8.3f}")
    print(f"{'Min (mA)':30s} {dma['min_ma']:>8.3f}  {poll['min_ma']:>8.3f}")
    print(f"{'Max (mA)':30s} {dma['max_ma']:>8.3f}  {poll['max_ma']:>8.3f}")
    print(f"{'99th pct (mA)':30s} {dma['p99_ma']:>8.3f}  {poll['p99_ma']:>8.3f}")
    print(f"{'─'*55}")
    print(f"{'Baseline (sleep, mA)':30s} {BASELINE_MA:>8.3f}")
    print(f"{'Gate overhead DMA (mA)':30s} {dma_gate_ma:>8.3f}")
    print(f"{'Gate overhead POLL (mA)':30s} {poll_gate_ma:>8.3f}")
    print(f"{'─'*55}")
    print(f"{'DMA power saving':30s} {overhead_ma:>7.3f} mA  ({saving_pct:.1f}% vs POLL)")
    print(f"{'='*55}")

    verdict = "✅ DMA saves power" if overhead_ma > 0.1 else "⚠  Difference < 0.1 mA — check setup"
    print(f"\nVerdict: {verdict}")

    # Save CSV for records
    import os, datetime
    results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(results_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(results_dir, f"power_compare_{ts}.csv")
    with open(csv_path, "w") as f:
        f.write("mode,mean_ma,min_ma,max_ma,p99_ma,samples\n")
        for r in [dma, poll]:
            f.write(f"{r['label']},{r['mean_ma']:.4f},{r['min_ma']:.4f},"
                    f"{r['max_ma']:.4f},{r['p99_ma']:.4f},{r['n']}\n")
    print(f"Saved: {csv_path}")


def main():
    print("=== Guardian Week 5 Day 5 — DMA vs Polling Power Compare ===\n")
    print(f"Supply voltage : {VOLTAGE_MV} mV")
    print(f"Duration/phase : {DURATION_S} s")
    print(f"Baseline       : {BASELINE_MA:.3f} mA (nRF52840 sleep)\n")

    port = find_ppk2_port()
    ppk2 = PPK2_API(port, timeout=1)  # each measure_phase re-inits fully

    # ── Phase 1: DMA mode ─────────────────────────────────────────────────────
    print("\nPhase 1 — DMA mode (POLL_MODE_TEST 0)")
    print("Make sure firmware with POLL_MODE_TEST=0 is flashed.")
    dma_result = measure_phase(ppk2, "DMA", DURATION_S)

    # ── Phase 2: Polling mode ─────────────────────────────────────────────────
    print("\nPhase 2 — Polling mode (POLL_MODE_TEST 1)")
    print("Steps:")
    print("  1. Disconnect PPK2 VDD wire from nRF52840 P2 header")
    print("  2. Connect USB to nRF52840-DK")
    print("  3. Edit main.c: #define POLL_MODE_TEST 1")
    print("  4. west build -b nrf52840dk/nrf52840 && west flash")
    print("  5. Disconnect USB, reconnect PPK2 VDD wire")
    poll_result = measure_phase(ppk2, "POLL", DURATION_S)

    # ── Results ───────────────────────────────────────────────────────────────
    print_results(dma_result, poll_result)


if __name__ == "__main__":
    main()
