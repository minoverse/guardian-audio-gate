#!/usr/bin/env python3
"""
Guardian — Instruction-Level Power Model

Computes theoretical battery life from nRF52840 datasheet values and measured
gate timing, without requiring P22 isolation or SB40 removal.

Sources:
  nRF52840 PS v1.7, Section 6.1 "Electrical specification"
  - CPU Active (Flash, 64 MHz, Vdd=3.3V): 4.8 mA
  - System ON Idle (no RAM retention):     0.7 µA  (0.0007 mA)
  - PDM peripheral active:                 0.5 mA  (Table 68 — PDM current)
  - DMA (EasyDMA) active:                  0.2 mA  (EasyDMA overhead)
  - HFXO (crystal oscillator, required):   0.5 mA

Measured values (from serial logs):
  Gate execution time (WCET): 1,515 µs
  Gate average time:          1,467 µs
  Frame period:              20,000 µs
  Frame budget:              20,000 µs (320 samples @ 16 kHz)

Usage:
    python3 power_model.py
"""
import os
import math
import datetime

try:
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False

# ── nRF52840 datasheet values (mA) ────────────────────────────────────────────
I_CPU_ACTIVE_MA   = 4.8      # CPU running from Flash @ 64 MHz, Vdd=3.3V
I_SLEEP_MA        = 0.0007   # System ON idle (WFE, no RAM retention)
I_PDM_MA          = 0.5      # PDM peripheral always active (audio capture)
I_DMA_MA          = 0.2      # EasyDMA overhead during PDM transfer
I_HFXO_MA         = 0.5      # High-frequency crystal oscillator (required for PDM)

# ── Measured timing (µs) ──────────────────────────────────────────────────────
T_GATE_AVG_US     = 1467     # gate_decide() average measured execution time
T_GATE_WCET_US    = 1515     # worst-case execution time (Phase 1, 1572 frames)
T_FRAME_US        = 20000    # frame period (320 samples @ 16 kHz)
T_TINYML_US       = 100000   # mock TinyML (100ms busy-wait)

# Battery model
BATTERY_MAH       = 200      # CR2032 ~200 mAh  (or 1000 mAh for 18650)
BATTERY_MAH_LIPO  = 1000     # LiPo 1000 mAh

# Gate modes
ABORT_RATES = {
    'Quiet room (99.4% abort)':  0.994,
    'Subway noise (70% abort)':  0.70,
    'Worst case (0% abort)':     0.00,
}


# ── Energy per frame (µC = µA·s) ─────────────────────────────────────────────

def compute_frame_energy_uc(abort_rate, gate_avg_us, tinyml_us):
    """
    Compute charge consumed per 20ms frame (µC).

    Always-on components:
      PDM + DMA + HFXO run the entire frame (audio capture is continuous).

    CPU active during:
      - Gate decision (gate_avg_us every frame)
      - TinyML inference (tinyml_us, only when gate does NOT abort)

    CPU idle (WFE sleep) during:
      - Remainder of frame
    """
    t_frame_s = T_FRAME_US * 1e-6

    # Always-on peripherals (full frame)
    q_peripherals = (I_PDM_MA + I_DMA_MA + I_HFXO_MA) * 1e3 * t_frame_s  # µC

    # CPU: gate decision every frame (µC)
    t_gate_s = gate_avg_us * 1e-6
    q_gate   = I_CPU_ACTIVE_MA * 1e3 * t_gate_s  # µC

    # CPU: TinyML only on wake events
    wake_rate = 1.0 - abort_rate
    t_tinyml_s = tinyml_us * 1e-6
    q_tinyml   = I_CPU_ACTIVE_MA * 1e3 * t_tinyml_s * wake_rate  # µC

    # CPU sleep (remaining frame time)
    t_sleep_s = t_frame_s - t_gate_s - (t_tinyml_s * wake_rate)
    t_sleep_s = max(t_sleep_s, 0.0)
    q_sleep   = I_SLEEP_MA * 1e3 * t_sleep_s  # µC

    q_total = q_peripherals + q_gate + q_tinyml + q_sleep
    return q_total, {
        'peripherals': q_peripherals,
        'gate':        q_gate,
        'tinyml':      q_tinyml,
        'sleep':       q_sleep,
    }


def battery_life_hours(q_per_frame_uc, battery_mah):
    """Battery life in hours given µC per frame and mAh capacity."""
    frames_per_sec  = 1.0 / (T_FRAME_US * 1e-6)          # 50 frames/s
    current_ma      = q_per_frame_uc * 1e-3 * frames_per_sec  # mA average
    return battery_mah / current_ma, current_ma


def main():
    print("=" * 65)
    print("GUARDIAN — INSTRUCTION-LEVEL POWER MODEL")
    print("nRF52840 datasheet + measured gate timing")
    print("=" * 65)

    print(f"\nDatasheet constants (nRF52840 PS v1.7):")
    print(f"  CPU active (64 MHz, flash) : {I_CPU_ACTIVE_MA:.1f} mA")
    print(f"  CPU sleep (System ON idle) : {I_SLEEP_MA*1000:.1f} µA")
    print(f"  PDM peripheral             : {I_PDM_MA:.1f} mA")
    print(f"  EasyDMA                    : {I_DMA_MA:.1f} mA")
    print(f"  HFXO oscillator            : {I_HFXO_MA:.1f} mA")
    print(f"  Always-on total            : {I_PDM_MA+I_DMA_MA+I_HFXO_MA:.1f} mA")

    print(f"\nMeasured timing:")
    print(f"  Gate avg     : {T_GATE_AVG_US:,} µs")
    print(f"  Gate WCET    : {T_GATE_WCET_US:,} µs")
    print(f"  Frame period : {T_FRAME_US:,} µs (50 frames/s)")
    print(f"  TinyML mock  : {T_TINYML_US//1000:,} ms")

    print(f"\nFrame duty cycle:")
    print(f"  Gate: {T_GATE_AVG_US/T_FRAME_US*100:.2f}% CPU")
    print(f"  Idle: {(T_FRAME_US-T_GATE_AVG_US)/T_FRAME_US*100:.2f}% sleep")

    print(f"\n{'='*65}")
    print(f"BATTERY LIFE PROJECTIONS")
    print(f"{'='*65}")
    header = f"{'Scenario':<32} {'Avg (mA)':>8}  {'CR2032 200mAh':>14}  {'LiPo 1000mAh':>13}"
    print(header)
    print(f"{'─'*65}")

    rows = []
    for label, abort_rate in ABORT_RATES.items():
        q_uc, breakdown = compute_frame_energy_uc(abort_rate, T_GATE_AVG_US, T_TINYML_US)
        life_cr, avg_ma = battery_life_hours(q_uc, BATTERY_MAH)
        life_lipo, _    = battery_life_hours(q_uc, BATTERY_MAH_LIPO)
        print(f"{label:<32} {avg_ma:>8.3f}  {life_cr:>11.0f}h  {life_lipo:>11.0f}h")
        rows.append({
            'label':     label,
            'abort_rate': abort_rate,
            'avg_ma':    avg_ma,
            'life_cr':   life_cr,
            'life_lipo': life_lipo,
            'breakdown': breakdown,
            'q_uc':      q_uc,
        })

    print(f"{'─'*65}")
    print(f"\nPower breakdown — quiet room (99.4% abort):")
    q_uc, bd = compute_frame_energy_uc(0.994, T_GATE_AVG_US, T_TINYML_US)
    total_q = sum(bd.values())
    for name, val in bd.items():
        print(f"  {name:<15}: {val/total_q*100:5.1f}%  ({val:.4f} µC/frame)")
    print(f"  {'Total':<15}: {'100.0%':>6}  ({total_q:.4f} µC/frame)")

    _, avg_ma_quiet = battery_life_hours(q_uc, BATTERY_MAH)
    print(f"\nAverage current (quiet room): {avg_ma_quiet:.3f} mA")
    print(f"  = {avg_ma_quiet*1000:.0f} µA — dominated by PDM+DMA+HFXO ({I_PDM_MA+I_DMA_MA+I_HFXO_MA:.1f} mA always-on)")

    print(f"\nGate power saving (quiet vs worst-case):")
    _, avg_worst = battery_life_hours(
        compute_frame_energy_uc(0.00, T_GATE_AVG_US, T_TINYML_US)[0], BATTERY_MAH)
    saving_pct = (avg_worst - avg_ma_quiet) / avg_worst * 100
    life_quiet, _ = battery_life_hours(q_uc, BATTERY_MAH)
    life_worst, _ = battery_life_hours(
        compute_frame_energy_uc(0.00, T_GATE_AVG_US, T_TINYML_US)[0], BATTERY_MAH)
    print(f"  Worst case (no abort): {avg_worst:.3f} mA → {life_worst:.0f}h on CR2032")
    print(f"  Quiet room (99% abort): {avg_ma_quiet:.3f} mA → {life_quiet:.0f}h on CR2032")
    print(f"  Gate extends battery life {life_quiet/life_worst:.1f}× in quiet environments")

    print(f"{'='*65}")

    # ── Save results ──────────────────────────────────────────────────────────
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../results')
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(out_dir, f'power_model_{ts}.csv')
    png_path = os.path.join(out_dir, f'power_model_{ts}.png')

    import csv
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['scenario', 'abort_rate', 'avg_ma', 'life_cr2032_h', 'life_lipo_h', 'q_uc_per_frame'])
        for r in rows:
            w.writerow([r['label'], r['abort_rate'], f"{r['avg_ma']:.4f}",
                        f"{r['life_cr']:.1f}", f"{r['life_lipo']:.1f}", f"{r['q_uc']:.6f}"])
    print(f"\nCSV: {csv_path}")

    if not HAS_PLOT:
        return

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Guardian Power Model — nRF52840 Datasheet + Measured Timing',
                 fontsize=13, fontweight='bold')

    labels    = [r['label'].split('(')[0].strip() for r in rows]
    avg_mas   = [r['avg_ma'] for r in rows]
    life_crs  = [r['life_cr'] for r in rows]
    life_lipos= [r['life_lipo'] for r in rows]
    colors    = ['#2ca02c', '#ff7f0e', '#d62728']

    # Average current
    ax = axes[0]
    bars = ax.bar(labels, avg_mas, color=colors, alpha=0.82, edgecolor='black')
    ax.set_ylabel('Average current (mA)')
    ax.set_title('Current Consumption vs Gate Mode')
    ax.grid(axis='y', alpha=0.3)
    for bar, val in zip(bars, avg_mas):
        ax.text(bar.get_x() + bar.get_width()/2, val + 0.01,
                f'{val:.3f}', ha='center', fontsize=8, fontweight='bold')

    # CR2032 battery life
    ax = axes[1]
    bars = ax.bar(labels, life_crs, color=colors, alpha=0.82, edgecolor='black')
    ax.set_ylabel('Battery life (hours)')
    ax.set_title(f'CR2032 {BATTERY_MAH} mAh Battery Life')
    ax.grid(axis='y', alpha=0.3)
    for bar, val in zip(bars, life_crs):
        days = val / 24
        ax.text(bar.get_x() + bar.get_width()/2, val + 1,
                f'{val:.0f}h\n({days:.0f}d)', ha='center', fontsize=8, fontweight='bold')

    # Power breakdown pie (quiet room)
    ax = axes[2]
    q_uc, bd = compute_frame_energy_uc(0.994, T_GATE_AVG_US, T_TINYML_US)
    pie_labels = ['PDM+DMA+HFXO\n(always-on)', 'Gate decision\n(CPU active)',
                  'TinyML\n(0.6% wake rate)', 'CPU sleep\n(WFE idle)']
    pie_vals   = [bd['peripherals'], bd['gate'], bd['tinyml'], bd['sleep']]
    pie_colors = ['#d62728', '#1f77b4', '#ff7f0e', '#2ca02c']
    wedges, texts, autotexts = ax.pie(pie_vals, labels=pie_labels,
                                       autopct='%1.1f%%', colors=pie_colors,
                                       startangle=90)
    ax.set_title('Power Breakdown — Quiet Room\n(99.4% abort rate)')

    plt.tight_layout()
    plt.savefig(png_path, dpi=300)
    print(f"Plot: {png_path}")
    plt.show()


if __name__ == '__main__':
    main()
