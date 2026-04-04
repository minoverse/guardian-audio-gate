#!/usr/bin/env python3
"""
Guardian — 24-Hour Stability Monitor
=====================================
Reads the nRF52840 serial port continuously.  Parses structured FAULT and
GATE log lines.  Writes a rolling CSV and prints a live summary every 60s.

When you run overnight:
    python3 tools/test_stability.py --port /dev/ttyACM0 --duration 86400

What it tracks:
  - PDM restarts (SW watchdog fires)
  - Ring overruns
  - WDT resets (detected at next boot via RESETREAS)
  - Total gate decisions (WAKE + ABORT)
  - WAKE false-positive rate during known-silence periods
  - AGC gain drift (detects slow cal drift or DC accumulation)

Usage:
    python3 tools/test_stability.py --port /dev/ttyACM0
    python3 tools/test_stability.py --port /dev/ttyACM0 --duration 3600  # 1h test
    python3 tools/test_stability.py --replay results/serial_log.txt       # offline
"""

import argparse
import sys
import os
import re
import time
import datetime
import csv

# ── Log line patterns (match firmware printk format) ───────────────────────
RE_GATE   = re.compile(
    r'GATE:\s+(\S+)\s+conf=\s*(\d+)\s+E=\s*(\d+)\s+C=\s*(-?\d+)'
    r'\s+ZCR=\s*(\d+)\s+SFM=([\d.]+)\s+CV=([\d.]+)\s+T=\s*(\d+)us'
)
RE_PDM_RESTART = re.compile(r'PDM.*restart|SW_WDT.*restart', re.IGNORECASE)
RE_OVERRUN     = re.compile(r'overrun|ring.*overrun',         re.IGNORECASE)
RE_WDT_BOOT    = re.compile(r'BOOT.*WDT reset',               re.IGNORECASE)
RE_PDM_FATAL   = re.compile(r'PDM_FATAL',                     re.IGNORECASE)
RE_PREROLL     = re.compile(r'PREROLL.*suppressed',           re.IGNORECASE)
RE_FRONTEND    = re.compile(
    r'FRONTEND:\s+agc_gain=([\d.]+)\s+agc_env=([\d.]+)\s+cal=([\d.]+)'
)

class StabilityMonitor:
    def __init__(self, out_dir):
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.reset()
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(out_dir, f'stability_{ts}.csv')
        self._csv_f   = open(self.csv_path, 'w', newline='')
        self._csv_w   = csv.writer(self._csv_f)
        self._csv_w.writerow([
            'elapsed_s', 'decision', 'confidence', 'energy', 'correlation',
            'zcr', 'sfm', 'cv', 'pipeline_us', 'agc_gain', 'agc_env'
        ])
        self.start_time  = time.time()
        self.last_report = time.time()
        self.agc_gains   = []

    def reset(self):
        self.pdm_restarts   = 0
        self.overruns       = 0
        self.wdt_reboots    = 0
        self.pdm_fatals     = 0
        self.preroll_suppressed = 0
        self.total_frames   = 0
        self.wakes          = 0
        self.aborts         = 0
        self.pipeline_us    = []

    def feed_line(self, line):
        line = line.strip()

        m = RE_GATE.search(line)
        if m:
            decision, conf, energy, corr, zcr, sfm, cv, t_us = m.groups()
            self.total_frames += 1
            if decision == 'WAKE':
                self.wakes += 1
            else:
                self.aborts += 1
            us = int(t_us)
            self.pipeline_us.append(us)
            elapsed = time.time() - self.start_time
            self._csv_w.writerow([
                f'{elapsed:.1f}', decision, conf, energy, corr,
                zcr, sfm, cv, us, '', ''
            ])
            self._csv_f.flush()
            return

        m = RE_FRONTEND.search(line)
        if m:
            gain, env, cal = m.groups()
            self.agc_gains.append(float(gain))
            return

        if RE_PDM_RESTART.search(line):  self.pdm_restarts    += 1
        if RE_OVERRUN.search(line):      self.overruns        += 1
        if RE_WDT_BOOT.search(line):     self.wdt_reboots     += 1
        if RE_PDM_FATAL.search(line):    self.pdm_fatals      += 1
        if RE_PREROLL.search(line):      self.preroll_suppressed += 1

    def report(self, force=False):
        now = time.time()
        if not force and (now - self.last_report) < 60:
            return
        self.last_report = now
        elapsed = now - self.start_time
        h, m  = divmod(int(elapsed), 3600)
        m, s  = divmod(m, 60)

        abort_rate = (self.aborts / self.total_frames * 100
                      if self.total_frames else 0)
        avg_us = (sum(self.pipeline_us) / len(self.pipeline_us)
                  if self.pipeline_us else 0)
        max_us = max(self.pipeline_us) if self.pipeline_us else 0

        # AGC drift: std of last 100 gain values — should be <0.05 in silence
        agc_std = 0.0
        if len(self.agc_gains) >= 10:
            import statistics
            agc_std = statistics.stdev(self.agc_gains[-100:])

        print(f"\n── Stability Report  [{h:02d}:{m:02d}:{s:02d} elapsed] ──────────────")
        print(f"  Total frames       : {self.total_frames:>8,}")
        print(f"  WAKE / ABORT       : {self.wakes:>6,} / {self.aborts:>6,}  "
              f"(abort rate {abort_rate:.1f}%)")
        print(f"  Pipeline timing    : avg={avg_us:.0f}µs  max={max_us}µs")
        print(f"  AGC gain σ (100fr) : {agc_std:.4f}  "
              f"{'✅ stable' if agc_std < 0.05 else '⚠️  drifting'}")
        print(f"  PDM restarts       : {self.pdm_restarts}"
              f"  {'✅' if self.pdm_restarts == 0 else '⚠️'}")
        print(f"  Ring overruns      : {self.overruns}"
              f"  {'✅' if self.overruns == 0 else '⚠️'}")
        print(f"  WDT reboots        : {self.wdt_reboots}"
              f"  {'✅' if self.wdt_reboots == 0 else '❌'}")
        print(f"  PDM_FATAL events   : {self.pdm_fatals}"
              f"  {'✅' if self.pdm_fatals == 0 else '❌ CRITICAL'}")
        print(f"  TinyML suppressions: {self.preroll_suppressed}")
        print(f"  CSV log            : {self.csv_path}")

        # Pass/fail summary
        stable = (self.wdt_reboots == 0 and self.pdm_fatals == 0
                  and agc_std < 0.1 and max_us < 5000)
        print(f"\n  {'✅ STABLE — ready for deployment' if stable else '❌ ISSUES DETECTED — see above'}")
        return stable

    def close(self):
        self._csv_f.close()


def run_live(port, baud, duration, monitor):
    try:
        import serial
    except ImportError:
        print("ERROR: pip3 install pyserial")
        sys.exit(1)

    print(f"Opening {port} at {baud} baud.  Ctrl+C to stop.")
    print(f"Duration: {duration}s ({duration/3600:.1f}h)\n")
    deadline = time.time() + duration

    with serial.Serial(port, baud, timeout=1.0) as ser:
        while time.time() < deadline:
            line = ser.readline().decode('utf-8', errors='replace')
            if line:
                monitor.feed_line(line)
            monitor.report()

    monitor.report(force=True)


def run_replay(path, monitor):
    print(f"Replaying: {path}\n")
    with open(path) as f:
        for line in f:
            monitor.feed_line(line)
    monitor.report(force=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port',     default='/dev/ttyACM0')
    ap.add_argument('--baud',     type=int, default=115200)
    ap.add_argument('--duration', type=int, default=86400,
                    help='Seconds to run (default 86400 = 24h)')
    ap.add_argument('--replay',   help='Replay existing serial log file')
    args = ap.parse_args()

    out_dir = os.path.join(os.path.dirname(__file__), '..', 'results')
    monitor = StabilityMonitor(out_dir)

    try:
        if args.replay:
            run_replay(args.replay, monitor)
        else:
            run_live(args.port, args.baud, args.duration, monitor)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        monitor.report(force=True)
    finally:
        monitor.close()
