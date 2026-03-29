#!/usr/bin/env python3
import time
import sys

try:
    from ppk2_api.ppk2_api import PPK2_API
    import numpy as np
except ImportError:
    print("Install: pip3 install ppk2-api numpy --break-system-packages")
    sys.exit(1)

print("=== Guardian Week 5 Power Measurement ===\n")

# Find PPK2 port
import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())
ppk2_port = None
for p in ports:
    if 'PPK2' in p.description or 'JLink' in p.description:
        print(f"Found device: {p.device} - {p.description}")
        response = input(f"Use {p.device}? (y/n): ")
        if response.lower() == 'y':
            ppk2_port = p.device
            break

if not ppk2_port:
    print("PPK2 not found. Enter port manually (e.g., /dev/ttyACM0):")
    ppk2_port = input("Port: ")

print(f"\nConnecting to PPK2 on {ppk2_port}...")
ppk2 = PPK2_API(ppk2_port, timeout=1)
ppk2.get_modifiers()          # load calibration — required before measuring
ppk2.use_source_meter()
ppk2.set_source_voltage(3000) # 3000 mV = 3V supply

print("Powering DUT and waiting 3s for boot...")
ppk2.toggle_DUT_power("ON")
time.sleep(3)

DURATION = 30  # seconds
print(f"\n=== Measuring DMA mode for {DURATION}s ===")

all_samples = []  # values in µA
ppk2.start_measuring()
t_start = time.time()

while time.time() - t_start < DURATION:
    raw = ppk2.get_data()
    if raw != b'':
        samples, _ = ppk2.get_samples(raw)  # decode raw bytes → µA list
        all_samples.extend(samples)
    time.sleep(0.01)  # 10ms poll — PPK2 samples at ~100kHz internally

ppk2.stop_measuring()
ppk2.toggle_DUT_power("OFF")

if not all_samples:
    print("ERROR: no samples collected — check PPK2 connection")
    sys.exit(1)

arr = np.array(all_samples)          # µA
avg_ma  = np.mean(arr)  / 1000.0
min_ma  = np.min(arr)   / 1000.0
max_ma  = np.max(arr)   / 1000.0
std_ma  = np.std(arr)   / 1000.0

# Theoretical polling estimate (+15% overhead from memcpy + busy-wait wakeups)
polling_est = avg_ma * 1.15

print(f"\n{'='*50}")
print(f"Samples collected : {len(arr):,}")
print(f"DMA average       : {avg_ma:.3f} mA")
print(f"DMA min/max       : {min_ma:.3f} / {max_ma:.3f} mA")
print(f"Std deviation     : {std_ma:.3f} mA")
print(f"Polling estimate  : ~{polling_est:.2f} mA  (+15%)")
print(f"Estimated saving  : ~{avg_ma * 0.15:.2f} mA")
print(f"{'='*50}")
print(f"\nExpected DMA range: 2.6 – 3.0 mA  (UART off)")
print(f"                    3.0 – 4.0 mA  (UART on)")
