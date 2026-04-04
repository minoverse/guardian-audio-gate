#!/usr/bin/env python3
"""Hardware tests 2b/2c/3/4 — run as: python3 test_hardware.py <test>
   Tests: 2b  2c  3  4
"""
import sys, serial, time

PORT = '/dev/ttyACM0'
BAUD = 115200

def capture(seconds, key):
    s = serial.Serial(PORT, BAUD, timeout=1)
    s.reset_input_buffer()
    lines = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        l = s.readline().decode('utf-8', errors='replace').strip()
        if key in l:
            lines.append(l)
    s.close()
    return lines

test = sys.argv[1] if len(sys.argv) > 1 else ''

if test == '2b':
    print("SPEAK NORMALLY into mic now (8 seconds)...")
    for l in capture(8, 'FRONTEND'): print(l)

elif test == '2c':
    print("SHOUT into mic now (8 seconds)...")
    for l in capture(8, 'FRONTEND'): print(l)

elif test == '3':
    print("SPEAK for 5s, then go QUIET for 5s...")
    for l in capture(12, 'GATE:'): print(l)

elif test == '4':
    print("SPEAK for 5s, then put FAN near mic for 5s...")
    for l in capture(12, 'GATE:'): print(l)

else:
    print("Usage: python3 test_hardware.py <2b|2c|3|4>")
