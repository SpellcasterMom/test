#!/usr/bin/env python3
"""Endpoint-control demo: TCP=(200,200,200) стоит, инструмент идёт +X -> вниз.

10 шагов G1 (просчитаны в двойнике: dPos<=1мм на всех шагах).
После каждого шага читает M115 и печатает фактическую невязку.

Использование:
    python demo_endpoint.py                 # порт по умолчанию
    python demo_endpoint.py /dev/ttyUSB0    # Linux
    python demo_endpoint.py COM11           # Windows

Требования: pip install pyserial
"""

import re
import sys
import time

import serial

STEPS = [
    "G1 X200 Y200 Z200 A90.0 B-0.0 C90.0 F300",
    "G1 X200 Y200 Z200 A98.2 B7.1 C98.2 F300",
    "G1 X200 Y200 Z200 A107.6 B12.9 C107.6 F300",
    "G1 X200 Y200 Z200 A118.0 B17.0 C118.0 F300",
    "G1 X200 Y200 Z200 A129.2 B19.2 C129.2 F300",
    "G1 X200 Y200 Z200 A140.8 B19.2 C140.8 F300",
    "G1 X200 Y200 Z200 A152.0 B17.0 C152.0 F300",
    "G1 X200 Y200 Z200 A162.4 B12.9 C162.4 F300",
    "G1 X200 Y200 Z200 A171.8 B7.1 C171.8 F300",
    "G1 X200 Y200 Z200 A-180.0 B-0.0 C-180.0 F300",
]

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
BAUD = 115200


def send(ser, cmd, timeout=30.0):
    ser.write((cmd + "\n").encode())
    t0 = time.time()
    buf = ""
    while time.time() - t0 < timeout:
        buf += ser.read_all().decode(errors="ignore")
        for line in buf.split("\n"):
            s = line.strip()
            if s == "OK":
                return True, ""
            if s.startswith("ERROR"):
                return False, s
        time.sleep(0.05)
    return False, "TIMEOUT"


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.dtr = False
    ser.rts = False
    time.sleep(8)  # ESP32 проходит setup()
    ser.read_all()
    ok, err = send(ser, "G90")
    print("G90:", "OK" if ok else err)
    print(f"{'шаг':>4} {'dPos':>6} {'dOri':>6}  итог")
    for k, cmd in enumerate(STEPS):
        ok, err = send(ser, cmd, timeout=60.0)
        if not ok:
            print(f"{k:>4} {'-':>6} {'-':>6}  {err}")
            continue
        ser.read_all()
        ser.write(b"M115\n")   # M115 не шлёт OK — читаем сырьём
        time.sleep(1.0)
        out = ser.read_all().decode(errors="ignore")
        m = re.search(r"dist=([\d.]+)mm dOri=([\d.]+)deg", out)
        if m:
            print(f"{k:>4} {float(m.group(1)):>6.1f} {float(m.group(2)):>6.1f}  OK")
        else:
            print(f"{k:>4} {'?':>6} {'?':>6}  moved, no RESIDUAL parsed")
    ser.close()


if __name__ == "__main__":
    main()
