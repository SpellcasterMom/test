#!/usr/bin/env python3
"""Circle demo: TCP рисует круг r=60 в плоскости YZ (X=230 const),
инструмент всё время +X. Валидировано двойником (accept_loops.py):
все 20 точек dPos<=0.9, хлыстов нет (continuity-биас в выборе).

Центр (230,0,300) выбран так, чтобы весь круг лежал в декстерусной
зоне (низко-впереди с горизонтальным инструментом — дыра лимитов,
доказано шотганом 40 сидов).

Использование: python demo_circle.py [/dev/ttyUSB0]
Требования: pip install pyserial
"""

import re
import sys
import time
import math

import serial

CX, CY, CZ, R, N = 230.0, 0.0, 300.0, 60.0, 20
EU = (90.0, 0.0, 90.0)

STEPS = [
    "G1 X%(x).0f Y%(y).0f Z%(z).0f A%(a).1f B%(b).1f C%(c).1f F300" % {
        "x": CX,
        "y": CY + R * math.cos(2 * math.pi * k / N),
        "z": CZ + R * math.sin(2 * math.pi * k / N),
        "a": EU[0], "b": EU[1], "c": EU[2],
    }
    for k in range(N)
]

PORT = sys.argv[1] if len(sys.argv) > 1 else None  # None = автопоиск
BAUD = 115200


def find_port():
    """Автопоиск порта ESP32 (как в demo.py)."""
    import serial.tools.list_ports
    cands = []
    for p in serial.tools.list_ports.comports():
        s = ((p.description or "") + (p.manufacturer or "")).lower()
        if any(k in s for k in
               ["cp210", "ch340", "ch341", "ft232", "usb-serial", "uart",
                "silicon labs", "espressif", "esp32"]):
            cands.append(p.device)
    if not cands:
        print("Порт ESP32 не найден. Укажите вручную:")
        print("  python demo_circle.py /dev/ttyUSB0")
        sys.exit(1)
    if len(cands) > 1:
        print(f"Портов несколько {cands}, беру {cands[0]}")
    return cands[0]


def send(ser, cmd, timeout=120.0):
    ser.read_all()  # сброс несъеденного
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
    sys.stdout.reconfigure(line_buffering=True)
    port = PORT or find_port()
    print(f"Порт: {port}")
    print(f"Круг: центр=({CX:.0f},{CY:.0f},{CZ:.0f}) r={R:.0f}, X const, инструмент +X")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    ser.dtr = False
    ser.rts = False
    time.sleep(8)  # ESP32 проходит setup()
    ser.read_all()
    ok, err = send(ser, "G90")
    print("G90:", "OK" if ok else err)
    print(f"{'т.':>4} {'dPos':>6} {'dOri':>6}  {'время':>14}  итог")
    t_start = time.time()
    for k, cmd in enumerate(STEPS):
        ok, err = send(ser, cmd, timeout=60.0)
        stamp = time.strftime("%H:%M:%S") + f" (+{time.time()-t_start:6.1f}c)"
        if not ok:
            print(f"{k:>4} {'-':>6} {'-':>6}  {stamp}  {err}")
            continue
        ser.read_all()
        ser.write(b"M115\n")
        time.sleep(1.0)
        out = ser.read_all().decode(errors="ignore")
        m = re.search(r"dist=([\d.]+)mm dOri=([\d.]+)deg", out)
        if m:
            print(f"{k:>4} {float(m.group(1)):>6.1f} {float(m.group(2)):>6.1f}  {stamp}  OK")
        else:
            print(f"{k:>4} {'?':>6} {'?':>6}  {stamp}  moved, no RESIDUAL parsed")
    ser.close()


if __name__ == "__main__":
    main()
