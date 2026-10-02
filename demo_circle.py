#!/usr/bin/env python3
"""Circle demo: TCP рисует круг в наклонной плоскости, инструмент +X.
Валидировано двойником (accept_loops.py + шотган): все точки dPos<=1,
хлыстов нет (continuity-биас в выборе).

Круг r=100, центр (230,0,300), нормаль (0.3,0.4,0.87) — наклон по всем
тремя осям. r=100 с X const невозможен (низ — дыра лимитов,
доказано шотганом 40 сидов), поэтому плоскость наклонная.
Порядок обратный (23..0) — так гладче (проверено симуляцией).

Использование: python demo_circle.py [/dev/ttyUSB0]
Требования: pip install pyserial
"""

import re
import sys
import time
import math

import serial

CX, CY, CZ, R, N = 230.0, 0.0, 300.0, 100.0, 24
NORMAL = (0.3, 0.4, 0.87)   # наклон по всем осям
EU = (90.0, 0.0, 90.0)      # инструмент +X


def _norm(v):
    s = math.sqrt(sum(c * c for c in v))
    return tuple(c / s for c in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


_NN = _norm(NORMAL)
_UU = _norm(_cross(_NN, (0, 0, 1)))
_VV = _cross(_NN, _UU)

STEPS = [
    "G1 X%(x).0f Y%(y).0f Z%(z).0f A%(a).1f B%(b).1f C%(c).1f F300" % {
        "x": CX + R * (_UU[0] * math.cos(2 * math.pi * k / N) +
                       _VV[0] * math.sin(2 * math.pi * k / N)),
        "y": CY + R * (_UU[1] * math.cos(2 * math.pi * k / N) +
                       _VV[1] * math.sin(2 * math.pi * k / N)),
        "z": CZ + R * (_UU[2] * math.cos(2 * math.pi * k / N) +
                       _VV[2] * math.sin(2 * math.pi * k / N)),
        "a": EU[0], "b": EU[1], "c": EU[2],
    }
    for k in list(range(N))[::-1]
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


def open_serial(port):
    """Открытие без сброса ESP32 (как в demo.py): DTR/RTS до open, пауза 15с."""
    print(f"Открываю {port} @ {BAUD} (без сброса ESP32)...")
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.1
    ser.dtr = False
    ser.rts = False
    ser.open()
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass
    ser.reset_input_buffer()
    print("Порт открыт. Ждём 15 сек (setup ESP32)...")
    time.sleep(15)
    ser.reset_input_buffer()
    print("готово.\n")
    return ser


def send(ser, cmd, timeout=120.0):
    ser.read_all()  # сброс несъеденного
    ser.write((cmd + "\n").encode())
    ser.flush()
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
    print(f"Круг r={R:.0f} в наклонной плоскости, центр=({CX:.0f},{CY:.0f},{CZ:.0f}),")
    print(f"  нормаль={tuple(round(c, 2) for c in _NN)}, инструмент +X")
    ser = open_serial(port)
    ser.read_all()
    #ok, err = send(ser, "G90")
    #print("G90:", "OK" if ok else err)
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
