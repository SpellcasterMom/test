#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW)
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Date        : 2026-10-05
Last modify : 2026-10-05

Circle demo: TCP рисует круг в наклонной плоскости.
Круг r=100, центр (230,0,300), нормаль (0.3,0.4,0.87).
Порядок обратный (23..0) — так гладче (проверено симуляцией).

Пресеты ориентации (второй аргумент, по умолчанию tilt):
  tilt  — tilt-профиль A/B/C под каждую точку (трекинг-модель двойника:
          0 отказов, пик 9.4°; строгий +X на дуге 14-21 упирается в лимиты
          запястья — см. MATH.md §3 и логи);
          ЖЕЛЕЗО 05.10 (circle_20261005_170235.log): 24/24 OK, 0 отказов,
          пик dOri=0.3° — лучше прогноза (полный набор кандидатов FW
          сильнее жадного грида модели);
  plusx — фиксированный инструмент +X (90,0,90): воспроизводит отказы
          No accurate safe IK на дуге (позиция везде <=1мм, падает только
          ориентация) — демо честных ворот 10мм/15°.

Каждый шаг: G1 -> M115 целиком в лог-файл + dPos/dOri в консоль.

Использование:
    python demo_circle.py [/dev/ttyUSB0] [tilt|plusx]
Требования: pip install pyserial
"""

import os
import re
import sys
import time
import math

import serial

CX, CY, CZ, R, N = 230.0, 0.0, 300.0, 100.0, 24
NORMAL = (0.3, 0.4, 0.87)   # наклон по всем осям
EU_PLUSX = (90.0, 0.0, 90.0)  # инструмент строго +X
# Tilt-профиль (A,B,C) на 24 точки (индексы 0..23 = k 23..0):
# подобран трекинг-моделью двойника (совпала с железом до 0.1°):
# на каждой точке сетка A{85,90,95,100}xB{-20..20} от предыдущей позы,
# минимум dPos+ax+0.02*jump в воротах 10/15.
EU_TILT = [
    (100.0, -10.0, 90.0), (100.0, -10.0, 90.0),
    (100.0, -10.0, 90.0), (100.0, -10.0, 90.0),
    (100.0, -20.0, 90.0), (95.0, -20.0, 90.0),
    (95.0, -20.0, 90.0), (90.0, -20.0, 90.0),
    (90.0, -20.0, 90.0), (90.0, -10.0, 90.0),
    (90.0, -20.0, 90.0), (100.0, -10.0, 90.0),
    (100.0, -20.0, 90.0), (100.0, -20.0, 90.0),
    (100.0, 10.0, 90.0), (100.0, 10.0, 90.0),
    (100.0, 10.0, 90.0), (100.0, 10.0, 90.0),
    (100.0, 10.0, 90.0), (100.0, 0.0, 90.0),
    (100.0, 0.0, 90.0), (100.0, 0.0, 90.0),
    (100.0, -10.0, 90.0), (100.0, -10.0, 90.0),
]


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

def build_steps(eu_list):
    return [
        "G1 X%(x).0f Y%(y).0f Z%(z).0f A%(a).1f B%(b).1f C%(c).1f F300" % {
            "x": CX + R * (_UU[0] * math.cos(2 * math.pi * k / N) +
                           _VV[0] * math.sin(2 * math.pi * k / N)),
            "y": CY + R * (_UU[1] * math.cos(2 * math.pi * k / N) +
                           _VV[1] * math.sin(2 * math.pi * k / N)),
            "z": CZ + R * (_UU[2] * math.cos(2 * math.pi * k / N) +
                           _VV[2] * math.sin(2 * math.pi * k / N)),
            "a": eu_list[i][0], "b": eu_list[i][1], "c": eu_list[i][2],
        }
        for i, k in enumerate(list(range(N))[::-1])
    ]


PRESETS = {
    "tilt": [EU_TILT[i] for i in range(N)],
    "plusx": [EU_PLUSX] * N,
}

PORT = None
PRESET = "tilt"
for arg in sys.argv[1:]:
    if arg.lower() in PRESETS:
        PRESET = arg.lower()
    else:
        PORT = arg  # иначе считаем портом
if PORT is None:
    PORT = None  # автопоиск в main()
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
            if s == "OK" or s.startswith("OK:"):
                return True, ""
            if s.startswith("ERROR"):
                return False, s
        time.sleep(0.05)
    return False, "TIMEOUT"


def main():
    sys.stdout.reconfigure(line_buffering=True)
    port = PORT or find_port()
    steps = build_steps(PRESETS[PRESET])
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "circle_" + time.strftime("%Y%m%d_%H%M%S") + ".log")
    log = open(log_path, "w", encoding="utf-8")
    print(f"Порт: {port}")
    print(f"Круг r={R:.0f} в наклонной плоскости, центр=({CX:.0f},{CY:.0f},{CZ:.0f}),")
    print(f"  нормаль={tuple(round(c, 2) for c in _NN)}, пресет={PRESET}")
    print(f"  лог M115: {log_path}")
    log.write(f"# preset={PRESET} r={R} center=({CX},{CY},{CZ})\n")
    ser = open_serial(port)
    ser.read_all()
    ok, err = send(ser, "G90")
    print("G90:", "OK" if ok else err)
    log.write(f">> G90\n<< {'OK' if ok else err}\n")
    print(f"{'т.':>4} {'dPos':>6} {'dOri':>6}  {'время':>14}  итог")
    t_start = time.time()
    for k, cmd in enumerate(steps):
        ok, err = send(ser, cmd, timeout=60.0)
        stamp = time.strftime("%H:%M:%S") + f" (+{time.time()-t_start:6.1f}c)"
        log.write(f">> {cmd}\n")
        if not ok:
            print(f"{k:>4} {'-':>6} {'-':>6}  {stamp}  {err}")
            log.write(f"<< {err}\n")
            continue
        ser.read_all()
        ser.write(b"M115\n")   # M115 не шлёт OK — читаем сырьём
        time.sleep(1.0)
        out = ser.read_all().decode(errors="ignore")
        log.write(out + ("\n" if not out.endswith("\n") else ""))
        m = re.search(r"dist=([\d.]+)mm dOri=([\d.]+)deg", out)
        if m:
            print(f"{k:>4} {float(m.group(1)):>6.1f} {float(m.group(2)):>6.1f}  {stamp}  OK")
        else:
            print(f"{k:>4} {'?':>6} {'?':>6}  {stamp}  moved, no RESIDUAL parsed")
    log.close()
    print(f"Лог записан: {log_path}")
    ser.close()


if __name__ == "__main__":
    main()
