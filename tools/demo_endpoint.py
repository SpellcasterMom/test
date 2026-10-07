#!/usr/bin/env python3
"""Endpoint-control demo: TCP=(200,200,200) стоит, инструмент идёт +X -> вниз.

10 шагов G1 (просчитаны в двойнике: dPos<=1мм на всех шагах).
После каждого шага читает M115 и печатает фактическую расхождение.

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
    "G1 X200 Y200 Z200 A96.7 B5.9 C96.7 F300",
    "G1 X200 Y200 Z200 A104.1 B11.0 C104.1 F300",
    "G1 X200 Y200 Z200 A112.2 B15.0 C112.2 F300",
    "G1 X200 Y200 Z200 A121.0 B17.8 C121.0 F300",
    "G1 X200 Y200 Z200 A130.3 B19.3 C130.3 F300",
    "G1 X200 Y200 Z200 A139.7 B19.3 C139.7 F300",
    "G1 X200 Y200 Z200 A149.0 B17.8 C149.0 F300",
    "G1 X200 Y200 Z200 A157.8 B15.0 C157.8 F300",
    "G1 X200 Y200 Z200 A165.9 B11.0 C165.9 F300",
    "G1 X200 Y200 Z200 A173.3 B5.9 C173.3 F300",
    "G1 X200 Y200 Z200 A-180.0 B-0.0 C-180.0 F300",
]

PORT = sys.argv[1] if len(sys.argv) > 1 else None  # None = автопоиск
BAUD = 921600


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
        print("  python demo_endpoint.py /dev/ttyUSB0")
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
    ser.read_all()  # сброс несъеденного (чужой OK не должен подтвердить чужую команду)
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
    sys.stdout.reconfigure(line_buffering=True)  # вывод сразу, без буфера
    port = PORT or find_port()
    print(f"Порт: {port}")
    ser = open_serial(port)
    ser.read_all()
    ok, err = send(ser, "G90")
    print("G90:", "OK" if ok else err)
    print(f"{'шаг':>4} {'dPos':>6} {'dOri':>6}  {'время':>14}  итог")
    t_start = time.time()
    for k, cmd in enumerate(STEPS):
        ok, err = send(ser, cmd, timeout=60.0)
        stamp = time.strftime("%H:%M:%S") + f" (+{time.time()-t_start:6.1f}c)"
        if not ok:
            print(f"{k:>4} {'-':>6} {'-':>6}  {stamp}  {err}")
            continue
        ser.read_all()
        ser.write(b"M115\n")   # M115 не шлёт OK — читаем сырьём
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
