#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW)
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Date        : 2026-10-05
Last modify : 2026-10-11

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

Режим исполнения — поток M450 (стриминг, прошивка feat-stream):
  M450 S1 → все точки подряд (OK придерживается пока нет места) →
  M450 S0 (докачать, исполнить без остановок, встать).
Буфер — 180 точек (~3 авто-чанка по 60), голодание — hold-and-resume.
В конце — отчёт M115 (STREAM + CHUNKS + RESIDUAL).

Использование:
    python demo_circle.py [/dev/ttyUSB0] [tilt|plusx] [log]
    без log — только движения, без файла;
    с log — плюс лог-файл (команды + батч-отчёт M115).
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
LOG = False
for arg in sys.argv[1:]:
    low = arg.lower()
    if low in PRESETS:
        PRESET = low
    elif low == "log":
        LOG = True
    else:
        PORT = arg  # иначе считаем портом
if PORT is None:
    PORT = None  # автопоиск в main()
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
        print("  python demo_circle.py /dev/ttyUSB0")
        sys.exit(1)
    if len(cands) > 1:
        print(f"Портов несколько {cands}, беру {cands[0]}")
    return cands[0]


def open_serial(port):
    """Открытие без лишнего сброса + СИНХРОНИЗАЦИЯ со стартом контроллера.
    Возвращает (ser, rebooted). Открытие порта часто ребутит ESP32 —
    тогда ждём конца boot-драйва, иначе команды уходят в пустоту."""
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
    buf = ""
    t0 = time.time()
    rebooted = False
    while time.time() - t0 < 3.0:
        buf += ser.read_all().decode(errors="ignore")
        if "6-DOF Robot Arm Controller" in buf:
            rebooted = True
            break
        time.sleep(0.05)
    if not rebooted:
        ser.write(b"M105\n")
        ser.flush()
        t0 = time.time()
        alive = False
        while time.time() - t0 < 8.0:
            buf += ser.read_all().decode(errors="ignore")
            if "Angles:" in buf or "6-DOF Robot Arm Controller" in buf:
                if "6-DOF Robot Arm Controller" in buf:
                    rebooted = True
                else:
                    alive = True
                break
            time.sleep(0.05)
        if alive:
            print("Контроллер жив, ребута не было — короткая пауза и едем.")
            time.sleep(2.0)
            ser.reset_input_buffer()
            print("готово.\n")
            return ser, False
        print("Живого ответа нет — ждём конца boot-драйва (OK: Home)...")
        rebooted = True
    print("Контроллер ребутнулся — ждём конца boot-драйва (OK: Home)...")
    t0 = time.time()
    while time.time() - t0 < 90.0:
        buf += ser.read_all().decode(errors="ignore")
        if "OK: Home" in buf:
            break
        time.sleep(0.1)
    else:
        print("Boot не завершился за 90с — вслепую не едем.")
        ser.close()
        sys.exit(1)
    print("Boot готов. Отстой 5с (refresh + калибровка IMU)...")
    time.sleep(5.0)
    ser.reset_input_buffer()
    print("готово.\n")
    return ser, True


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


def m115_raw(ser):
    """Сырой дамп M115 (не шлёт OK в конце)."""
    ser.read_all()
    ser.write(b"M115\n")
    ser.flush()
    time.sleep(1.0)
    return ser.read_all().decode(errors="ignore")


def main():
    sys.stdout.reconfigure(line_buffering=True)
    port = PORT or find_port()
    steps = build_steps(PRESETS[PRESET])
    log = None
    log_path = None
    if LOG:
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "circle_" + time.strftime("%Y%m%d_%H%M%S") + ".log")
        log = open(log_path, "w", encoding="utf-8")
    print(f"Порт: {port}")
    print(f"Круг r={R:.0f} в наклонной плоскости, центр=({CX:.0f},{CY:.0f},{CZ:.0f}),")
    print(f"  нормаль={tuple(round(c, 2) for c in _NN)}, пресет={PRESET}, лог={'вкл' if LOG else 'выкл'}")
    if LOG:
        print(f"  лог: {log_path}")
        log.write(f"# preset={PRESET} r={R} center=({CX},{CY},{CZ})\n")
    ser, rebooted = open_serial(port)
    ser.read_all()

    def emit(cmd, timeout=120.0):
        """Отправить + отразить в лог. Возвращает (ok, err)."""
        ok, err = send(ser, cmd, timeout)
        print(f"{cmd.split()[0] if ' ' in cmd else cmd}:", "OK" if ok else err)
        if LOG:
            log.write(f">> {cmd}\n<< {'OK' if ok else err}\n")
        return ok, err

    if not rebooted:
        # Ребута не было — режим мог остаться REL: G90 явно.
        # После свежего boot и так ABS — не нужен.
        ok, _ = emit("G90")
        if not ok:
            print("G90 не прошёл — дальше вслепую не едем.")
            if log:
                log.close()
            ser.close()
            sys.exit(1)
    # Старт всегда из парка, затем раскладка в home (декартов трекинг
    # не стартует из сложенной позы; парк→home — штатный путь boot).
    try:
        ok, _ = emit("G28 H1", timeout=180.0)
        if not ok:
            print("Парковка не удалась — круг не запускаю.")
            sys.exit(1)
        ok, _ = emit("G28", timeout=180.0)
        if not ok:
            print("Раскладка в home не удалась — круг не запускаю.")
            sys.exit(1)

        # Поток: копим все точки подряд (буфер прошивки держит ~3 чанка,
        # OK придерживается сам пока нет места — чанки тут не нужны).
        t_start = time.time()
        nok = True
        ok, err = send(ser, "M450 S1", timeout=30.0)
        print("M450 S1:", "OK" if ok else err)
        if LOG:
            log.write(f">> M450 S1\n<< {'OK' if ok else err}\n")
        if not ok:
            print("Поток не открылся (нужна прошивка со стримингом) — выхожу.")
            sys.exit(1)
        for k, cmd in enumerate(steps):
            ok, err = send(ser, cmd, timeout=120.0)
            stamp = time.strftime("%H:%M:%S") + f" (+{time.time()-t_start:6.1f}c)"
            if LOG:
                log.write(f">> {cmd}\n<< {'OK' if ok else err}\n")
            if not ok:
                print(f"{k:>4}  ERROR {stamp}  {err} — поток сброшен, выхожу.")
                sys.exit(1)
            print(f"{k:>4}  buffered  {stamp}")
        # Барьер: докачать, исполнить всё без остановок, встать.
        print("Исполняю поток...")
        ok, err = send(ser, "M450 S0", timeout=300.0)
        print("M450 S0:", "OK" if ok else err)
        if LOG:
            log.write(f">> M450 S0\n<< {'OK' if ok else err}\n")
        if not ok:
            print("Поток не доехал — смотрю M115.")
            nok = False
        out = m115_raw(ser)
        if LOG:
            log.write(out + ("\n" if not out.endswith("\n") else ""))
        m = re.search(r"dist=([\d.]+)mm dOri=([\d.]+)deg", out)
        st = re.search(r"STREAM:\s*state=(\w+).*starved=(\d+)", out)
        print("M115:", (f"stream {st.group(1)} starved={st.group(2)}, " if st else "") +
              (f"dist={m.group(1)}mm dOri={m.group(2)}deg" if m else "без RESIDUAL"))
        if not nok:
            print("Круг не докатан — дальше вслепую не едем.")
            sys.exit(1)
    finally:
        # Ночуем в парке: следующий boot считает руку в G28 H1.
        # Работает и при обрыве: лишний G28 H1 из парка — no-op.
        try:
            ok, err = send(ser, "G28 H1", timeout=180.0)
            print("Финал G28 H1:", "OK" if ok else err)
            if LOG:
                log.write(f">> G28 H1 (финал)\n<< {'OK' if ok else err}\n")
        except Exception as e:
            print("Финал: запарковать не вышло:", e)
        if log:
            log.close()
            print(f"Лог записан: {log_path}")
        ser.close()


if __name__ == "__main__":
    main()
