#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW)
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Date        : 2026-10-05
Last modify : 2026-10-15

Endpoint demo: TCP=(180,0,280) стоит мёртво, едут большие суставы:
P1 −41..+23°, P2 −59..+45°, P3 −123..+98° (суммарно 521° travel, запястье
свободно). Маршрут связный: 14 точек пачкой через поток M450
(S1 → G1… → S0, без остановок), подача F30 — медленно и зрелищно.
Валидация двойником: ворота 10мм/15° на точках (худший dOri 10.3°),
bulge ног ≤1.9мм, выбор ветвей совпадает с прошивкой (min-residual
+ continuity) — смена ветви исключена. Возвратной ноги нет специально
(см. историю bulge 117мм): паркуемся из конца штатным G28 H1.

14 шагов G1 F30 (связный маршрут P1–P3, валидация выше).
Диагностики нет: точки идут пачкой через поток M450
(S1 → G1… → S0, без остановок на стыках), в конце — отчёт прошивки.

Старт — как в demo_circle.py: синхронизация с boot
(баннер / M105 / OK: Home + 5с отстоя), G90 только без ребута,
G28 H1 → G28, финал — ночёвка в парке G28 H1 (try/finally,
работает и при обрыве).

Использование:
    python demo_endpoint.py [/dev/ttyUSB0] [log]
    без log — только движения; с log — плюс лог-файл endpoint_*.log (команды).
Требования: pip install pyserial
"""

import os
import sys
import time

import serial

STEPS = [
    "G1 X180 Y0 Z280 A120 B0 C90 F30",
    "G1 X180 Y0 Z280 A130 B10 C90 F30",
    "G1 X180 Y0 Z280 A140 B20 C90 F30",
    "G1 X180 Y0 Z280 A150 B30 C90 F30",
    "G1 X180 Y0 Z280 A160 B40 C90 F30",
    "G1 X180 Y0 Z280 A170 B50 C90 F30",
    "G1 X180 Y0 Z280 A170 B40 C45 F30",
    "G1 X180 Y0 Z280 A180 B30 C90 F30",
    "G1 X180 Y0 Z280 A170 B20 C45 F30",
    "G1 X180 Y0 Z280 A180 B20 C90 F30",
    "G1 X180 Y0 Z280 A170 B30 C90 F30",
    "G1 X180 Y0 Z280 A160 B30 C90 F30",
    "G1 X180 Y0 Z280 A150 B40 C90 F30",
    "G1 X180 Y0 Z280 A140 B30 C90 F30",
]

PORT = None
LOG = False
for arg in sys.argv[1:]:
    low = arg.lower()
    if low == "log":
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
        print("  python demo_endpoint.py /dev/ttyUSB0")
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
    """Послать команду, ждать OK/ERROR."""
    ser.read_all()  # сброс несъеденного
    ser.write((cmd + "\n").encode())
    ser.flush()
    t0 = time.time()
    buf = ""
    while time.time() - t0 < timeout:
        buf += ser.read_all().decode(errors="ignore")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            s = line.strip()
            if s == "OK" or s.startswith("OK:"):
                return True, ""
            if s.startswith("ERROR"):
                return False, s
        time.sleep(0.05)
    # Хвост без перевода строки (прошивка всегда шлёт \n, но дёшево проверить).
    s = buf.strip()
    if s == "OK" or s.startswith("OK:"):
        return True, ""
    if s.startswith("ERROR"):
        return False, s
    return False, "TIMEOUT"


def main():
    sys.stdout.reconfigure(line_buffering=True)  # вывод сразу, без буфера
    port = PORT or find_port()
    log = None
    log_path = None
    if LOG:
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "endpoint_" + time.strftime("%Y%m%d_%H%M%S") + ".log")
        log = open(log_path, "w", encoding="utf-8")
    print(f"Порт: {port}")
    print("Точка (180,0,280), качаем P1–P3, 14 шагов G1 F30,")
    print(f"  пачка M450 (без остановок), лог={'вкл' if LOG else 'выкл'}")
    if LOG:
        print(f"  лог: {log_path}")
        log.write("# endpoint TCP=(180,0,280) steps=14 F30 stream\n")
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
            print("Парковка не удалась — прогон не запускаю.")
            sys.exit(1)
        ok, _ = emit("G28", timeout=180.0)
        if not ok:
            print("Раскладка в home не удалась — прогон не запускаю.")
            sys.exit(1)

        # Поток: копим все точки подряд (буфер прошивки держит ~3 чанка,
        # OK придерживается сам пока нет места — чанки тут не нужны).
        t_start = time.time()
        ok, err = send(ser, "M450 S1", timeout=30.0)
        print("M450 S1:", "OK" if ok else err)
        if LOG:
            log.write(f">> M450 S1\n<< {'OK' if ok else err}\n")
        if not ok:
            print("Поток не открылся (нужна прошивка со стримингом) — выхожу.")
            sys.exit(1)
        for k, cmd in enumerate(STEPS):
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
            print("Поток не доехал — дальше вслепую не едем.")
            sys.exit(1)
        print("Поток докатан.")
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
