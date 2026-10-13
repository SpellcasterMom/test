#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW)
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Date        : 2026-10-05
Last modify : 2026-10-12

Endpoint-control demo: TCP=(200,200,200) стоит, инструмент идёт +X -> вниз.

12 шагов G1 (просчитаны в двойнике: dPos<=1мм на всех шагах).
После каждого шага — M115: фактическая невязка в таблицу и в лог.
В конце — итог против ворот 10мм/15°.

Старт и проверка — как в demo_circle.py: синхронизация с boot
(баннер / M105 / OK: Home + 5с отстоя), G90 только без ребута,
G28 H1 → G28, в LOG-режиме живой гироскоп M119 S=2 на время прогона,
финал — ночёвка в парке G28 H1 (try/finally, работает и при обрыве).

Использование:
    python demo_endpoint.py [/dev/ttyUSB0] [log]
    без log — только движения и таблица в консоль;
    с log — плюс лог-файл endpoint_*.log (команды + дампы M115 + #IMU).
Требования: pip install pyserial
"""

import os
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


def send(ser, cmd, timeout=120.0, imu_log=None):
    """Послать команду, ждать OK/ERROR. imu_log (файл, LOG-режим): пролетающие
    мимо #IMU-строки живого режима — туда же (иначе потеряются)."""
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
            if imu_log is not None and s.startswith("#IMU"):
                imu_log.write(s + "\n")
            if s == "OK" or s.startswith("OK:"):
                return True, ""
            if s.startswith("ERROR"):
                return False, s
        time.sleep(0.05)
    # Хвост без перевода строки (прошивка всегда шлёт \n, но дёшево проверить).
    # #IMU тут не пишем: оборванная строка испортит лог, а целый ответ
    # мы бы уже разобрали выше.
    s = buf.strip()
    if s == "OK" or s.startswith("OK:"):
        return True, ""
    if s.startswith("ERROR"):
        return False, s
    return False, "TIMEOUT"


def m115_raw(ser):
    """Сырой дамп M115 (не шлёт OK в конце)."""
    ser.read_all()
    ser.write(b"M115\n")
    ser.flush()
    time.sleep(1.0)
    return ser.read_all().decode(errors="ignore")


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
    print("Точка (200,200,200), инструмент +X -> вниз, 12 шагов G1,")
    print(f"  проверка M115 после каждого, ворота 10мм/15°, лог={'вкл' if LOG else 'выкл'}")
    if LOG:
        print(f"  лог: {log_path}")
        log.write("# endpoint TCP=(200,200,200) steps=12\n")
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

        if LOG:
            # Живой гироскоп на время прогона: пишется только при движении,
            # гаснет повторным S=2 в конце. Пролетающие строки ловим в send().
            live_on = False
            ok_live, err_live = send(ser, "M119 S=2", timeout=30.0)
            print("M119 S=2 (live):", "OK" if ok_live else err_live)
            log.write(f">> M119 S=2\n<< {'OK' if ok_live else err_live}\n")
            live_on = ok_live

        print(f"{'шаг':>4} {'dPos':>6} {'dOri':>6}  {'время':>14}  итог")
        t_start = time.time()
        fails = 0
        maxd = 0.0
        maxo = 0.0
        for k, cmd in enumerate(STEPS):
            ok, err = send(ser, cmd, timeout=60.0, imu_log=log if LOG else None)
            stamp = time.strftime("%H:%M:%S") + f" (+{time.time()-t_start:6.1f}c)"
            if LOG:
                log.write(f">> {cmd}\n<< {'OK' if ok else err}\n")
            if not ok:
                print(f"{k:>4} {'-':>6} {'-':>6}  {stamp}  {err}")
                fails += 1
                continue
            out = m115_raw(ser)
            if LOG:
                log.write(out + ("\n" if not out.endswith("\n") else ""))
            m = re.search(r"dist=([\d.]+)mm dOri=([\d.]+)deg", out)
            if m:
                d, o = float(m.group(1)), float(m.group(2))
                maxd, maxo = max(maxd, d), max(maxo, o)
                print(f"{k:>4} {d:>6.1f} {o:>6.1f}  {stamp}  OK")
            else:
                print(f"{k:>4} {'?':>6} {'?':>6}  {stamp}  moved, no RESIDUAL parsed")
        if LOG and live_on:
            ok_live, err_live = send(ser, "M119 S=2", timeout=30.0)
            print("M119 S=2 (live off):", "OK" if ok_live else err_live)
            log.write(f">> M119 S=2\n<< {'OK' if ok_live else err_live}\n")
            live_on = False
        verdict = "ПРОШЁЛ" if (fails == 0 and maxd <= 10.0 and maxo <= 15.0) else "НЕ ПРОШЁЛ"
        print(f"Итог: {len(STEPS)-fails}/{len(STEPS)} шагов, "
              f"max dPos={maxd:.1f}мм max dOri={maxo:.1f}° "
              f"(ворота 10/15) — {verdict}")
        if LOG:
            log.write(f"# итог: fails={fails} maxd={maxd:.1f} maxo={maxo:.1f} {verdict}\n")
        if fails:
            print("Есть несъеденные шаги — дальше вслепую не едем.")
            sys.exit(1)
    finally:
        # Живой режим гасим до финала (иначе парковка тоже польётся в эфир
        # и останется включённым после выхода). live_on может не существовать
        # при раннем обрыве — через locals().
        try:
            if LOG and locals().get("live_on"):
                send(ser, "M119 S=2", timeout=30.0)
                log.write(">> M119 S=2 (live off, финал)\n")
        except Exception as e:
            print("Финал: live не погашен:", e)
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
