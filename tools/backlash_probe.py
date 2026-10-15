#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW)
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Date        : 2026-10-13
Last modify : 2026-10-13

Зонд люфтов: качает каждый сустав треугольником вокруг текущей позы
и меряет мёртвую зону гироскопом (IMU на фланце видит все 6 суставов).
Детект — по модулю |w| (ориентация монтажа не важна; бонус: в нулях
серв оси гиро сонаправлены осям робота, R_mount=I — смещён только ноль,
а для гироскопа это не значит ничего).

Как ставить руку: мачта вверх глазами/джогом (про скрипт позу не знает —
центр качания = текущая поза из M105). Вертикаль желательна (гравитация
не грузит), но ширина гистерезиса от статической просадки не зависит.

Старт и финал — как в demo_circle.py: boot-sync, G90 только без ребута
(после ребута рука в парке — мачты нет, выходим с подсказкой),
парковка G28 H1 в finally (следующий boot считает руку в парке).
Компенсация на время замера обнуляется (M400 нули — иначе подпор
компенсатора исказит замер!); в конце измеренное применяется M400
(рантайм), M500 — руками.

Использование:
    python backlash_probe.py [/dev/ttyUSB0] [P1 P3 ...] [log]
    без log — таблица в консоль; с log — плюс файл backlash_*.log.
Требования: pip install pyserial
"""

import os
import re
import sys
import time
import math

import serial

AMP = 2.0       # полуразмах качания, градусы
STEP = 0.5      # шаг, градусы (разрешение замера)
CYCLES = 2      # треугольников на сустав (реверсов: 2 на цикл)
FEED = 60       # подача качания (тихо, без звона)
SETTLE = 0.2    # отстой после шага, с (затухание звона)
BLANK = 0.1     # глухота после реверса, с (переходный)

PORT = None
JOINTS = None  # None = все P1..P6
LOG = False
for arg in sys.argv[1:]:
    low = arg.lower()
    if low == "log":
        LOG = True
    elif re.fullmatch(r"p[1-6]", low):
        JOINTS = (JOINTS or []) + [int(low[1])]
    else:
        PORT = arg  # иначе считаем портом
if JOINTS is None:
    JOINTS = [1, 2, 3, 4, 5, 6]
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
        print("  python backlash_probe.py /dev/ttyUSB0")
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


def send(ser, cmd, timeout=120.0, imu_hook=None):
    """Послать команду, ждать OK/ERROR. imu_hook(line, t) — на каждую
    прилетевшую #IMU-строку живого режима (t — время приёма)."""
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
            if imu_hook is not None and s.startswith("#IMU"):
                imu_hook(s, time.time())
            if s == "OK" or s.startswith("OK:"):
                return True, ""
            if s.startswith("ERROR"):
                return False, s
        time.sleep(0.05)
    s = buf.strip()
    if s == "OK" or s.startswith("OK:"):
        return True, ""
    if s.startswith("ERROR"):
        return False, s
    return False, "TIMEOUT"


def parse_imu_line(line):
    """#IMU ... gx=.. gy=.. gz=.. -> (gx, gy, gz) counts или None."""
    m = re.search(r"gx=(-?\d+)\s+gy=(-?\d+)\s+gz=(-?\d+)", line)
    if not m:
        return None
    return (int(m.group(1)), int(m.group(2)), int(m.group(3)))


def detect_dead(samples, reversals, thr_dps, gyro_lsb, bias):
    """Чистая функция детекта (юнит-тестируема без железа).
    samples — [(t, gx, gy, gz)] counts; reversals — [(t_rev, a_tip,
    leg=[(t_send, a_target), ...])]; thr_dps — порог модуля.
    Возвращает [(direction, dead_deg or None)] — None = отклика не было
    (мёртвая зона шире ноги, верхняя граница = её длина)."""
    bx, by, bz = bias
    mags = []
    for t, gx, gy, gz in samples:
        dx = (gx - bx) * gyro_lsb / 1000.0
        dy = (gy - by) * gyro_lsb / 1000.0
        dz = (gz - bz) * gyro_lsb / 1000.0
        mags.append((t, math.sqrt(dx * dx + dy * dy + dz * dz)))
    out = []
    for t_rev, a_tip, leg in reversals:
        direction = 1 if leg and leg[0][1] > a_tip else -1
        hit = None
        n = len(mags)
        for i in range(n - 1):
            t, m = mags[i]
            if t <= t_rev + BLANK:
                continue
            if m > thr_dps and mags[i + 1][1] > thr_dps:
                for t_send, a_tgt in leg:
                    if t_send >= t:
                        hit = abs(a_tgt - a_tip)
                        break
                else:
                    hit = abs(leg[-1][1] - a_tip) if leg else None
                break
        out.append((direction, hit))
    return out


def main():
    sys.stdout.reconfigure(line_buffering=True)
    port = PORT or find_port()
    log = None
    log_path = None
    if LOG:
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "backlash_" + time.strftime("%Y%m%d_%H%M%S") + ".log")
        log = open(log_path, "w", encoding="utf-8")
    print(f"Порт: {port}")
    print(f"Качание P{JOINTS}: ±{AMP}° шагом {STEP}°, {CYCLES} цикла, F{FEED}, лог={'вкл' if LOG else 'выкл'}")
    ser, rebooted = open_serial(port)
    ser.read_all()

    def emit(cmd, timeout=120.0):
        ok, err = send(ser, cmd, timeout)
        print(f"{cmd.split()[0] if ' ' in cmd else cmd}:", "OK" if ok else err)
        if LOG:
            log.write(f">> {cmd}\n<< {'OK' if ok else err}\n")
        return ok, err

    try:
        if rebooted:
            print("Был ребут — рука в парке, а не в мачте.")
            print("Поставь мачту вручную (джог) и перезапусти зонд.")
            sys.exit(1)
        ok, _ = emit("G90")
        if not ok:
            print("G90 не прошёл — дальше вслепую не едем.")
            sys.exit(1)
        # Обнуляем компенсацию: подпор на реверсе иначе исказит замер.
        # Старые значения нигде не прочитать — считаем, что были нули
        # (если что-то стояло — верни руками, зонд предупредит в итоге).
        ok, _ = emit("M400 " + " ".join(f"P{i}=0" for i in range(1, 7)))
        if not ok:
            print("M400 не прошёл — замер без обнуления врёт, выхожу.")
            sys.exit(1)
        # Шкалы и байас из шапки дампа (быстрый, с децимацией).
        ser.read_all()
        ser.write(b"M119 S=1 D=64\n")
        ser.flush()
        time.sleep(2.0)
        head = ser.read_all().decode(errors="ignore")
        if LOG:
            log.write(">> M119 S=1 D=64\n" + head + "\n")
        ms = re.search(r"gyro_mdps_lsb=([\d.]+)", head)
        mb = re.search(r"#BIAS dps gx=([-\d.]+) gy=([-\d.]+) gz=([-\d.]+)", head)
        if not ms or not mb:
            print("Не разобрал #SCALE/#BIAS — проверь M119 S=1 руками.")
            sys.exit(1)
        gyro_lsb = float(ms.group(1))
        bias = (float(mb.group(1)), float(mb.group(2)), float(mb.group(3)))
        print(f"Шкала гиро {gyro_lsb} mdps/LSB, байас {tuple(round(b, 2) for b in bias)} dps")
        # Центры качания — текущая поза.
        ser.read_all()
        ser.write(b"M105\n")
        ser.flush()
        time.sleep(0.5)
        m105 = ser.read_all().decode(errors="ignore")
        c = [float(v) for v in re.findall(r"P[1-6]=([-\d.]+)", m105)[:6]]
        if len(c) < 6:
            print("Не разобрал M105")
            sys.exit(1)
        print("Центры:", " ".join(f"P{i+1}={c[i]:.1f}" for i in range(6)))
        # Живой гироскоп на время замера.
        live_on = False
        ok, err = send(ser, "M119 S=2", timeout=30.0)
        print("M119 S=2 (live):", "OK" if ok else err)
        if LOG:
            log.write(f">> M119 S=2\n<< {'OK' if ok else err}\n")
        live_on = ok
        if not live_on:
            print("Live не включился (нужна прошивка с S=2) — выхожу.")
            sys.exit(1)
        samples = []

        def hook(line, t):
            g = parse_imu_line(line)
            if g:
                samples.append((t,) + g)
            if LOG:
                log.write(f"## {t:.2f} {line}\n")

        # Шум покоя 2 с: порог = среднее + 4σ, пол 1.0 dps.
        print("Шум покоя 2 с...")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            for line in ser.read_all().decode(errors="ignore").split("\n"):
                s = line.strip()
                if s.startswith("#IMU"):
                    hook(s, time.time())
            time.sleep(0.05)
        rest = []
        for t, gx, gy, gz in [(s[0], s[1], s[2], s[3]) for s in samples]:
            dx = (gx - bias[0]) * gyro_lsb / 1000.0
            dy = (gy - bias[1]) * gyro_lsb / 1000.0
            dz = (gz - bias[2]) * gyro_lsb / 1000.0
            rest.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        mean = sum(rest) / len(rest)
        var = sum((v - mean) ** 2 for v in rest) / len(rest)
        thr = max(mean + 4 * math.sqrt(var), 1.0)
        print(f"Покой: mean={mean:.2f} σ={math.sqrt(var):.2f} → порог {thr:.2f} dps (n={len(rest)})")
        if LOG:
            log.write(f"# порог {thr:.2f} dps\n")
        # Качание по суставам.
        results = {}
        n_steps = int(round(AMP / STEP))
        for j in JOINTS:
            cen = c[j - 1]
            lo, hi = cen - AMP, cen + AMP
            print(f"--- P{j} (центр {cen:.1f}) ---")
            reversals = []
            # Разгонная нога к верху (слабина неизвестно с какой стороны).
            ok, _ = send(ser, f"M104 P{j}={hi:.2f} F{FEED}", timeout=60.0,
                         imu_hook=hook)
            if not ok:
                print(f"P{j}: разгон не удался — пропускаю сустав.")
                continue
            time.sleep(SETTLE)
            tip = hi
            failed = False
            for _ in range(CYCLES):
                for tgt in (lo, hi):
                    leg = []
                    a = tip
                    while abs(tgt - a) > 1e-9:
                        a = round(a + (STEP if tgt > a else -STEP), 2)
                        if (tgt > tip and a > tgt) or (tgt < tip and a < tgt):
                            a = tgt
                        ok, _ = send(ser, f"M104 P{j}={a:.2f} F{FEED}",
                                     timeout=60.0, imu_hook=hook)
                        if not ok:
                            failed = True
                            break
                        leg.append((time.time(), a))
                        time.sleep(SETTLE)
                    if failed:
                        break
                    reversals.append((leg[0][0] if leg else time.time(), tip, leg))
                    tip = tgt
                if failed:
                    break
            res = detect_dead(samples, reversals, thr, gyro_lsb, bias)
            pos = [d for dr, d in res if dr > 0 and d is not None]
            neg = [d for dr, d in res if dr < 0 and d is not None]
            nan = sum(1 for _, d in res if d is None)
            mp = sum(pos) / len(pos) if pos else None
            mn = sum(neg) / len(neg) if neg else None
            vals = [d for _, d in res if d is not None]
            mean_dead = sum(vals) / len(vals) if vals else None
            results[j] = (mp, mn, mean_dead, len(res), nan)
            mp_s = "?" if mp is None else round(mp, 2)
            mn_s = "?" if mn is None else round(mn, 2)
            md_s = "?" if mean_dead is None else round(mean_dead, 2)
            print(f"P{j}: +: {mp_s}  -: {mn_s}  среднее: {md_s}  (реверсов {len(res)}, без отклика {nan})")
            # Возврат в центр (без детекта).
            send(ser, f"M104 P{j}={cen:.2f} F{FEED}", timeout=60.0, imu_hook=hook)
            time.sleep(SETTLE)
        if live_on:
            send(ser, "M119 S=2", timeout=30.0)
            print("M119 S=2 (live off): OK")
            live_on = False
        print("=== Итог (градусы) ===")
        sug = []
        for j in JOINTS:
            mp, mn, mean_dead, n, nan = results.get(j, (None, None, None, 0, 0))
            if mean_dead is not None:
                print(f"P{j}: люфт ≈ {mean_dead:.2f}  (+{mp:.2f} / -{mn:.2f})")
                sug.append(f"P{j}={mean_dead:.2f}")
            else:
                print(f"P{j}: люфт ≈ ? (нет данных)")
        if sug:
            line = "M400 " + " ".join(sug)
            print("Применить (рантайм, без сохранения):", line)
            ok, _ = send(ser, line, timeout=30.0)
            print("M400:", "OK" if ok else "не встал — введи руками")
            print("Сохранить навсегда: M500 (после проверки реверсом!)")
        print("Старые значения M400 занулены зондом — если что-то стояло, верни руками.")
    finally:
        try:
            if LOG and locals().get("live_on"):
                send(ser, "M119 S=2", timeout=30.0)
                log.write(">> M119 S=2 (live off, финал)\n")
        except Exception as e:
            print("Финал: live не погашен:", e)
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
