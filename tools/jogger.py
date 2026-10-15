#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW)
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Date        : 2026-10-05
Last modify : 2026-10-07

Джоггер 6-DOF руки с клавиатуры.

Пары клавиш на серву (+/-):  q/a=P1  w/s=P2  e/d=P3  r/f=P4
                              t/g=P5  y/h=P6  u/j=P7
Строчная буква — шаг 5°, ЗАГЛАВНАЯ (Shift) — шаг 25°.
`0` — все сервы в 0 (M104 P1=0 .. P6=0; P7=7 закрыт).
`~` — опросить M105 и показать что вернулось.
`?` — помощь.  Esc / Ctrl-C — выход.

Движение: M104 Pn=<угол> (едет интерполятором, ждём OK).
Углы помним локально, старт — с опроса M105.

Использование:
    python3 tools/jogger.py /dev/ttyUSB0 [baud]

ESP32 НЕ перезагружается при подключении (DTR/RTS сняты до open).
"""

import re
import select
import sys
import time

try:
    import serial
except ImportError:
    print("нужен pyserial: pip install pyserial")
    sys.exit(1)

BAUD = 921600
STEP_SMALL = 5.0
STEP_BIG = 25.0

# Лимиты math-углов (зеркало DEFAULT_SERVO_CONFIG из src/config.h):
# P1-P3 ±135, P4-P6 ±90, P7 7..38 (закрыт..открыт). Джог дальше лимита не шлёт —
# иначе локальный угол убегает от реального (прошивка режет молча).
SERVO_LIMITS = [(-135.0, 135.0)] * 3 + [(-90.0, 90.0)] * 3 + [(7.0, 38.0)]


def clamp_angle(idx, value):
    lo, hi = SERVO_LIMITS[idx]
    limited = value < lo or value > hi
    return max(lo, min(hi, value)), limited

# (плюс-клавиша, минус-клавиша) -> индекс сервы 0..6 (P1..P7)
KEYS = {
    'q': (0, +1), 'a': (0, -1),
    'w': (1, +1), 's': (1, -1),
    'e': (2, +1), 'd': (2, -1),
    'r': (3, +1), 'f': (3, -1),
    't': (4, +1), 'g': (4, -1),
    'y': (5, +1), 'h': (5, -1),
    'u': (6, +1), 'j': (6, -1),
}

HELP = """клавиши: q/a=P1 w/s=P2 e/d=P3 r/f=P4 t/g=P5 y/h=P6 u/j=P7 (+/-)
  строчная = шаг %.0f°, ЗАГЛАВНАЯ = шаг %.0f°   `0` = все в 0   `~` = M105   `?` = помощь   Esc = выход""" % (STEP_SMALL, STEP_BIG)


def open_serial(port, baud):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
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
    return ser


def send(ser, cmd, timeout):
    """Отправка + ожидание OK/ERROR (для M105 — строк с Angles:)."""
    ser.reset_input_buffer()
    ser.write((cmd.strip() + "\n").encode())
    ser.flush()
    lines = []
    start = time.time()
    while time.time() - start < timeout:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="ignore").strip()
        if not line or line.upper() == cmd.strip().upper():
            continue  # пусто / эхо
        lines.append(line)
        up = line.upper()
        if up == "OK" or up.startswith("OK:") or up.startswith("ERROR"):
            return line, lines
        if line.startswith("Angles:") and cmd.strip().upper() == "M105":
            return line, lines
    return "TIMEOUT", lines


def parse_angles(line):
    """'Angles: P1=0.0 ... P7=50.0' -> [7] float (None если нет)."""
    found = {int(m.group(1)) - 1: float(m.group(2))
             for m in re.finditer(r"P(\d)=(-?[\d.]+)", line or "")}
    return [found.get(i) for i in range(7)]


class Keyboard:
    """Один символ без Enter. Linux: termios; Windows: msvcrt."""

    def __init__(self):
        self.win = sys.platform.startswith("win")
        if not self.win:
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setraw(self.fd)
            # setraw гасит OPOST/ONLCR всего терминала — print("\n")
            # перестаёт возвращать каретку (строки «лесенкой»).
            # Возвращаем обработку ВЫВОДА, ввод остаётся raw.
            mode = termios.tcgetattr(self.fd)
            mode[1] |= termios.OPOST | termios.ONLCR
            termios.tcsetattr(self.fd, termios.TCSADRAIN, mode)

    def get(self, timeout=0.1):
        if self.win:
            import msvcrt
            if not msvcrt.kbhit():
                time.sleep(timeout)
                return None
            ch = msvcrt.getch()
            try:
                return ch.decode()
            except Exception:
                return None
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        return sys.stdin.read(1)

    def restore(self):
        if not self.win:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def drain(self):
        """Выбросить всё, что нажали пока ждали ответ (иначе команды копятся и рвут вывод)."""
        if self.win:
            import msvcrt
            while msvcrt.kbhit():
                msvcrt.getch()
            return
        import select
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                break
            sys.stdin.read(1)


def main():
    if len(sys.argv) < 2:
        print("использование: python3 tools/jogger.py PORT [baud]")
        sys.exit(1)
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD

    print(f"Открываю {port} @ {baud} (без сброса ESP32)...")
    ser = open_serial(port, baud)
    time.sleep(1.0)

    print("Спрашиваю M105 (стартовые углы)...")
    _, lines = send(ser, "M105", 8)
    for l in lines:
        print("  <", l)
    angles = parse_angles(next((l for l in lines if l.startswith("Angles:")), ""))
    if all(a is None for a in angles):
        print("M105 не ответил углами — стартую с нулей (осторожно!)")
        angles = [0.0] * 7
    else:
        angles = [a if a is not None else 0.0 for a in angles]
    print("Текущие:", " ".join(f"P{i+1}={a:.1f}" for i, a in enumerate(angles)))
    print(HELP)

    kb = Keyboard()
    try:
        while True:
            ch = kb.get(0.1)
            if ch is None:
                continue
            if ch in ('\x1b', '\x03'):  # Esc / Ctrl-C
                print("\nвыход.")
                break
            if ch == '?':
                print(HELP)
                continue
            if ch == '0':
                cmd = " ".join([f"P{i+1}=0" for i in range(6)] + ["P7=7"])
                cmd = "M104 " + cmd
                print(f">> {cmd}   (все в 0, захват закрыт)", flush=True)
                res, rlines = send(ser, cmd, 30)
                err = next((l for l in rlines if l.upper().startswith("ERROR")), None)
                if err:
                    print(f"  < {err} — углы НЕ обновлены", flush=True)
                else:
                    angles = [0.0] * 6 + [7.0]
                    print(f"  < {res}", flush=True)
                print(flush=True)
                kb.drain()
                continue
            if ch == '~' or ch == '`':
                print(">> M105", flush=True)
                _, rlines = send(ser, "M105", 8)
                for l in rlines:
                    print("  <", l, flush=True)
                got = parse_angles(next((l for l in rlines if l.startswith("Angles:")), ""))
                if not all(a is None for a in got):
                    angles = [a if a is not None else angles[i] for i, a in enumerate(got)]
                    print("Текущие:", " ".join(f"P{i+1}={a:.1f}" for i, a in enumerate(angles)), flush=True)
                print(flush=True)
                kb.drain()
                continue
            low = ch.lower()
            if low in KEYS and ch.isalpha():
                idx, direction = KEYS[low]
                step = STEP_BIG if ch.isupper() else STEP_SMALL
                target, limited = clamp_angle(idx, angles[idx] + direction * step)
                lim_mark = " [ЛИМИТ!]" if limited else ""
                cmd = f"M104 P{idx+1}={target:.1f}"
                print(f">> {cmd}   (P{idx+1}: {angles[idx]:.1f} -> {target:.1f}){lim_mark}", flush=True)
                res, rlines = send(ser, cmd, 30)
                err = next((l for l in rlines if l.upper().startswith("ERROR")), None)
                if err:
                    print(f"  < {err} — угол НЕ обновлён", flush=True)
                else:
                    angles[idx] = target
                    print(f"  < {res}", flush=True)
                print(flush=True)
                kb.drain()
    finally:
        kb.restore()


if __name__ == "__main__":
    main()
