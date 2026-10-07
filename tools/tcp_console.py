#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW)
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Date        : 2026-10-06
Last modify : 2026-10-06

TCP-консоль контроллера (мост socat/ser2net на стороне железа).
Использование:
    python3 tools/tcp_console.py HOST PORT "M105" [timeout]
    python3 tools/tcp_console.py HOST PORT          # интерактив, пустая строка = выход
Ответ собирается до OK / OK:* / ERROR / TIMEOUT (M114/M115/M105 без OK —
успех = нетривиальный вывод + тишина 1.5с).
"""

import socket
import sys
import time

DEF_TIMEOUT = 30.0
QUIET_GAP = 1.5  # тишина после данных = конец ответа без OK


def open_tcp(host, port):
    s = socket.socket()
    s.settimeout(10)
    s.connect((host, int(port)))
    return s


def transact(s, cmd, timeout=DEF_TIMEOUT):
    """Отправка + сбор ответа. Возвращает (ok, lines)."""
    want_ok = not cmd.strip().upper().startswith(("M114", "M115", "M105"))
    for skip in ("",):
        pass
    s.sendall((cmd.strip() + "\n").encode())
    lines = []
    buf = b""
    start = time.time()
    last_rx = start
    while time.time() - start < timeout:
        try:
            d = s.recv(4096)
        except socket.timeout:
            d = b""
        if d:
            last_rx = time.time()
            buf += d
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="ignore").strip().strip("\r")
                if not line:
                    continue
                # эхо моста "> CMD" пропускаем
                if line == cmd.strip() or line == "> " + cmd.strip():
                    continue
                lines.append(line)
                up = line.upper()
                if up == "OK" or up.startswith("OK:") or up.startswith("ERROR"):
                    return line, lines
        else:
            time.sleep(0.05)
        if not want_ok and lines and (time.time() - last_rx > QUIET_GAP):
            return lines[-1], lines
    return "TIMEOUT", lines


def main():
    if len(sys.argv) < 3:
        print("использование: tcp_console.py HOST PORT [\"CMD\" [timeout]]")
        sys.exit(1)
    host, port = sys.argv[1], int(sys.argv[2])
    s = open_tcp(host, port)
    if len(sys.argv) > 3:
        timeout = float(sys.argv[4]) if len(sys.argv) > 4 else DEF_TIMEOUT
        res, lines = transact(s, sys.argv[3], timeout)
        for l in lines:
            print("<", l)
        if res == "TIMEOUT":
            print("TIMEOUT")
        s.close()
        return
    print("интерактив (пустая строка = выход)")
    while True:
        try:
            cmd = input("> ")
        except EOFError:
            break
        if not cmd.strip():
            break
        res, lines = transact(s, cmd)
        for l in lines:
            print("<", l)
        if res == "TIMEOUT":
            print("TIMEOUT")
    s.close()


if __name__ == "__main__":
    main()
