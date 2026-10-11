#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW) — MCP transport
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Group       : UTS/b-23-1-o
Mentor      : Lipko I.Y.
Date        : 2026-10-11
Last modify : 2026-10-11

Транспорт MCP-сервера к контроллеру: Serial (pyserial) или TCP-мост
(socat на машине с железом). Семантика ответов 1:1 с tools/demo.py и
tools/tcp_console.py:
  - терминаторы OK / OK:* / ERROR;
  - M105/M114/M115 без OK — успех = нетривиальный вывод + тишина QUIET_GAP;
  - эхо "> CMD" пропускается;
  - открытие Serial без DTR/RTS (не дёргать авто-reset ESP32).
Хозяин порта один: пока жив сервер — монитор/раннеры/socat на том же
порту закрыты.
"""

import os
import socket
import threading
import time

DEFAULT_BAUD = 921600
DEF_TIMEOUT = 30.0
QUIET_GAP = 1.5  # тишина после данных = конец ответа без OK
SERIAL_READ_TIMEOUT = 0.1
INTER_POLL_DELAY = 0.02

# Команды, не присылающие OK (успех = вывод + тишина).
NO_OK_PREFIXES = ("M114", "M115", "M105")


def is_no_ok(cmd):
    """True, если команда не завершается строкой OK."""
    return cmd.strip().upper().startswith(NO_OK_PREFIXES)


def is_echo(line, cmd):
    """True, если строка — эхо команды мостом/прошивкой."""
    c = cmd.strip()
    return line == c or line == "> " + c or line.startswith(">")


def classify(line):
    """Терминатор строки ответа: 'ok' / 'error' / None."""
    up = line.upper()
    if up == "OK" or up.startswith("OK:") or up.startswith("OK "):
        return "ok"
    if up.startswith("ERROR"):
        return "error"
    return None


def finish_no_ok(lines):
    """Результат тишины для команд без OK: (ok, status)."""
    if lines:
        return True, "QUIET"
    return False, "TIMEOUT"


class Config:
    """Конфиг из env (см. mcp/README.md)."""

    def __init__(self):
        self.transport = os.environ.get("ROBOT_TRANSPORT", "serial").lower()
        self.serial_port = os.environ.get("ROBOT_SERIAL_PORT", "/dev/ttyUSB0")
        self.baud = int(os.environ.get("ROBOT_BAUD", str(DEFAULT_BAUD)))
        self.tcp_host = os.environ.get("ROBOT_TCP_HOST", "127.0.0.1")
        self.tcp_port = int(os.environ.get("ROBOT_TCP_PORT", "5000"))
        self.timeout = float(os.environ.get("ROBOT_TIMEOUT", str(DEF_TIMEOUT)))

    def describe(self):
        if self.transport == "tcp":
            return "tcp %s:%d" % (self.tcp_host, self.tcp_port)
        return "serial %s @ %d" % (self.serial_port, self.baud)


class RobotLink:
    """Единоличное соединение с контроллером + глобальный lock."""

    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self._lock = threading.Lock()
        self._ser = None
        self._sock = None

    # -- Serial --

    def _open_serial(self):
        import serial

        if self._ser is not None and self._ser.is_open:
            return self._ser
        try:
            ser = serial.Serial()
            ser.port = self.cfg.serial_port
            ser.baudrate = self.cfg.baud
            ser.bytesize = serial.EIGHTBITS
            ser.parity = serial.PARITY_NONE
            ser.stopbits = serial.STOPBITS_ONE
            ser.timeout = SERIAL_READ_TIMEOUT
            # DTR/RTS до open — не дёргать авто-reset (как в demo.py).
            ser.dtr = False
            ser.rts = False
            ser.open()
        except Exception as e:
            raise RuntimeError(
                "Не открыт %s @ %d (%s). Хозяин порта один: закрой "
                "pio monitor / socat / tools/*.py и проверь, что узел — "
                "crw-, а не обычный файл." % (self.cfg.serial_port,
                                              self.cfg.baud, e)
            )
        try:
            ser.setDTR(False)
            ser.setRTS(False)
        except Exception:
            pass
        ser.reset_input_buffer()
        self._ser = ser
        return ser

    def _transact_serial(self, ser, cmd, timeout):
        ser.reset_input_buffer()
        ser.write((cmd.strip() + "\n").encode())
        ser.flush()
        want_quiet = is_no_ok(cmd)
        lines = []
        start = time.time()
        last_rx = start
        while time.time() - start < timeout:
            raw = ser.readline()
            if raw:
                line = raw.decode(errors="ignore").strip()
                if not line or is_echo(line, cmd):
                    continue
                lines.append(line)
                last_rx = time.time()
                term = classify(line)
                if term == "ok":
                    return True, line, lines
                if term == "error":
                    return False, line, lines
            else:
                time.sleep(INTER_POLL_DELAY)
            if want_quiet and lines and (time.time() - last_rx > QUIET_GAP):
                ok, status = finish_no_ok(lines)
                return ok, status, lines
        if want_quiet:
            ok, status = finish_no_ok(lines)
            return ok, status, lines
        return False, "TIMEOUT", lines

    # -- TCP --

    def _open_tcp(self):
        if self._sock is not None:
            return self._sock
        try:
            s = socket.socket()
            s.settimeout(10)
            s.connect((self.cfg.tcp_host, self.cfg.tcp_port))
            s.settimeout(0.5)
        except Exception as e:
            raise RuntimeError(
                "Не подключён TCP %s:%d (%s). На машине с железом должен "
                "стоять мост: socat TCP-LISTEN:5000,fork,reuseaddr "
                "/dev/ttyUSB0,raw,echo=0,b921600" % (self.cfg.tcp_host,
                                                     self.cfg.tcp_port, e)
            )
        self._sock = s
        return s

    def _transact_tcp(self, s, cmd, timeout):
        try:
            s.sendall((cmd.strip() + "\n").encode())
        except Exception as e:
            self._sock = None
            raise RuntimeError("Обрыв TCP-моста: %s" % e)
        want_quiet = is_no_ok(cmd)
        lines = []
        buf = b""
        start = time.time()
        last_rx = start
        while time.time() - start < timeout:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                chunk = b""
            except Exception as e:
                self._sock = None
                raise RuntimeError("Обрыв TCP-моста: %s" % e)
            if chunk:
                last_rx = time.time()
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode(errors="ignore").strip().strip("\r")
                    if not line or is_echo(line, cmd):
                        continue
                    lines.append(line)
                    term = classify(line)
                    if term == "ok":
                        return True, line, lines
                    if term == "error":
                        return False, line, lines
            else:
                time.sleep(INTER_POLL_DELAY)
            if want_quiet and lines and (time.time() - last_rx > QUIET_GAP):
                ok, status = finish_no_ok(lines)
                return ok, status, lines
        if want_quiet:
            ok, status = finish_no_ok(lines)
            return ok, status, lines
        return False, "TIMEOUT", lines

    # -- Публичное API --

    def transact(self, cmd, timeout=None):
        """Одна команда. Возвращает (ok, status, lines). Потокобезопасно."""
        timeout = self.timeout if timeout is None else timeout
        with self._lock:
            if self.cfg.transport == "tcp":
                s = self._open_tcp()
                return self._transact_tcp(s, cmd, timeout)
            ser = self._open_serial()
            return self._transact_serial(ser, cmd, timeout)

    def close(self):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None


def format_result(cmd, ok, status, lines, max_lines=200):
    """Текст ответа MCP-тула: статус + строки прошивки."""
    head = "OK %s" % status if ok else "FAIL %s" % status
    body = "\n".join(lines[:max_lines])
    if len(lines) > max_lines:
        body += "\n... (%d строк всего)" % len(lines)
    return "%s\n%s" % (head, body) if body else head
