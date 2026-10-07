#!/usr/bin/env python3
"""
Демонстрация G-code команд для 6-DOF robot arm на ESP32.

Отправляет последовательность команд из шапки main.cpp по Serial,
ждёт ответ OK/ERROR после каждой команды, логирует прогресс.

ESP32 НЕ перезагружается принудительно при подключении (DTR/RTS снимаются
до open). После открытия порта скрипт ждёт 8 секунд — чтобы ESP32 успел
пройти setup() (там delay(5000) + инициализация I2C/EEPROM) и начал
слушать Serial в core0Task.

Использование:
    python demo.py                        # порт по умолчанию, пауза 8 сек
    python demo.py /dev/ttyUSB0           # Linux
    python demo.py COM11                  # Windows
    python demo.py /dev/tty.usbserial-*   # macOS
    python demo.py /dev/ttyUSB0 921600 12 # порт, скорость, пауза (сек)

Требования:
    pip install pyserial
"""

import sys
import time
import serial
import serial.tools.list_ports


# ===== Настройки =====
DEFAULT_BAUD = 921600
DEFAULT_STARTUP_DELAY = 15.0   # пауза после open, чтобы ESP32 прошёл setup()
TIMEOUT_SEC = 30              # максимум ожидания ответа на команду
INTER_CMD_DELAY = 0.2         # пауза между командами (сек)


# ===== Демонстрация Endpoint Control (валидирована в двойнике) =====
# TCP=(200,200,200) стоит (dPos<=1мм на всех шагах twin),
# инструмент идёт +X -> вниз. DH-v2 + палец 105.

DEMO_COMMANDS = [
    ("G90",                                  "Absolute mode"),
    ("G28",                                  "Home all axes"),
    ("G1 X200 Y200 Z200 A90.0 B-0.0 C90.0 F300",     "Шаг 0: инструмент +X"),
    ("G1 X200 Y200 Z200 A96.7 B5.9 C96.7 F300",      "Шаг 1"),
    ("G1 X200 Y200 Z200 A104.1 B11.0 C104.1 F300",   "Шаг 2"),
    ("G1 X200 Y200 Z200 A112.2 B15.0 C112.2 F300",   "Шаг 3"),
    ("G1 X200 Y200 Z200 A121.0 B17.8 C121.0 F300",   "Шаг 4"),
    ("G1 X200 Y200 Z200 A130.3 B19.3 C130.3 F300",   "Шаг 5"),
    ("G1 X200 Y200 Z200 A139.7 B19.3 C139.7 F300",   "Шаг 6"),
    ("G1 X200 Y200 Z200 A149.0 B17.8 C149.0 F300",   "Шаг 7"),
    ("G1 X200 Y200 Z200 A157.8 B15.0 C157.8 F300",   "Шаг 8"),
    ("G1 X200 Y200 Z200 A165.9 B11.0 C165.9 F300",   "Шаг 9"),
    ("G1 X200 Y200 Z200 A173.3 B5.9 C173.3 F300",    "Шаг 10"),
    ("G1 X200 Y200 Z200 A-180.0 B-0.0 C-180.0 F300", "Шаг 11: инструмент вниз"),
    ("M115",                                 "Разбор последнего шага"),
    ("G28",                                  "Home again"),
    ("M114",                                 "Report position"),
    ("M105",                                 "Report servo angles"),
]


def find_port():
    """Автоматический поиск порта ESP32 (USB-Serial / CP210x / CH340)."""
    ports = serial.tools.list_ports.comports()
    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        mfg  = (p.manufacturer or "").lower()
        if any(k in desc + mfg for k in
               ["cp210", "ch340", "ch341", "ft232", "usb-serial", "uart",
                "silicon labs", "espressif", "esp32"]):
            candidates.append(p.device)

    if not candidates:
        print("Не найден порт ESP32. Доступные порты:")
        for p in ports:
            print(f"  {p.device}  — {p.description}")
        print("\nУкажи порт вручную: python demo.py <порт>")
        sys.exit(1)

    if len(candidates) > 1:
        print(f"Найдено несколько подходящих портов: {candidates}")
        print(f"Использую первый: {candidates[0]}")
        print("Если не тот — укажи вручную: python demo.py <порт>")

    return candidates[0]


def open_serial(port, baud=DEFAULT_BAUD, startup_delay=DEFAULT_STARTUP_DELAY):
    """
    Открывает Serial без принудительного сброса ESP32.
    DTR/RTS снимаются ДО open(), чтобы не дёрнуть схему авто-сброса.
    После открытия ждёт startup_delay секунд — чтобы ESP32 завершил setup()
    и был готов принимать команды (в setup() есть delay(5000)).
    """
    print(f"Открываю {port} @ {baud} (без сброса ESP32)...")

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.1

    # Снимаем DTR/RTS ДО open — не дёргаем авто-reset
    ser.dtr = False
    ser.rts = False

    ser.open()

    # Ещё раз после open — на случай, если драйвер поднял
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    # Сбрасываем буфер — там может быть мусор от ESP32
    ser.reset_input_buffer()

    # Пауза, чтобы ESP32 завершил setup() и был готов принимать команды
    print(f"Порт открыт. Ждём {startup_delay:.0f} сек (setup ESP32)...")
    for remaining in range(int(startup_delay), 0, -1):
        print(f"  {remaining}...", end="", flush=True)
        time.sleep(1.0)
    print(" готово.\n")

    # Очищаем буфер ещё раз — на случай, если во время паузы
    # ESP32 что-то печатал (boot-log, "Ready.", и т.д.)
    ser.reset_input_buffer()

    return ser


def send_command(ser, cmd, timeout=TIMEOUT_SEC):
    """
    Отправляет G-code команду и ждёт OK / ERROR.

    M114/M115/M105 в ответ шлют данные без OK — для них успех =
    получен нетривиальный вывод (иначе вечное ожидание таймаута).

    Возвращает (ok, response_lines).
    """
    ser.reset_input_buffer()

    # Отправка команды с \n
    full_cmd = cmd.strip() + "\n"
    ser.write(full_cmd.encode())
    ser.flush()

    # Команды без OK в ответе
    no_ok = full_cmd.strip().upper().startswith(("M114", "M115", "M105"))

    # Ждём ответа
    start = time.time()
    last_rx = start
    response_lines = []
    result = None

    while time.time() - start < timeout:
        if ser.in_waiting:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if not line:
                continue

            response_lines.append(line)
            last_rx = time.time()

            # Эхо команды
            if line.startswith(">"):
                continue

            # Результат
            if line.startswith("OK"):
                result = True
                break
            if line.startswith("ERROR"):
                result = False
                break
            if line.startswith("Angles:") or line.startswith("Position:"):
                # Информационные ответы M105/M114 — печатаем и продолжаем ждать OK
                print(f"      {line}")

        # Команды без OK (M114/M115/M105): тишина 1.5с после вывода = готово,
        # иначе висим весь timeout впустую (M115 молча ел 30с).
        if no_ok and response_lines and (time.time() - last_rx) > 1.5:
            break

        time.sleep(0.02)

    if result is None:
        if no_ok and len(response_lines) >= 3:
            return True, response_lines
        return False, response_lines

    return result, response_lines


def print_banner():
    print("=" * 70)
    print("  6-DOF Robot Arm — Serial Demo")
    print("=" * 70)
    print()


def main():
    sys.stdout.reconfigure(line_buffering=True)  # вывод сразу, без буфера
    # Аргументы: port [baud] [startup_delay]
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD
    startup_delay = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_STARTUP_DELAY

    print_banner()

    try:
        ser = open_serial(port, baud, startup_delay=startup_delay)
    except serial.SerialException as e:
        print(f"Ошибка открытия порта: {e}")
        sys.exit(1)

    ok_count = 0
    err_count = 0
    t_start = time.time()

    def stamp():
        dt = time.time() - t_start
        return time.strftime("%H:%M:%S") + f" (+{dt:6.1f}c)"

    try:
        print(f"Отправляю {len(DEMO_COMMANDS)} команд...\n")

        for idx, (cmd, description) in enumerate(DEMO_COMMANDS, 1):
            print(f"[{idx:2}/{len(DEMO_COMMANDS)}] [{stamp()}] {description}")
            print(f"      → {cmd}")

            ok, lines = send_command(ser, cmd)

            if ok:
                ok_count += 1
                print(f"      ✓ OK [{stamp()}]")
            else:
                err_count += 1
                print(f"      ✗ FAIL [{stamp()}]")
                for line in lines:
                    if not line.startswith(">"):
                        print(f"      | {line}")

            time.sleep(INTER_CMD_DELAY)

    except KeyboardInterrupt:
        print("\n\nПрервано пользователем (Ctrl+C)")
    finally:
        ser.close()
        print()

    print("=" * 70)
    print(f"  Итог: OK = {ok_count} / {len(DEMO_COMMANDS)},  FAIL = {err_count}")
    print("=" * 70)

    sys.exit(0 if err_count == 0 else 1)


if __name__ == "__main__":
    main()