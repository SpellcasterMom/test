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
    python demo.py /dev/ttyUSB0 115200 12 # порт, скорость, пауза (сек)

Требования:
    pip install pyserial
"""

import sys
import time
import serial
import serial.tools.list_ports


# ===== Настройки =====
DEFAULT_BAUD = 115200
DEFAULT_STARTUP_DELAY = 15.0   # пауза после open, чтобы ESP32 прошёл setup()
TIMEOUT_SEC = 30              # максимум ожидания ответа на команду
INTER_CMD_DELAY = 0.2         # пауза между командами (сек)


# ===== Демонстрационная последовательность (из шапки main.cpp) =====
#   G0 P7=... заменено на M104 P7=... — прямое управление захватом
#   без пересчёта IK (иначе safety ругается на reach=415 > MAX=395).
#   M412 R=420 в начале — поднимаем maxReach, чтобы IK-точки тоже влезали.

DEMO_COMMANDS = [
    ("M412 R=420",                           "Set max reach 420mm"),
    ("G28",                                  "Home all axes"),
    ("G0 X200 Y50 Z250 A10 B-20 C5 F300",    "Точка 1"),
    ("G0 X180 Y-80 Z150 A-15 B10 C-20 F300", "Точка 2"),
    ("G0 X-50 Y-150 Z200 A25 B0 C30 F300",   "Точка 3"),
    ("G0 X-100 Y120 Z300 A-10 B-30 C-15 F300","Точка 4"),
    ("G0 X100 Y0 Z100 A0 B20 C0 F250",       "Точка 5"),
    ("M104 P7=20",                           "Захват 20%"),
    ("G0 X50 Y-200 Z280 A30 B-25 C40 F300",  "Точка 6"),
    ("M104 P7=90",                           "Захват 90%"),
    ("G0 X150 Y180 Z180 A-20 B15 C-35 F300", "Точка 7"),
    ("G0 X250 Y0 Z200 A0 B-40 C0 F250",      "Точка 8"),
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

    Возвращает (ok, response_lines).
    """
    ser.reset_input_buffer()

    # Отправка команды с \n
    full_cmd = cmd.strip() + "\n"
    ser.write(full_cmd.encode())
    ser.flush()

    # Ждём ответа
    start = time.time()
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

        time.sleep(0.02)

    if result is None:
        return False, response_lines

    return result, response_lines


def print_banner():
    print("=" * 70)
    print("  6-DOF Robot Arm — Serial Demo")
    print("=" * 70)
    print()


def main():
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

    try:
        print(f"Отправляю {len(DEMO_COMMANDS)} команд...\n")

        for idx, (cmd, description) in enumerate(DEMO_COMMANDS, 1):
            print(f"[{idx:2}/{len(DEMO_COMMANDS)}] {description}")
            print(f"      → {cmd}")

            ok, lines = send_command(ser, cmd)

            if ok:
                ok_count += 1
                print(f"      ✓ OK")
            else:
                err_count += 1
                print(f"      ✗ FAIL")
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