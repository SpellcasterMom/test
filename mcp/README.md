# mcp — MCP-сервер робота CLAW

12 тулов: чтение (`status_angles`, `fk`, `diag`, `imu`, `help_text`)
+ движение (`move_joints`, `move_tcp`, `move_linear`, `move_stream`,
`go_home`, `gripper`, `e_stop`). Протокол ответов 1:1 с `tools/tcp_console.py`.

## Где запускать

На машине с USB (helper), рядом с железом. В контейнере без
`/dev/ttyUSB*` работать не будет (только через TCP-мост).

## Установка (helper)

```bash
sudo apt install -y python3 python3-pip
pip3 install -r mcp/requirements.txt
```

## Запуск

```bash
# прямой Serial (дефолт)
ROBOT_TRANSPORT=serial ROBOT_SERIAL_PORT=/dev/ttyUSB0 python3 mcp/server.py
# через socat-мост
ROBOT_TRANSPORT=tcp ROBOT_TCP_HOST=127.0.0.1 ROBOT_TCP_PORT=5000 python3 mcp/server.py
```

Env: `ROBOT_TRANSPORT=serial|tcp`, `ROBOT_SERIAL_PORT`, `ROBOT_BAUD`
(921600 = `SERIAL_BAUD`, рассинхрон = тишина), `ROBOT_TCP_HOST`,
`ROBOT_TCP_PORT`, `ROBOT_TIMEOUT` (30 с).

## Правила

- Хозяин порта один: перед стартом закрой `pio monitor`, `socat`
  на том же порту, `tools/*.py`.
- Движение только с `confirm=true`; `G1` без `F` — отказ.
- Пачки (`move_stream`, M450): точки — строки `G1 … F…` (лимит 500);
  сервер шлёт `S1 → точки → S0`, при обрыве — `S2`, в конце — сводка
  `M115`. Прошивка без стриминга ответит `ERROR` на `S1` — тогда
  по точкам обычными `move_linear`.
- `M111`/`M500`/`M502` не экспонируются (home/EEPROM — только человек).
- `/dev/ttyUSB0` должен быть `crw-`, не обычным файлом (см. грабли
  в истории: `rm` + перетык платы).

## Проверка без железа

```bash
python3 -m pytest mcp/tests/ -q
```

## Подключение к opencode

В проектном `opencode.json`:

```json
{ "mcp": { "claw": {
  "type": "local",
  "command": ["python3", "mcp/server.py"],
  "environment": { "ROBOT_TRANSPORT": "serial" }
} } }
```
