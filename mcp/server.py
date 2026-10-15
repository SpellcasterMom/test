#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW) — MCP server
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Group       : УТС/б-23-1-о
Mentor      : Lipko I.Y.
Date        : 2026-10-11
Last modify : 2026-10-13

MCP-сервер робота: 12 тулов (5 чтение + 7 движение). Транспорт — Serial
или TCP-мост (ROBOT_TRANSPORT), скорость 921600 = SERIAL_BAUD.
Пачки точек — потоком M450 (move_stream: S1 → G1… → S0, при обрыве S2).
Правила: хозяин порта один; движение только с confirm=true; G1 без F —
отказ; M111/M500/M502 намеренно не экспонируются (home/EEPROM трогает
только человек). Запуск: python3 mcp/server.py (stdio).
"""

import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# Свой mcp/ затеняет установленный пакет mcp (локальный каталог раньше
# site-packages): вычищаем оба хвоста ДО импорта FastMCP, иначе
# `mcp.server` резолвится в этот же server.py. Возвращаем только свой
# каталог — для `import commands` ниже.
for _p in (os.path.dirname(_THIS_DIR), _THIS_DIR):
    while _p in sys.path:
        sys.path.remove(_p)
# pytest грузит этот файл как часть локального пакета (mcp/tests/ —
# пакет): тень сидит уже в sys.modules — вычищаем и её, кроме модуля,
# который исполняется прямо сейчас.
for _m in [m for m in sys.modules
           if m == "mcp" or m.startswith("mcp.")]:
    if _m != __name__:
        del sys.modules[_m]
try:
    from mcp.server import FastMCP
except ImportError:  # старые 1.x
    from mcp.server.fastmcp import FastMCP

sys.path.insert(0, _THIS_DIR)

import commands
from commands import ValidationError
from transport import Config, RobotLink, format_result

mcp = FastMCP("claw-6dof")
_link = None


def link():
    global _link
    if _link is None:
        _link = RobotLink(Config())
    return _link


def run(cmd, timeout=None):
    ok, status, lines = link().transact(cmd, timeout)
    return format_result(cmd, ok, status, lines)


def need_confirm(confirm):
    if not confirm:
        return ("Отказ: движение требует confirm=true. "
                "Проверь цель по diag/fk, затем повтори с confirm=true.")
    return None


# -- Чтение --

@mcp.tool()
def status_angles() -> str:
    """Углы сервоприводов (M105)."""
    return run("M105")


@mcp.tool()
def fk() -> str:
    """Прямая кинематика: TCP + цепочка J0..J6 (M114)."""
    return run("M114")


@mcp.tool()
def diag(full: bool = False) -> str:
    """Разбор M115: цель/кандидаты/факт/расхождение. full=true → M115 S=1."""
    return run(commands.build_m115(full), timeout=45.0)


@mcp.tool()
def imu(dump: bool = False, decimate: int = 0) -> str:
    """IMU BMI160: живьё (M119) или дамп кольца (dump=true, decimate=N)."""
    return run(commands.build_m119(dump, decimate), timeout=45.0)


@mcp.tool()
def help_text() -> str:
    """Справка прошивки (?): команды и версии FW/HW."""
    return run("?")


# -- Движение (только с confirm=true) --

@mcp.tool()
def move_joints(P1: float = None, P2: float = None, P3: float = None,
                P4: float = None, P5: float = None, P6: float = None,
                P7: float = None, feed: float = None,
                confirm: bool = False) -> str:
    """Плавный заезд суставов M104 (лимиты P1–P3 ±135, P4–P6 ±90, P7 0–100)."""
    err = need_confirm(confirm)
    if err:
        return err
    try:
        cmd = commands.build_m104(
            {"P1": P1, "P2": P2, "P3": P3, "P4": P4,
             "P5": P5, "P6": P6, "P7": P7}, feed)
    except ValidationError as e:
        return "Отказ: %s" % e
    return run(cmd, timeout=120.0)


@mcp.tool()
def move_tcp(x: float = None, y: float = None, z: float = None,
             a: float = None, b: float = None, c: float = None,
             feed: float = None, relative: bool = False,
             confirm: bool = False) -> str:
    """Ход TCP к точке G0 (незаданные оси держит; без A/B/C — инструмент +X).
    relative=true — дельты от текущего TCP (G91, затем возврат в G90)."""
    err = need_confirm(confirm)
    if err:
        return err
    try:
        cmd = commands.build_g0_xyz({"X": x, "Y": y, "Z": z},
                                    {"A": a, "B": b, "C": c}, feed)
    except ValidationError as e:
        return "Отказ: %s" % e
    if relative:
        r1 = run("G91")
        if r1.startswith("FAIL"):
            return r1
        r2 = run(cmd, timeout=120.0)
        run("G90")
        return r2
    return run(cmd, timeout=120.0)


@mcp.tool()
def move_linear(x: float = None, y: float = None, z: float = None,
                a: float = None, b: float = None, c: float = None,
                feed: float = None, confirm: bool = False) -> str:
    """Декартов ход G1 строго по прямой. Подача feed обязательна."""
    err = need_confirm(confirm)
    if err:
        return err
    try:
        cmd = commands.build_g1_xyz({"X": x, "Y": y, "Z": z},
                                    {"A": a, "B": b, "C": c}, feed)
    except ValidationError as e:
        return "Отказ: %s" % e
    return run(cmd, timeout=180.0)


@mcp.tool()
def go_home(park: bool = False, feed: float = None,
            confirm: bool = False) -> str:
    """Дом (G28) или парковка (park=true → G28 H1). Держи зону свободной:
    при старте рука едет парк→home сама, сейчас поедет сразу."""
    err = need_confirm(confirm)
    if err:
        return err
    return run(commands.build_g28(park, feed), timeout=120.0)


@mcp.tool()
def gripper(percent: float, confirm: bool = False) -> str:
    """Захват M108: 0=закрыт, 100=открыт (проценты условные)."""
    err = need_confirm(confirm)
    if err:
        return err
    try:
        cmd = commands.build_m108(percent)
    except ValidationError as e:
        return "Отказ: %s" % e
    return run(cmd, timeout=60.0)


@mcp.tool()
def move_stream(points: list, confirm: bool = False) -> str:
    """Пачка точек одним потоком M450: S1 → все G1 подряд → S0 (без
    остановок на стыках). points — строки 'G1 X=.. Y=.. Z=.. [A=.. B=.. C=..]
    F..' (F обязательна, лимит 500). При обрыве — S2 (сброс потока).
    В конце — сводка M115."""
    err = need_confirm(confirm)
    if err:
        return err
    if not points:
        return "Отказ: пустая пачка"
    if len(points) > 500:
        return "Отказ: пачка %d > 500 точек" % len(points)
    try:
        pts = [commands.check_g1(p) for p in points]
    except ValidationError as e:
        return "Отказ: %s" % e
    r = run(commands.build_m450("open"), timeout=30.0)
    if r.startswith("FAIL"):
        return r
    for p in pts:
        r = run(p, timeout=120.0)
        if r.startswith("FAIL"):
            run(commands.build_m450("abort"), timeout=30.0)
            return r
    r = run(commands.build_m450("drain"), timeout=300.0)
    if r.startswith("FAIL"):
        return r
    return run("M115", timeout=45.0)


@mcp.tool()
def e_stop() -> str:
    """Аварийный стоп M112. Без confirm, всегда доступен."""
    return run("M112", timeout=15.0)


if __name__ == "__main__":
    cfg = Config()
    print("claw-6dof MCP: транспорт %s" % cfg.describe(), file=sys.stderr)
    mcp.run()
