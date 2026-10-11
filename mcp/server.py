#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW) — MCP server
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Group       : UTS/b-23-1-o
Mentor      : Lipko I.Y.
Date        : 2026-10-11
Last modify : 2026-10-11

MCP-сервер робота: 11 тулов (5 чтение + 6 движение). Транспорт — Serial
или TCP-мост (ROBOT_TRANSPORT), скорость 921600 = SERIAL_BAUD.
Правила: хозяин порта один; движение только с confirm=true; G1 без F —
отказ; M111/M500/M502 намеренно не экспонируются (home/EEPROM трогает
только человек). Запуск: python3 mcp/server.py (stdio).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mcp.server.fastmcp import FastMCP

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
    """IMU GY-80: живьё (M119) или дамп кольца (dump=true, decimate=N)."""
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
def e_stop() -> str:
    """Аварийный стоп M112. Без confirm, всегда доступен."""
    return run("M112", timeout=15.0)


if __name__ == "__main__":
    cfg = Config()
    print("claw-6dof MCP: транспорт %s" % cfg.describe(), file=sys.stderr)
    mcp.run()
