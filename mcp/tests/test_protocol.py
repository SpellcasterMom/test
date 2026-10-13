#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW) — MCP protocol tests
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Group       : УТС/б-23-1-о
Mentor      : Lipko I.Y.
Date        : 2026-10-11
Last modify : 2026-10-13

Офлайн-тесты: парсинг ответов и построители команд. Железо не нужно.
Запуск из корня репо: python3 -m pytest mcp/tests/ -q
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest

import commands
from commands import ValidationError
from transport import classify, finish_no_ok, is_echo, is_no_ok


def test_no_ok_prefixes():
    assert is_no_ok("M105")
    assert is_no_ok("m114")
    assert is_no_ok("M115 S=1")
    assert not is_no_ok("M104 P1=10")
    assert not is_no_ok("G0 X200")
    assert not is_no_ok("?")


def test_echo_skip():
    assert is_echo("> M105", "M105")
    assert is_echo("M105", "M105")
    assert is_echo("> G28", "G28")
    assert not is_echo("OK", "M105")
    assert not is_echo("Angles: P1=0", "M105")


def test_classify():
    assert classify("OK") == "ok"
    assert classify("OK: queued") == "ok"
    assert classify("ERROR: No safe IK") == "error"
    assert classify("Angles: P1=0") is None
    assert classify("TARGET X=200") is None


def test_finish_no_ok():
    ok, status = finish_no_ok(["Angles: P1=0", "Position: X=1"])
    assert ok and status == "QUIET"
    ok, status = finish_no_ok([])
    assert not ok and status == "TIMEOUT"


def test_m104_ok():
    assert commands.build_m104({"P1": 10, "P4": -5}) == "M104 P1=10 P4=-5"
    assert commands.build_m104({"P7": 50}, 300) == "M104 P7=50 F300"


def test_m104_limits():
    with pytest.raises(ValidationError):
        commands.build_m104({"P1": 200})
    with pytest.raises(ValidationError):
        commands.build_m104({"P4": 91})
    with pytest.raises(ValidationError):
        commands.build_m104({"P7": -1})
    with pytest.raises(ValidationError):
        commands.build_m104({})


def test_g0_g1():
    assert commands.build_g0_xyz({"X": 200, "Y": 200, "Z": 200}) == \
        "G0 X=200 Y=200 Z=200"
    assert commands.build_g0_xyz({"X": 20}, {"A": 90, "B": 0, "C": 90},
                                 300) == "G0 X=20 A=90 B=0 C=90 F300"
    with pytest.raises(ValidationError):
        commands.build_g0_xyz({})
    with pytest.raises(ValidationError):
        commands.build_g1_xyz({"X": 1})  # F обязательна
    assert commands.build_g1_xyz({"X": 1}, None, 100) == "G1 X=1 F100"


def test_g28_m108_m119_m115():
    assert commands.build_g28() == "G28"
    assert commands.build_g28(True, 15) == "G28 H1 F15"
    assert commands.build_m108(50) == "M108 S=50"
    with pytest.raises(ValidationError):
        commands.build_m108(101)
    assert commands.build_m119() == "M119"
    assert commands.build_m119(True, 10) == "M119 S=1 D=10"
    assert commands.build_m115() == "M115"
    assert commands.build_m115(True) == "M115 S=1"


def test_m450():
    assert commands.build_m450("open") == "M450 S1"
    assert commands.build_m450("drain") == "M450 S0"
    assert commands.build_m450("abort") == "M450 S2"
    with pytest.raises(ValidationError):
        commands.build_m450("explode")


def test_check_g1():
    assert commands.check_g1("G1 X=1 F100") == "G1 X=1 F100"
    assert commands.check_g1("g1 x=1 y=2 f300") == "g1 x=1 y=2 f300"
    with pytest.raises(ValidationError):
        commands.check_g1("G0 X=1 F100")  # не G1
    with pytest.raises(ValidationError):
        commands.check_g1("G1 X=1")  # без F
    with pytest.raises(ValidationError):
        commands.check_g1("")
    with pytest.raises(ValidationError):
        commands.check_g1(None)


def test_move_stream_sequence(monkeypatch):
    import server
    calls = []

    def fake_run(cmd, timeout=None):
        calls.append(cmd)
        if cmd == "M115":
            return "OK QUIET\nRESIDUAL: dist=0.5mm dOri=0.3deg"
        return "OK"

    monkeypatch.setattr(server, "run", fake_run)
    fn = getattr(server.move_stream, "fn", server.move_stream)
    out = fn(["G1 X=1 F100", "G1 X=2 F100"], confirm=True)
    assert calls == ["M450 S1", "G1 X=1 F100", "G1 X=2 F100",
                     "M450 S0", "M115"]
    assert "RESIDUAL" in out
    # без confirm — отказ, в порт ничего не ушло
    del calls[:]
    out = fn(["G1 X=1 F100"], confirm=False)
    assert out.startswith("Отказ") and not calls
    # битая точка — отказ до S1
    out = fn(["G0 X=1 F100"], confirm=True)
    assert out.startswith("Отказ") and not calls
    # обрыв посреди — S2 и возврат FAIL
    def fake_fail(cmd, timeout=None):
        calls.append(cmd)
        return "FAIL TIMEOUT" if cmd.startswith("G1") else "OK"
    monkeypatch.setattr(server, "run", fake_fail)
    del calls[:]
    out = fn(["G1 X=1 F100"], confirm=True)
    assert calls == ["M450 S1", "G1 X=1 F100", "M450 S2"]
    assert out.startswith("FAIL")
