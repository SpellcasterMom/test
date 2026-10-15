#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW) — MCP command builders
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Group       : УТС/б-23-1-о
Mentor      : Lipko I.Y.
Date        : 2026-10-11
Last modify : 2026-10-13

Чистые построители G-code для MCP-тулов (без железа и без MCP-зависимостей,
покрыты офлайн-тестами). Лимиты — MATH-лимиты из README: P1–P3 ±135°,
P4–P6 ±90°, P7 7–38° (закрыт..открыт; проценты M108 — отдельно 0–100).
Прошивка остаётся авторитетом (режет и проверяет
safety сама); здесь — ранний отказ с понятным текстом вместо прыжка.
"""

P123_MIN, P123_MAX = -135.0, 135.0
P456_MIN, P456_MAX = -90.0, 90.0
P7_MIN, P7_MAX = 7.0, 38.0  # градусы сервы (закрыт..открыт)
PCT_MIN, PCT_MAX = 0.0, 100.0  # проценты M108 (не путать с градусами!)

JOINT_LIMITS = {
    "P1": (P123_MIN, P123_MAX),
    "P2": (P123_MIN, P123_MAX),
    "P3": (P123_MIN, P123_MAX),
    "P4": (P456_MIN, P456_MAX),
    "P5": (P456_MIN, P456_MAX),
    "P6": (P456_MIN, P456_MAX),
    "P7": (P7_MIN, P7_MAX),
}


class ValidationError(ValueError):
    pass


def check_joint(name, value):
    lo, hi = JOINT_LIMITS[name]
    if value is None:
        return None
    if not (lo <= value <= hi):
        raise ValidationError(
            "%s=%.2f вне лимита [%.0f, %.0f]" % (name, value, lo, hi))
    return value


def fmt_num(v):
    if float(v).is_integer():
        return str(int(v))
    return ("%.3f" % v).rstrip("0").rstrip(".")


def build_m104(joints, feed=None):
    """M104 P1=.. [F..]. joints: dict P1..P7 (хотя бы один)."""
    parts = []
    for i in range(1, 8):
        name = "P%d" % i
        if joints.get(name) is not None:
            check_joint(name, joints[name])
            parts.append("%s=%s" % (name, fmt_num(joints[name])))
    if not parts:
        raise ValidationError("M104: задай хотя бы один сустав P1..P7")
    cmd = "M104 " + " ".join(parts)
    if feed is not None:
        cmd += " F%s" % fmt_num(feed)
    return cmd


def build_g0_xyz(axes, orient=None, feed=None):
    """G0 X.. Y.. Z.. [A.. B.. C..] [F..]."""
    parts = []
    for a in ("X", "Y", "Z"):
        if axes.get(a) is not None:
            parts.append("%s=%s" % (a, fmt_num(axes[a])))
    if not parts:
        raise ValidationError("G0: задай хотя бы одну ось X/Y/Z")
    orient = orient or {}
    for a in ("A", "B", "C"):
        if orient.get(a) is not None:
            parts.append("%s=%s" % (a, fmt_num(orient[a])))
    cmd = "G0 " + " ".join(parts)
    if feed is not None:
        cmd += " F%s" % fmt_num(feed)
    return cmd


def build_g1_xyz(axes, orient=None, feed=None):
    """G1 как G0, но F обязательна (требование прошивки)."""
    if feed is None:
        raise ValidationError("G1: подача F обязательна")
    return build_g0_xyz(axes, orient, feed).replace("G0 ", "G1 ", 1)


def build_g28(park=False, feed=None):
    cmd = "G28"
    if park:
        cmd += " H1"
    if feed is not None:
        cmd += " F%s" % fmt_num(feed)
    return cmd


def build_m108(percent):
    if not (PCT_MIN <= percent <= PCT_MAX):
        raise ValidationError(
            "M108: S=%.2f вне 0–100" % percent)
    return "M108 S=%s" % fmt_num(percent)


def build_m119(dump=False, decimate=0):
    if not dump:
        return "M119"
    if decimate and decimate > 1:
        return "M119 S=1 D=%d" % int(decimate)
    return "M119 S=1"


def build_m450(action):
    """M450: open=S1 (открыть поток), drain=S0 (исполнить), abort=S2."""
    cmds = {"open": "M450 S1", "drain": "M450 S0", "abort": "M450 S2"}
    if action not in cmds:
        raise ValidationError("M450: действие open/drain/abort")
    return cmds[action]


def check_g1(cmd):
    """Сырая строка точки потока: 'G1 ...' + обязательная F."""
    if not isinstance(cmd, str) or not cmd.strip():
        raise ValidationError("точка потока — непустая строка 'G1 ...'")
    parts = cmd.strip().split()
    if parts[0].upper() != "G1":
        raise ValidationError(
            "точка потока должна начинаться с G1: %r" % cmd)
    if not any(p.upper().startswith("F") for p in parts[1:]):
        raise ValidationError("G1 без F — отказ: %r" % cmd)
    return cmd.strip()


def build_m115(full=False):
    return "M115 S=1" if full else "M115"
