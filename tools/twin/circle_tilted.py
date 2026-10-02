"""Круг в наклонной плоскости: подбор центра/нормали/радиуса + проверка.
Инструмент ⟂ плоскости (как ручка по доске), ориентация одна на весь контур.
"""
import math
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from accept_loops import hw_step, DH2, F
from ik import euler_to_matrix, matrix_to_euler


def norm(v):
    s = math.sqrt(sum(c * c for c in v))
    return tuple(c / s for c in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def mmul(A, B):
    return [sum(A[i * 4 + k] * B[k * 4 + j] for k in range(4))
            for i in range(4) for j in range(4)]


def tool_euler(td):
    """Ориентация: инструмент = td, минимальный доворот от нулевой позы."""
    R0 = euler_to_matrix(90.0, 0.0, 90.0)
    z0 = (R0[2], R0[6], R0[10])
    d = max(-1.0, min(1.0, td[0] * z0[0] + td[1] * z0[1] + td[2] * z0[2]))
    if d > 0.9999:
        return (90.0, 0.0, 90.0)
    ax = cross(z0, td)
    s = math.sqrt(sum(c * c for c in ax))
    ax = tuple(c / s for c in ax)
    th = math.acos(d)
    c, ss = math.cos(th), math.sin(th)
    x, y, z = ax
    Rr = [c + x * x * (1 - c), x * y * (1 - c) - z * ss, x * z * (1 - c) + y * ss, 0,
          y * x * (1 - c) + z * ss, c + y * y * (1 - c), y * z * (1 - c) - x * ss, 0,
          z * x * (1 - c) - y * ss, z * y * (1 - c) + x * ss, c + z * z * (1 - c), 0,
          0, 0, 0, 1]
    return matrix_to_euler(mmul(Rr, R0))


def check_circle(cx, cy, cz, r, n_raw, npts=24):
    n = norm(n_raw)
    td = tuple(-c for c in n)
    eu = tool_euler(td)
    u = norm(cross(n, (0, 0, 1)))
    v = cross(n, u)
    qprev = [0.0] * 6
    ok = True
    rows = []
    for k in range(npts):
        a = 2 * math.pi * k / npts
        pos = (cx + r * (u[0] * math.cos(a) + v[0] * math.sin(a)),
               cy + r * (u[1] * math.cos(a) + v[1] * math.sin(a)),
               cz + r * (u[2] * math.cos(a) + v[2] * math.sin(a)))
        res = hw_step(pos, eu, qprev)
        if res is None:
            rows.append((k, pos, None))
            ok = False
            continue
        q, dp2, th2, mm = res
        rows.append((k, pos, (dp2, th2, mm["iters"], math.dist(q, qprev), list(q))))
        qprev = list(q)
    return ok, rows, eu, n


if __name__ == "__main__":
    ok, rows, eu, n = check_circle(230, 0, 280, 60, (0.3, 0.4, 0.87))
    print("нормаль:", tuple(round(c, 2) for c in n),
          "eu:", tuple(round(c, 1) for c in eu))
    for k, pos, res in rows:
        if res is None:
            print(f"  т.{k}: ОТКАЗ")
        else:
            dp2, th2, it, jump, q = res
            print(f"  т.{k}: dPos={dp2:.2f} ax={th2:.1f} it={it} jump={jump:.0f}")
    print("ИТОГ:", "ВСЕ СОШЛИСЬ" if ok else "ЕСТЬ СРЫВЫ")
