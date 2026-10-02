"""Приёмка как на железе 1:1: кандидаты solve_all + refine-from-prev,
фильтр safety, выбор min-residual, порог 10/15. Без этого twin врёт
(показывает прыжки ветвей, которых железо не делает, и наоборот).

Запуск: python3 accept_loops.py
"""
import math
from fk import fk_chain
from solve import solve_all
from ik import refine
from fit3 import axis_err

DH2 = [[0, 90, 134, 0], [165, 0, 0, 90], [161, 0, 0, 0],
       [0, -90, 0, -90], [53, 90, 54, 90], [0, 0, 27, 0]]
F = 105.0
ZMIN = [0, 0, 10, 10, 10, 10]
BOX = 560.0
ZMAX = 700.0
MAXR = 560.0


def safe(q):
    r = fk_chain(q, DH2, F)
    pts = list(r["joints"]) + [r["tcp"]]
    for k in range(len(pts) - 1):
        a, b = pts[k], pts[k + 1]
        for s in range(6):
            t = s / 5.0
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            z = a[2] + (b[2] - a[2]) * t
            if not (-BOX <= x <= BOX and -BOX <= y <= BOX and 0 <= z <= ZMAX):
                return False
            if math.hypot(x, y) > MAXR:
                return False
            if z < ZMIN[min(k, 5)]:
                return False
    return True


def hw_step(pos, eu, qprev):
    """Один шаг как прошивка: кандидаты, safe-фильтр, min-residual, порог."""
    cands = list(solve_all(pos, eu, DH2, finger=F))
    meta = {}
    qc, cc = refine(pos, eu, list(qprev), DH2, F, meta=meta)
    cands.append((qc, cc, 0, 0, meta))
    best = None
    for q, conv, dp, do, mm in cands:
        if not safe(q):
            continue
        dp2, th = axis_err(q, pos, eu, DH2, finger=F)
        sc = dp2 + th + 0.02 * math.dist(q, qprev)  # как прошивка: continuity-биас
        if best is None or sc < best[0]:
            best = (sc, q, conv, mm, dp2, th)
    if best is None:
        return None
    sc, q, conv, mm, dp2, th = best
    if dp2 > 10.0 or th > 15.0:
        return None
    return q, dp2, th, mm


def loop(cx, cy, cz, r, eu, n=20, q0=None):
    qprev = list(q0) if q0 else [0.0] * 6
    ok = True
    rows = []
    for k in range(n):
        a = 2 * math.pi * k / n
        pos = (cx, cy + r * math.cos(a), cz + r * math.sin(a))
        res = hw_step(pos, eu, qprev)
        if res is None:
            rows.append((k, pos, None))
            ok = False
            continue
        q, dp2, th, mm = res
        rows.append((k, pos, (dp2, th, mm["iters"], math.dist(q, qprev), list(q))))
        qprev = list(q)
    return ok, rows


if __name__ == "__main__":
    EU = (90.0, 0.0, 90.0)
    for name, cx, cy, cz, r in [("A r=80 @(220,0,320)", 220, 0, 320, 80),
                                ("B r=60 @(230,0,300)", 230, 0, 300, 60)]:
        print(f"=== круг {name}, инструмент +X ===")
        ok, rows = loop(cx, cy, cz, r, EU)
        for k, pos, res in rows:
            if res is None:
                print(f"  т.{k}: ОТКАЗ (прошивка бы тоже отказала)")
            else:
                dp2, th, it, jump, q = res
                print(f"  т.{k}: dPos={dp2:.2f} ax={th:.1f} it={it} jump={jump:.0f}")
        print("ИТОГ:", "ВСЕ СОШЛИСЬ" if ok else "ЕСТЬ СРЫВЫ")
