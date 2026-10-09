"""Низкие цели + зеркало safety: доказать, что ниже zMin ничего не проскакивает.
Цели у стола и ПОД столом (ожидаем отказ/недостижимость, а не тихий провал).
Зеркало checkAllLinks: короб ±560/700, maxReach 560, zMin {0,0,10*4},
сэмплинг звеньев 5 точек + палец.

Запуск: python3 verify_low.py
"""
import math
import sys
sys.path.insert(0, '/workspace/test/robot-arm-6dof/tools/twin')
from solve import solve_all
from ik import refine
from fit3 import axis_err
from fk import fk_chain, mat_mul, dh_transform

DH2 = [[0, 90, 134, 0], [165, 0, 0, 90], [161, 0, 0, 0],
       [0, -90, 0, -90], [53, 90, 54, 90], [0, 0, 27, 0]]
F = 105.0
ZMIN = [0, 0, 10, 10, 10, 10]


def links_ok(q):
    """Зеркало SafetyChecker: звенья + палец, сэмплинг как в прошивке."""
    # Один проход fk_chain с пальцем: joints + tcp сразу (раньше joints
    # из fk_chain + второй полный проход по матрицам ради tcp).
    r = fk_chain(q, DH2, finger=F)
    J = list(r["joints"])
    tcp = r["tcp"]
    segs = [(J[i], J[i + 1]) for i in range(6)] + [(J[6], tcp)]
    for idx, (a, b) in enumerate(segs):
        lim = ZMIN[5] if idx == 6 else ZMIN[min(idx, 5)]
        for s in range(6):
            t = s / 5.0
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            z = a[2] + (b[2] - a[2]) * t
            if not (-560 <= x <= 560 and -560 <= y <= 560 and 0 <= z <= 700):
                return False, f"seg{idx} вне короба ({x:.0f},{y:.0f},{z:.0f})"
            if math.hypot(x, y) > 560:
                return False, f"seg{idx} reach"
            if z < lim:
                return False, f"seg{idx} z={z:.1f}<{lim}"
    return True, "ok"


if __name__ == "__main__":
    targets = [
        ("низ clean", (200, 0, 120), (0, 90, 0)),
        ("низ грань", (150, 0, 60), (0, 90, 0)),
        ("под столом", (200, 0, -30), (0, 90, 0)),
        ("низ бок", (250, 120, 80), (-90, 0, 0)),
        ("ноль", (0, 0, 0), (0, 90, 0)),
    ]
    for tname, pos, eu in targets:
        cands = list(solve_all(pos, eu, DH2, finger=F))
        meta = {}
        qc, cc = refine(pos, eu, [0.0] * 6, DH2, F, meta=meta)
        cands.append((qc, cc, 0, 0, meta))
        print(f"--- {tname} {pos} ---")
        for q, conv, dp, do, mm in cands:
            dp2, th = axis_err(q, pos, eu, DH2, finger=F)
            ok, why = links_ok(q)
            r = fk_chain(q, DH2, finger=F)
            print(f"  dPos={dp2:.1f} ax={th:.1f} conv={conv} "
                  f"TCPz={r['tcp'][2]:.1f} safety={'OK' if ok else 'FAIL:' + why} "
                  f"q={[round(v, 1) for v in q]}")
