"""Приёмочный тест = ТОЧНОЕ зеркало прошивки:
solve_all: 2 компенсированных сида (down/up) + DLS;
+ 3-й кандидат: refine от текущей позы (как добавит G0-хендлер).
Холодный старт: current = home {0,-30,-135,0,0,0}[0:6].
Критерий пользователя: лучший dPos <= 2мм.

Запуск: python3 accept.py
"""
from dh_draw import DH_DRAW, FINGER_KIN
from solve import solve_all
from ik import refine, fk_pose
from fit3 import axis_err

HOME = [0.0, -30.0, -135.0, 0.0, 0.0, 0.0]
ZEROS = [0.0] * 6
_, EU_ZERO = fk_pose(ZEROS, DH_DRAW, FINGER_KIN)  # истинная ориентация нулей (не identity!)
TARGETS = [
    ('zeros (132,53,514)+eu_zero', (132, 53, 514), EU_ZERO),
    ('T1 (200,200,200)+A0B146C45', (200, 200, 200), (0, 146, 45)),
    ('T2 (200,200,200)+A-90B0C0', (200, 200, 200), (-90, 0, 0)),
    ('T3 (150,0,150)+A180B0C0', (150, 0, 150), (180, 0, 0)),
    ('mid (260,0,200)+down', (260, 0, 200), (0, 90, 0)),
    ('mid (260,80,200)+fwd', (260, 80, 200), (-90, 0, 0)),
    ('mid (180,0,320)+down', (180, 0, 320), (0, 90, 0)),
    ('mid (340,-80,120)+fwd', (340, -80, 120), (-90, 0, 0)),
]

if __name__ == "__main__":
    print(f"cold start current=home {HOME}")
    hp, he = fk_pose(HOME, DH_DRAW, FINGER_KIN)
    print(f"home FK: TCP={[round(v,1) for v in hp]}")
    allpass = True
    for tname, pos, eu in TARGETS:
        cands = solve_all(pos, eu, DH_DRAW, finger=FINGER_KIN)
        meta = {}
        qc, convc = refine(pos, eu, list(HOME), DH_DRAW, FINGER_KIN, meta=meta)
        cands.append((qc, convc, 0, 0, meta))
        best = None
        for q, conv, dp, do, meta in cands:
            dp2, th = axis_err(q, pos, eu, DH_DRAW, finger=FINGER_KIN)
            score = dp2 + th
            if best is None or score < best[0]:
                best = (score, q, conv, dp2, th, meta)
        score, q, conv, dp2, th, meta = best
        ok = dp2 <= 2.0
        allpass &= ok
        print(f"[{'PASS' if ok else 'FAIL'}] {tname}: dPos={dp2:.1f} ax={th:.1f} "
              f"conv={conv} it={meta['iters']} q={[round(v,1) for v in q]}")
    print("ИТОГ:", "ПРИНЯТО (все dPos<=2мм)" if allpass else "НЕ ПРИНЯТО")
