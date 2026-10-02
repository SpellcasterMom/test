"""Проверка сходимости FK/IK на двойнике (DH_DRAW, палец 132).
Тест 1 (round-trip): случайные q -> FK -> solve_all -> невязка/итерации.
Тест 2 (цели): сетка декартовых целей + ориентации -> доля PASS.

Запуск: python3 check_conv.py
"""
import random
from dh_draw import DH_DRAW, FINGER_KIN
from solve import solve_all
from fit3 import axis_err
from ik import fk_pose

random.seed(11)
LIM = [(-135, 135)] * 3 + [(-90, 90)] * 3


def rand_q():
    return [random.uniform(*LIM[i]) for i in range(6)]


print("=== ТЕСТ 1: round-trip (n=30) ===")
conv_n = it_sum = 0
dp_max = th_max = 0.0
fails = []
for t in range(30):
    qt = rand_q()
    pos, eu = fk_pose(qt, DH_DRAW, FINGER_KIN)
    sols = solve_all(pos, eu, DH_DRAW, finger=FINGER_KIN)
    if not sols:
        fails.append((qt, "NOSEED"))
        continue
    q, conv, dp, do, meta = sols[0]
    dp2, th = axis_err(q, pos, eu, DH_DRAW, finger=FINGER_KIN)
    dp_max, th_max = max(dp_max, dp2), max(th_max, th)
    if conv:
        conv_n += 1
        it_sum += meta["iters"]
    elif not (dp2 < 2.0 and th < 6.0):
        fails.append((qt, f"dPos={dp2:.1f} ax={th:.1f}"))
print(f"conv=True: {conv_n}/30, средний it={it_sum/max(conv_n,1):.1f}, "
      f"max dPos={dp_max:.2f} max ax={th_max:.2f}, жёстких провалов={len(fails)}")
for qt, why in fails[:8]:
    print(f"  FAIL q={[round(v,1) for v in qt]}: {why}")

print("=== ТЕСТ 1b: round-trip, запястье operational (|q4-6|<45, n=30) ===")
conv_n = it_sum = 0
dp_max = th_max = 0.0
fails = []
made = 0
while made < 30:
    qt = rand_q()
    if any(abs(qt[i]) > 45 for i in (3, 4, 5)):
        continue
    made += 1
    pos, eu = fk_pose(qt, DH_DRAW, FINGER_KIN)
    sols = solve_all(pos, eu, DH_DRAW, finger=FINGER_KIN)
    if not sols:
        fails.append((qt, "NOSEED"))
        continue
    q, conv, dp, do, meta = sols[0]
    dp2, th = axis_err(q, pos, eu, DH_DRAW, finger=FINGER_KIN)
    dp_max, th_max = max(dp_max, dp2), max(th_max, th)
    if conv:
        conv_n += 1
        it_sum += meta["iters"]
    elif not (dp2 < 2.0 and th < 6.0):
        fails.append((qt, f"dPos={dp2:.1f} ax={th:.1f}"))
print(f"conv=True: {conv_n}/30, средний it={it_sum/max(conv_n,1):.1f}, "
      f"max dPos={dp_max:.2f} max ax={th_max:.2f}, жёстких провалов={len(fails)}")
for qt, why in fails[:8]:
    print(f"  FAIL q={[round(v,1) for v in qt]}: {why}")

print("=== ТЕСТ 2b: operational-сетка (середина хода, инструмент вниз/вперёд) ===")
oris = {
    "down": (0, 90, 0),
    "fwd": (-90, 0, 0),
    "tilt45": (-45, 45, 0),
}
grid = [(x, y, z) for x in (180, 260, 340) for y in (-80, 0, 80)
        for z in (80, 200, 320)]
tot = p = convt = 0
bad = []
for pos in grid:
    for oname, eu in oris.items():
        sols = solve_all(pos, eu, DH_DRAW, finger=FINGER_KIN)
        tot += 1
        ok = False
        if sols:
            q, conv, dp, do, meta = sols[0]
            dp2, th = axis_err(q, pos, eu, DH_DRAW, finger=FINGER_KIN)
            ok = dp2 < 2.0 and th < 6.0
            convt += conv
        if ok:
            p += 1
        else:
            bad.append((pos, oname,
                        f"{dp2:.1f}/{th:.1f}" if sols else "NOSEED"))
print(f"PASS: {p}/{tot} ({100.0*p/tot:.0f}%), строгих conv=True: {convt}/{tot}")
for pos, oname, why in bad[:15]:
    print(f"  FAIL {pos} {oname}: {why}")
