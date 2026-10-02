"""Constrained-фит DH под чертёж (нули) + валидация полным solve().
Длины звеньев ПРИБИТЫ к чертежу, свободны офсеты/твисты/база.
Регуляризация к чертежным значениям ломает вырожденные жульничества.

Ворота приёмки: solve_all T1/T2/T3 -> dPos<2мм и ось-угол<6 (паритет железа).

Запуск: python3 fit2.py
"""
import random
from fk import fk_chain, vec_sub, vec_norm, angle_deg, reach_xy
from solve import solve_all
import math

TCP_REF = (132.0, 53.0, 514.0)
DIR_REF = (1.0, 0.0, 0.0)

# [lo, hi, default(чертёж), scale] — scale для регуляризации
BOUNDS = [
    (-180.0, 180.0, 150.0, 30.0),   # 0 off0 (базовый разворот; старт у оптимума)
    (-30.0, 30.0, 0.0, 10.0),       # 1 a1
    (-80.0, 80.0, 0.0, 20.0),       # 2 d1
    (-20.0, 20.0, 0.0, 10.0),       # 3 d3
    (10.0, 40.0, 27.0, 10.0),       # 4 a4
    (-20.0, 20.0, 0.0, 10.0),       # 5 d4
    (-180.0, 180.0, 90.0, 30.0),    # 6 al3
    (-180.0, 180.0, -90.0, 30.0),   # 7 al4
    (-180.0, 180.0, 0.0, 30.0),     # 8 al5
    (-45.0, 45.0, 0.0, 15.0),       # 9 al1
    (-180.0, 180.0, 0.0, 30.0),     # 10 off4
    (-180.0, 180.0, 0.0, 30.0),     # 11 off5
    (10.0, 40.0, 27.0, 10.0),       # 12 a5
    (-20.0, 20.0, 0.0, 10.0),       # 13 d5
]
NAMES = ["off0", "a1", "d1", "d3", "a4", "d4", "al3", "al4", "al5", "al1",
         "off4", "off5", "a5", "d5"]
REG_W = 0.3


def build_dh(p):
    return [
        [0, 90, 134, p[0]],
        [p[1], p[9], p[2], 90],
        [165, 0, 0, 0],
        [161, 0, p[3], p[6]],
        [p[4], p[7], p[5], p[10]],
        [p[12], p[8], p[13], p[11]],
    ]


def score_of(p):
    dh = build_dh(p)
    r = fk_chain([0.0] * 6, dh, finger=105.0)
    J = r["joints"]
    s = vec_norm(vec_sub(r["tcp"], TCP_REF))
    s += abs(reach_xy(J[5]) - 54.0) * 3.0
    s += abs(vec_norm(vec_sub(J[6], J[4])) - 54.0) * 3.0
    s += (abs(J[2][2] - 134.0) + abs(J[3][2] - 299.0) + abs(J[4][2] - 460.0)) * 0.5
    s += angle_deg(r["dir"], DIR_REF) * 3.0
    for k in range(14):
        _, _, dflt, sc = BOUNDS[k]
        s += REG_W * ((p[k] - dflt) / sc) ** 2
    return s


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def optimize(n_restarts=40, n_iters=800, seed=7):
    random.seed(seed)
    best, best_p = 1e18, None
    for rs in range(n_restarts):
        if rs == 0:
            p = [b[2] for b in BOUNDS]
        else:
            p = [random.uniform(b[0], b[1]) for b in BOUNDS]
        st = [(b[1] - b[0]) * 0.05 for b in BOUNDS]
        sc = score_of(p)
        stagn = 0
        for _ in range(n_iters):
            q = [clamp(p[k] + random.uniform(-st[k], st[k]), BOUNDS[k][0], BOUNDS[k][1])
                 for k in range(14)]
            s2 = score_of(q)
            if s2 < sc:
                p, sc = q, s2
                stagn = 0
            else:
                stagn += 1
                if stagn >= 60:
                    stagn = 0
                    st = [s * 0.5 for s in st]
        if sc < best:
            best, best_p = sc, list(p)
    return best, best_p


def axis_err(q, pos, euler, dh, finger=105.0):
    from ik import euler_to_matrix
    from fk import dh_transform, mat_mul
    Rt = euler_to_matrix(*euler)
    T = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]
    for i in range(6):
        a, al, d, off = dh[i]
        T = mat_mul(T, dh_transform(a, al, d, q[i] + off))
    tc = (T[3] + T[2] * finger, T[7] + T[6] * finger, T[11] + T[10] * finger)
    dp = math.dist(pos, tc)
    tr = sum(Rt[i * 4 + k] * T[i * 4 + k] for i in range(3) for k in range(3))
    th = math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2))))
    return dp, th


if __name__ == "__main__":
    best, p = optimize()
    print(f"BEST score={best:.2f} (регуляризация включена)")
    for k in range(14):
        print(f"  {NAMES[k]:5s} = {p[k]:8.2f}")
    dh = build_dh(p)
    print("DH_PARAMS = [")
    for row in dh:
        print(f"    [{row[0]:.2f}, {row[1]:.2f}, {row[2]:.2f}, {row[3]:.2f}],")
    print("]")
    targets = [
        ('T1 (200,200,200)+A0B146C45', (200, 200, 200), (0, 146, 45)),
        ('T2 (200,200,200)+A-90B0C0', (200, 200, 200), (-90, 0, 0)),
        ('T3 (150,0,150)+A180B0C0', (150, 0, 150), (180, 0, 0)),
    ]
    print("=== ворота solve_all ===")
    ok_all = True
    for tname, pos, eu in targets:
        sols = solve_all(pos, eu, dh)
        print(f"--- {tname}: сидов={len(sols)} ---")
        if not sols:
            print("  НЕТ СИДОВ — запястье недостижимо"); ok_all = False
            continue
        for q, conv, dp, do, meta in sols:
            dp2, th = axis_err(q, pos, eu, dh)
            gate = "PASS" if (dp2 < 2.0 and th < 6.0) else "FAIL"
            print(f"  [{gate}] conv={conv} it={meta['iters']} dPos={dp2:.1f} axori={th:.1f} "
                  f"q={[round(v,1) for v in q]}")
        if not any(axis_err(q, pos, eu, dh)[0] < 2.0 and axis_err(q, pos, eu, dh)[1] < 6.0
                   for q, _, _, _, _ in sols):
            ok_all = False
    print("ИТОГ:", "ВСЕ ВОРОТА ПРОЙДЕНЫ" if ok_all else "ЕСТЬ ПРОВАЛЫ — в прошивку рано")
