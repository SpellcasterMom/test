"""Фит DH в FW-совместимой компоновке + ворота solve_all.
Компоновка: длинные звенья строго a[1]=165, a[2]=161 (seed-модель прошки),
ноль плеча вертикаль (off1=90). Свободны: база, мелкие офсеты, запястье.

Ворота: T1 обязан PASS (dPos<2, axori<6) — иначе регресс против текущего
железа (0.3мм/2.8 на FW-DH). T2/T3 — stretch.

Запуск: python3 fit3.py
"""
import random, math
from fk import fk_chain, vec_sub, vec_norm, angle_deg, reach_xy
from solve import solve_all

TCP_REF = (132.0, 53.0, 514.0)
DIR_REF = (1.0, 0.0, 0.0)

# param: [lo, hi, default, scale]
BOUNDS = [
    (-180, 180, 150.0, 30.0),   # 0 off0
    (-30, 30, 0.0, 10.0),       # 1 a0
    (100, 170, 134.0, 10.0),    # 2 d0
    (-45, 45, 0.0, 15.0),       # 3 al1
    (-30, 30, 0.0, 10.0),       # 4 d1
    (-45, 45, 0.0, 15.0),       # 5 al2
    (-30, 30, 0.0, 10.0),       # 6 d2
    (-45, 45, 0.0, 15.0),       # 7 off2
    (5, 50, 27.0, 10.0),        # 8 a3
    (-180, 180, 90.0, 30.0),    # 9 al3
    (-30, 30, 0.0, 10.0),       # 10 d3
    (-180, 180, 0.0, 30.0),     # 11 off3
    (5, 50, 27.0, 10.0),        # 12 a4
    (-180, 180, -90.0, 30.0),   # 13 al4
    (-30, 30, 0.0, 10.0),       # 14 d4
    (-180, 180, 0.0, 30.0),     # 15 off4
    (5, 50, 27.0, 10.0),        # 16 a5
    (-180, 180, 0.0, 30.0),     # 17 al5
    (-30, 30, 0.0, 10.0),       # 18 d5
    (-180, 180, 0.0, 30.0),     # 19 off5
]
NAMES = ["off0", "a0", "d0", "al1", "d1", "al2", "d2", "off2",
         "a3", "al3", "d3", "off3", "a4", "al4", "d4", "off4",
         "a5", "al5", "d5", "off5"]
REG_W = 0.3
N = len(BOUNDS)


def build_dh(p):
    return [
        [p[1], 90, p[2], p[0]],
        [165, p[3], p[4], 90],
        [161, p[5], p[6], p[7]],
        [p[8], p[9], p[10], p[11]],
        [p[12], p[13], p[14], p[15]],
        [p[16], p[17], p[18], p[19]],
    ]


def score_of(p):
    dh = build_dh(p)
    r = fk_chain([0.0] * 6, dh, finger=105.0)
    J = r["joints"]
    s = vec_norm(vec_sub(r["tcp"], TCP_REF))
    s += abs(reach_xy(J[5]) - 54.0) * 3.0
    s += abs(vec_norm(vec_sub(J[6], J[4])) - 54.0) * 3.0
    s += (abs(J[1][2] - 134.0) + abs(J[2][2] - 299.0) + abs(J[3][2] - 460.0)) * 0.5
    s += angle_deg(r["dir"], DIR_REF) * 3.0
    for k in range(N):
        _, _, dflt, sc = BOUNDS[k]
        s += REG_W * ((p[k] - dflt) / sc) ** 2
    return s


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def optimize(n_restarts=40, n_iters=800, seed=7):
    random.seed(seed)
    results = []
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
                 for k in range(N)]
            s2 = score_of(q)
            if s2 < sc:
                p, sc = q, s2
                stagn = 0
            else:
                stagn += 1
                if stagn >= 60:
                    stagn = 0
                    st = [s * 0.5 for s in st]
        results.append((sc, list(p)))
    results.sort(key=lambda t: t[0])
    return results


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
    return dp, math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2))))


if __name__ == "__main__":
    cands = optimize()
    print("топ-5 по скору нулей:")
    for sc, _ in cands[:5]:
        print(f"  score={sc:.2f}")
    targets = [
        ('T1 (200,200,200)+A0B146C45', (200, 200, 200), (0, 146, 45)),
        ('T2 (200,200,200)+A-90B0C0', (200, 200, 200), (-90, 0, 0)),
        ('T3 (150,0,150)+A180B0C0', (150, 0, 150), (180, 0, 0)),
    ]
    winner = None
    for sc, p in cands[:8]:
        dh = build_dh(p)
        print(f"=== кандидат score={sc:.2f} ===")
        t1pass = False
        for tname, pos, eu in targets:
            sols = solve_all(pos, eu, dh)
            if not sols:
                print(f"  {tname}: НЕТ СИДОВ")
                continue
            flags = []
            for q, conv, dp, do, meta in sols:
                dp2, th = axis_err(q, pos, eu, dh)
                ok = dp2 < 2.0 and th < 6.0
                flags.append(ok)
                print(f"  {tname}: [{'PASS' if ok else 'FAIL'}] conv={conv} "
                      f"it={meta['iters']} dPos={dp2:.1f} axori={th:.1f} "
                      f"q={[round(v,1) for v in q]}")
            if tname.startswith('T1') and any(flags):
                t1pass = True
        if t1pass and winner is None:
            winner = (sc, p)
    if winner:
        sc, p = winner
        print(f"\n*** WINNER score={sc:.2f} ***")
        for k in range(N):
            print(f"  {NAMES[k]:5s} = {p[k]:8.2f}")
        print("DH_PARAMS = {")
        for row in build_dh(p):
            print(f"    {{{row[0]:.2f}, {row[1]:.2f}, {row[2]:.2f}, {row[3]:.2f}}},")
        print("};")
    else:
        print("\n*** НИКТО НЕ ПРОШЁЛ T1 — в прошивку рано ***")
