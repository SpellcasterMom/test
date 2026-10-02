"""Подбор DH под цифры пользователя (hill-climb с рестартами).

Параметры P: [off0, a1, d1, d3, a4, d4, al3, al4, al5]
Фикс.: a0=0,d0=134,al0=90, off1=90, a2=165,d2=0,al2=0, a3=161,
        a5=27,d5=0, fing=105, off2..5=0
Цели: см. check.py (TCP, DIR, J5rad, |J6J4|, J1, Zstack, AX4/5/6).

Запуск: python3 opt.py
"""
import random
from fk import fk_chain, vec_sub, vec_norm, angle_deg, reach_xy

TCP_REF = (132.0, 53.0, 514.0)
DIR_REF = (1.0, 0.0, 0.0)

BOUNDS = [
    (-180.0, 180.0),  # 0 off0
    (-80.0, 80.0),    # 1 a1
    (-80.0, 80.0),    # 2 d1
    (-40.0, 40.0),    # 3 d3
    (-40.0, 40.0),    # 4 a4
    (-40.0, 40.0),    # 5 d4
    (-180.0, 180.0),  # 6 al3
    (-180.0, 180.0),  # 7 al4
    (-180.0, 180.0),  # 8 al5
    (-180.0, 180.0),  # 9 al1
    (-180.0, 180.0),  # 10 off4
    (-180.0, 180.0),  # 11 off5
    (10.0, 70.0),     # 12 a5 (длина J5->J6 свободна!)
    (-70.0, 70.0),    # 13 d5 (латераль J5->J6 свободна!)
]

NAMES = ["off0", "a1", "d1", "d3", "a4", "d4", "al3", "al4", "al5", "al1",
         "off4", "off5", "a5", "d5"]


def build_dh(p):
    """Собрать полный DH из вектора параметров."""
    return [
        [0, 90, 134, p[0]],
        [p[1], p[9], p[2], 90],
        [165, 0, 0, 0],
        [161, 0, p[3], p[6]],
        [p[4], p[7], p[5], p[10]],
        [p[12], p[8], p[13], p[11]],
    ]


def score_of(p):
    """Взвешенная невязка (0 = идеал)."""
    dh = build_dh(p)
    r = fk_chain([0.0] * 6, dh, finger=105.0)
    J, A = r["joints"], r["axes"]
    tcp = r["tcp"]
    s = 0.0
    s += vec_norm(vec_sub(tcp, TCP_REF))
    s += abs(reach_xy(J[5]) - 54.0) * 3.0
    d46 = vec_norm(vec_sub(J[6], J[4]))
    s += abs(d46 - 54.0) * 3.0
    # Оси J4/J5/J6 убраны: это были корпуса гондол, не валы (вариант а).
    # Остаётся только направление инструмента (задано явно).
    s += (abs(J[2][2] - 134.0) + abs(J[3][2] - 299.0) + abs(J[4][2] - 460.0)) * 0.5
    s += angle_deg(r["dir"], DIR_REF) * 3.0
    return s


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def optimize(n_restarts=30, n_iters=500, seed=7):
    random.seed(seed)
    best, best_p = 1e18, None
    seeds = [
        [-27.0, 0.0, -59.5, 0.0, 27.0, 0.0, 90.0, -90.0, 0.0, 0.0, 0.0, 0.0, 27.0, 0.0],
        [153.0, 0.0, 59.5, 0.0, 27.0, 0.0, 90.0, -90.0, 0.0, 0.0, 0.0, 0.0, 27.0, 0.0],
        [90.0, 0.0, -53.0, 0.0, 27.0, 0.0, 90.0, -90.0, 0.0, 0.0, 0.0, 0.0, 27.0, 0.0],
        [-90.0, 0.0, 53.0, 0.0, 27.0, 0.0, 90.0, -90.0, 0.0, 0.0, 0.0, 0.0, 27.0, 0.0],
        [-27.0, 0.0, -59.5, 0.0, 0.0, 27.0, -65.0, -90.0, 0.0, 0.0, 0.0, 0.0, 27.0, 0.0],
        [90.0, 0.0, 27.0, 0.0, 27.0, 0.0, 90.0, -90.0, 0.0, 90.0, 0.0, 0.0, 27.0, 0.0],
        [-27.0, 0.0, -59.5, 0.0, 27.0, 0.0, 90.0, -90.0, 0.0, 0.0, 90.0, 0.0, 27.0, 0.0],
        [-27.0, 0.0, -59.5, 0.0, 27.0, 0.0, 90.0, -90.0, 0.0, 0.0, -90.0, 0.0, 27.0, 0.0],
    ]
    for rs in range(n_restarts):
        if rs < len(seeds):
            p = list(seeds[rs])
        else:
            p = [random.uniform(*BOUNDS[k]) for k in range(14)]
        st = [(HI - LO) * 0.05 for LO, HI in BOUNDS]
        sc = score_of(p)
        stagn = 0
        for _ in range(n_iters):
            q = [clamp(p[k] + random.uniform(-st[k], st[k]), *BOUNDS[k])
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


if __name__ == "__main__":
    best, p = optimize()
    print(f"BEST score={best:.2f}")
    for k in range(14):
        print(f"  {NAMES[k]:5s} = {p[k]:8.2f}")
    import check
    check.check("WINNER", build_dh(p), finger=105.0)
