"""Прототип авто-объезда запретных коробов (только двойник, в прошивку НЕ едет).
Алгоритм — ровно тот, что ляжет в interpolator позже:
  1. Прямо: IK цели + проверка ломаной в joint-space (20 проб, как
     checkTrajectory). Чисто — едем, вейпоинтов нет.
  2. Иначе объезд: кандидаты = точки у 6 граней короба-нарушителя
     (±запас) + над верхом; для каждой: IK + проверка двух плеч
     (старт→via, via→цель). Первая чистая побеждает.
  3. Нет чистой — отказ (как сейчас прошивка).

Зеркало safety: короб ±560/700, maxReach 560, zMin {0,0,10*4},
сэмплинг звеньев 5 точек + палец + СПИСОК ЗОН (этого в accept_loops
не было — добавлено здесь).

Запуск: python3 detour.py
"""
import math
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from solve import solve_all
from ik import refine
from fit3 import axis_err
from fk import fk_chain, mat_mul, dh_transform

DH2 = [[0, 90, 134, 0], [165, 0, 0, 90], [161, 0, 0, 0],
       [0, -90, 0, -90], [53, 90, 54, 90], [0, 0, 27, 0]]
F = 105.0
ZMIN = [0, 0, 10, 10, 10, 10]
BOX = 560.0
ZMAX = 700.0
MAXR = 560.0
CLEAR = 30.0  # запас вокруг короба для via-точек


def fk_all(q):
    r = fk_chain(q, DH2, finger=0.0)
    J = list(r["joints"])
    T = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]
    for i in range(6):
        a, al, d, off = DH2[i]
        T = mat_mul(T, dh_transform(a, al, d, q[i] + off))
    tcp = (T[3] + T[2] * F, T[7] + T[6] * F, T[11] + T[10] * F)
    return J, tcp


def in_box(p, z):
    return (z[0] <= p[0] <= z[1] and z[2] <= p[1] <= z[3] and
            z[4] <= p[2] <= z[5])


def pose_safe(q, zones):
    """Зеркало checkAllLinks + зоны."""
    J, tcp = fk_all(q)
    for i in range(1, 7):
        if math.hypot(J[i][0], J[i][1]) > MAXR:
            return False
    segs = [(J[i], J[i + 1]) for i in range(1, 6)] + [(J[6], tcp)]
    for idx, (a, b) in enumerate(segs):
        li = min(idx + 1, 5)
        for s in range(6):
            t = s / 5.0
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t,
                 a[2] + (b[2] - a[2]) * t)
            if not (-BOX <= p[0] <= BOX and -BOX <= p[1] <= BOX and
                    0 <= p[2] <= ZMAX):
                return False
            if math.hypot(p[0], p[1]) > MAXR:
                return False
            if p[2] < ZMIN[li]:
                return False
            for z in zones:
                if in_box(p, z):
                    return False
    return True


def path_safe(qa, qb, zones, n=20):
    for s in range(n + 1):
        t = s / n
        q = [qa[i] + (qb[i] - qa[i]) * t for i in range(6)]
        if not pose_safe(q, zones):
            return False
    return True


def solve_pose(pos, eu, qseed, zones):
    """Выбор как прошивка: среди безопасных min(dPos+dOri+0.02·прыжок),
    затем порог 10/15. Нет годного — None."""
    import math as _m
    cands = list(solve_all(pos, eu, DH2, finger=F))
    meta = {}
    qc, cc = refine(pos, eu, list(qseed), DH2, F, meta=meta)
    cands.append((qc, cc, 0, 0, meta))
    best = None
    for q, conv, dp, do, mm in cands:
        if not pose_safe(q, zones):
            continue
        dp2, th = axis_err(q, pos, eu, DH2, finger=F)
        sc = dp2 + th + 0.02 * _m.dist(q, qseed)
        if best is None or sc < best[0]:
            best = (sc, q, dp2, th)
    if best is None:
        return None
    sc, q, dp2, th = best
    if dp2 > 10.0 or th > 15.0:
        return None
    return q, dp2, th


def solve_via(vp, eu, qseed, zones):
    """Via-точка: важна только ПОЗИЦИЯ (dPos<=5) + safety.
    Ориентация — какая получилась (eu_solved для команды)."""
    from ik import fk_pose
    cands = list(solve_all(vp, eu, DH2, finger=F))
    meta = {}
    qc, cc = refine(vp, eu, list(qseed), DH2, F, meta=meta)
    cands.append((qc, cc, 0, 0, meta))
    best = None
    for q, conv, dp, do, mm in cands:
        if not pose_safe(q, zones):
            continue
        dp2, th = axis_err(q, vp, eu, DH2, finger=F)
        if dp2 > 5.0:
            continue
        if best is None or dp2 < best[0]:
            best = (dp2, q, th)
    if best is None:
        return None
    _, q, th = best
    _, eu_solved = fk_pose(q, DH2, F)
    return q, eu_solved


def plan(qstart, pos, eu, zones, depth=2):
    """Возвращает ([via...], q_end) или None. Via — TCP-точки.
    depth: глубина рекурсии (0 = только прямо)."""
    r0 = solve_pose(pos, eu, qstart, zones)
    if r0 is not None and path_safe(qstart, r0[0], zones):
        return [], r0[0]
    if depth <= 0:
        return None
    # ищем короб-нарушитель: первый бокс, который цепляет прямую TCP
    _, tcp0 = fk_all(qstart)
    for z in zones:
        hit = False
        for s in range(21):
            t = s / 20.0
            p = (tcp0[0] + (pos[0] - tcp0[0]) * t,
                 tcp0[1] + (pos[1] - tcp0[1]) * t,
                 tcp0[2] + (pos[2] - tcp0[2]) * t)
            if in_box(p, z):
                hit = True
                break
        if not hit:
            continue
        cx = (z[0] + z[1]) / 2
        cy = (z[2] + z[3]) / 2
        cz = (z[4] + z[5]) / 2
        mid = ((tcp0[0] + pos[0]) / 2, (tcp0[1] + pos[1]) / 2,
               (tcp0[2] + pos[2]) / 2)
        cand_pts = [
            (z[0] - CLEAR, cy, cz), (z[1] + CLEAR, cy, cz),
            (cx, z[2] - CLEAR, cz), (cx, z[3] + CLEAR, cz),
            (cx, cy, z[4] - CLEAR), (cx, cy, z[5] + CLEAR),
            mid,
        ]
        for vp in cand_pts:
            if any(in_box(vp, zz) for zz in zones):
                continue
            r1 = solve_via(vp, eu, qstart, zones)
            if r1 is None:
                continue
            q_via, eu_via = r1
            if not path_safe(qstart, q_via, zones):
                continue
            sub = plan(q_via, pos, eu, zones, depth - 1)
            if sub is None:
                continue
            return [(vp, eu_via)] + sub[0], sub[1]
    # Не нашли обход гранями — пробуем RRT в joint-space.
    return plan_rrt(qstart, pos, eu, zones)


def plan_rrt(qstart, pos, eu, zones, max_nodes=400):
    """RRT в joint-space: узлы — безопасные q, рёбра — прямые отрезки.
    Возвращает ([(vp, eu)..], q_end) через TCP-точки узлов или None.
    Вейпоинты выдаём TCP-позами узлов пути (для G-кода)."""
    import random as _r
    from ik import fk_pose
    r0 = solve_pose(pos, eu, qstart, zones)
    if r0 is None:
        return None
    q_goal = r0[0]
    if not pose_safe(qstart, zones):
        return None
    LIM = [(-135, 135)] * 3 + [(-90, 90)] * 3
    nodes = [list(qstart)]
    parents = [-1]
    goal_idx = -1

    def nearest(q):
        bi, bd = 0, 1e18
        for i, n in enumerate(nodes):
            d = sum((a - b) ** 2 for a, b in zip(n, q)) ** 0.5
            if d < bd:
                bd, bi = d, i
        return bi

    def step_toward(qa, qb, step=30.0):
        import math as _m
        d = _m.dist(qa, qb)
        if d < 1e-9:
            return list(qa)
        k = min(1.0, step / d)
        out = []
        for i in range(6):
            v = qa[i] + (qb[i] - qa[i]) * k
            lo, hi = LIM[i]
            out.append(max(lo, min(hi, v)))
        return out

    for _ in range(max_nodes):
        if _r.random() < 0.3:
            qr = list(q_goal)
        else:
            qr = [_r.uniform(*LIM[i]) for i in range(6)]
        ni = nearest(qr)
        qn = step_toward(nodes[ni], qr)
        if not pose_safe(qn, zones):
            continue
        if not path_safe(nodes[ni], qn, zones):
            continue
        nodes.append(qn)
        parents.append(ni)
        if path_safe(qn, q_goal, zones):
            nodes.append(list(q_goal))
            parents.append(len(nodes) - 2)
            goal_idx = len(nodes) - 1
            break
    if goal_idx < 0:
        return None
    # цепочка + сглаживание (жадные срезы)
    chain = []
    i = goal_idx
    while i >= 0:
        chain.append(i)
        i = parents[i]
    chain.reverse()
    smooth = [chain[0]]
    i = 0
    while i < len(chain) - 1:
        j = len(chain) - 1
        while j > i + 1 and not path_safe(nodes[chain[i]], nodes[chain[j]], zones):
            j -= 1
        smooth.append(chain[j])
        i = j
    # вейпоинты TCP (все узлы кроме старта), ориентация — по факту узла
    vias = []
    for idx in smooth[1:-1]:
        p, e = fk_pose(nodes[idx], DH2, F)
        vias.append((p, e))
    return vias, list(q_goal)


if __name__ == "__main__":
    def show(r):
        if r is None:
            return "ОТКАЗ"
        if not r[0]:
            return "прямо, вейпоинтов 0"
        return "ОБЪЕЗД через " + str([(tuple(round(v) for v in p),
                                       tuple(round(v, 1) for v in e)) for p, e in r[0]])
    CUBE = (150, 250, 150, 250, 150, 250)  # короб 100^3
    print("=== сцена 1: прямая свободна (мимо куба) ===")
    print("  итог:", show(plan([0.0] * 6, (400, 0, 300), (90.0, 0.0, 90.0), [CUBE])))
    print("=== сцена 2: прямая через куб (старт — нули, цель за кубом) ===")
    print("  итог:", show(plan([0.0] * 6, (150, 350, 200), (90.0, 0.0, 90.0), [CUBE])))
    print("=== сцена 2b: TCP сквозь куб вниз ===")
    print("  итог:", show(plan([0.0] * 6, (132, 53, 200), (90.0, 0.0, 90.0),
                               [(80, 180, 0, 110, 280, 380)])))
    print("=== сцена 3: цель ВНУТРИ куба (нельзя в принципе) ===")
    print("  итог:", show(plan([0.0] * 6, (200, 200, 200), (90.0, 0.0, 90.0), [CUBE])) or "ОТКАЗ (верно)")
