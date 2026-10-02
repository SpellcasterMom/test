"""Полный конвейер solve() прошивки: TCP->запястье, геометрические сиды
(elbow down/up), solveOrientation через R03, DLS-доводка, выбор min-residual.
Всё параметризовано DH — можно гонять кандидатов офлайн.

Запуск: python3 solve.py
"""
import math
from ik import (euler_to_matrix, refine, clamp_angle, ORI_W,
                DH_WINNER, fk_pose)
from fk import dh_transform, mat_mul

FW_DH = [[25, 90, 110, 0], [160, 0, 0, 90], [155, 0, 0, 0],
         [25, 90, 0, 0], [25, -90, 0, 0], [25, 0, 0, 0]]
FINGER = 105.0


def t03(q1, q2, q3, dh):
    """T03 = T01*T12*T23, q математические градусы."""
    T = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    for i, q in enumerate((q1, q2, q3)):
        a, alpha, d, off = dh[i]
        T = mat_mul(T, dh_transform(a, alpha, d, q + off))
    return T


def decompose(R):
    """Декомпозиция R36 -> (q4,q5,q6). 1:1 с прошивкой (включая кардан)."""
    sp = -R[8]
    if sp > 0.9999:
        return (90.0, math.degrees(math.atan2(R[1], R[5])), 0.0)
    if sp < -0.9999:
        return (-90.0, math.degrees(math.atan2(-R[1], R[5])), 0.0)
    return (math.degrees(math.asin(sp)),
            math.degrees(math.atan2(R[9], R[10])),
            math.degrees(math.atan2(R[4], R[0])))


def solve_position(wx, wy, wz, dh, clamped=False):
    """Геометрия позиции запястья. Возвращает (q1,q2,q3) elbow-down или None.
    clamped=True: вместо None вернуть зажатое решение (рука максимально
    вытянута/сложена в сторону цели) — сид для DLS, не точное решение."""
    d0, a0 = dh[0][2], dh[0][0]
    L1, L2 = dh[1][0], dh[2][0]
    q1 = math.degrees(math.atan2(wy, wx))
    r = math.hypot(wx, wy) - a0
    s = wz - d0
    dist = math.hypot(r, s)
    if dist > (L1 + L2) or dist < abs(L1 - L2):
        if not clamped:
            return None
        # Зажать (r,s) в достижимое кольцо — сид тянется к цели как может
        if dist < 1e-9:
            return None
        lo = abs(L1 - L2) + 1e-3
        hi = (L1 + L2) - 1e-3
        k = (hi if dist > L1 + L2 else lo) / dist
        r, s = r * k, s * k
        dist = math.hypot(r, s)
    cos_q3 = max(-1.0, min(1.0, (r * r + s * s - L1 * L1 - L2 * L2) / (2 * L1 * L2)))
    q3 = -math.degrees(math.acos(cos_q3))
    q3r = math.radians(q3)
    q2 = math.degrees(math.atan2(s, r) - math.atan2(L2 * math.sin(q3r),
                                                   L1 + L2 * math.cos(q3r)))
    return (q1, q2 - dh[1][3], q3)


def seed_arm(w, dh):
    """Сид q1-q3 с компенсацией запястья (fixed-point, 3 итерации):
    J4_target = O6_target - (O6_straight - J4), где straight = FK(q123,0,0,0).
    Убирает систематические ~107мм ошибки seed-модели под сферическое запястье.
    O6 = фланец joints[6] (в DH-v2 это фланец, не J6-центр)."""
    from fk import fk_chain as _fk
    g = solve_position(*w, dh)
    if g is None:
        g = solve_position(*w, dh, clamped=True)
    if g is None:
        return None, w
    jt = w
    for _ in range(3):
        q1, q2, q3 = g
        r = _fk([q1, q2, q3, 0.0, 0.0, 0.0], dh, finger=0.0)
        J4 = r["joints"][3]
        O6s = r["joints"][6]
        jt = (w[0] - (O6s[0] - J4[0]),
              w[1] - (O6s[1] - J4[1]),
              w[2] - (O6s[2] - J4[2]))
        g2 = solve_position(*jt, dh)
        if g2 is None:
            g2 = solve_position(*jt, dh, clamped=True)
        if g2 is None:
            break
        g = g2
    return g, jt


def solve_orientation(A, B, C, q1, q2, q3, dh):
    R06 = euler_to_matrix(A, B, C)
    T = t03(q1, q2, q3, dh)
    # R36 = R03^T * R06
    R36 = [0.0] * 16
    for i in range(3):
        for j in range(3):
            R36[i * 4 + j] = sum(T[k * 4 + i] * R06[k * 4 + j] for k in range(3))
    R36[15] = 1.0
    return decompose(R36)


def solve_all(pos, euler, dh, finger=FINGER):
    """Полный solve: 2 сида -> refine -> min-residual. Возвращает список
    (q, conv, dpos, dori_deg)."""
    A, B, C = euler
    R06 = euler_to_matrix(A, B, C)
    tool = (R06[2], R06[6], R06[10])
    w = (pos[0] - tool[0] * finger, pos[1] - tool[1] * finger, pos[2] - tool[2] * finger)
    seeds = []
    g, jt = seed_arm(w, dh)
    if g is not None:
        q1, q2, q3 = g
        q4, q5, q6 = solve_orientation(A, B, C, q1, q2, q3, dh)
        seeds.append([clamp_angle(i, v) for i, v in enumerate((q1, q2, q3, q4, q5, q6))])
        # elbow up (на компенсированной цели jt)
        q3u = -q3
        q3r = math.radians(q3u)
        r = math.hypot(jt[0], jt[1]) - dh[0][0]
        s = jt[2] - dh[0][2]
        L1, L2 = dh[1][0], dh[2][0]
        q2u = math.degrees(math.atan2(s, r) - math.atan2(L2 * math.sin(q3r),
                                                        L1 + L2 * math.cos(q3r))) - dh[1][3]
        q4, q5, q6 = solve_orientation(A, B, C, q1, q2u, q3u, dh)
        seeds.append([clamp_angle(i, v) for i, v in enumerate((q1, q2u, q3u, q4, q5, q6))])
    # + сид «нули» (центральная конфигурация — сильная ветвь для верхней полусферы)
    seeds.append([0.0] * 6)
    out = []
    for sd in seeds:
        meta = {}
        q, conv = refine(pos, euler, sd, dh, finger, meta=meta)
        p2, e2 = fk_pose(q, dh, finger)
        dp = math.dist(pos, p2)
        do = math.dist(euler, e2)
        out.append((q, conv, dp, do, meta))
    out.sort(key=lambda t: t[2] + t[3])
    return out


if __name__ == "__main__":
    targets = [
        ('T1 (200,200,200)+A0B146C45', (200, 200, 200), (0, 146, 45)),
        ('T2 (200,200,200)+A-90B0C0', (200, 200, 200), (-90, 0, 0)),
        ('T3 (150,0,150)+A180B0C0', (150, 0, 150), (180, 0, 0)),
    ]
    for dh, name in ((FW_DH, 'FW-DH'), (DH_WINNER, 'DH_WINNER')):
        print(f"########## {name} ##########")
        for tname, pos, eu in targets:
            sols = solve_all(pos, eu, dh)
            print(f"--- {tname}: сидов={len(sols)} ---")
            for q, conv, dp, do, meta in sols:
                print(f"  conv={conv} it={meta['iters']} dPos={dp:.1f} dOri~{do:.1f} "
                      f"seed=({meta['seed_pos']:.0f},{meta['seed_ori']:.0f}) "
                      f"q={[round(v,1) for v in q]}")
