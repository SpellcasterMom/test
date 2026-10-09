"""Численный IK (damped least-squares) — зеркало refineSolution из прошивки.

Работает с ПОЛНОЙ FK-цепочкой (не الكرة запястья), поэтому точность не зависит
от сферичности запястья. Используется для spot-check DH-кандидатов офлайн:
если тут сходится — сойдётся и в прошивке (тот же алгоритм).

Запуск: python3 ik.py
"""
import math
from fk import fk_chain, mat_mul, dh_transform


def euler_to_matrix(roll, pitch, yaw):
    """Эйлер ZYX -> матрица 4x4. R = Rz(yaw)*Ry(pitch)*Rx(roll), градусы.
    !!! Конвенция должна совпадать с прошивкой — см. euler_from_matrix ниже;
    при расхождении поправить обе функции по fk_solver.cpp !!!"""

# Параметры DLS — 1:1 как NLS_* в ik_solver.cpp прошивки
LAMBDA = 0.3
ORI_W = 30.0
DELTA = 1e-4
MAX_STEP = 0.2
MAX_ITER = 100
TOL_POS = 1.0
TOL_ORI = 1.0
WEPS = 1e-4

# Лимиты серв (математические градусы), как DEFAULT_SERVO_CONFIG
SERVO_LIMITS = [(-135.0, 135.0)] * 3 + [(-90.0, 90.0)] * 3


def clamp_angle(i, angle):
    lo, hi = SERVO_LIMITS[i]
    return max(lo, min(hi, angle))


def euler_to_matrix(roll, pitch, yaw):
    """Эйлер ZYX -> матрица 4x4. R = Rz(yaw)*Ry(pitch)*Rx(roll), градусы."""
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0,
        -sp, cp * sr, cp * cr, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]


def solve6(A, b):
    """Гаусс с частичным выбором ведущего для 6x6. Возвращает x или None."""
    m = [row[:] + [bi] for row, bi in zip([list(r) for r in A], b)]
    for col in range(6):
        piv = max(range(col, 6), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-9:
            return None
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, 6):
            f = m[r][col] / m[col][col]
            for k in range(col, 7):
                m[r][k] -= f * m[col][k]
    x = [0.0] * 6
    for i in range(5, -1, -1):
        s = m[i][6] - sum(m[i][k] * x[k] for k in range(i + 1, 6))
        x[i] = s / m[i][i]
    return x


def ik_error(qrad, Rt, pt, dh, finger):
    """Вектор ошибки 6x1: позиция TCP (мм) + ориентация axis-angle x ORI_W.
    1:1 с IKSolver::ikError (включая ветку th≈π)."""
    qdeg = [r * 180.0 / math.pi for r in qrad]
    T = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    for i in range(6):
        a, alpha, d, off = dh[i]
        T = mat_mul(T, dh_transform(a, alpha, d, qdeg[i] + off))
    # TCP = конец цепочки + палец вдоль оси фланца (третья колонка R06)
    p = (T[3] + T[2] * finger, T[7] + T[6] * finger, T[11] + T[10] * finger)
    # R_err = Rt * R^T
    Re = [[sum(Rt[i * 4 + k] * T[j * 4 + k] for k in range(3))
           for j in range(3)] for i in range(3)]
    tr = Re[0][0] + Re[1][1] + Re[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    th = math.acos(c)
    ax = Re[2][1] - Re[1][2]
    ay = Re[0][2] - Re[2][0]
    az = Re[1][0] - Re[0][1]
    s = math.sqrt(ax * ax + ay * ay + az * az)
    if s > 1e-6:
        ex, ey, ez = ax / s * th, ay / s * th, az / s * th
    elif c < 0:
        ex, ey, ez = th, 0.0, 0.0
    else:
        ex, ey, ez = 0.0, 0.0, 0.0
    return [pt[0] - p[0], pt[1] - p[1], pt[2] - p[2],
            ex * ORI_W, ey * ORI_W, ez * ORI_W]


def refine(target_pos, target_euler, q_seed, dh, finger=105.0, meta=None):
    """DLS-доводка seed'а. Возвращает (q_out[6], converged). q_seed: градусы."""
    Rt = euler_to_matrix(*target_euler)
    pt = target_pos
    qr = [math.radians(clamp_angle(i, q_seed[i])) for i in range(6)]
    qmin = [math.radians(SERVO_LIMITS[i][0]) for i in range(6)]
    qmax = [math.radians(SERVO_LIMITS[i][1]) for i in range(6)]
    e = ik_error(qr, Rt, pt, dh, finger)
    best_qr, best_n = list(qr), sum(v * v for v in e) ** 0.5
    enorm = best_n
    if meta is not None:
        meta.update(
            seed_pos=math.sqrt(e[0] ** 2 + e[1] ** 2 + e[2] ** 2),
            seed_ori=math.sqrt(e[3] ** 2 + e[4] ** 2 + e[5] ** 2) / ORI_W * 180.0 / math.pi,
        )
    tol_ori_w = math.radians(TOL_ORI) * ORI_W
    lam, converged, it, stall = LAMBDA, False, 0, 0
    for it in range(MAX_ITER):
        ep = math.sqrt(e[0] ** 2 + e[1] ** 2 + e[2] ** 2)
        eo = math.sqrt(e[3] ** 2 + e[4] ** 2 + e[5] ** 2)
        if ep <= TOL_POS and eo <= tol_ori_w:
            converged = True
            break
        J = [[0.0] * 6 for _ in range(6)]
        for j in range(6):
            qp = list(qr)
            qp[j] += DELTA
            e2 = ik_error(qp, Rt, pt, dh, finger)
            for i in range(6):
                J[i][j] = (e2[i] - e[i]) / DELTA
        lam2 = lam * lam
        dq = [0.0] * 6
        dir_ok = False
        for _round in range(6):
            A = [[(lam2 if i == j else 0.0) + sum(J[i][k] * J[j][k] for k in range(6))
                  for j in range(6)] for i in range(6)]
            y = solve6(A, e)
            if y is None:
                break
            for k in range(6):
                dq[k] = -sum(J[i][k] * y[i] for i in range(6))
            viol = -1
            for k in range(6):
                if (qr[k] <= qmin[k] + WEPS and dq[k] < 0) or \
                   (qr[k] >= qmax[k] - WEPS and dq[k] > 0):
                    viol = k
                    break
            if viol < 0:
                dir_ok = True
                break
            for i in range(6):
                J[i][viol] = 0.0
        if not dir_ok:
            lam = min(lam * 4.0, 1e4)
            stall += 1
            if stall >= 15:
                break
            continue
        for k in range(6):
            dq[k] = max(-MAX_STEP, min(MAX_STEP, dq[k]))
        step, nn, improved = 1.0, enorm, False
        en = e
        for _ in range(10):
            qtry = [min(qmax[k], max(qmin[k], qr[k] + dq[k] * step)) for k in range(6)]
            en = ik_error(qtry, Rt, pt, dh, finger)
            nn = sum(v * v for v in en) ** 0.5
            if nn < enorm:
                improved = True
                break
            step *= 0.5
        if not improved:
            lam = min(lam * 4.0, 1e4)
            stall += 1
            if stall >= 15:
                break
            continue
        qr, e, enorm = qtry, en, nn
        lam = max(lam * 0.5, 1e-3)
        stall = 0
        if nn < best_n:
            best_n, best_qr = nn, list(qr)
    if not converged:
        qr = best_qr
    q_out = [clamp_angle(i, qr[i] * 180.0 / math.pi) for i in range(6)]
    if meta is not None:
        meta.update(iters=it, converged=converged)
    return q_out, converged


import math  # noqa: E402  (re-import для __main__ блока)
from fk import fk_chain as _fk_chain  # noqa: E402


def matrix_to_euler(R):
    """R (16 чисел) -> (roll, pitch, yaw) ZYX, градусы. Обратное euler_to_matrix."""
    sp = max(-1.0, min(1.0, -R[8]))
    pitch = math.degrees(math.asin(sp))
    roll = math.degrees(math.atan2(R[9], R[10]))
    yaw = math.degrees(math.atan2(R[4], R[0]))
    return (roll, pitch, yaw)


def fk_pose(qdeg, dh, finger=105.0):
    """FK-поза: (pos, euler). Для генерации целей из известных q."""
    T = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    for i in range(6):
        a, alpha, d, off = dh[i]
        T = mat_mul(T, dh_transform(a, alpha, d, qdeg[i] + off))
    pos = (T[3] + T[2] * finger, T[7] + T[6] * finger, T[11] + T[10] * finger)
    return pos, matrix_to_euler(T)


DH_WINNER = [
    [0, 90, 134, 158.73],
    [1.19, -18.07, 46.32, 90],
    [165, 0, 0, 0],
    [161, 0, -3.52, 4.31],
    [22.68, -55.30, 12.84, -7.82],
    [25.59, 5.20, -5.78, -16.50],
]


if __name__ == "__main__":
    import random
    random.seed(7)
    q_trues = [
        [0, 0, 0, 0, 0, 0],
        [30, -20, 40, 0, 30, 0],
        [-40, 30, -30, 20, -40, 10],
        [60, 45, -60, -30, 45, 20],
    ]
    print("=== spot-check DLS на DH_WINNER: цель=FK(q_true), сид=q_true+возмущение ===")
    for qt in q_trues:
        pos, eu = fk_pose(qt, DH_WINNER)
        for trial in range(3):
            seed = [clamp_angle(i, qt[i] + random.uniform(-15, 15)) for i in range(6)]
            meta = {}
            q, conv = refine(pos, eu, seed, DH_WINNER, meta=meta)
            p2, e2 = fk_pose(q, DH_WINNER)
            dp = math.dist(pos, p2)
            do = math.dist(eu, e2)
            print(f"q_true={qt} seedΔ±15° -> conv={conv} it={meta.get('iters')} "
                  f"dPos={dp:.2f} dOri~{do:.2f} q={[round(v,1) for v in q]}")
    print("=== те же цели с сидом из нулей ===")
    for qt in q_trues:
        pos, eu = fk_pose(qt, DH_WINNER)
        meta = {}
        q, conv = refine(pos, eu, [0]*6, DH_WINNER, meta=meta)
        p2, e2 = fk_pose(q, DH_WINNER)
        print(f"q_true={qt} seed=0 -> conv={conv} it={meta.get('iters')} "
              f"dPos={math.dist(pos,p2):.2f} q={[round(v,1) for v in q]}")
