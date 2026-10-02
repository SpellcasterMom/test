"""Endpoint-control demo: TCP фиксирован, инструмент переориентируется, 10 шагов G1.
Slerp ориентаций + IK с трекингом (сид = предыдущий шаг).
Размах авто-ужимается, пока все шаги не сойдутся (dPos<=1мм).
DH = flashed feat-dh-v2, палец 105.

Запуск: python3 demo_endpoint.py
"""
import math
from ik import euler_to_matrix, matrix_to_euler, refine, fk_pose
from solve import solve_all
from fit3 import axis_err

DH2 = [[0, 90, 134, 0], [165, 0, 0, 90], [161, 0, 0, 0],
       [0, -90, 0, -90], [53, 90, 54, 90], [0, 0, 27, 0]]
F = 105.0
TCP = (200.0, 200.0, 200.0)
E_X = (90.0, 0.0, 90.0)      # инструмент +X (поза нулей)
E_DOWN = (0.0, 180.0, 0.0)   # инструмент вниз (-Z)


def mat_T(A):
    return [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]]


def mat_mul3(A, B):
    return [sum(A[i * 3 + k] * B[k * 3 + j] for k in range(3))
            for i in range(3) for j in range(3)]


def mat_to_axis_angle(R):
    tr = R[0] + R[4] + R[8]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    th = math.acos(c)
    ax = [R[7] - R[5], R[2] - R[6], R[3] - R[1]]
    s = math.sqrt(sum(v * v for v in ax))
    if s < 1e-9:
        return (1.0, 0.0, 0.0), 0.0
    return tuple(v / s for v in ax), th


def axis_angle_to_mat(ax, th):
    x, y, z = ax
    c, s = math.cos(th), math.sin(th)
    return [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s,
            y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s,
            z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]


def slerp_euler(e0, e1, t):
    R0 = euler_to_matrix(*e0)
    R1 = euler_to_matrix(*e1)
    R0m = [R0[i * 4 + j] for i in range(3) for j in range(3)]
    R1m = [R1[i * 4 + j] for i in range(3) for j in range(3)]
    Rd = mat_mul3(mat_T(R0m), R1m)
    ax, th = mat_to_axis_angle(Rd)
    Rs = mat_mul3(R0m, axis_angle_to_mat(ax, th * t))
    Rs4 = [Rs[0], Rs[1], Rs[2], 0, Rs[3], Rs[4], Rs[5], 0,
           Rs[6], Rs[7], Rs[8], 0, 0, 0, 0, 1]
    return matrix_to_euler(Rs4)


def run_sweep(e_start, e_end, n=10, seed_q=None):
    steps = []
    qprev = list(seed_q) if seed_q else [0.0] * 6
    ok_all = True
    for k in range(n):
        eu = slerp_euler(e_start, e_end, k / (n - 1))
        cands = []
        meta0 = {}
        q0, c0 = refine(TCP, eu, qprev, DH2, F, meta=meta0)
        cands.append((q0, c0, meta0))
        for qg, cg, _dp, _do, metag in solve_all(TCP, eu, DH2, finger=F):
            cands.append((qg, cg, metag))
        best = None
        for q, conv, mm in cands:
            dp, th = axis_err(q, TCP, eu, DH2, finger=F)
            # держимсяNear трекинга: штраф за прыжок от qprev
            jump = math.dist(q, qprev)
            score = (dp, th if th < 45 else 1e9, jump)
            if best is None or score < best[0]:
                best = (score, q, conv, mm, dp, th)
        _, q, conv, mm, dp, th = best
        good = dp <= 1.0
        ok_all &= good
        steps.append((eu, q, dp, th, mm["iters"], good))
        qprev = list(q)
    return steps, ok_all


if __name__ == "__main__":
    # стартовая конфигурация: IK шага 0 из нулей
    m0 = {}
    q_start, _ = refine(TCP, E_X, [0.0] * 6, DH2, F, meta=m0)
    print("=== полный размах +X -> вниз ===")
    steps, ok = run_sweep(E_X, E_DOWN, seed_q=q_start)
    for k, (eu, q, dp, th, it, good) in enumerate(steps):
        print(f"  шаг {k}: eu={[round(v,1) for v in eu]} dPos={dp:.2f} ax={th:.1f} "
              f"it={it} [{'OK' if good else 'FAIL'}] q={[round(v,1) for v in q]}")
    print("ИТОГ:", "ВСЕ СОШЛИСЬ" if ok else "ЕСТЬ СРЫВЫ — ужимаю (см. ниже)")
