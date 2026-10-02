"""Валидация декартовой математики moveToPose 1:1 с прошивкой:
eulerToR vs euler_to_matrix, rToEuler round-trip, slerp, unwrap,
и полный прогон декартова движения с замером схода TCP с отрезка.

Запуск: python3 cart_test.py
"""
import math
import random
from ik import euler_to_matrix, matrix_to_euler, refine, fk_pose
from fit3 import axis_err

DH2 = [[0, 90, 134, 0], [165, 0, 0, 90], [161, 0, 0, 0],
       [0, -90, 0, -90], [53, 90, 54, 90], [0, 0, 27, 0]]
F = 105.0


def e2r(r, p, y):
    cr, sr = math.cos(math.radians(r)), math.sin(math.radians(r))
    cp, sp = math.cos(math.radians(p)), math.sin(math.radians(p))
    cy, sy = math.cos(math.radians(y)), math.sin(math.radians(y))
    return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr]


def r2e(R):
    sp = -R[6]
    if sp > 0.9999:
        # pitch=+90: слились (roll-yaw); кладем все в yaw, roll=0
        return (0.0, 90.0, math.degrees(math.atan2(-R[1], R[2])))
    if sp < -0.9999:
        return (0.0, -90.0, math.degrees(math.atan2(-R[1], -R[2])))
    return (math.degrees(math.atan2(R[7], R[8])),
            math.degrees(math.asin(sp)),
            math.degrees(math.atan2(R[3], R[0])))


def mmul(A, B):
    return [sum(A[i * 3 + k] * B[k * 3 + j] for k in range(3))
            for i in range(3) for j in range(3)]


def mT(A):
    return [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]]


print("=== 1. eulerToR == euler_to_matrix (200 случайных) ===")
w = 0.0
for _ in range(200):
    e = (random.uniform(-180, 180), random.uniform(-179, 179), random.uniform(-180, 180))
    R = euler_to_matrix(*e)
    R9 = [R[i * 4 + j] for i in range(3) for j in range(3)]
    E = e2r(*e)
    w = max(w, max(abs(a - b) for a, b in zip(R9, E)))
print("max diff:", w)

print("=== 2. rToEuler round-trip (матрицы совпадают) ===")
w = 0.0
for _ in range(200):
    e = (random.uniform(-180, 180), random.uniform(-179, 179), random.uniform(-180, 180))
    E = e2r(*e)
    r, p, y = r2e(E)
    E2 = e2r(r, p, y)
    w = max(w, max(abs(a - b) for a, b in zip(E, E2)))
print("max matrix diff:", w)

print("=== 3. декартов прогон zeros -> T1-поза, сход TCP с отрезка ===")


def fk_tcp(q):
    from fk import dh_transform, mat_mul
    T = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]
    for i in range(6):
        a, al, d, off = DH2[i]
        T = mat_mul(T, dh_transform(a, al, d, q[i] + off))
    return (T[3] + T[2] * F, T[7] + T[6] * F, T[11] + T[10] * F)


def seg_dist(pt, a, b):
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    L2 = sum(v * v for v in ab)
    if L2 < 1e-9:
        return math.dist(pt, a)
    t = max(0.0, min(1.0, sum((pt[i] - a[i]) * ab[i] for i in range(3)) / L2))
    return math.dist(pt, (a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t))


q0 = [0.0] * 6
p0, e0 = fk_pose(q0, DH2, F)
pT, eT = (200.0, 200.0, 200.0), (0.0, 146.0, 45.0)
R0 = e2r(*e0)
R1 = e2r(*eT)
Rd = mmul(mT(R0), R1)
tr = Rd[0] + Rd[4] + Rd[8]
th = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2)))
ax = [Rd[7] - Rd[5], Rd[2] - Rd[6], Rd[3] - Rd[1]]
s = math.sqrt(sum(v * v for v in ax))
ax = [v / s for v in ax]
N = 60
qprev = list(q0)
eprev = list(e0)
maxdev, maxdp, fails = 0.0, 0.0, 0
for step in range(N + 1):
    t = step / N
    ee = 2 * t * t if t < 0.5 else 1 - (-2 * t + 2) ** 2 / 2
    pp = tuple(p0[i] + ((pT[i] - p0[i]) * ee) for i in range(3))
    ang = th * ee
    ca, sa = math.cos(ang), math.sin(ang)
    x, y, z = ax
    Rm = [ca + x * x * (1 - ca), x * y * (1 - ca) - z * sa, x * z * (1 - ca) + y * sa,
          y * x * (1 - ca) + z * sa, ca + y * y * (1 - ca), y * z * (1 - ca) - x * sa,
          z * x * (1 - ca) - y * sa, z * y * (1 - ca) + x * sa, ca + z * z * (1 - ca)]
    R = mmul(R0, Rm)
    eu = r2e(R)
    # unwrap к eprev (только ±360 здесь; flip не нужен — slerp гладок)
    best, bd = None, 1e18
    for fr in (-1, 0, 1):
        for fy in (-1, 0, 1):
            c = (eu[0] + 360 * fr, eu[1], eu[2] + 360 * fy)
            d = sum(abs(c[i] - eprev[i]) for i in range(3))
            if d < bd:
                bd, best = d, c
    eu = best
    eprev = list(eu)
    meta = {}
    q, conv = refine(pp, eu, qprev, DH2, F, meta=meta)
    jump = math.dist(q, qprev)
    if jump > 45.0:
        fails += 1
        continue
    qprev = list(q)
    tcp = fk_tcp(q)
    maxdev = max(maxdev, seg_dist(tcp, p0, pT))
    dp, th2 = axis_err(q, pp, eu, DH2, finger=F)
    maxdp = max(maxdp, dp)
print(f"шагов: {N+1}, срывов трекинга: {fails}, макс. сход TCP с отрезка: {maxdev:.2f} мм, макс dPos шага: {maxdp:.2f} мм")
