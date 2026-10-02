"""Прямая кинематика 6DOF через DH-параметры. Зеркало FKSolver из прошивки.

Соглашения (как в config.h):
  DH звена: [a (мм), alpha (град), d (мм), theta_offset (град)]
  Углы суставов q — математические (градусы), как currentAngles в прошивке.
  Полный угол звена: theta = q[i] + theta_offset[i].
  Матрицы — список из 16 чисел, row-major: m[a][b] = M[a*4+b].
"""

import math


def dh_transform(a, alpha_deg, d, theta_deg):
    """Одна DH-матрица 4x4 (standard Denavit-Hartenberg)."""
    ct = math.cos(math.radians(theta_deg))
    st = math.sin(math.radians(theta_deg))
    ca = math.cos(math.radians(alpha_deg))
    sa = math.sin(math.radians(alpha_deg))
    return [
        ct, -st * ca, st * sa, a * ct,
        st, ct * ca, -ct * sa, a * st,
        0.0, sa, ca, d,
        0.0, 0.0, 0.0, 1.0,
    ]


def mat_mul(A, B):
    """Произведение матриц 4x4: C = A * B."""
    C = [0.0] * 16
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += A[i * 4 + k] * B[k * 4 + j]
            C[i * 4 + j] = s
    return C


def fk_chain(q, dh, finger=0.0):
    """Вся цепочка от базы.

    Возвращает словарь:
      joints: 7 точек [(x,y,z)] — J0 (база) + J1..J6
      axes:   оси суставов + фланец: axes[0] = ось J1 (всегда Z),
              axes[k] (k>=1) = третья колонка R0k = ось сустава k+1;
              axes[6] = ось фланца (направление пальца без длины)
      tcp:    позиция кончика пальца = J6 + finger * axes[6]
    """
    T = [1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]
    joints = [(0.0, 0.0, 0.0)]
    axes = [(0.0, 0.0, 1.0)]  # ось J1 — всегда Z базы
    for i in range(6):
        a, alpha, d, off = dh[i]
        Ti = dh_transform(a, alpha, d, q[i] + off)
        T = mat_mul(T, Ti)
        joints.append((T[3], T[7], T[11]))
        axes.append((T[2], T[6], T[10]))
    fx, fy, fz = axes[6]
    x6, y6, z6 = joints[6]
    tcp = (x6 + fx * finger, y6 + fy * finger, z6 + fz * finger)
    return {"joints": joints, "axes": axes, "tcp": tcp, "dir": axes[6]}


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def angle_deg(a, b):
    """Угол между единичными векторами в градусах."""
    d = max(-1.0, min(1.0, vec_dot(a, b)))
    return math.degrees(math.acos(d))


def reach_xy(p):
    """Цилиндрический радиус от оси Z."""
    return math.hypot(p[0], p[1])
