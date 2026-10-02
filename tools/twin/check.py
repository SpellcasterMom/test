"""Приёмка DH-кандидата по цифрам пользователя (нулевая поза = середины серв).

Запуск: python3 check.py
"""
from fk import fk_chain, vec_sub, vec_norm, angle_deg, reach_xy

# Цель из чертежа/замеров (нулевая поза):
TCP_REF = (132.0, 53.0, 514.0)
DIR_REF = (1.0, 0.0, 0.0)      # инструмент +X
J5RAD_REF = 54.0
J6J4_REF = 54.0
J1_REF = (0.0, 0.0, 134.0)
Z_REF = (134.0, 299.0, 460.0)  # высоты J2/J3/J4
AX4_REF = (0.0, 1.0, 0.0)      # ось J4 = +Y
AX5_REF = (0.0, 0.0, 1.0)      # ось J5 = +Z
AX6_REF = (1.0, 0.0, 0.0)      # ось J6 = +X


def check(name, dh, finger=0.0, verbose=True):
    """Прогоняет FK в нулях и сверяет с эталоном. Возвращает (ok, report)."""
    q = [0.0] * 6
    r = fk_chain(q, dh, finger)
    J = r["joints"]
    rep = []
    ok_all = True

    def item(label, cond, detail):
        nonlocal ok_all
        ok_all = ok_all and cond
        rep.append((label, cond, detail))

    tcp = r["tcp"]
    item("TCP", vec_norm(vec_sub(tcp, TCP_REF)) <= 3.0,
         f"{tcp[0]:.1f},{tcp[1]:.1f},{tcp[2]:.1f} err={vec_norm(vec_sub(tcp, TCP_REF)):.1f}")
    item("DIR", angle_deg(r["dir"], DIR_REF) <= 10.0,
         f"({r['dir'][0]:.3f},{r['dir'][1]:.3f},{r['dir'][2]:.3f}) err={angle_deg(r['dir'], DIR_REF):.1f}deg")
    item("J5rad", abs(reach_xy(J[5]) - J5RAD_REF) <= 2.0,
         f"{reach_xy(J[5]):.1f}")
    item("|J6J4|", abs(vec_norm(vec_sub(J[6], J[4])) - J6J4_REF) <= 1.5,
         f"{vec_norm(vec_sub(J[6], J[4])):.1f}")
    item("J1", vec_norm(vec_sub(J[1], J1_REF)) <= 1.0,
         f"({J[1][0]:.1f},{J[1][1]:.1f},{J[1][2]:.1f})")
    item("Zstack", abs(J[2][2] - Z_REF[0]) <= 2.0 and abs(J[3][2] - Z_REF[1]) <= 2.0
         and abs(J[4][2] - Z_REF[2]) <= 2.0,
         f"J2z={J[2][2]:.1f} J3z={J[3][2]:.1f} J4z={J[4][2]:.1f}")
    A = r["axes"]
    item("AX4~Y", angle_deg(A[3], AX4_REF) <= 10.0, f"{angle_deg(A[3], AX4_REF):.1f}deg")
    item("AX5~Z", angle_deg(A[4], AX5_REF) <= 10.0, f"{angle_deg(A[4], AX5_REF):.1f}deg")
    item("AX6~X", angle_deg(A[5], AX6_REF) <= 10.0, f"{angle_deg(A[5], AX6_REF):.1f}deg")

    if verbose:
        print(f"--- {name}: {'PASS' if ok_all else 'FAIL'}")
        for label, cond, detail in rep:
            print(f"  [{'OK ' if cond else 'FAIL'}] {label:8s} {detail}")
        for i in range(7):
            print(f"    J{i}=({J[i][0]:.1f},{J[i][1]:.1f},{J[i][2]:.1f})")
    return ok_all, rep


# Старый DH из прошивки (регресс-референс; палец 0, нули старые)
DH_OLD = [
    [25, 90, 110, 0],
    [160, 0, 0, 0],
    [155, 0, 0, 0],
    [25, 90, 0, 0],
    [25, -90, 0, 0],
    [25, 0, 0, 0],
]

if __name__ == "__main__":
    check("DH_OLD (прошивка master, finger=0)", DH_OLD, finger=0.0)
