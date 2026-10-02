"""DH, выведенный из чертежа ВРУЧНУЮ (без подбора).
Нулевая поза = вертикальная рука:
  J2=(0,0,134) J3=(0,0,299) J4=(0,0,460) J5=(0,0,514) J6=(0,53,514)
  фланец=(27,53,514) TCP=(132,53,514) инструмент=+X
Оси нулей: J1=Z, J2=J3=J4=-Y, J5=+X, J6=+Z.
Кинематический палец (O6->TCP) = 27+105 = 132.

Проверка: FK нулей обязан дать TCP=(132,53,514) и инструмент +X ТОЧНО.
Ворота: solve_all T1/T2/T3.

Запуск: python3 dh_draw.py
"""
from fk import fk_chain
from solve import solve_all
from fit3 import axis_err

DH_DRAW = [
    [0, 90, 134, 0],
    [165, 0, 0, 90],
    [161, 0, 0, 0],
    [54, 90, 0, 0],
    [53, -90, 0, -90],
    [0, 90, 0, 0],
]
FINGER_KIN = 132.0  # 27 (J6->фланец) + 105 (захват)

if __name__ == "__main__":
    r = fk_chain([0.0] * 6, DH_DRAW, finger=FINGER_KIN)
    print("нули FK: TCP =", tuple(round(v, 2) for v in r["tcp"]),
          "tool =", tuple(round(v, 3) for v in r["dir"]))
    print("суставы:")
    for i, j in enumerate(r["joints"]):
        print(f"  J{i}:", tuple(round(v, 1) for v in j))
    print("ожидалось: TCP=(132,53,514) tool=(1,0,0)")
    targets = [
        ('T1 (200,200,200)+A0B146C45', (200, 200, 200), (0, 146, 45)),
        ('T2 (200,200,200)+A-90B0C0', (200, 200, 200), (-90, 0, 0)),
        ('T3 (150,0,150)+A180B0C0', (150, 0, 150), (180, 0, 0)),
    ]
    print("=== ворота solve_all (finger=132) ===")
    for tname, pos, eu in targets:
        sols = solve_all(pos, eu, DH_DRAW, finger=FINGER_KIN)
        print(f"--- {tname}: сидов={len(sols)} ---")
        for q, conv, dp, do, meta in sols:
            dp2, th = axis_err(q, pos, eu, DH_DRAW, finger=FINGER_KIN)
            print(f"  [{'PASS' if (dp2 < 2.0 and th < 6.0) else 'FAIL'}] conv={conv} "
                  f"it={meta['iters']} dPos={dp2:.1f} axori={th:.1f} "
                  f"q={[round(v,1) for v in q]}")
