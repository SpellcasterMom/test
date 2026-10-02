# tools — питон-инструменты

Раннеры демо (Serial) и двойник кинематики (офлайн-валидация без железа).

## Раннеры (запуск из этого каталога)

- `demo.py` — endpoint-демо из шапки `main.cpp`
  (12 шагов, TCP стоит).
- `demo_endpoint.py` — то же + разбор M115 по шагам.
- `demo_circle.py` — круг r=100 в наклонной
  плоскости (24 точки G1 + разбор M115).

Всем нужен `pip install pyserial`, порт
автоопределяется (`... /dev/ttyUSB0` вручную).

## twin/ — двойник FK/IK (зеркало прошивки 1:1)

- `fk.py`, `ik.py`, `solve.py` — FK, DLS, полный
  `solve()` (сиды + min-residual).
- `accept.py`, `accept_loops.py`, `check_conv.py` —
  приёмки сходимости и кругов.
- `dh_draw.py`, `check.py`, `opt.py`, `fit2.py`,
  `fit3.py` — история вывода DH (архив).
- `demo_endpoint.py` → `endpoint_sweep.py` — свип
  ориентаций (архив).
- `cart_test.py` — математика декартового G1.
- `circle_tilted.py` — подбор наклонного круга.
- `workspace_stl.py` — воксель → `reach.stl`.

Правило: любая правка кинематики сначала
доказывается здесь, потом едет в `src/`.
