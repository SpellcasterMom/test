# Как работает прошивка 6DOF — блок-схемы

Рамка: `+X` вперёд, `+Y` влево, `+Z` вверх. Стол = XY (`z=0`).
Порог годности IK: **10 мм / 15°**. Палец захвата 105 мм.

## 1. Тракт команды G0 (декарт)

```mermaid
flowchart TD
    S["Serial: G0 X.. Y.. Z.. A.. B.. C.. J.. F.."] --> P["GCodeParser<br/>params + флаги hasPosition/hasGripper"]
    P --> D["Диспетчер COMMANDS[]<br/>имя → handler"]
    D --> H["handleG0 / handleG1"]
    H --> ARC["Подъём-дуга: отрезок старт→цель<br/>+ синус-горб h=120, n=24"]
    ARC --> SOL["solve_all на каждом сэмпле<br/>2 гео-сида + нули → refine DLS"]
    SOL --> TH{"dPos ≤ 10 мм<br/>и dOri ≤ 15°?"}
    TH -- "нет" --> FAIL["ERROR: No accurate safe IK<br/>best dPos/dOri в M115"]
    TH -- "да" --> SF["SafetyChecker<br/>короб, zMin звеньев, бокс колонны"]
    SF -- "нарушение" --> FAIL
    SF -- "чисто" --> DET["DetourPlanner<br/>плохие сэмплы → WPS nwp"]
    DET --> MOV["Interpolator.moveToDetour<br/>lerp + slerp, IK на каждом шаге"]
    MOV --> SRV["ServoController.setAngle<br/>math → ШИМ → PCA9685"]
    SRV --> OK["OK + M115: INTERP DET nwp/minZ/WPS"]

    H -. "M104 P7=.." .-> DIR["moveTo напрямую<br/>без IK"]
    DIR --> SRV
    H -. "M108 S=.." .-> GR["Сустав 6 = S%<br/>через interpolator"]
    GR --> SRV
```

Параллельно крутится задача дисплея: TCP XYZ + Roll/Pitch/Yaw, ABS над PCA,
последняя команда в статус-баре. Диагностика — всегда `M115`.

## 2. Ядро IK: solve_all + refine

```mermaid
flowchart TD
    T["Цель: TCP + Эйлер A/B/C"] --> W["Точка запястья<br/>TCP − toolDir × 105"]
    W --> SEED["Сиды: локоть вниз / локоть вверх<br/>+ нули (верхняя полусфера)"]
    SEED --> GEO["Геометрия плеча: q1,q2,q3<br/>по треугольнику 165/161"]
    GEO --> WR["Ориентация: q4,q5,q6<br/>R36 = R03ᵀ × R06 → разложение"]
    WR --> DLS["refine: DLS-итерации<br/>clamp ±135° / ±90°"]
    DLS --> FK["FK-проверка: dPos + ось-ошибка"]
    FK --> SORT["Сортировка по min-residual<br/>+ штраф за дальность от текущей позы"]
    SORT --> BEST["Лучшее решение → порог 10/15"]
```

## 3. Отказ — это штатно

Неточного решения НЕТ в прошивке специально: лучше честный
`ERROR` + цифры в M115, чем прыжок в мусор. Частые причины:

- точка слишком близко к телу (локоть складывается за ±135°);
- середина дуги проходит через бокс колонны `[-65,65]²×[0,180]`;
- ориентация по умолчанию `+X (90,0,90)` не вытягивается — задать A/B/C явно.
