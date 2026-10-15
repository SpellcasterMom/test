/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-10-01
 *
 * Описание   : Реализация решателя обратной кинематики (IK) для 6-DOF
 *              манипулятора. Алгоритм разделяется на два независимых этапа:
 *
 *              ЭТАП 1 — ГЕОМЕТРИЧЕСКОЕ РЕШЕНИЕ ПОЗИЦИИ (суставы q1–q3):
 *                - Определение угла поворота основания (q1) по координатам x, y
 *                - Проекция задачи на 2D-плоскость (r, s)
 *                - Применение теоремы косинусов для двухзвенного манипулятора
 *                - Получение углов плеча (q2) и локтя (q3)
 *
 *              ЭТАП 2 — DH-ДЕКОМПОЗИЦИЯ ОРИЕНТАЦИИ (суставы q4–q6):
 *                - Вычисление матрицы R03 (поворот от основания до 3-го сустава)
 *                - Преобразование целевой ориентации (Эйлер ZYX) в матрицу R06
 *                - Вычисление матрицы запястья: R36 = R03⁻¹ · R06
 *                - Декомпозиция R36 на углы Эйлера (q4, q5, q6) с обработкой замка кардана
 *
 * Математическая основа:
 *   - DH-параметры (Denavit-Hartenberg) для описания кинематической цепи
 *   - Теорема косинусов для решения треугольника в 2D-проекции
 *   - Матрицы поворота 3×3 (в составе матриц 4×4) для описания ориентации
 *   - Декомпозиция Эйлера ZYX: R = Rz(ψ) · Ry(θ) · Rx(φ)
 *
 * =====================================================================
 * ВАЖНОЕ ИСПРАВЛЕНИЕ (Last modify: 2026-09-25):
 * =====================================================================
 * IK решает позицию ЗАПЯСТЬЯ (сустав 6), а не TCP (кончик пальца).
 * Пользователь задаёт TCP, поэтому перед solvePosition() вычитается
 * длина пальца вдоль направления целевой ориентации:
 *
 *   wrist = TCP − fingerLength × toolDir
 *
 * где toolDir — ось Z инструмента в базовой СК (третий столбец R06).
 *
 * Без этой коррекции G0 X100 Y100 Z100 давал Position X=83 Y=127 Z=116
 * (ошибка ~20 мм: IK решал для запястья, а M114 выводил TCP).
 */

#include "ik_solver.h"
#include <cmath>

// ============================================================================
// Параметры DLS-уточнения (см. refineSolution)
// ============================================================================
// NLS_LAMBDA   — демпфирование (λ² добавляется к диагонали J·Jᵀ)
// NLS_ORI_W    — вес ориентации в векторе ошибки (1° ≈ 0.5 усл.мм)
// NLS_DELTA    — шаг численного якобиана (рад)
// NLS_MAX_STEP — максимальный шаг одного сустава за итерацию (рад)
static const float NLS_LAMBDA = 0.3f;
static const float NLS_ORI_W = 30.0f;
static const float NLS_DELTA = 1e-4f;
static const float NLS_MAX_STEP = 0.2f;

// ============================================================================
// КОНСТРУКТОР И ИНИЦИАЛИЗАЦИЯ
// ============================================================================

/**
 * Конструктор IKSolver по умолчанию.
 *
 * Инициализирует DH-параметры и ограничения сервоприводов из глобальных
 * конфигурационных массивов DH_PARAMS, LINK_LENGTHS и DEFAULT_CONFIG.
 *
 * Дополнительно: fingerLength = GRIPPER_FINGER_LENGTH — длина пальца захвата,
 * которая вычитается из целевой позиции TCP в solve()/solveAll()/isReachable().
 */
IKSolver::IKSolver() {
    // Копирование DH-параметров: [a, alpha, d] для каждого из 6 звеньев
    for (int i = 0; i < 6; i++) {
        a[i]     = DH_PARAMS[i][0];  // длина звена (мм)
        alpha[i] = DH_PARAMS[i][1];  // угол поворота (градусы)
        d[i]     = DH_PARAMS[i][2];  // осевое смещение (мм)
        thetaOff[i] = DH_PARAMS[i][3]; // сдвиг нуля (J2: +90° = вертикаль)
        linkLen[i] = LINK_LENGTHS[i]; // длина звена для геометрического IK (мм)
    }
    // Копирование ограничений для 7 каналов (6 суставов + захват)
    for (int i = 0; i < 7; i++) {
        servoLimits[i] = DEFAULT_CONFIG.servos[i];
    }
    for (int i = 0; i < 6; i++) {
        qminRad[i] = servoLimits[i].minAngle * RAD_PER_DEG;
        qmaxRad[i] = servoLimits[i].maxAngle * RAD_PER_DEG;
    }
    // Длина пальца (для смещения TCP → запястье)
    fingerLength = GRIPPER_FINGER_LENGTH;
    // Параметры численного решателя по умолчанию
    numMaxIter = 100;
    numTolPos = 1.0f;
    numTolOri = 1.0f;
    // Внутренний FK должен повторять ту же цепочку, что общий fkSolver
    syncFk();
}

// ============================================================================
// МЕТОДЫ НАСТРОЙКИ ПАРАМЕТРОВ
// ============================================================================

/**
 * Установка DH-параметров извне (перезаписывает значения по умолчанию).
 *
 * @param params — массив [6][4], где для каждого звена заданы:
 *                 [a, alpha, d, theta_offset]
 */
void IKSolver::setDHParams(const float params[6][4]) {
    for (int i = 0; i < 6; i++) {
        a[i]     = params[i][0];
        alpha[i] = params[i][1];
        d[i]     = params[i][2];
        thetaOff[i] = params[i][3];
    }
    syncFk();
}

/**
 * Установка длин звеньев (для геометрического IK).
 * Эти значения используются в solvePosition() как L1 и L2
 * для расчёта треугольника двухзвенного манипулятора.
 */
void IKSolver::setLinkLengths(const int lengths[6]) {
    for (int i = 0; i < 6; i++)
        linkLen[i] = lengths[i];
    syncFk();
}

/**
 * Установка механических ограничений сервоприводов.
 * Ограничения задаются в градусах и используются методом clampAngle().
 */
void IKSolver::setServoLimits(const ServoConfig servos[7]) {
    for (int i = 0; i < 7; i++)
        servoLimits[i] = servos[i];
    for (int i = 0; i < 6; i++) {
        qminRad[i] = servoLimits[i].minAngle * RAD_PER_DEG;
        qmaxRad[i] = servoLimits[i].maxAngle * RAD_PER_DEG;
    }
}

/**
 * Настройка численного решателя.
 *
 * @param maxIter   — лимит итераций DLS
 * @param tolPosMM  — допуск по позиции TCP (мм)
 * @param tolOriDeg — допуск по ориентации (градусы)
 */
void IKSolver::setNumericParams(int maxIter, float tolPosMM, float tolOriDeg) {
    numMaxIter = maxIter;
    numTolPos = tolPosMM;
    numTolOri = tolOriDeg;
}

/**
 * Протолкнуть DH/длина/палец во внутренний fkSolver, чтобы численное
 * уточнение работало по ТОЙ ЖЕ цепочке, что общий FK (M114) и safety.
 */
void IKSolver::syncFk() {
    float dh[6][4];
    for (int i = 0; i < 6; i++) {
        dh[i][0] = a[i];
        dh[i][1] = alpha[i];
        dh[i][2] = d[i];
        dh[i][3] = thetaOff[i];
    }
    fkSolver.setDHParams(dh);
    fkSolver.setLinkLengths(linkLen);
    fkSolver.setFingerLength(fingerLength);
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ РАБОТЫ С МАТРИЦАМИ
// ============================================================================

/**
 * R36 = R03ᵀ · R06 по блоку 3×3: 27 умножений вместо полного 4×4
 * (64 + транспонирование). Остальное в R36 — как раньше: единичная база + [3][3]=1.
 */
void IKSolver::matMul3x3T(const Mat4 &R03, const Mat4 &R06, Mat4 &R36) {
    R36 = Mat4::identity();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R36.m[i][j] = R03.m[0][i] * R06.m[0][j]
                        + R03.m[1][i] * R06.m[1][j]
                        + R03.m[2][i] * R06.m[2][j];
}

// ============================================================================
// ПРЕОБРАЗОВАНИЕ ЭЙЛЕРОВЫХ УГЛОВ (ZYX) В МАТРИЦУ ПОВОРОТА
// ============================================================================

/**
 * Преобразование углов Эйлера ZYX в матрицу поворота 4×4.
 *
 * Порядок обхода осей (внутри-наружу): X → Y → Z
 * Итоговая матрица: R = Rz(ψ) · Ry(θ) · Rx(φ)
 */
Mat4 IKSolver::eulerToMatrix(float roll, float pitch, float yaw) {
    float cr = cos(roll),  sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);
    float cy = cos(yaw),   sy = sin(yaw);

    Mat4 R;

    // Формула произведения Rz · Ry · Rx (ZYX Euler angles)
    R.m[0][0] = cy*cp;
    R.m[1][0] = sy*cp;
    R.m[2][0] = -sp;

    R.m[0][1] = cy*sp*sr - sy*cr;
    R.m[1][1] = sy*sp*sr + cy*cr;
    R.m[2][1] = cp*sr;

    R.m[0][2] = cy*sp*cr + sy*sr;
    R.m[1][2] = sy*sp*cr - cy*sr;
    R.m[2][2] = cp*cr;

    R.m[3][3] = 1.0f;

    return R;
}

// ============================================================================
// ДЕКОМПОЗИЦИЯ МАТРИЦЫ ПОВОРОТА НА УГЛЫ ЭЙЛЕРА ZYX
// ============================================================================

/**
 * Декомпозиция матрицы поворота R36 на углы Эйлера ZYX (q4, q5, q6).
 * Обрабатывает замок кардана при pitch = ±90°.
 */
void IKSolver::decomposeRotation(const Mat4 &R, float &q4, float &q5, float &q6) {
    float sp = -R.m[2][0];

    if (sp > 0.9999f) {
        // Замок кардана: pitch = +90°
        q4 = 90.0f;
        q5 = atan2(R.m[0][1], R.m[1][1]) * DEG_PER_RAD;
        q6 = 0.0f;
    } else if (sp < -0.9999f) {
        // Замок кардана: pitch = -90°
        q4 = -90.0f;
        q5 = atan2(-R.m[0][1], R.m[1][1]) * DEG_PER_RAD;
        q6 = 0.0f;
    } else {
        // Обычный случай
        q4 = asin(sp) * DEG_PER_RAD;
        q5 = atan2(R.m[2][1], R.m[2][2]) * DEG_PER_RAD;
        q6 = atan2(R.m[1][0], R.m[0][0]) * DEG_PER_RAD;
    }
}

// ============================================================================
// ОСНОВНОЙ МЕТОД РЕШЕНИЯ ОБРАТНОЙ КИНЕМАТИКИ (solve)
// ============================================================================

/**
 * Основной метод solve() — двухэтапное решение обратной кинематики.
 *
 * ВХОД: целевая поза target = (x, y, z, roll, pitch, yaw)
 * ВЫХОД: углы суставов result = (q1, q2, q3, q4, q5, q6, gripper)
 *
 * ВАЖНО (Last modify: 2026-09-25):
 *   IK решает позицию ЗАПЯСТЬЯ (сустав 6), а не TCP (кончик пальца).
 *   Пользователь задаёт TCP, поэтому перед solvePosition() вычитаем
 *   длину пальца вдоль направления целевой ориентации:
 *     wrist = TCP − fingerLength × toolDir
 *
 * @param target — целевая поза TCP (позиция + ориентация)
 * @param result — массив углов суставов (6 суставов + захват)
 * @return true если решение найдено, false если позиция недостижима
 *
 * ЭТАП 3 — refineSolution(): доводка по полной FK-цепочке (DLS),
 * устраняет ошибку несферического запястья.
 */
bool IKSolver::solve(const Pose &target, JointAngles &result) {
    float q1, q2, q3, q4, q5, q6;

    // ================================================================
    // КОРРЕКЦИЯ ЦЕЛИ: TCP → ЗАПЯСТЬЕ
    // ================================================================
    // Целевая ориентация → матрица R06, из неё извлекаем ось Z
    // (направление выдвижения пальца в базовой СК).
    Mat4 R06 = eulerToMatrix(target.orientation.x * RAD_PER_DEG,
                             target.orientation.y * RAD_PER_DEG,
                             target.orientation.z * RAD_PER_DEG);
    // Ось Z инструмента — третий столбец матрицы R06
    Vec3 toolDir = {R06.m[0][2], R06.m[1][2], R06.m[2][2]};

    // Смещаем цель с TCP на запястье:
    //   wrist = TCP − fingerLength × toolDir
    float wx = target.position.x - toolDir.x * fingerLength;
    float wy = target.position.y - toolDir.y * fingerLength;
    float wz = target.position.z - toolDir.z * fingerLength;

    // ================================================================
    // ЭТАП 1: Решение задачи позиции (суставы 1–3) для ЗАПЯСТЬЯ
    // ================================================================
    if (!solvePosition(wx, wy, wz, q1, q2, q3)) {
        return false;
    }

    // ================================================================
    // ЭТАП 2: Решение задачи ориентации (суставы 4–6)
    // ================================================================
    // Rt уже посчитана выше — матричное ядро без повторного eulerToMatrix.
    solveOrientationM(R06, q1, q2, q3, q4, q5, q6);

    // ================================================================
    // ПОСТ-ОБРАБОТКА: Ограничение углов механическими пределами
    // ================================================================
    result.angles[0] = clampAngle(0, q1);  // основание
    result.angles[1] = clampAngle(1, q2);  // плечо
    result.angles[2] = clampAngle(2, q3);  // локоть
    result.angles[3] = clampAngle(3, q4);  // запястье (pitch)
    result.angles[4] = clampAngle(4, q5);  // запястье (yaw)
    result.angles[5] = clampAngle(5, q6);  // запястье (roll)
    result.angles[6] = GRIP_CLOSED_DEG;  // захват управляется отдельно

    // ЭТАП 3: доводка по полной цепочке — матричное ядро напрямую
    // (обёртка refineSolution пересчитала бы eulerToMatrix ещё раз).
    refineSolutionM(target.position, R06, result, nullptr);

    return true;
}

// ============================================================================
// ГЕОМЕТРИЧЕСКОЕ РЕШЕНИЕ ЗАДАЧИ ПОЗИЦИИ (СУСТАВЫ 1–3)
// ============================================================================

/**
 * Геометрическое решение задачи позиции (суставы q1, q2, q3).
 *
 * ЗАДАЧА: Найти углы q1, q2, q3, чтобы ЗАПЯСТЬЕ (конец 3-го звена)
 *         оказалось в точке (x, y, z).
 *
 * ВНИМАНИЕ: входные (x, y, z) — это позиция ЗАПЯСТЬЯ, а не TCP.
 * Вызывающий код (solve()/solveAll()/isReachable()) уже вычел длину
 * пальца из целевого TCP.
 */
bool IKSolver::solvePosition(float x, float y, float z, float &q1, float &q2, float &q3) {
    return solveArm(x, y, z, false, q1, q2, q3);
}

// ============================================================================
// ЗАЖАТАЯ ВЕРСИЯ solvePosition (только сид для DLS)
// ============================================================================

/**
 * Та же геометрия, но вместо отказа при недостижимости (r,s) зажимаются
 * в достижимое кольцо [|L1−L2|, L1+L2]. Двойник: solve.py solve_position(clamped).
 */
bool IKSolver::solvePositionClamped(float x, float y, float z, float &q1, float &q2, float &q3) {
    return solveArm(x, y, z, true, q1, q2, q3);
}

/**
 * Общий мотор solvePosition/solvePositionClamped (см. .h).
 * Шаги 1–3, 5–7 идентичны; разница только в шаге 4: отказ vs зажатие (r,s).
 */
bool IKSolver::solveArm(float x, float y, float z, bool clamped,
                         float &q1, float &q2, float &q3) {
    float d0 = d[0];
    float a0 = a[0];

    q1 = atan2(y, x);

    float r = sqrt(x * x + y * y) - a0;
    float s = z - d0;

    float L1 = a[1];
    float L2 = a[2];

    float dist = sqrt(r * r + s * s);
    if (dist > (L1 + L2) || dist < fabs(L1 - L2)) {
        if (!clamped) return false;      // строгий путь: отказ (isReachable/solve)
        if (dist < 1e-9f) return false;
        float lo = fabs(L1 - L2) + 1e-3f;
        float hi = (L1 + L2) - 1e-3f;
        float k = (dist > L1 + L2) ? hi / dist : lo / dist;
        r *= k;
        s *= k;
    }

    float cosQ3 = (r * r + s * s - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cosQ3 = constrain(cosQ3, -1.0f, 1.0f);
    q3 = -acos(cosQ3);   // «elbow down» — q3 < 0

    float sinQ3 = sin(q3);
    float C = L1 + L2 * cosQ3;
    float S = L2 * sinQ3;
    q2 = atan2(s, r) - atan2(S, C);

    // Ноль плеча — ВЕРТИКАЛЬ (как в solvePosition)
    q1 = q1 * DEG_PER_RAD;
    q2 = q2 * DEG_PER_RAD - thetaOff[1];
    q3 = q3 * DEG_PER_RAD;

    return true;
}

// ============================================================================
// СИД РУКИ С КОМПЕНСАЦИЕЙ ЗАПЯСТЬЯ (fixed-point)
// ============================================================================

bool IKSolver::seedArm(float wx, float wy, float wz,
                       float &q1, float &q2, float &q3,
                       float &jx, float &jy, float &jz) {
    bool haveArm = false;
    if (solvePosition(wx, wy, wz, q1, q2, q3)) haveArm = true;
    else if (solvePositionClamped(wx, wy, wz, q1, q2, q3)) haveArm = true;
    if (!haveArm) return false;

    jx = wx; jy = wy; jz = wz;
    float qq[6] = {0, 0, 0, 0, 0, 0};
    for (int it = 0; it < 3; it++) {
        qq[0] = q1; qq[1] = q2; qq[2] = q3;
        // Сустав 3 в joints = J4 (конец звеньев 165+161),
        // сустав 6 = фланец O6 — один проход вместо двух getJointPosition.
        Vec3 joints[7], tcp, fdir;
        Mat4 T06;
        fkSolver.getPose(qq, joints, tcp, fdir, T06);
        (void)tcp; (void)fdir; (void)T06;
        Vec3 pJ4 = joints[3];
        Vec3 pO6 = joints[6];
        jx = wx - (pO6.x - pJ4.x);
        jy = wy - (pO6.y - pJ4.y);
        jz = wz - (pO6.z - pJ4.z);
        float n1, n2, n3;
        if (solvePosition(jx, jy, jz, n1, n2, n3)) { q1 = n1; q2 = n2; q3 = n3; }
        else if (solvePositionClamped(jx, jy, jz, n1, n2, n3)) { q1 = n1; q2 = n2; q3 = n3; }
        else break;
    }
    return true;
}

// ============================================================================
// РЕШЕНИЕ ЗАДАЧИ ОРИЕНТАЦИИ (СУСТАВЫ 4–6): DH-ДЕКОМПОЗИЦИЯ
// ============================================================================

/**
 * Решение задачи ориентации (суставы q4, q5, q6) методом DH-декомпозиции.
 *
 * АЛГОРИТМ:
 *   1. Вычисляем T03 через fkSolver.getTransform (полные DH + theta_offset),
 *      R03 — верхний левый блок 3×3.
 *   2. R06 из целевых углов Эйлера (roll, pitch, yaw).
 *   3. R36 = R03ᵀ · R06 (R03 ортогональна → R03⁻¹ = R03ᵀ).
 *   4. Декомпозиция R36 → q4, q5, q6 (с обработкой замка кардана).
 */
bool IKSolver::solveOrientation(float A, float B, float C,
                                 float q1, float q2, float q3,
                                 float &q4, float &q5, float &q6) {
    // Обёртка: один перевод эйлеры→матрица, дальше матричное ядро.
    Mat4 Rt = eulerToMatrix(A * RAD_PER_DEG, B * RAD_PER_DEG, C * RAD_PER_DEG);
    return solveOrientationM(Rt, q1, q2, q3, q4, q5, q6);
}

/**
 * Ядро solveOrientation по готовой Rt (см. .h): R03 из FK, R36 3×3, декомпозиция.
 */
bool IKSolver::solveOrientationM(const Mat4 &Rt, float q1, float q2, float q3,
                                 float &q4, float &q5, float &q6) {
    // T03 через общий FK-движок: учитывает theta_offset и полные DH
    // (вручную собранные T01/T12/T23 их игнорировали и дублировали FK).
    // q1..q3 здесь МАТЕМАТИЧЕСКИЕ (градусы), офсеты добавит computeFK.
    float qq[6] = {q1, q2, q3, 0, 0, 0};
    Mat4 T03 = fkSolver.getTransform(qq, 2);

    // Извлечение R03 (верхний левый блок 3×3)
    Mat4 R03 = Mat4::identity();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R03.m[i][j] = T03.m[i][j];

    // --------------------------------------------------
    // R36 = R03ᵀ · Rt (3×3, без 4×4-накладных)
    // --------------------------------------------------
    Mat4 R36;
    matMul3x3T(R03, Rt, R36);

    // --------------------------------------------------
    // Декомпозиция R36 → q4, q5, q6
    // --------------------------------------------------
    decomposeRotation(R36, q4, q5, q6);

    return true;
}

// ============================================================================
// ПРОВЕРКА ДОСТИЖИМОСТИ ЦЕЛЕВОЙ ПОЗИЦИИ
// ============================================================================

/**
 * Проверка достижимости целевой позиции.
 *
 * ВНИМАНИЕ: target — это TCP. Перед проверкой вычитаем длину пальца,
 * чтобы проверять достижимость ЗАПЯСТЬЯ (как в solve()).
 */
bool IKSolver::isReachable(const Pose &target) {
    // Смещаем TCP → запястье
    Mat4 R06 = eulerToMatrix(target.orientation.x * RAD_PER_DEG,
                             target.orientation.y * RAD_PER_DEG,
                             target.orientation.z * RAD_PER_DEG);
    Vec3 toolDir = {R06.m[0][2], R06.m[1][2], R06.m[2][2]};

    float wx = target.position.x - toolDir.x * fingerLength;
    float wy = target.position.y - toolDir.y * fingerLength;
    float wz = target.position.z - toolDir.z * fingerLength;

    float q1, q2, q3;
    return solvePosition(wx, wy, wz, q1, q2, q3);
}

// ============================================================================
// ПОИСК ВСЕХ ВОЗМОЖНЫХ РЕШЕНИЙ (ELBOW UP / ELBOW DOWN)
// ============================================================================

/**
 * Поиск всех возможных решений обратной кинематики.
 *
 * Для задачи позиции (суставы 1–3) существует два решения:
 *   1) «elbow down» — q3 < 0 (стандартная конфигурация)
 *   2) «elbow up»  — q3 > 0 (альтернативная)
 *
 * ВНИМАНИЕ: target — это TCP. Перед решением вычитаем длину пальца.
 *
 * ИСПРАВЛЕНО: в решении «elbow up» q3 нужно вернуть в радианы перед
 * вызовом sin/cos (C++ ожидает радианы). Раньше был баг: q3 уже в градусах
 * после solvePosition(), а sin/cos применялись к градусам → мусор в q2.
 *
 * @param target     — целевая поза TCP
 * @param solutions  — массив найденных решений (максимум 4, но обычно 1–2)
 * @return количество найденных решений (0, 1 или 2)
 */
int IKSolver::solveAll(const Pose &target, JointAngles solutions[4],
                       IKSolutionMeta *meta) {
    int count = 0;
    float q1, q2, q3, q4, q5, q6;

    // === Коррекция TCP → запястье (Rt считается ОДИН раз на цель;
    // сиды ниже переиспользуют её через solveOrientationM/refineSolutionM) ===
    Mat4 R06 = eulerToMatrix(target.orientation.x * RAD_PER_DEG,
                             target.orientation.y * RAD_PER_DEG,
                             target.orientation.z * RAD_PER_DEG);
    Vec3 toolDir = {R06.m[0][2], R06.m[1][2], R06.m[2][2]};

    float wx = target.position.x - toolDir.x * fingerLength;
    float wy = target.position.y - toolDir.y * fingerLength;
    float wz = target.position.z - toolDir.z * fingerLength;

    // ================================================================
    // РЕШЕНИЯ 1-2: elbow down/up на КОМПЕНСИРОВАННОЙ цели J4
    // ================================================================
    // Запястье не сферическое: чистый solvePosition(wx,wy,wz)
    // кладёт J4 в точку фланца O6 — сид уходит мимо и DLS не дотягивает.
    // seedArm() итеративно переносит цель на J4. DH при этом НЕ трогаем.
    float q1c, q2c, q3c, jx, jy, jz;
    if (seedArm(wx, wy, wz, q1c, q2c, q3c, jx, jy, jz)) {
        // --- Решение 1: elbow down (Rt уже готова — без повторного eulerToMatrix) ---
        q1 = q1c; q2 = q2c; q3 = q3c;
        solveOrientationM(R06, q1, q2, q3, q4, q5, q6);
        solutions[count].setAll(
            clampAngle(0, q1), clampAngle(1, q2), clampAngle(2, q3),
            clampAngle(3, q4), clampAngle(4, q5), clampAngle(5, q6),
            0);
        count++;

        // --- Решение 2: elbow up (на той же компенсированной цели jx,jy,jz) ---
        // q3 после seedArm() уже в градусах; для sin/cos переводим в радианы.
        q3 = -q3;   // elbow down → elbow up (градусы)

        float r = sqrt(jx * jx + jy * jy) - a[0];
        float s = jz - d[0];
        float L1 = a[1];
        float L2 = a[2];

        float q3rad = q3 * RAD_PER_DEG;
        float q2rad = atan2(s, r) - atan2(L2 * sin(q3rad), L1 + L2 * cos(q3rad));
        q2 = q2rad * DEG_PER_RAD - thetaOff[1];   // в градусы + ноль плеча

        solveOrientationM(R06, q1, q2, q3, q4, q5, q6);
        solutions[count].setAll(
            clampAngle(0, q1), clampAngle(1, q2), clampAngle(2, q3),
            clampAngle(3, q4), clampAngle(4, q5), clampAngle(5, q6),
            0);
        count++;
    }

    // ================================================================
    // РЕШЕНИЕ 3: сид «нули» — центральная конфигурация
    // ================================================================
    // Сильная ветвь для верхней полусферы; для цели «нули» даёт 0 итераций.
    // Место в solutions[4] есть (сейчас занято ≤2), 4-й слот — для кандидата
    // от текущей позы (добавляет обработчик G0).
    if (count < 4) {
        solutions[count].setAll(0, 0, 0, 0, 0, 0, 0);
        count++;
    }

    // ================================================================
    // ЭТАП 3: ЧИСЛЕННОЕ УТОЧНЕНИЕ (DLS)
    // ================================================================
    // Запястье НЕ сферическое (J4→O6 ~107 мм по чертежу robot1),
    // поэтому геометрические решения доводим по полной FK-цепочке. Seed'ы уже в solutions[].
    // Матричное ядро напрямую: обёртка refineSolution пересчитала бы Rt ещё раз на сид.
    for (int i = 0; i < count; i++) {
        IKSolutionMeta m;
        refineSolutionM(target.position, R06, solutions[i], &m);
        if (meta) meta[i] = m;
    }

    return count;
}

// ============================================================================
// ОГРАНИЧЕНИЕ УГЛА МЕХАНИЧЕСКИМИ ПРЕДЕЛАМИ
// ============================================================================

/**
 * Ограничение угла механическими пределами сервопривода.
 *
 * @param servoId — номер сервопривода (0–6)
 * @param angle   — вычисленный угол (градусы)
 * @return угол, обрезанный до [minAngle, maxAngle]
 */
float IKSolver::clampAngle(int servoId, float angle) {
    float minA = servoLimits[servoId].minAngle;
    float maxA = servoLimits[servoId].maxAngle;
    return constrain(angle, minA, maxA);
}

// ============================================================================
// ВЕКТОР ОШИБКИ ДЛЯ ЧИСЛЕННОГО РЕШАТЕЛЯ
// ============================================================================

/**
 * Вектор ошибки 6x1 для конфигурации q (радианы).
 *
 * e[0..2] — ошибка позиции TCP (мм): цель минус факт по FK.
 * e[3..5] — ошибка ориентации × NLS_ORI_W в формате axis-angle (θ·u):
 *   R_err = Rt·Rᵀ, θ — угол, u — ось. Честная метрика без сворачивания
 *   через синус (0.5·Σcross при θ→180° показывает 0 и тянет решатель
 *   к ложному минимуму «вверх ногами»).
 */
void IKSolver::ikError(const float qrad[6], const Mat4 &Rt, const Vec3 &pt, float e[6]) {
    // q уже в радианах — однопроходная поза без кругового перевода рад→град→рад.
    Vec3 joints[7];
    Vec3 p, fdir;
    Mat4 T06;
    fkSolver.getPoseRad(qrad, joints, p, fdir, T06);
    (void)joints;
    (void)fdir;
    // R06 — верхний левый 3×3 из T06 (getRotationMatrix(q,5) давал то же).
    Mat4 R = Mat4::identity();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R.m[i][j] = T06.m[i][j];

    e[0] = pt.x - p.x;
    e[1] = pt.y - p.y;
    e[2] = pt.z - p.z;

    // R_err = Rt · Rᵀ (поворот, переводящий текущую ориентацию в целевую)
    float Re[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Re[i][j] = Rt.m[i][0]*R.m[j][0] + Rt.m[i][1]*R.m[j][1] + Rt.m[i][2]*R.m[j][2];

    float tr = Re[0][0] + Re[1][1] + Re[2][2];
    float c = (tr - 1.0f) * 0.5f;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    float th = acos(c);

    // Ось из кососимметричной части: (R−Rᵀ)∨ = 2·sinθ·u
    float ax = Re[2][1] - Re[1][2];
    float ay = Re[0][2] - Re[2][0];
    float az = Re[1][0] - Re[0][1];
    float s = sqrt(ax*ax + ay*ay + az*az);
    float ex = 0, ey = 0, ez = 0;
    if (s > 1e-6f) {
        // Обычный случай θ∈(0,π): e_o = θ·u
        ex = ax / s * th;
        ey = ay / s * th;
        ez = az / s * th;
    } else if (c < 0) {
        // θ≈π: ось вырождена. Берём ось X текущей СК как направление —
        // грубо, но сохраняет градиент и не даёт ложный ноль.
        ex = th; ey = 0; ez = 0;
    }
    // else θ≈0: e_o = 0, всё верно
    e[3] = ex * NLS_ORI_W;
    e[4] = ey * NLS_ORI_W;
    e[5] = ez * NLS_ORI_W;
}

// ============================================================================
// РЕШЕНИЕ СЛАУ 6x6
// ============================================================================

/**
 * Решение СЛАУ A·x = b методом Гаусса с частичным выбором ведущего элемента.
 *
 * @return false если матрица вырождена (ведущий элемент ~0).
 */
bool IKSolver::solve6(const float A[6][6], const float b[6], float x[6]) {
    float m[6][7];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) m[i][j] = A[i][j];
        m[i][6] = b[i];
    }
    for (int col = 0; col < 6; col++) {
        int piv = col;
        for (int r = col + 1; r < 6; r++)
            if (fabs(m[r][col]) > fabs(m[piv][col])) piv = r;
        if (fabs(m[piv][col]) < 1e-9f) return false;
        if (piv != col) {
            for (int k = col; k < 7; k++) {
                float t = m[col][k]; m[col][k] = m[piv][k]; m[piv][k] = t;
            }
        }
        for (int r = col + 1; r < 6; r++) {
            float f = m[r][col] / m[col][col];
            for (int k = col; k < 7; k++) m[r][k] -= f * m[col][k];
        }
    }
    for (int i = 5; i >= 0; i--) {
        float s = m[i][6];
        for (int k = i + 1; k < 6; k++) s -= m[i][k] * x[k];
        x[i] = s / m[i][i];
    }
    return true;
}

// ============================================================================
// ЧИСЛЕННОЕ УТОЧНЕНИЕ РЕШЕНИЯ (DAMPED LEAST-SQUARES)
// ============================================================================

/**
 * Доводка seed-решения по полной FK-цепочке.
 *
 * Итерация DLS:
 *   1) e = ошибка (позиция мм + ориентация × вес), FK — эталон;
 *   2) J — численный якобиан 6x6 (возмущение NLS_DELTA по каждому суставу);
 *   3) (J·Jᵀ + λ²I)·y = e  →  dq = −Jᵀ·y (МИНУС! шаг против градиента,
 *      обрезан NLS_MAX_STEP);
 *   4) q += dq с обрезкой по лимитам серв (line search принимает шаг
 *      только если ошибка строго упала).
 *
 * Возвращает лучшее найденное (не обязательно последнее) q.
 * Захват (angles[6]) не трогается.
 */
/**
 * Численное уточнение по матричной цели (ядро DLS, без эйлеров).
 * Вектор ошибки: позиция TCP (мм) + axis-angle(Rt·Rᵀ)×вес —
 * непрерывная метрика, без разрывов эйлеровой свёртки (±180°),
 * которые давали мусорные всплески якобиана («большой градиент»).
 * Эйлеры живут только на границах (G-code/M115); внутри — матрицы.
 */
bool IKSolver::refineSolutionM(const Vec3 &pt, const Mat4 &Rt,
                               JointAngles &q, IKSolutionMeta *meta) {
    if (meta) { meta->iters = 0; meta->converged = false; meta->dbgDq0 = -1; meta->dbgN1 = -1; }

    float qr[6];
    // Лимиты в радианах — из кэша setServoLimits (без 12 mults на вызов).
    const float *qmin = qminRad;
    const float *qmax = qmaxRad;
    for (int i = 0; i < 6; i++) {
        qr[i] = clampAngle(i, q.angles[i]) * RAD_PER_DEG;
    }

    float e[6];
    ikError(qr, Rt, pt, e);

    // Нормы держим в КВАДРАТАХ (сравнения nn < enorm эквивалентны,
    // sqrt — только для meta-диагностики). Было 2 sqrt/итерацию + до 10 в line search.
    float bestQr[6], bestN2 = 0;
    for (int k = 0; k < 6; k++) bestQr[k] = qr[k];
    for (int i = 0; i < 6; i++) bestN2 += e[i] * e[i];

    float tolOriW = numTolOri * RAD_PER_DEG * NLS_ORI_W;
    float tolPos2 = numTolPos * numTolPos;
    float tolOriW2 = tolOriW * tolOriW;
    float enorm2 = bestN2;

    // Расхождение seed'а для диагностики (M115 покажет seed → best)
    if (meta) {
        meta->seedPos = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
        meta->seedOri = sqrt(e[3]*e[3] + e[4]*e[4] + e[5]*e[5]) / NLS_ORI_W * DEG_PER_RAD;
    }

    // Левенберг-Марквардт: λ адаптивная, шаг принимается только если
    // ошибка строго упала (backtracking line search). Без этого шаги
    // Гаусса-Ньютона перелетают и лучшей остаётся seed-точка.
    float lam = NLS_LAMBDA;
    bool converged = false;
    int it = 0, stall = 0;
    for (it = 0; it < numMaxIter; it++) {
        float ep2 = e[0]*e[0] + e[1]*e[1] + e[2]*e[2];
        float eo2 = e[3]*e[3] + e[4]*e[4] + e[5]*e[5];
        if (ep2 <= tolPos2 && eo2 <= tolOriW2) { converged = true; break; }

        // Численный якобиан: столбец j = (e(q+δj) − e(q)) / δ
        // (× invDelta вместо деления — 36 делений/итерацию убраны).
        static const float INV_DELTA = 1.0f / NLS_DELTA;
        float J[6][6], e2[6], qp[6];
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) qp[k] = qr[k];
            qp[j] += NLS_DELTA;
            ikError(qp, Rt, pt, e2);
            for (int i = 0; i < 6; i++) J[i][j] = (e2[i] - e[i]) * INV_DELTA;
        }

        // DLS-направление: (J·Jᵀ + λ²I)·y = e ; dq = −Jᵀ·y.
        // МИНУС обязателен: минимизируем ||e + J·dq||² + λ²||dq||², т.е.
        // (JᵀJ + λ²I)·dq = −Jᵀe. Без минуса — подъём вместо спуска:
        // все шаги бракуются line search и итогом всегда остаётся seed.
        // Active-set: сустав, стоящий на лимите и толкающий наружу,
        // исключаем из задачи (обнуляем столбец J) и пересчитываем dq —
        // иначе зажатое направление душит спуск по свободным суставам.
        float lam2 = lam * lam;
        float A[6][6], y[6], dq[6];
        bool dirOk = false;
        for (int round = 0; round < 6; round++) {
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 6; j++) {
                    float s = (i == j) ? lam2 : 0.0f;
                    for (int k = 0; k < 6; k++) s += J[i][k] * J[j][k];
                    A[i][j] = s;
                }
            }
            if (!solve6(A, e, y)) break;
            for (int k = 0; k < 6; k++) {
                float s = 0;
                for (int i = 0; i < 6; i++) s += J[i][k] * y[i];
                dq[k] = -s;   // МИНУС: шаг против градиента (см. выше)
            }
            int viol = -1;
            const float WEPS = 1e-4f;
            for (int k = 0; k < 6; k++) {
                if ((qr[k] <= qmin[k] + WEPS && dq[k] < 0) ||
                    (qr[k] >= qmax[k] - WEPS && dq[k] > 0)) { viol = k; break; }
            }
            if (viol < 0) { dirOk = true; break; }
            for (int i = 0; i < 6; i++) J[i][viol] = 0;
        }
        if (!dirOk) {
            lam = lam * 4.0f; if (lam > 1e4f) lam = 1e4f;
            if (++stall >= 15) break;
            continue;
        }
        for (int k = 0; k < 6; k++) {
            if (dq[k] > NLS_MAX_STEP) dq[k] = NLS_MAX_STEP;
            if (dq[k] < -NLS_MAX_STEP) dq[k] = -NLS_MAX_STEP;
        }
        if (it == 0 && meta) {
            float d = 0;
            for (int k = 0; k < 6; k++) d += dq[k] * dq[k];
            meta->dbgDq0 = sqrt(d);
        }

        // Backtracking: пробуем полный шаг, половинки, ... — берём первый,
        // который строго уменьшил ошибку. Иначе — растим λ и повторяем.
        // Сравнение в квадратах (nn2 < enorm2 ⟺ nn < enorm), sqrt — только в meta.
        float qtry[6], en[6];
        float step = 1.0f, nn2 = enorm2;
        bool improved = false;
        for (int ls = 0; ls < 10; ls++) {
            for (int k = 0; k < 6; k++)
                qtry[k] = constrain(qr[k] + dq[k] * step, qmin[k], qmax[k]);
            ikError(qtry, Rt, pt, en);
            nn2 = 0;
            for (int i = 0; i < 6; i++) nn2 += en[i] * en[i];
            if (ls == 0 && it == 0 && meta) meta->dbgN1 = sqrt(nn2);
            if (nn2 < enorm2) { improved = true; break; }
            step *= 0.5f;
        }
        if (!improved) {
            lam = lam * 4.0f; if (lam > 1e4f) lam = 1e4f;
            if (++stall >= 15) break;
            continue;
        }

        // Шаг принят
        for (int k = 0; k < 6; k++) qr[k] = qtry[k];
        for (int i = 0; i < 6; i++) e[i] = en[i];
        enorm2 = nn2;
        lam = lam * 0.5f; if (lam < 1e-3f) lam = 1e-3f;
        stall = 0;
        if (nn2 < bestN2) {
            bestN2 = nn2;
            for (int k = 0; k < 6; k++) bestQr[k] = qr[k];
        }
    }

    // Возвращаем лучшее (при несходимости — не последнее, а лучшее)
    if (!converged)
        for (int k = 0; k < 6; k++) qr[k] = bestQr[k];
    for (int i = 0; i < 6; i++)
        q.angles[i] = clampAngle(i, qr[i] * DEG_PER_RAD);

    if (meta) { meta->iters = it; meta->converged = converged; }
    return converged;
}

/**
 * Обёртка для позы: единственный перевод эйлеры→матрица на входе,
 * дальше всё матричное (см. refineSolutionM).
 */
bool IKSolver::refineSolution(const Pose &target, JointAngles &q,
                              IKSolutionMeta *meta) {
    Mat4 Rt = eulerToMatrix(target.orientation.x * RAD_PER_DEG,
                            target.orientation.y * RAD_PER_DEG,
                            target.orientation.z * RAD_PER_DEG);
    return refineSolutionM(target.position, Rt, q, meta);
}
