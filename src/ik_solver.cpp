/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-25
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
 * Транспонирование матрицы 4×4.
 *
 * Для ортогональных матриц поворота транспонирование эквивалентно
 * обратной матрице: R⁻¹ = Rᵀ. Используется в solveOrientation()
 * для вычисления R03ᵀ при расчёте R36 = R03ᵀ · R06.
 */
Mat4 IKSolver::matTranspose(const Mat4 &m) {
    Mat4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = m.m[j][i];
    return r;
}

/**
 * Умножение двух матриц 4×4: r = a · b.
 *
 * Используется для:
 *   - Вычисления произведения DH-матриц: T03 = T01 · T12 · T23
 *   - Вычисления R36 = R03ᵀ · R06
 */
Mat4 IKSolver::matMultiply(const Mat4 &a, const Mat4 &b) {
    Mat4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
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
        q5 = atan2(R.m[0][1], R.m[1][1]) * RAD_TO_DEG;
        q6 = 0.0f;
    } else if (sp < -0.9999f) {
        // Замок кардана: pitch = -90°
        q4 = -90.0f;
        q5 = atan2(-R.m[0][1], R.m[1][1]) * RAD_TO_DEG;
        q6 = 0.0f;
    } else {
        // Обычный случай
        q4 = asin(sp) * RAD_TO_DEG;
        q5 = atan2(R.m[2][1], R.m[2][2]) * RAD_TO_DEG;
        q6 = atan2(R.m[1][0], R.m[0][0]) * RAD_TO_DEG;
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
    Mat4 R06 = eulerToMatrix(target.orientation.x * DEG_TO_RAD,
                             target.orientation.y * DEG_TO_RAD,
                             target.orientation.z * DEG_TO_RAD);
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
    solveOrientation(target.orientation.x, target.orientation.y, target.orientation.z,
                     q1, q2, q3, q4, q5, q6);

    // ================================================================
    // ПОСТ-ОБРАБОТКА: Ограничение углов механическими пределами
    // ================================================================
    result.angles[0] = clampAngle(0, q1);  // основание
    result.angles[1] = clampAngle(1, q2);  // плечо
    result.angles[2] = clampAngle(2, q3);  // локоть
    result.angles[3] = clampAngle(3, q4);  // запястье (pitch)
    result.angles[4] = clampAngle(4, q5);  // запястье (yaw)
    result.angles[5] = clampAngle(5, q6);  // запястье (roll)
    result.angles[6] = 0;                  // захват управляется отдельно

    // ЭТАП 3: доводка по полной цепочке (см. refineSolution)
    refineSolution(target, result, nullptr);

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
    // DH-параметры основания
    float d0 = d[0];  // высота основания (мм) — смещение вдоль оси Z
    float a0 = a[0];  // смещение основания (мм) — длина звена основания

    // ==================================================================
    // ШАГ 1: Угол поворота основания (q1)
    // ==================================================================
    q1 = atan2(y, x);

    // ==================================================================
    // ШАГ 2: Проекция на 2D-плоскость (r, s)
    // ==================================================================
    float r = sqrt(x * x + y * y) - a0;
    float s = z - d0;

    // ==================================================================
    // ШАГ 3: Длины звеньев двухзвенного манипулятора
    // ==================================================================
    float L1 = a[1];  // плечо — локоть
    float L2 = a[2];  // локоть — запястье

    float dist = sqrt(r * r + s * s);

    // ==================================================================
    // ШАГ 4: Проверка достижимости
    // ==================================================================
    if (dist > (L1 + L2)) return false;      // слишком далеко
    if (dist < fabs(L1 - L2)) return false;  // слишком близко

    // ==================================================================
    // ШАГ 5: Угол локтя (q3) по теореме косинусов
    // ==================================================================
    float cosQ3 = (r * r + s * s - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cosQ3 = constrain(cosQ3, -1.0f, 1.0f);
    q3 = -acos(cosQ3);   // «elbow down» — q3 < 0

    // ==================================================================
    // ШАГ 6: Угол плеча (q2)
    // ==================================================================
    float sinQ3 = sin(q3);
    float C = L1 + L2 * cosQ3;
    float S = L2 * sinQ3;
    q2 = atan2(s, r) - atan2(S, C);

    // ==================================================================
    // ШАГ 7: Перевод из радиан в градусы + ноль плеча
    // ====================================================================
    // Геометрия выше считает q2 от ГОРИЗОНТАЛИ (DH-кадр), а математический
    // ноль плеча — ВЕРТИКАЛЬ: q2_math = q2_DH − thetaOff[J2].
    q1 = q1 * RAD_TO_DEG;
    q2 = q2 * RAD_TO_DEG - thetaOff[1];
    q3 = q3 * RAD_TO_DEG;

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
    // R06 из целевой ориентации
    // --------------------------------------------------
    Mat4 R06 = eulerToMatrix(A * DEG_TO_RAD, B * DEG_TO_RAD, C * DEG_TO_RAD);

    // --------------------------------------------------
    // R36 = R03ᵀ · R06
    // --------------------------------------------------
    Mat4 R03T = matTranspose(R03);
    Mat4 R36 = matMultiply(R03T, R06);

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
    Mat4 R06 = eulerToMatrix(target.orientation.x * DEG_TO_RAD,
                             target.orientation.y * DEG_TO_RAD,
                             target.orientation.z * DEG_TO_RAD);
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

    // === Коррекция TCP → запястье ===
    Mat4 R06 = eulerToMatrix(target.orientation.x * DEG_TO_RAD,
                             target.orientation.y * DEG_TO_RAD,
                             target.orientation.z * DEG_TO_RAD);
    Vec3 toolDir = {R06.m[0][2], R06.m[1][2], R06.m[2][2]};

    float wx = target.position.x - toolDir.x * fingerLength;
    float wy = target.position.y - toolDir.y * fingerLength;
    float wz = target.position.z - toolDir.z * fingerLength;

    // ================================================================
    // РЕШЕНИЕ 1: elbow down (q3 < 0)
    // ================================================================
    if (solvePosition(wx, wy, wz, q1, q2, q3)) {
        solveOrientation(target.orientation.x, target.orientation.y, target.orientation.z,
                         q1, q2, q3, q4, q5, q6);
        solutions[count].setAll(
            clampAngle(0, q1), clampAngle(1, q2), clampAngle(2, q3),
            clampAngle(3, q4), clampAngle(4, q5), clampAngle(5, q6),
            0);
        count++;
    }

    // ================================================================
    // РЕШЕНИЕ 2: elbow up (q3 > 0)
    // ================================================================
    // ИСПРАВЛЕНО: q3 после solvePosition() уже в градусах.
    // Для вызова sin/cos переводим обратно в радианы (q3rad),
    // иначе atan2 получит некорректные аргументы.
    if (solvePosition(wx, wy, wz, q1, q2, q3)) {
        q3 = -q3;   // elbow down → elbow up (градусы)

        float r = sqrt(wx * wx + wy * wy) - a[0];
        float s = wz - d[0];
        float L1 = a[1];
        float L2 = a[2];

        // Переводим q3 обратно в радианы для тригонометрии
        float q3rad = q3 * DEG_TO_RAD;
        float q2rad = atan2(s, r) - atan2(L2 * sin(q3rad), L1 + L2 * cos(q3rad));
        q2 = q2rad * RAD_TO_DEG - thetaOff[1];   // в градусы + ноль плеча

        solveOrientation(target.orientation.x, target.orientation.y, target.orientation.z,
                         q1, q2, q3, q4, q5, q6);
        solutions[count].setAll(
            clampAngle(0, q1), clampAngle(1, q2), clampAngle(2, q3),
            clampAngle(3, q4), clampAngle(4, q5), clampAngle(5, q6),
            0);
        count++;
    }

    // ================================================================
    // ЭТАП 3: ЧИСЛЕННОЕ УТОЧНЕНИЕ (DLS)
    // ================================================================
    // Запястье НЕ сферическое (вылеты 3×25 мм), поэтому геометрические
    // решения доводим по полной FK-цепочке. Seed'ы уже в solutions[].
    for (int i = 0; i < count; i++) {
        IKSolutionMeta m;
        refineSolution(target, solutions[i], &m);
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
    float qdeg[6];
    for (int i = 0; i < 6; i++) qdeg[i] = qrad[i] * RAD_TO_DEG;

    Vec3 p = fkSolver.getEndEffector(qdeg);
    Mat4 R = fkSolver.getRotationMatrix(qdeg, 5);

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
bool IKSolver::refineSolution(const Pose &target, JointAngles &q, IKSolutionMeta *meta) {
    if (meta) { meta->iters = 0; meta->converged = false; meta->dbgDq0 = -1; meta->dbgN1 = -1; }

    Mat4 Rt = eulerToMatrix(target.orientation.x * DEG_TO_RAD,
                            target.orientation.y * DEG_TO_RAD,
                            target.orientation.z * DEG_TO_RAD);
    Vec3 pt = target.position;

    float qr[6], qmin[6], qmax[6];
    for (int i = 0; i < 6; i++) {
        qr[i] = clampAngle(i, q.angles[i]) * DEG_TO_RAD;
        qmin[i] = servoLimits[i].minAngle * DEG_TO_RAD;
        qmax[i] = servoLimits[i].maxAngle * DEG_TO_RAD;
    }

    float e[6];
    ikError(qr, Rt, pt, e);

    float bestQr[6], bestN = 0;
    for (int k = 0; k < 6; k++) bestQr[k] = qr[k];
    for (int i = 0; i < 6; i++) bestN += e[i] * e[i];
    bestN = sqrt(bestN);

    float tolOriW = numTolOri * DEG_TO_RAD * NLS_ORI_W;
    float enorm = bestN;

    // Невязка seed'а для диагностики (M115 покажет seed → best)
    if (meta) {
        meta->seedPos = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
        meta->seedOri = sqrt(e[3]*e[3] + e[4]*e[4] + e[5]*e[5]) / NLS_ORI_W * RAD_TO_DEG;
    }

    // Левенберг-Марквардт: λ адаптивная, шаг принимается только если
    // ошибка строго упала (backtracking line search). Без этого шаги
    // Гаусса-Ньютона перелетают и лучшей остаётся seed-точка.
    float lam = NLS_LAMBDA;
    bool converged = false;
    int it = 0, stall = 0;
    for (it = 0; it < numMaxIter; it++) {
        float ep = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
        float eo = sqrt(e[3]*e[3] + e[4]*e[4] + e[5]*e[5]);
        if (ep <= numTolPos && eo <= tolOriW) { converged = true; break; }

        // Численный якобиан: столбец j = (e(q+δj) − e(q)) / δ
        float J[6][6], e2[6], qp[6];
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) qp[k] = qr[k];
            qp[j] += NLS_DELTA;
            ikError(qp, Rt, pt, e2);
            for (int i = 0; i < 6; i++) J[i][j] = (e2[i] - e[i]) / NLS_DELTA;
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
        float qtry[6], en[6];
        float step = 1.0f, nn = enorm;
        bool improved = false;
        for (int ls = 0; ls < 10; ls++) {
            for (int k = 0; k < 6; k++)
                qtry[k] = constrain(qr[k] + dq[k] * step, qmin[k], qmax[k]);
            ikError(qtry, Rt, pt, en);
            nn = 0;
            for (int i = 0; i < 6; i++) nn += en[i] * en[i];
            nn = sqrt(nn);
            if (ls == 0 && it == 0 && meta) meta->dbgN1 = nn;
            if (nn < enorm) { improved = true; break; }
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
        enorm = nn;
        lam = lam * 0.5f; if (lam < 1e-3f) lam = 1e-3f;
        stall = 0;
        if (nn < bestN) {
            bestN = nn;
            for (int k = 0; k < 6; k++) bestQr[k] = qr[k];
        }
    }

    // Возвращаем лучшее (при несходимости — не последнее, а лучшее)
    if (!converged)
        for (int k = 0; k < 6; k++) qr[k] = bestQr[k];
    for (int i = 0; i < 6; i++)
        q.angles[i] = clampAngle(i, qr[i] * RAD_TO_DEG);

    if (meta) { meta->iters = it; meta->converged = converged; }
    return converged;
}
