/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-10-01
 */

/*
 * fk_solver.cpp — Реализация решателя прямой кинематики для 6-осевого робота.
 *
 * Основные принципы:
 *  1. Каждое звено робота описывается DH-параметрами: [a, alpha, d, theta_offset].
 *  2. Матрица перехода между двумя последовательными СК строится формулой DH:
 *
 *       T(a, α, d, θ) =
 *         | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) |
 *         | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) |
 *         |   0         sin(α)           cos(α)          d     |
 *         |   0           0                0              1     |
 *
 *  3. Полная матрица T0i вычисляется последовательным перемножением:
 *       T0i = T01 × T12 × T23 × ... × T(i-1,i)
 *
 *  4. Позиция TCP (кончик пальца) = позиция запястья (сустав 6) + длина пальца
 *     вдоль направления выдвижения пальца (ось Z 6-го звена В БАЗОВОЙ СК).
 *
 * =====================================================================
 * ВАЖНОЕ ИСПРАВЛЕНИЕ (Last modify: 2026-09-25):
 * =====================================================================
 * Функция getFingerDirection() раньше возвращала ось Z в ЛОКАЛЬНОЙ
 * системе координат 6-го звена (без накопления T01·T12·...·T56).
 * Из-за этого getEndEffector() складывал:
 *   - позицию запястья (в базовой СК)       ✓
 *   - направление пальца (в локальной СК)   ✗
 * → ошибка ~20 мм в TCP.
 *
 * Теперь getFingerDirection() вычисляет ПОЛНУЮ матрицу T06 через
 * computeFK(q, 5) и извлекает из неё ось Z — уже в базовой СК.
 * Это согласовано с getJointPosition(q, 6), которая тоже в базовой СК.
 */

#include "fk_solver.h"
#include <cmath>

/**
 * Конструктор FKSolver.
 *
 * Инициализирует DH-параметры из конфигурационного массива DH_PARAMS,
 * длины звеньев из LINK_LENGTHS и длину пальца из GRIPPER_FINGER_LENGTH.
 * Все значения по умолчанию определены в config.h.
 */
FKSolver::FKSolver() {
    fingerLength = GRIPPER_FINGER_LENGTH;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 4; j++) {
            dh[i][j] = DH_PARAMS[i][j];
        }
        linkLen[i] = LINK_LENGTHS[i];
    }
    refreshTrigCache();
}

/**
 * Установка DH-параметров для всех 6 звеньев робота.
 *
 * @param params — массив 6×4:
 *   params[i][0] = a_i     — длина звена (мм)
 *   params[i][1] = alpha_i — угол закручивания (градусы)
 *   params[i][2] = d_i     — смещение вдоль оси Z (мм)
 *   params[i][3] = theta_offset_i — начальный сдвиг угла (градусы)
 */
void FKSolver::setDHParams(const float params[6][4]) {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 4; j++)
            dh[i][j] = params[i][j];
    refreshTrigCache();
}

/**
 * Установка длин звеньев (суставов) робота.
 *
 * @param lengths — массив из 6 целых чисел, длины в миллиметрах.
 */
void FKSolver::setLinkLengths(const int lengths[6]) {
    for (int i = 0; i < 6; i++)
        linkLen[i] = lengths[i];
}

/**
 * Установка длины пальца захвата.
 *
 * Длина пальца напрямую влияет на вычисление TCP:
 *   TCP = позиция_запястья + fingerLength × направление_пальца
 *
 * При fingerLength = 0 TCP совпадает с запястьем.
 * При большой fingerLength TCP значительно удалён от запястья.
 *
 * @param len — длина пальца в миллиметрах.
 */
void FKSolver::setFingerLength(float len) {
    fingerLength = len;
}

/**
 * Кэш тригонометрии DH: вызывается из конструктора и setDHParams.
 * alpha/off меняются только там, поэтому cos/sin(alpha) и off в радианах
 * считаем один раз, а не на каждое звено каждого FK-прохода.
 */
void FKSolver::refreshTrigCache() {
    for (int i = 0; i < 6; i++) {
        offRad[i] = dh[i][3] * RAD_PER_DEG;
        float al = dh[i][1] * RAD_PER_DEG;
        calpha[i] = cos(al);
        salpha[i] = sin(al);
    }
}

/**
 * DH-матрица звена i из готовых cos/sin(alpha) — та же формула, что dhTransform.
 */
Mat4 FKSolver::dhFast(float a, int i, float d, float theta) {
    float ct = cos(theta);
    float st = sin(theta);
    float ca = calpha[i];
    float sa = salpha[i];

    Mat4 t;
    t.m[0][0] = ct;   t.m[0][1] = -st * ca;  t.m[0][2] =  st * sa;  t.m[0][3] = a * ct;
    t.m[1][0] = st;   t.m[1][1] =  ct * ca;  t.m[1][2] = -ct * sa;  t.m[1][3] = a * st;
    t.m[2][0] = 0;    t.m[2][1] =  sa;        t.m[2][2] =  ca;       t.m[2][3] = d;
    t.m[3][0] = 0;    t.m[3][1] =  0;         t.m[3][2] =  0;        t.m[3][3] = 1;
    return t;
}

/**
 * Вычисление прямой кинематики — последовательное перемножение DH-матриц.
 *
 * Алгоритм:
 *   T = E (единичная матрица 4×4 — начальное положение в базе)
 *   для каждого звена i = 0 .. upToJoint:
 *     1. Вычислить полный угол: θ = q[i] + theta_offset (градусы → радианы)
 *     2. Построить DH-матрицу Ti для этого звена
 *     3. Умножить: T = T × Ti (накопление преобразований)
 *
 * @param q          — углы конфигурации (градусы).
 * @param upToJoint  — индекс последнего звена (0..5).
 * @return Матрица 4×4 T0i.
 */
Mat4 FKSolver::computeFK(const float q[6], int upToJoint) {
    Mat4 T = Mat4::identity();

    for (int i = 0; i <= upToJoint && i < 6; i++) {
        // Суммируем угол сочленения и начальный сдвиг (сдвиг уже в радианах из кэша)
        float theta = q[i] * RAD_PER_DEG + offRad[i];
        // DH-матрица звена i (cos/sin(alpha) из кэша)
        Mat4 Ti = dhFast(dh[i][0], i, dh[i][2], theta);
        // Накопление: текущая T умножается на Ti
        T = T * Ti;
    }

    return T;
}

/**
 * Однопроходная цепочка: один проход по 6 DH-матрицам даёт и все
 * позиции суставов, и накопленную T06. Все геттеры ниже — тонкие
 * обёртки над ней (поведение то же, трансформов в разы меньше).
 */
void FKSolver::computeChain(const float q[6], Vec3 joints[7], Mat4 &T06) {
    Mat4 T = Mat4::identity();
    joints[0] = {0, 0, 0};
    for (int i = 0; i < 6; i++) {
        float theta = q[i] * RAD_PER_DEG + offRad[i];
        Mat4 Ti = dhFast(dh[i][0], i, dh[i][2], theta);
        T = T * Ti;
        joints[i + 1] = T.getPosition();
    }
    T06 = T;
}

/**
 * То же, но входные углы уже в радианах (численный IK держит q в радианах).
 */
void FKSolver::computeChainRad(const float qrad[6], Vec3 joints[7], Mat4 &T06) {
    Mat4 T = Mat4::identity();
    joints[0] = {0, 0, 0};
    for (int i = 0; i < 6; i++) {
        float theta = qrad[i] + offRad[i];
        Mat4 Ti = dhFast(dh[i][0], i, dh[i][2], theta);
        T = T * Ti;
        joints[i + 1] = T.getPosition();
    }
    T06 = T;
}

/**
 * Однопроходная поза: joints + TCP + направление пальца + T06.
 */
void FKSolver::getPose(const float q[6], Vec3 joints[7], Vec3 &tcp, Vec3 &fingerDir, Mat4 &T06) {
    computeChain(q, joints, T06);
    fingerDir = {T06.m[0][2], T06.m[1][2], T06.m[2][2]};
    const Vec3 &wrist = joints[6];
    tcp = wrist + fingerDir * fingerLength;
}

/**
 * Однопроходная поза из радиан (путь refineSolutionM/ikError).
 */
void FKSolver::getPoseRad(const float qrad[6], Vec3 joints[7], Vec3 &tcp, Vec3 &fingerDir, Mat4 &T06) {
    computeChainRad(qrad, joints, T06);
    fingerDir = {T06.m[0][2], T06.m[1][2], T06.m[2][2]};
    const Vec3 &wrist = joints[6];
    tcp = wrist + fingerDir * fingerLength;
}

/**
 * Получение позиции конкретного сустава в базовой СК.
 *
 * Позиция сустава joint — это вектор переноса из матрицы T0,(joint-1).
 *
 * @param q     — углы конфигурации (градусы).
 * @param joint — номер сустава (0 = база, 1..6 = суставы).
 * @return Вектор позиции {x, y, z} в миллиметрах.
 */
Vec3 FKSolver::getJointPosition(const float q[6], int joint) {
    if (joint <= 0) return {0, 0, 0};
    if (joint > 6) joint = 6;
    Vec3 joints[7];
    Mat4 T06;
    computeChain(q, joints, T06);
    return joints[joint];
}

/**
 * Получение позиций всех 7 точек (база + 6 суставов).
 *
 * joints[0] = {0,0,0} — начало координат базы.
 * joints[i] = позиция i-го сустава (i=1..6).
 *
 * @param q      — углы конфигурации.
 * @param joints — выходной массив из 7 Vec3.
 */
void FKSolver::getAllJointPositions(const float q[6], Vec3 joints[7]) {
    Mat4 T06;
    computeChain(q, joints, T06);
    (void)T06;
}

/**
 * Вычисление позиции конечного эффектора (TCP — Tool Center Point).
 *
 * Формула:
 *   TCP = позиция_запястья + fingerLength × направление_пальца
 *
 * Где:
 *   позиция_запястья   — позиция 6-го сустава в базовой СК
 *   направление_пальца — ось Z 6-го звена В БАЗОВОЙ СК
 *   fingerLength       — длина пальца захвата
 *
 * ВАЖНО: обе величины (wrist и dir) должны быть в ОДНОЙ системе координат —
 * в базовой. Раньше getFingerDirection() возвращал направление в локальной
 * СК 6-го звена, что давало ошибку ~20 мм. Исправлено (см. getFingerDirection).
 *
 * @param q — углы конфигурации.
 * @return Позиция кончика пальца {x, y, z} в мм (базовая СК).
 */
Vec3 FKSolver::getEndEffector(const float q[6]) {
    Vec3 joints[7];
    Vec3 tcp, dir;
    Mat4 T06;
    getPose(q, joints, tcp, dir, T06);
    return tcp;
}

/**
 * Получение направляющего вектора пальца (ось Z 6-го звена В БАЗОВОЙ СК).
 *
 * =====================================================================
 * ИСПРАВЛЕНО (Last modify: 2026-09-25):
 * =====================================================================
 * Раньше здесь строилась только матрица 6-го звена:
 *   Mat4 T6 = dhTransform(...);
 * → это ось Z в ЛОКАЛЬНОЙ СК 6-го звена.
 *
 * getEndEffector() складывал её с позицией запястья в БАЗОВОЙ СК →
 * смешение систем координат → ошибка ~20 мм.
 *
 * Теперь используется полная матрица T06 = computeFK(q, 5):
 * она накапливает T01·T12·T23·T34·T45·T56, и ось Z из неё уже
 * в БАЗОВОЙ СК. Согласовано с getJointPosition(q, 6).
 *
 * Используется для:
 *   - Вычисления TCP (getEndEffector)
 *   - Проверок безопасности (столкновения с препятствиями)
 *   - Визуализации направления захвата
 *
 * @param q — углы конфигурации.
 * @return Единичный вектор направления выдвижения пальца (базовая СК).
 */
Vec3 FKSolver::getFingerDirection(const float q[6]) {
    // Ось Z из накопленной T06 — уже в базовой СК (см. getPose).
    Vec3 joints[7];
    Mat4 T06;
    computeChain(q, joints, T06);
    (void)joints;

    // Третий столбец матрицы вращения R06 — это ось Z 6-го звена
    // в координатах базовой СК. Единичный вектор (для матриц поворота).
    return {T06.m[0][2], T06.m[1][2], T06.m[2][2]};
}

/**
 * Получение полной матрицы перехода 4×4 от базы до указанного сустава.
 *
 * Матрица содержит:
 *   - Верхний левый блок 3×3 — матрица вращения (ориентация)
 *   - Верхний правый столбец 3×1 — вектор переноса (позиция)
 *   - Нижняя строка [0 0 0 1] — однородные координаты
 *
 * @param q          — углы конфигурации (градусы).
 * @param upToJoint  — номер звена (0..5).
 * @return Матрица 4×4 T0i.
 */
Mat4 FKSolver::getTransform(const float q[6], int upToJoint) {
    return computeFK(q, upToJoint);
}

/**
 * Получение матрицы вращения 3×3 (из матрицы перехода 4×4).
 *
 * Матрица вращения — это верхний левый блок 3×3 матрицы T0i.
 * Она описывает ориентацию локальной СК сустава относительно базовой СК.
 *
 * @param q          — углы конфигурации.
 * @param upToJoint  — номер звена (0..5).
 * @return Матрица 4×4, содержащая матрицу вращения 3×3 в верхнем левом углу.
 */
Mat4 FKSolver::getRotationMatrix(const float q[6], int upToJoint) {
    Mat4 R = Mat4::identity();
    if (upToJoint >= 5) {
        // Горячий путь (ikError, moveToPose): берём R06 из однопроходной T06.
        Vec3 joints[7];
        Mat4 T06;
        computeChain(q, joints, T06);
        (void)joints;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                R.m[i][j] = T06.m[i][j];
        return R;
    }
    Mat4 T = computeFK(q, upToJoint);
    // Копируем только верхний левый блок 3×3 (матрица вращения)
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R.m[i][j] = T.m[i][j];
    return R;
}

/**
 * Получение DH-параметров для использования в обратной кинематике.
 *
 * Возвращает текущие DH-параметры всех 6 звеньев:
 *   out[i][0] = a_i     — длина звена (мм)
 *   out[i][1] = alpha_i — угол закручивания (градусы)
 *   out[i][2] = d_i     — смещение вдоль оси Z (мм)
 *   out[i][3] = theta_offset_i — начальный сдвиг угла (градусы)
 *
 * @param out — выходной массив 6×4, заполняется текущими DH-параметрами.
 */
void FKSolver::getDHParams(float out[6][4]) const {
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 4; j++)
            out[i][j] = dh[i][j];
}