/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       :
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-18
 */

#include "safety_checker.h"
#include <cmath>

/**
 * @brief Конструктор по умолчанию.
 *
 * Инициализирует проверку безопасности с параметрами по умолчанию из конфигурации:
 * - DEFAULT_LINK_SAFETY: безопасные расстояния для звеньев
 * - WORKSPACE: границы рабочей области
 * - GRIPPER_FINGER_LENGTH: длина пальца захватчика
 * Количество запрещенных зон устанавливается в 0, проверки включены.
 */
SafetyChecker::SafetyChecker()
    : linkSafety(DEFAULT_LINK_SAFETY),
      workspace(WORKSPACE),
      zoneCount(0),
      enabled(true),
      fingerLength(GRIPPER_FINGER_LENGTH) {}

/**
 * @brief Установка конфигурации безопасности.
 * Заменяет текущие параметры безопасности звеньев и границы рабочей области.
 */
void SafetyChecker::setConfig(const LinkSafety &ls, const WorkspaceLimits &ws) {
    linkSafety = ls;
    workspace = ws;
}

/**
 * @brief Установка минимальной высоты стола (общий Z минимум).
 * Это предотвращает столкновение любых частей робота со столом.
 */
void SafetyChecker::setTableZMin(float zMin) {
    workspace.zMin = zMin;
}

/**
 * @brief Установка максимального радиуса дотягивания.
 * Определяет максимальное расстояние от вертикальной оси основания (ось Z)
 * до любой точки робота. Защита от чрезмерного вытягивания.
 */
void SafetyChecker::setMaxReach(float maxR) {
    linkSafety.maxReach = maxR;
}

/**
 * @brief Установка минимального радиуса дотягивания.
 * Определяет минимальное расстояние от вертикальной оси основания.
 * Защита от слишком близкого расположения к основанию (самопересечение).
 */
void SafetyChecker::setMinReach(float minR) {
    linkSafety.minReach = minR;
}

/**
 * @brief Установка минимальной высоты Z для конкретного звена.
 * Каждое звено может иметь свой порог минимальной высоты для
 * предотвращения столкновения с препятствиями на рабочей поверхности.
 * Допустимые индексы: 0-5 (6 основных звеньев).
 */
void SafetyChecker::setLinkZMin(int link, float zMin) {
    if (link >= 0 && link < 6) {
        linkSafety.zMin[link] = zMin;
    }
}

/**
 * @brief Установка длины пальца захватчика.
 * Используется для проверки безопасности виртуального 7-го звена (палец).
 */
void SafetyChecker::setFingerLength(float len) {
    fingerLength = len;
    fkSolver.setFingerLength(len);
}

void SafetyChecker::setDHParams(const float dh[6][4]) {
    fkSolver.setDHParams(dh);
}

/**
 * @brief Добавление запрещенной зоны (прямоугольный параллелепипед).
 * Координаты границ автоматически сортируются (min/max), поэтому
 * порядок подачи x1/x2, y1/y2, z1/z2 не важен.
 * Максимальное количество зон: MAX_FORBIDDEN_ZONES.
 */
void SafetyChecker::addForbiddenZone(float x1, float x2, float y1, float y2,
                                      float z1, float z2) {
    if (zoneCount >= MAX_FORBIDDEN_ZONES) return;
    zones[zoneCount] = {
        .xMin = fmin(x1, x2), .xMax = fmax(x1, x2),
        .yMin = fmin(y1, y2), .yMax = fmax(y1, y2),
        .zMin = fmin(z1, z2), .zMax = fmax(z1, z2),
        .active = true,
    };
    zoneCount++;
}

/**
 * @brief Очистка всех запрещенных зон.
 * Устанавливает счетчик зон в 0, фактически деактивируя все зоны.
 */
void SafetyChecker::clearForbiddenZones() {
    zoneCount = 0;
}

/**
 * @brief Включение/отключение проверок безопасности.
 * При отключении все методы проверки (checkPosition, checkAllLinks,
 * checkTrajectory) будут безусловно возвращать true.
 */
void SafetyChecker::enable(bool en) {
    enabled = en;
}

bool SafetyChecker::isEnabled() const {
    return enabled;
}

String SafetyChecker::getLastError() const {
    return lastError;
}

/**
 * @brief Проверка попадания точки в рабочую область.
 * Рабочая область определяется прямоугольным параллелепипедом
 * с границами: x ∈ [xMin, xMax], y ∈ [yMin, yMax], z ∈ [zMin, zMax].
 */
bool SafetyChecker::insideWorkspace(const Vec3 &p) const {
    return (p.x >= workspace.xMin && p.x <= workspace.xMax &&
            p.y >= workspace.yMin && p.y <= workspace.yMax &&
            p.z >= workspace.zMin && p.z <= workspace.zMax);
}

/**
 * @brief Проверка попадания точки в запрещенную зону.
 * Перебирает все активные запрещенные зоны. Если точка попадает
 * внутрь хотя бы одной зоны (все 6 координат в пределах), возвращает true.
 * Неактивные зоны пропускаются.
 */
bool SafetyChecker::insideForbiddenZone(const Vec3 &p) const {
    for (int i = 0; i < zoneCount; i++) {
        if (!zones[i].active) continue;
        if (p.x >= zones[i].xMin && p.x <= zones[i].xMax &&
            p.y >= zones[i].yMin && p.y <= zones[i].yMax &&
            p.z >= zones[i].zMin && p.z <= zones[i].zMax) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Линейная интерполяция точки на отрезке.
 * Формула: result = a + (b - a) * t
 * При t=0 возвращает точку a, при t=1 — точку b.
 */
Vec3 SafetyChecker::sampleOnLink(const Vec3 &a, const Vec3 &b, float t) const {
    return a + (b - a) * t;
}

/**
 * @brief Проверка безопасности одного звена.
 *
 * Метод сэмплирует 5 точек равномерно вдоль звена (от jointA до jointB)
 * и для каждой точки выполняет комплексную проверку:
 *
 * 1. Минимальная высота Z: для каждого звена задается свой порог zMin.
 *    Для виртуального 7-го звена (палец) используется zMin звена 5.
 *    Это предотвращает столкновение звена с рабочей поверхностью.
 *
 * 2. Границы рабочей области: точка должна находиться внутри
 *    прямоугольной области workspace.
 *
 * 3. Цилиндрический радиус дотягивания (reach):
 *    - maxReach: максимальное расстояние от оси Z основания (защита от перерастяжения)
 *    - minReach: минимальное расстояние от оси Z основания (защита от самопересечения)
 *    Радиус вычисляется как sqrt(x² + y²) — расстояние в плоскости XY.
 *
 * 4. Запрещенные зоны: точка не должна попадать в ранее определенные зоны.
 *
 * Если хотя бы одна проверка не проходит, метод возвращает false
 * и записывает описание ошибки в lastError.
 */
bool SafetyChecker::checkLinkSafety(const Vec3 &jointA, const Vec3 &jointB, int linkIdx) {
    if (linkIdx < 0 || linkIdx >= MAX_LINKS) return true;

    // Для виртуального 7-го звена (палец захватчика) используем zMin звена 5,
    // так как палец является продолжением последнего реального звена
    float zMin = (linkIdx < 6) ? linkSafety.zMin[linkIdx] : linkSafety.zMin[5];

    // Количество точек для сэмплирования вдоль звена
    const int SAMPLES = 5;
    for (int s = 0; s <= SAMPLES; s++) {
        float t = (float)s / SAMPLES;
        Vec3 point = sampleOnLink(jointA, jointB, t);

        // Проверка 1: минимальная высота Z для данного звена
        if (point.z < zMin) {
            lastError = "Link " + String(linkIdx) +
                        " Z=" + String(point.z, 1) + " < Z_MIN=" + String(zMin, 1);
            return false;
        }

        // Проверка 2: границы рабочей области
        if (!insideWorkspace(point)) {
            lastError = "Link " + String(linkIdx) +
                        " outside workspace at (" + String(point.x, 1) + ", " +
                        String(point.y, 1) + ", " + String(point.z, 1) + ")";
            return false;
        }

        // Проверка 3: цилиндрический радиус дотягивания (расстояние от оси Z основания)
        float reachXY = sqrt(point.x * point.x + point.y * point.y);
        if (reachXY > linkSafety.maxReach) {
            lastError = "Link " + String(linkIdx) +
                        " reach=" + String(reachXY, 1) + " > MAX=" + String(linkSafety.maxReach, 1);
            return false;
        }
        if (reachXY < linkSafety.minReach) {
            lastError = "Link " + String(linkIdx) +
                        " reach=" + String(reachXY, 1) + " < MIN=" + String(linkSafety.minReach, 1);
            return false;
        }

        // Проверка 4: запрещенные зоны
        if (insideForbiddenZone(point)) {
            lastError = "Link " + String(linkIdx) +
                        " in forbidden zone at (" + String(point.x, 1) + ", " +
                        String(point.y, 1) + ", " + String(point.z, 1) + ")";
            return false;
        }
    }

    return true;
}

/**
 * @brief Проверка позиции конечного эффектора.
 * Проверяет только попадание заданной точки в рабочую область
 * и отсутствие в запрещенных зонах. Не проверяет звенья —
 * для этого предназначен метод checkAllLinks().
 * Если проверки отключены (enabled=false), всегда возвращает true.
 */
bool SafetyChecker::checkPosition(const Vec3 &pos) const {
    if (!enabled) return true;

    if (!insideWorkspace(pos)) {
        return false;
    }

    if (insideForbiddenZone(pos)) {
        return false;
    }

    return true;
}

/**
 * @brief Проверка безопасности всех звеньев для заданной конфигурации.
 *
 * Последовательно выполняет:
 *
 * 1. Прямая кинематика: вычисляет позиции всех 7 сочленений (6 реальных + TCP)
 *    с помощью FKSolver.
 *
 * 2. Проверка радиуса дотягивания для каждого сочленения (индексы 1-6):
 *    Каждое сочленение должно находиться в пределах [minReach, maxReach]
 *    от оси Z основания.
 *
 * 3. Проверка 6 основных звеньев (индексы 0-5):
 *    Для каждого звена (от сочленения i до i+1) вызывается checkLinkSafety(),
 *    которая сэмплирует 5 точек и проверяет Z минимум, границы, радиус и запрещенные зоны.
 *
 * 4. Проверка виртуального 7-го звена (палец захватчика):
 *    Звено от 6-го сочленения до TCP (Tool Center Point).
 *    Палец также должен быть безопасен по всем параметрам.
 *
 * @param q Массив углов 6 сочленений (в градусах)
 * @return true если все звенья безопасны
 */
bool SafetyChecker::checkAllLinks(const float q[6]) {
    if (!enabled) return true;

    // Вычисляем позиции всех сочленений с помощью прямой кинематики
    Vec3 joints[7];
    fkSolver.getAllJointPositions(q, joints);

    // Проверка радиуса дотягивания для каждого сочленения (1-6)
    // Соединение 0 (основание) находится на оси, его не проверяем
    for (int i = 1; i <= 6; i++) {
        float reachXY = sqrt(joints[i].x * joints[i].x + joints[i].y * joints[i].y);
        if (reachXY > linkSafety.maxReach) {
            lastError = "Joint " + String(i) + " reach=" + String(reachXY, 1) +
                        " > MAX=" + String(linkSafety.maxReach, 1);
            return false;
        }
        if (reachXY < linkSafety.minReach) {
            lastError = "Joint " + String(i) + " reach=" + String(reachXY, 1) +
                        " < MIN=" + String(linkSafety.minReach, 1);
            return false;
        }
    }

    // Проверка каждого из 6 основных звеньев (от сочленения i до i+1)
    // Каждое звено проверяется по 5 точкам на Z минимум, границы, радиус и запрещенные зоны
    for (int i = 0; i < 6; i++) {
        if (!checkLinkSafety(joints[i], joints[i + 1], i)) {
            return false;
        }
    }

    // Проверка виртуального 7-го звена: палец захватчика (от 6-го сочленения до TCP)
    Vec3 tcp = fkSolver.getEndEffector(q);
    if (!checkLinkSafety(joints[6], tcp, 6)) {
        return false;
    }

    return true;
}

/**
 * @brief Проверка безопасной траектории между двумя конфигурациями.
 *
 * Линейно интерполирует углы сочленений между qStart и qEnd
 * и проверяет безопасность всего манипулятора в каждом промежуточном шаге.
 *
 * Алгоритм:
 * 1. Делит траекторию на steps промежуточных шагов (по умолчанию 20)
 * 2. Для каждого шага вычисляет углы: q = qStart + (qEnd - qStart) * t
 * 3. Вызывает checkAllLinks(q) для проверки безопасности в этой точке
 * 4. Если хотя бы одна точка небезопасна — возвращает false
 *
 * Это гарантирует, что между безопасными начальной и конечной позициями
 * нет столкновений на промежуточных этапах движения.
 *
 * @param qStart Начальные углы (6 сочленений)
 * @param qEnd Конечные углы (6 сочленений)
 * @param steps Количество промежуточных шагов (по умолчанию 20)
 * @return true если вся траектория безопасна
 */
bool SafetyChecker::checkTrajectory(const float qStart[6], const float qEnd[6], int steps) {
    if (!enabled) return true;

    for (int step = 0; step <= steps; step++) {
        float t = (float)step / steps;
        float q[6];

        // Линейная интерполяция углов для текущего шага
        for (int i = 0; i < 6; i++) {
            q[i] = qStart[i] + (qEnd[i] - qStart[i]) * t;
        }

        // Проверка всех звеньев для текущей промежуточной конфигурации
        if (!checkAllLinks(q)) {
            lastError = "Collision at step " + String(step) + "/" + String(steps) +
                        ": " + lastError;
            return false;
        }
    }

    return true;
}
