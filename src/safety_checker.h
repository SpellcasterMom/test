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

#pragma once

#include "types.h"
#include "config.h"
#include "fk_solver.h"

/**
 * @brief Класс проверки безопасности для 6-осевого робота-манипулятора.
 *
 * Обеспечивает проверку безопасного положения робота во время движения.
 * Включает проверки:
 * - Границ рабочей области (Workspace bounds checking)
 * - Безопасности каждого звена (Link safety checking)
 *   - Минимальная высота по Z для каждого звена
 *   - Максимальный и минимальный радиус дотягивания (reach)
 *   - Запрещенные зоны (forbidden zones)
 * - Безопасности траектории (Trajectory safety checking)
 *   - Сэмплирование точек вдоль звеньев для обнаружения коллизий
 * - Виртуальное 7-е звено (пальцы захватчика)
 */
class SafetyChecker {
public:
    SafetyChecker();

    /**
     * @brief Установка конфигурации безопасности.
     * @param ls Параметры безопасности звеньев (Z минимумы, радиусы)
     * @param ws Границы рабочей области
     */
    void setConfig(const LinkSafety &ls, const WorkspaceLimits &ws);

    /**
     * @brief Установка минимальной высоты стола (общий Z минимум).
     * @param zMin Минимальная высота Z в мм
     */
    void setTableZMin(float zMin);

    /**
     * @brief Установка максимального радиуса дотягивания (максимальная вытянутость руки).
     * @param maxR Максимальное расстояние от оси Z основания в мм
     */
    void setMaxReach(float maxR);

    /**
     * @brief Установка минимального радиуса дотягивания (защита от сближения к оси основания).
     * @param minR Минимальное расстояние от оси Z основания в мм
     */
    void setMinReach(float minR);

    /**
     * @brief Установка минимальной высоты Z для конкретного звена.
     * @param link Индекс звена (0-5)
     * @param zMin Минимальная высота Z в мм для этого звена
     */
    void setLinkZMin(int link, float zMin);

    /**
     * @brief Установка DH-параметров для внутреннего FK-решателя.
     * Необходим для корректного вычисления позиций звеньев при проверке безопасности.
     * @param dh Массив 6×4 DH-параметров [a, alpha, d, theta_offset]
     */
    void setDHParams(const float dh[6][4]);

    /**
     * @brief Установка длины пальца захватчика (виртуальное 7-е звено).
     * @param len Длина пальца в мм
     */
    void setFingerLength(float len);

    /**
     * @brief Добавление запрещенной зоны (прямоугольный параллелепипед).
     * @param x1, x2 Границы по X (порядок не важен, будут отсортированы)
     * @param y1, y2 Границы по Y
     * @param z1, z2 Границы по Z
     */
    void addForbiddenZone(float x1, float x2, float y1, float y2, float z1, float z2,
                          uint8_t skipMask = 0);

    /**
     * @brief Очистка всех запрещенных зон.
     */
    void clearForbiddenZones();

    /**
     * @brief Включение/отключение проверок безопасности.
     * @param en true для включения, false для отключения
     */
    void enable(bool en);

    /**
     * @brief Проверка, включены ли проверки безопасности.
     * @return true если проверки включены
     */
    bool isEnabled() const;

    /**
     * @brief Проверка отдельной позиции (конечный эффектор).
     * Проверяет попадание точки в рабочую область и отсутствие в запрещенных зонах.
     * @param pos Позиция в декартовых координатах
     * @return true если позиция безопасна
     */
    bool checkPosition(const Vec3 &pos) const;

    /**
     * @brief Проверка всех звеньев для заданных углов сочленений.
     * Включает:
     * - Проверку радиуса дотягивания для каждого сочленения
     * - Проверку каждого звена (5 точек на звено) по Z, границам, радиусу и запрещенным зонам
     * - Проверку виртуального 7-го звена (пальцы захватчика)
     * @param q Массив углов 6 сочленений (в градусах)
     * @return true если все звенья безопасны
     */
    bool checkAllLinks(const float q[6]);

    /**
     * @brief Проверка безопасной траектории между двумя конфигурациями.
     * Линейно интерполирует углы между начальной и конечной позициями,
     * проверяя безопасность в промежуточных точках.
     * @param qStart Начальные углы (6 сочленений)
     * @param qEnd Конечные углы (6 сочленений)
     * @param steps Количество промежуточных шагов (по умолчанию 20)
     * @return true если вся траектория безопасна
     */
    bool checkTrajectory(const float qStart[6], const float qEnd[6], int steps = 20);

    /**
     * @brief Получение текста последней ошибки безопасности.
     * @return Строка с описанием ошибки
     */
    String getLastError() const;

    /**
     * @brief Получение количества активных запретных зон.
     * @return Количество активных запретных зон (0-10)
     */
    int getZoneCount() const;

    /**
     * @brief Получение параметров запретной зоны по индексу.
     * @param index Индекс зоны (0-9)
     * @param x1, x2 Границы по X
     * @param y1, y2 Границы по Y
     * @param z1, z2 Границы по Z
     * @param active Активна ли зона
     * @return true если зона существует
     */
    bool getForbiddenZone(int index, float &x1, float &x2, float &y1, float &y2, float &z1, float &z2, bool &active) const;

private:
    /** Максимальное количество звеньев: 6 основных + 1 виртуальное (палец захватчика) */
    static const int MAX_LINKS = 7;

    LinkSafety      linkSafety;     // Параметры безопасности звеньев
    WorkspaceLimits workspace;      // Границы рабочей области
    ForbiddenZone   zones[MAX_FORBIDDEN_ZONES]; // Массив запрещенных зон
    int             zoneCount;      // Текущее количество запрещенных зон
    bool            enabled;        // Флаг включения проверок безопасности
    float           fingerLength;   // Длина пальца захватчика (виртуальное 7-е звено)
    String          lastError;      // Текст последней ошибки

    FKSolver fkSolver; // Решатель прямой кинематики для вычисления позиций сочленений

    /**
     * @brief Проверка попадания точки в рабочую область.
     * Проверяет, что точка находится в пределах [xMin,xMax] x [yMin,yMax] x [zMin,zMax].
     * @param p Точка в декартовых координатах
     * @return true если точка внутри рабочей области
     */
    bool insideWorkspace(const Vec3 &p) const;

    /**
     * @brief Проверка попадания точки в запрещенную зону.
     * Перебирает все активные запрещенные зоны и проверяет, попадает ли точка хотя бы в одну.
     * @param p Точка в декартовых координатах
     * @return true если точка находится в какой-либо запрещенной зоне
     */
    bool insideForbiddenZone(const Vec3 &p, int linkIdx = -1) const;

    /**
     * @brief Проверка безопасности отдельного звена.
     * Сэмплирует 5 точек вдоль линии от jointA до jointB и для каждой проверяет:
     * - Минимальную высоту Z для данного звена
     * - Границы рабочей области
     * - Цилиндрический радиус дотягивания (maxReach / minReach)
     * - Попадание в запрещенные зоны
     * @param jointA Начальная точка звена (сочленение)
     * @param jointB Конечная точка звена (следующее сочленение)
     * @param linkIdx Индекс звена (0-6, где 6 = виртуальное звено пальца)
     * @return true если звено безопасно
     */
    bool checkLinkSafety(const Vec3 &jointA, const Vec3 &jointB, int linkIdx);

    /**
     * @brief Линейная интерполяция точки на отрезке.
     * Вычисляет точку на линии от a до b с параметром t (0..1).
     * @param a Начальная точка
     * @param b Конечная точка
     * @param t Параметр интерполяции (0 = a, 1 = b)
     * @return Интерполированная точка
     */
    Vec3 sampleOnLink(const Vec3 &a, const Vec3 &b, float t) const;
};
