/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       :
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-18
 */

/**
 * @file types.h
 * @brief Базовые типы данных для робота-манипулятора
 *
 * Содержит:
 * - Vec3 — трёхмерный вектор (позиция, ориентация)
 * - Mat4 — матрица4x4 (преобразования DH)
 * - JointAngles — углы сервоприводов
 * - Pose — целевая позиция + ориентация (для IK)
 * - GCodeCommand — структура G-кода
 * - CommandDef — определение команды в таблице
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <Arduino.h>

// ====================================================================
// Трёхмерный вектор
// ====================================================================
// Используется для:
// - Позиции точек в пространстве (x, y, z в мм)
// - Ориентации (roll, pitch, yaw в градусах)
// - Направлений (векторы в DH-преобразованиях)
struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Арифметические операции
    Vec3 operator+(const Vec3 &o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3 &o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(float s) const { return {x/s, y/s, z/s}; }

    // Длина вектора (расстояние от начала координат)
    float length() const { return sqrt(x*x + y*y + z*z); }

    // Расстояние до другой точки
    float distTo(const Vec3 &o) const { return (*this - o).length(); }

    // Середина между двумя точками (для safety-check)
    Vec3 midpoint(const Vec3 &o) const { return (*this + o) * 0.5f; }
};

// ====================================================================
// Матрица4x4
// ====================================================================
// Используется для:
// - DH-преобразований (T01, T12, ..., T56)
// - Накопления преобразований (T06 = T01 * T12 * ... * T56)
// - Извлечения позиции и ориентации
//
// Структура матрицы преобразования:
// [ R  p ]   R — матрица вращения3x3
// [ 0  1 ]   p — вектор переноса (позиция)
struct Mat4 {
    float m[4][4];

    Mat4() { memset(m, 0, sizeof(m)); }

    // Создать единичную матрицу
    static Mat4 identity() {
        Mat4 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
        return r;
    }

    // Извлечь позицию (последний столбец матрицы)
    Vec3 getPosition() const { return {m[0][3], m[1][3], m[2][3]}; }

    // Умножение матриц: this = A * B
    Mat4 operator*(const Mat4 &b) const {
        Mat4 r;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    r.m[i][j] += m[i][k] * b.m[k][j];
        return r;
    }
};

// ====================================================================
// Углы сервоприводов
// ====================================================================
// Хранит углы для всех7 сервоприводов:
//   angles[0] = Joint 1 (Base) — поворот вокруг Z
//   angles[1] = Joint 2 (Shoulder) — наклон плеча
//   angles[2] = Joint 3 (Elbow) — наклон локтя
//   angles[3] = Joint 4 (Wrist Pitch) — наклон запястья
//   angles[4] = Joint 5 (Wrist Roll) — вращение запястья
//   angles[5] = Joint 6 (Wrist Yaw) — поворот запястья
//   angles[6] = Gripper — захват (0-100%)
struct JointAngles {
    float angles[7];  //6 DOF + захват

    JointAngles() { memset(angles, 0, sizeof(angles)); }

    // Установить все углы сразу
    void setAll(float a0, float a1, float a2, float a3, float a4, float a5, float a6) {
        angles[0]=a0; angles[1]=a1; angles[2]=a2;
        angles[3]=a3; angles[4]=a4; angles[5]=a5; angles[6]=a6;
    }
};

// ====================================================================
// Целевая позиция (для обратной кинематики)
// ====================================================================
// Определяет целевую точку для IK:
//   position — куда переместить кончик пальцев (x, y, z в мм)
//   orientation — как ориентировать запястье (A=roll, B=pitch, C=yaw в градусах)
//   gripper — состояние захвата (0-100%)
struct Pose {
    Vec3 position;     // позиция (мм)
    Vec3 orientation;  // ориентация (градусы): A=roll, B=pitch, C=yaw
    float gripper;     // захват (0-100%)
};

// ====================================================================
// Тип G-кода
// ====================================================================
enum GCodeType {
    CMD_EMPTY,   // пустая строка или комментарий
    CMD_VALID,   // команда распознана и выполнена
    CMD_ERROR,   // ошибка парсинга или выполнения
};

// ====================================================================
// Стратегия интерполяции
// ====================================================================
// Определяет, как сервоприводы движутся к целевой позиции:
enum InterpolationStrategy {
    INTERP_LINEAR,      // все сервоприводы одновременно (быстро, но может нарушить безопасность)
    INTERP_SEQUENTIAL,  // по одному сервоприводу (медленно, но безопасно)
    INTERP_GROUPS,      // по группам (будущая реализация)
    INTERP_AUTO,        // сначала linear, если safety нарушается — sequential
};

// ====================================================================
// Структура G-кода
// ====================================================================
// Содержит распарсенную команду и все её параметры.
// Определяет режим выполнения (IK, Direct, Mixed).
struct GCodeCommand {
    GCodeType type = CMD_EMPTY;     // тип команды
    String    name;                 // имя команды ("G0", "M104", и т.д.)
    String    errorMsg;             // сообщение об ошибке

    // Режим выполнения
    enum Mode {
        IK_MODE,       // только IK (X, Y, Z, A, B, C)
        DIRECT_MODE,   // только прямое управление сервоприводами (P1..P7)
        MIXED_MODE     // комбинация IK и прямого управления
    };
    Mode mode = IK_MODE;

    // Целевая позиция (для IK и Mixed режимов)
    Pose pose;

    // Углы сервоприводов (для Direct и Mixed режимов)
    JointAngles servoAngles;
    bool servoOverride[7] = {};     // какие сервоприводы переопределены

    // Скорость перемещения (мм/с)
    float feedrate = 500.0;

    // Флаги наличия параметров
    bool hasPosition    = false;    // есть X, Y или Z
    bool hasOrientation = false;    // есть A, B или C
    bool hasGripper     = false;    // есть J (gripper)
    bool hasFeedrate    = false;    // есть F (feedrate)

    // Универсальные параметры (L, R, S, и т.д.)
    // Индексация по букве: params['L'-'A'] = params[11]
    float params[26] = {};
    bool  hasParam[26] = {};

    // Определение режима по наличию параметров
    void determineMode() {
        bool hasAnyServo = false;
        for (int i = 0; i < 7; i++) {
            if (servoOverride[i]) { hasAnyServo = true; break; }
        }

        if (hasPosition || hasOrientation) {
            if (hasAnyServo) mode = MIXED_MODE;
            else             mode = IK_MODE;
        } else if (hasAnyServo) {
            mode = DIRECT_MODE;
        }
    }
};

// ====================================================================
// Определение параметра команды
// ====================================================================
// Используется в таблице команд (CommandDef) для описания параметров:
//   letter — буква параметра ('X', 'Y', 'Z', 'P', и т.д.)
//   type   — тип значения (float, int, bool)
//   index  — индекс для P1..P7 (0-6), -1 = особый случай
//   defaultVal — значение по умолчанию
enum ParamType { PARAM_FLOAT, PARAM_INT, PARAM_BOOL };

struct ParamDef {
    char      letter;       // буква параметра
    ParamType type;         // тип значения
    int       index;        // индекс для P1..P7 (0-6), -1 = особый
    float     defaultVal;   // значение по умолчанию
};

// ====================================================================
// Определение команды в таблице
// ====================================================================
// Каждая G-команда описывается структурой CommandDef:
//   name        — имя команды ("G0", "M104", и т.д.)
//   description — описание (для вывода в ? help)
//   params[]    — массив параметров (до12)
//   paramCount  — количество параметров
//   handler     — указатель на функцию-обработчик
typedef void (*CommandHandler)(GCodeCommand &cmd);

struct CommandDef {
    const char*   name;          // имя команды
    const char*   description;   // описание
    ParamDef      params[12];    // параметры (до12)
    int           paramCount;    // количество параметров
    CommandHandler handler;      // обработчик
};
