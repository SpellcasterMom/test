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

#include <PCA9685.h>
#include "config.h"
#include "types.h"

/**
 * Класс ServoController — управление сервоприводами через драйвер PCA9685.
 *
 * Драйвер: janelia-arduino/PCA9685 (прямые микросекунды, без тиков).
 * Каждый канал генерирует импульсы с заданной шириной (мкс).
 *
 * Частота ШИМ 50 Гц (setToServoFrequency): период 20 мс (20000 мкс).
 * Длительность управляющего импульса 500–2500 мкс под конкретную серву
 * (minPWM/maxPWM из конфига).
 *
 * =====================================================================
 * ВАЖНО (Last modify: 2026-09-25):
 * =====================================================================
 * Введён homeOffset — сдвиг между МАТЕМАТИЧЕСКИМ углом (q_math, из IK)
 * и ФИЗИЧЕСКИМ углом сервы (servoAngle):
 *
 *   servoAngle = q_math + homeOffset
 *
 * При q_math = 0 физически серва стоит в homeOffset (135° для P1–P3,
 * 90° для P4–P6, 0° для gripper). Это позволяет IK/FK/safety работать
 * в «чистой» математике (0 = нейтральная поза), а физическое железо
 * получать корректные углы через ServoController::setAngle().
 *
 * minAngle/maxAngle в config.h остаются в МАТЕМАТИЧЕСКОМ диапазоне
 * (-135..+135, -90..+90, 0..100). homeOffset прибавляется ВНУТРИ
 * setAngle() и computePulseUs() перед конвертацией в PWM.
 */
class ServoController {
public:
    /**
     * Инициализация драйвера PCA9685.
     * Wire.begin(21, 22) + 400 кГц, проба адреса 0x40,
     * setupSingleDevice + частота сервоприводов 50 Гц.
     * Без железа — debug-режим (isPcaAvailable() == false).
     */
    void init();

    /**
     * Установка угла для одного сервопривода.
     *
     * Алгоритм (обновлён 2026-09-25):
     * 0. HOME OFFSET: servoAngle = q_math + homeOffset
     * 1. Валидация servoId (0–6) и проверка флага enabled.
     * 2. Применение корректирующего множителя (correctionFactor).
     *    Множитель задаётся в процентах: 100.0 — без коррекции,
     *    >100 — усиление, <100 — ослабление хода серво.
     * 3. Инвертирование направления (isInverted).
     *    Если серво установлено «зеркально», угол меняет знак.
     * 4. Ограничение (clamping) угла допустимыми пределами
     *    [minAngle+homeOffset, maxAngle+homeOffset] — в серво-диапазоне.
     * 5. Конвертация угла в ширину импульса (мкс) и запись в канал PCA9685.
     * 6. Запись значения PWM в соответствующий канал PCA9685.
     * 7. Сохранение МАТЕМАТИЧЕСКОГО угла (q_math) в currentAngles —
     *    для FK/IK/safety/display, чтобы они работали в единой СК.
     *
     * @param servoId  индекс сервопривода (0–6)
     * @param angle    МАТЕМАТИЧЕСКИЙ угол (q_math) в градусах
     */
    void setAngle(int servoId, float angle);

    /**
     * Пакетная установка углов для всех 7 сервоприводов.
     * Математика — через computePulseUs(), I2C — под одним take мьютекса.
     *
     * @param angles  структура с массивом МАТЕМАТИЧЕСКИХ углов для всех серво
     */
    void setAngles(const JointAngles &angles);

    /**
     * Возвращает текущие запомненные МАТЕМАТИЧЕСКИЕ углы всех сервоприводов.
     * Значения обновляются при каждом вызове setAngle().
     * Offset НЕ применён (это «чистая» математика для FK/IK/safety).
     */
    JointAngles getCurrentAngles();

    /**
     * Возвращает конфигурацию конкретного сервопривода.
     * Используется интерполятором для получения maxAngularSpeed и других параметров.
     *
     * @param servoId  индекс сервопривода (0-6)
     * @return         Константная ссылка на структуру ServoConfig
     */
    const ServoConfig& getConfig(int servoId) const;

    /**
     * Загрузка новой конфигурации для 7 сервоприводов.
     * Конфигурация содержит параметры каждого серво:
     * диапазон углов, диапазон PWM, корректирующий множитель,
     * флаг инвертирования, компенсацию люфта и homeOffset.
     *
     * @param configs  массив из 7 элементов ServoConfig
     */
    void setConfig(const ServoConfig configs[7]);

    /**
     * Включение сервоприводов.
     * После вызова setAngle() будет выдавать ШИМ-сигнал.
     */
    void enable();

    /**
     * Отключение сервоприводов.
     * Устанавливает флаг enabled = false, после чего setAngle()
     * игнорирует все команды. Также гасит все 7 каналов
     * (duty 0 — на выходе постоянный LOW, серво «разжимается»).
     */
    void disable();

    /**
     * Возвращает true, если PCA9685 обнаружен на I2C-шине и инициализирован.
     * false — модуль не найден, управление сервоприводами недоступно
     * (используется debug-режим без реального вывода ШИМ).
     *
     * Используется в main.cpp для отображения статуса PCA9685 на дисплее
     * (ON / OFF) и для диагностики проблем с подключением шины I2C.
     */
    bool isPcaAvailable() const { return pca9685Available; }

private:
    /** Объект драйвера PCA9685 (janelia-arduino), управляемый по I2C */
    PCA9685 pwm;

    /** Флаг: true если PCA9685 подключен и работает */
    bool pca9685Available = false;

    /** Массив конфигураций для 7 сервоприводов (6 осей + захват) */
    ServoConfig configs[7];

    /**
     * Текущие МАТЕМАТИЧЕСКИЕ углы всех сервоприводов (обновляется при setAngle).
     * Offset НЕ применён — это «чистая» математика для FK/IK/safety/display.
     */
    JointAngles currentAngles;

    /** Флаг включения: true — серво управляются, false — игнорируются */
    bool enabled = true;

    // Кэш линейной цепочки q_math → PWM (конфиг меняется только в
    // init()/setConfig(), поэтому делить на каждой записи незачем):
    //   corrK[i]    = correctionFactor/100 (точно как в setAngle)
    //   servoMinA[i]/servoMaxA[i] = min/maxAngle + homeOffset (серво-диапазон)
    float corrK[7];
    float servoMinA[7];
    float servoMaxA[7];
    void refreshPwmCache();

    /**
     * Чистая математика q_math → мкс ( offset → коррекция → инверсия →
     * clamp → линейная интерполяция). Без I2C и currentAngles —
     * общий мотор setAngle()/setAngles(). Бит-в-бит как раньше.
     */
    uint16_t computePulseUs(int servoId, float qmath);
};