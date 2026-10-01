/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-25
 */

#include "servo_controller.h"

/**
 * Инициализация сервоконтроллера.
 *
 * Шаг 1: Создание объекта PCA9685 с I2C-адресом 0x40.
 *   Адрес 0x40 — стандартный для модулей Adafruit PCA9685.
 *   Если на плате выпаяны перемычки A0–A5, адрес может быть другим
 *   (0x41–0x7F). Адрес формируется как: 0x40 | (A5<<5 | ... | A0).
 *
 * Шаг 2: pwm.begin() — инициализирует I2C-хост (Wire) и сбрасывает
 *   все регистры PCA9685 в начальное состояние.
 *
 * Шаг 3: pwm.setPWMFreq(50) — установка частоты ШИМ = 50 Гц.
 *   Стандартная частота для сервоприводов. Период = 20 мс.
 *   PCA9685 делит период на 4096 тиков, поэтому 1 тик ≈ 4.88 мкс.
 *   Управляющий импульс серво длится от ~1 мс (0°) до ~2 мс (180°).
 *
 * Шаг 4: Загрузка конфигураций из DEFAULT_CONFIG для каждого серво.
 */
void ServoController::init() {
    pwm = Adafruit_PWMServoDriver(0x40);
    
    // Проверяем, отвечает ли PCA9685 по I2C
    Wire.beginTransmission(0x40);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
        pca9685Available = true;
        pwm.begin();
        pwm.setPWMFreq(50);  // 50 Гц — стандартная частота для сервоприводов
    } else {
        pca9685Available = false;
        Serial.println("WARN: PCA9685 not found at 0x40, running in debug mode");
    }

    // Копируем конфигурации из глобального массива DEFAULT_CONFIG
    for (int i = 0; i < 7; i++) {
        configs[i] = DEFAULT_CONFIG.servos[i];
    }
}

/**
 * Установка конфигурации для всех 7 сервоприводов.
 *
 * Конфигурация каждого серво (ServoConfig) содержит:
 *   - minPWM / maxPWM     — ширина PWM-импульса в мкс для предельных углов
 *   - minAngle / maxAngle — предельные МАТЕМАТИЧЕСКИЕ углы серво (градусы)
 *   - correctionFactor    — корректирующий множитель (100.0 = без коррекции)
 *   - isInverted          — флаг инвертирования направления вращения
 *   - backlash            — величина люфта в градусах
 *   - homeOffset          — сдвиг math → servo (135° / 90° / 0°)
 *
 * minAngle/maxAngle — в МАТЕМАТИЧЕСКОМ диапазоне (-135..+135, -90..+90,
 * 0..100). homeOffset прибавляется внутри setAngle() и angleToPulse()
 * перед конвертацией в PWM.
 */
void ServoController::setConfig(const ServoConfig configs[7]) {
    for (int i = 0; i < 7; i++) {
        this->configs[i] = configs[i];
    }
}

/**
 * Установка угла для одного сервопривода.
 *
 * Полный алгоритм обработки угла (обновлён 2026-09-25):
 *
 * 0. HOME OFFSET (math → servo):
 *    q_math — математический угол из IK (0 = «нулевая поза»).
 *    servoAngle = q_math + homeOffset — физический угол сервы.
 *    P1–P3: homeOffset = 135° (math 0 → серва 135°)
 *    P4–P6: homeOffset =  90° (math 0 → серва  90°)
 *    P7:    homeOffset =   0°
 *
 * 1. ВАЛИДАЦИЯ: проверяем, что servoId в диапазоне [0, 6],
 *    и что сервоприводы включены (enabled = true).
 *
 * 2. КОРРЕКЦИЯ МНОЖИТЕЛЕМ (correctionFactor):
 *    correctedAngle = servoAngle * (correctionFactor / 100.0)
 *
 * 3. ИНВЕРТИРОВАНИЕ НАПРАВЛЕНИЯ (isInverted):
 *    correctedAngle = -correctedAngle (если isInverted)
 *
 * 4. ОГРАНИЧЕНИЕ УГЛА (clamping):
 *    constrain(correctedAngle, minAngle+homeOffset, maxAngle+homeOffset)
 *    — в СЕРВО-диапазоне (после offset).
 *
 * 5. КОНВЕРТАЦИЯ В PWM: angleToPulse() → тики PCA9685.
 *
 * 6. ЗАПИСЬ В PCA9685: pwm.setPWM(channel, 0, pulse).
 *
 * 7. СОХРАНЕНИЕ: в currentAngles хранится МАТЕМАТИЧЕСКИЙ угол (q_math),
 *    а не servo — чтобы FK/IK/safety/display работали в «чистой» СК.
 *
 * @param servoId  индекс сервопривода (0–5: оси, 6: захват)
 * @param angle    МАТЕМАТИЧЕСКИЙ угол (q_math) в градусах
 */
void ServoController::setAngle(int servoId, float angle) {
    // Проверка допустимости индекса сервопривода
    if (servoId < 0 || servoId > 6) return;
    // Сервоприводы отключены — игнорируем команду
    if (!enabled) return;

    // ШАГ 0: math → servo (прибавляем home offset).
    // Пример: q_math = 0 → servoAngle = 135° (P1) или 90° (P4).
    float servoAngle = angle + configs[servoId].homeOffset;

    // ШАГ 1: применение корректирующего множителя.
    // correctionFactor = 100.0 → без коррекции
    // correctionFactor = 105.0 → угол увеличивается на 5%
    float correction = configs[servoId].correctionFactor / 100.0f;
    float correctedAngle = servoAngle * correction;

    // ШАГ 2: инвертирование направления для зеркально установленных серво.
    if (configs[servoId].isInverted) {
        correctedAngle = -correctedAngle;
    }

    // ШАГ 3: ограничение угла в СЕРВО-диапазоне (после offset).
    // minAngle/maxAngle в config.h — математические, поэтому здесь
    // смещаем границы на homeOffset для получения фактического
    // физического диапазона сервы.
    float servoMin = (float)configs[servoId].minAngle + configs[servoId].homeOffset;
    float servoMax = (float)configs[servoId].maxAngle + configs[servoId].homeOffset;
    correctedAngle = constrain(correctedAngle, servoMin, servoMax);

    // ШАГ 4: конвертация угла в ширину PWM-импульса (тики PCA9685).
    // angleToPulse() также смещает границы на homeOffset внутри.
    uint16_t pulse = angleToPulse(servoId, correctedAngle);

    // ШАГ 5: запись PWM-значения в канал PCA9685.
    // Каналы 0–5 — сервоприводы 6DOF, канал 6 — захватчик.
    if (pca9685Available) {
        if (servoId < 6) {
            pwm.setPWM(servoId, 0, pulse);
        } else {
            pwm.setPWM(6, 0, pulse);
        }
    }

    // ШАГ 6: сохраняем МАТЕМАТИЧЕСКИЙ угол (без offset и коррекции) —
    // чтобы FK/IK/safety/display работали в единой системе координат.
    currentAngles.angles[servoId] = angle;
}

/**
 * Последовательная установка углов для всех 7 сервоприводов.
 * Просто вызывает setAngle() для каждого индекса 0–6.
 * Углы применяются последовательно, что важно для kinematic chain.
 *
 * @param angles — структура с МАТЕМАТИЧЕСКИМИ углами всех серво
 */
void ServoController::setAngles(const JointAngles &angles) {
    for (int i = 0; i < 7; i++) {
        setAngle(i, angles.angles[i]);
    }
}

/**
 * Возвращает текущие МАТЕМАТИЧЕСКИЕ углы всех сервоприводов.
 * Значения обновляются при каждом вызове setAngle().
 * Offset НЕ применён — это «чистая» математика для FK/IK/safety/display.
 */
JointAngles ServoController::getCurrentAngles() {
    return currentAngles;
}

const ServoConfig& ServoController::getConfig(int servoId) const {
    return configs[servoId];
}

/**
 * Включение сервоприводов.
 * После вызова все последующие setAngle() будут выдавать ШИМ-сигнал.
 */
void ServoController::enable() {
    enabled = true;
}

/**
 * Отключение сервоприводов.
 *
 * 1. Устанавливает enabled = false — дальнейшие setAngle() игнорируются.
 * 2. Обнуляет PWM на всех 7 каналах: pwm.setPWM(i, 0, 0).
 *    При on=0, off=0 сигнал на выходе канала постоянно LOW —
 *    сервопривод получает питание, но управляющий импульс отсутствует.
 *    Серво «разжимается» и удерживает текущую позицию без усилия.
 */
void ServoController::disable() {
    enabled = false;
    // Отключаем PWM на всех каналах — серво переходят в состояние покоя
    if (pca9685Available) {
        for (int i = 0; i < 7; i++) {
            pwm.setPWM(i, 0, 0);
        }
    }
}

/**
 * Конвертация СЕРВО-угла (градусы) в ширину PWM-импульса (тики PCA9685).
 *
 * ВАЖНО (Last modify: 2026-09-25): angle приходит как СЕРВО-угол
 * (уже после homeOffset). Границы minAngle/maxAngle в конфиге —
 * МАТЕМАТИЧЕСКИЕ, поэтому внутри функции они смещаются на homeOffset:
 *
 *   servoMinAngle = minAngle + homeOffset
 *   servoMaxAngle = maxAngle + homeOffset
 *
 * ШАГ 1: Линейная интерполяция угла → микросекунды.
 *   Формула:
 *     pulse_us = minPWM + (angle - servoMinAngle) * (maxPWM - minPWM)
 *                       / (servoMaxAngle - servoMinAngle)
 *
 *   где:
 *     minPWM, maxPWM         — ширина импульса в мкс для предельных углов
 *     servoMinAngle, servoMaxAngle — предельные СЕРВО-углы в градусах
 *
 *   Пример для 270°-сервы (P1):
 *     minAngle = -135, maxAngle = +135, homeOffset = 135
 *     servoMinAngle = 0, servoMaxAngle = 270
 *     minPWM = 500, maxPWM = 2500
 *     angle (servo) = 135 → pulse_us = 500 + (135 - 0) * 2000 / 270 = 1500 мкс
 *
 * ШАГ 2: Ограничение результата допустимым диапазоном PWM.
 *
 * ШАГ 3: Конвертация микросекунд в тики PCA9685.
 *   PCA9685 генерирует ШИМ с частотой 50 Гц:
 *     Период = 1/50 = 0.02 с = 20000 мкс
 *     Разрешение = 12 бит = 4096 тиков за период
 *     1 тик = 20000 / 4096 ≈ 4.8828 мкс
 *
 *   Формула конвертации:
 *     ticks = pulse_us * 4096 / 20000
 *
 *   Результат: 16-битное беззнаковое число (0–4096).
 *
 * @param servoId  индекс сервопривода (для доступа к конфигурации)
 * @param angle    СЕРВО-угол в градусах (уже с homeOffset)
 * @return         значение тиков для регистра ON/OFF PCA9685
 */
uint16_t ServoController::angleToPulse(int servoId, float angle) {
    // Извлечение параметров сервопривода из конфигурации
    float minPWM = configs[servoId].minPWM;
    float maxPWM = configs[servoId].maxPWM;

    // ВАЖНО: angle приходит как СЕРВО-угол (после offset),
    // поэтому границы тоже смещаем на homeOffset.
    float servoMinAngle = (float)configs[servoId].minAngle + configs[servoId].homeOffset;
    float servoMaxAngle = (float)configs[servoId].maxAngle + configs[servoId].homeOffset;

    // Линейная интерполяция: СЕРВО-угол → ширина импульса в микросекундах
    float pulse = minPWM + (angle - servoMinAngle) * (maxPWM - minPWM)
                          / (servoMaxAngle - servoMinAngle);
    pulse = constrain(pulse, (float)minPWM, (float)maxPWM);

    // Конвертация микросекунд в тики PCA9685.
    // Период ШИМ = 20000 мкс, разрешение = 4096 тиков.
    // 1 тик = 20000 / 4096 ≈ 4.8828 мкс.
    // Формула: ticks = pulse_us / (20000 / 4096) = pulse_us * 4096 / 20000
    return (uint16_t)(pulse * 4096.0f / 20000.0f);
}