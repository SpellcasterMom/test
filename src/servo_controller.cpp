/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       :
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-18
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
 *   - minPWM / maxPWM   — ширина PWM-импульса в мкс для предельных углов
 *   - minAngle / maxAngle — предельные углы серво в градусах
 *   - correctionFactor  — корректирующий множитель (100.0 = без коррекции)
 *   - isInverted        — флаг инвертирования направления вращения
 *   - backlash          — величина люфта в градусах
 *
 * Этот метод позволяет задать индивидуальные параметры для каждого серво
 * без перекомпиляции. Например, если серво 2 имеет искажённый ход,
 * можно подобрать correctionFactor = 105.3, чтобы компенсировать погрешность.
 */
void ServoController::setConfig(const ServoConfig configs[7]) {
    for (int i = 0; i < 7; i++) {
        this->configs[i] = configs[i];
    }
}

/**
 * Установка угла для одного сервопривода.
 *
 * Полный алгоритм обработки угла:
 *
 * 1. ВАЛИДАЦИЯ: проверяем, что servoId в диапазоне [0, 6],
 *    и что сервоприводы включены (enabled = true).
 *
 * 2. КОРРЕКЦИЯ МНОЖИТЕЛЕМ (correctionFactor):
 *    correctionFactor задаётся в процентах (по умолчанию 100.0).
 *    Формула: correctedAngle = angle * (correctionFactor / 100.0)
 *    Пример: correctionFactor = 105.0 компенсирует серво,
 *    которое проходит на 5% меньше заданного угла.
 *
 * 3. ИНВЕРТИРОВАНИЕ НАПРАВЛЕНИЯ (isInverted):
 *    Если серво установлено зеркально (например, правое плечо левого),
 *    инвертируем знак угла: correctedAngle = -correctedAngle.
 *    Это позволяет использовать единую кинематическую модель
 *    для зеркально-симметричных конфигураций.
 *
 * 4. ОГРАНИЧЕНИЕ УГЛА (clamping):
 *    Угол принудительно ограничивается допустимым диапазоном
 *    [minAngle, maxAngle]. Это защита от механического повреждения:
 *    серво не будет пытаться повернуться за допустимый предел.
 *
 * 5. КОНВЕРТАЦИЯ В PWM: вызов angleToPulse() для преобразования
 *    угла в ширину импульса в тиках PCA9685.
 *
 * 6. ЗАПИСЬ В PCA9685: pwm.setPWM(channel, 0, pulse) задаёт
 *    начало импульса в тике 0 и его конец в тике pulse.
 *    Для каналов 0–5 сервоприводы 6DOF, канал 6 — захватчик.
 *
 * 7. СОХРАНЕНИЕ СОСТОЯНИЯ: исходный угол (без коррекции) сохраняется
 *    в currentAngles, чтобы последующие вычисления (в том числе
 *    компенсация люфта) работали с «правильными» углами.
 *
 * @param servoId  индекс сервопривода (0–5: оси, 6: захват)
 * @param angle    целевой угол в градусах
 */
void ServoController::setAngle(int servoId, float angle) {
    // Проверка допустимости индекса сервопривода
    if (servoId < 0 || servoId > 6) return;
    // Сервоприводы отключены — игнорируем команду
    if (!enabled) return;

    // Применение корректирующего множителя:
    // correctionFactor = 100.0 → без коррекции
    // correctionFactor = 105.0 → угол увеличивается на 5%
    // correctionFactor = 95.0  → угол уменьшается на 5%
    float correction = configs[servoId].correctionFactor / 100.0f;
    float correctedAngle = angle * correction;

    // Инвертирование направления для зеркально установленных серво.
    // Например, если серво 4 (левое плечо) и серво 5 (правое плечо)
    // установлены зеркально, isInverted = true для одного из них.
    if (configs[servoId].isInverted) {
        correctedAngle = -correctedAngle;
    }

    // Ограничение угла допустимым диапазоном.
    // constrain() — функция Arduino, аналог clamp() из STL.
    // Защита от выхода за механические пределы сервопривода.
    correctedAngle = constrain(correctedAngle,
                               (float)configs[servoId].minAngle,
                               (float)configs[servoId].maxAngle);

    // Конвертация угла в ширину PWM-импульса (тиках PCA9685)
    uint16_t pulse = angleToPulse(servoId, correctedAngle);

    // Запись PWM-значения в канал PCA9685.
    // Каналы 0–5 — сервоприводы 6DOF, канал 6 — захватчик.
    // setPWM(channel, on_tick, off_tick): импульс начинается в тике 0
    // и заканчивается в тике pulse. Если pulse = 0, импульс не генерируется.
    if (pca9685Available) {
        if (servoId < 6) {
            pwm.setPWM(servoId, 0, pulse);
        } else {
            // Захватчик (грипер) всегда подключён к каналу 6
            pwm.setPWM(6, 0, pulse);
        }
    }

    // Сохраняем исходный угол (до коррекции и инверсии)
    // для последующих вычислений компенсации люфта
    currentAngles.angles[servoId] = angle;
}

/**
 * Последовательная установка углов для всех 7 сервоприводов.
 * Просто вызывает setAngle() для каждого индекса 0–6.
 * Углы применяются последовательно, что важно для kinematic chain.
 */
void ServoController::setAngles(const JointAngles &angles) {
    for (int i = 0; i < 7; i++) {
        setAngle(i, angles.angles[i]);
    }
}

/**
 * Возвращает текущие углы всех сервоприводов.
 * Значения обновляются при каждом вызове setAngle().
 * Используется для отслеживания текущей позиции манипулятора.
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
 * Конвертация угла (градусы) в ширину PWM-импульса (тики PCA9685).
 *
 * ШАГ 1: Линейная интерполяция угла → микросекунды.
 *   Формула:
 *     pulse_us = minPWM + (angle - minAngle) * (maxPWM - minPWM) / (maxAngle - minAngle)
 *
 *   где:
 *     minPWM, maxPWM — ширина импульса в мкс для minAngle и maxAngle
 *     minAngle, maxAngle — предельные углы серво в градусах
 *
 *   Пример для стандартного серво:
 *     minPWM=544, maxPWM=2400, minAngle=0, maxAngle=180
 *     angle=90 → pulse_us = 544 + (90-0) * (2400-544) / (180-0) = 1472 мкс
 *
 * ШАГ 2: Ограничение результата допустимым диапazonom PWM.
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
 *   Или в виде целочисленной арифметики:
 *     ticks = pulse_us * 4096.0 / 20000.0 * 1000.0 (здесь 1000.0 — неточность,
 *     вероятно, задумывалосьpulse_us уже в микросекундах)
 *
 *   Результат: 16-битное беззнаковое число (0–4096).
 *
 * @param servoId  индекс сервопривода (для доступа к конфигурации)
 * @param angle    угол в градусах (уже скорректированный и ограниченный)
 * @return         значение тиков для регистра ON/OFF PCA9685
 */
uint16_t ServoController::angleToPulse(int servoId, float angle) {
    // Извлечение параметров сервопривода из конфигурации
    float minPWM = configs[servoId].minPWM;
    float maxPWM = configs[servoId].maxPWM;
    float minAngle = configs[servoId].minAngle;
    float maxAngle = configs[servoId].maxAngle;

    // Линейная интерполяция: угол → ширина импульса в микросекундах
    float pulse = minPWM + (angle - minAngle) * (maxPWM - minPWM) / (maxAngle - minAngle);
    pulse = constrain(pulse, (float)minPWM, (float)maxPWM);

    // Конвертация микросекунд в тики PCA9685.
    // Период ШИМ = 20000 мкс, разрешение = 4096 тиков.
    // 1 тик = 20000 / 4096 ≈ 4.8828 мкс.
    // Формула: ticks = pulse_us / (20000 / 4096) = pulse_us * 4096 / 20000
    return (uint16_t)(pulse * 4096.0f / 20000.0f);
}
