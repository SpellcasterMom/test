/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-10-07
 */

#include "servo_controller.h"
#include "i2c_bus.h"

/**
 * Инициализация сервоконтроллера.
 *
 * Шаг 1: Wire.begin(I2C_*) + I2C_CLOCK_HZ (см. config.h), проба адреса 0x40
 *   (стандартный; перемычками A0–A5 может быть другим).
 *
 * Шаг 2: setupSingleDevice(Wire, 0x40) + setToServoFrequency() (50 Гц).
 *   Стандартная частота для сервоприводов. Период = 20 мс.
 *   Управляющий импульс серво длится от ~1 мс (0°) до ~2 мс (180°).
 *
 * Шаг 3: Загрузка конфигураций из DEFAULT_CONFIG для каждого серво.
 */
void ServoController::init() {
    // I2C явно: подъём из i2c_bus.h (пины/частота из config.h).
    bringUpI2CBus();

    // Проверяем, отвечает ли PCA9685 по I2C
    Wire.beginTransmission(0x40);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
        pca9685Available = true;
        pwm.setupSingleDevice(Wire, 0x40);
        pwm.setToServoFrequency();  // 50 Гц — стандарт для сервоприводов
    } else {
        pca9685Available = false;
        Serial.println("WARN: PCA9685 not found at 0x40, running in debug mode");
    }

    // Копируем конфигурации из глобального массива DEFAULT_CONFIG
    for (int i = 0; i < 7; i++) {
        configs[i] = DEFAULT_CONFIG.servos[i];
    }
    refreshPwmCache();
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
 * 0..100). homeOffset прибавляется внутри computePulseUs()
 * перед конвертацией в PWM.
 */
void ServoController::setConfig(const ServoConfig configs[7]) {
    for (int i = 0; i < 7; i++) {
        this->configs[i] = configs[i];
    }
    refreshPwmCache();
}

/**
 * refreshPwmCache — предрасчёт констант цепочки q_math → PWM.
 * Вызывается из init()/setConfig() (единственные места смены конфига).
 * Формулы — те же операции в том же порядке, что в setAngle(),
 * поэтому результат бит-в-бит совпадает с посчитанным на лету.
 */
void ServoController::refreshPwmCache() {
    for (int i = 0; i < 7; i++) {
        corrK[i] = configs[i].correctionFactor / 100.0f;
        servoMinA[i] = (float)configs[i].minAngle + configs[i].homeOffset;
        servoMaxA[i] = (float)configs[i].maxAngle + configs[i].homeOffset;
    }
}

/**
 * computePulseUs — чистая математика q_math → мкс (см. .h).
 * Деление (maxPWM−minPWM)/(servoMax−servoMin) оставлено как было —
 * замена на предрасчитанный множитель могла бы сдвинуть младший бит
 * после (uint16_t)-обрезки; экономия 350 дел/с того не стоит.
 */
uint16_t ServoController::computePulseUs(int servoId, float qmath) {
    // ШАГ 0: math → servo (прибавляем home offset).
    float servoAngle = qmath + configs[servoId].homeOffset;

    // ШАГ 1: корректирующий множитель (из кэша).
    float correctedAngle = servoAngle * corrK[servoId];

    // ШАГ 2: инверсия относительно центра серво-диапазона (см. доку setAngle).
    if (configs[servoId].isInverted) {
        correctedAngle = (servoMinA[servoId] + servoMaxA[servoId]) - correctedAngle;
    }

    // ШАГ 3: clamp в серво-диапазоне.
    correctedAngle = constrain(correctedAngle, servoMinA[servoId], servoMaxA[servoId]);

    // ШАГ 4: линейная интерполяция в микросекунды (внутри computePulseUs).
    float minPWM = configs[servoId].minPWM;
    float maxPWM = configs[servoId].maxPWM;
    float pulse = minPWM + (correctedAngle - servoMinA[servoId]) * (maxPWM - minPWM)
                           / (servoMaxA[servoId] - servoMinA[servoId]);
    pulse = constrain(pulse, (float)minPWM, (float)maxPWM);

    return (uint16_t)pulse;
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
 *    зеркало относительно центра серво-диапазона:
 *    correctedAngle = (servoMin+servoMax) - correctedAngle.
 *    Центр стоит (P1: q=0 → 135), края меняются местами.
 *
 * 4. ОГРАНИЧЕНИЕ УГЛА (clamping):
 *    constrain(correctedAngle, minAngle+homeOffset, maxAngle+homeOffset)
 *    — в СЕРВО-диапазоне (после offset).
 *
 * 5. КОНВЕРТАЦИЯ В PWM: computePulseUs() → микросекунды.
 *
 * 6. ЗАПИСЬ В PCA9685: setChannelServoPulseDuration(channel, мкс).
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

    // Вся математика q_math → мкс — в общем моторе (бит-в-бит как раньше).
    uint16_t pulseUs = computePulseUs(servoId, angle);

    // Запись ширины импульса в канал PCA9685 (janelia берёт мкс).
    // Каналы 0–5 — сервоприводы 6DOF, канал 6 — захватчик.
    // Шина I2C общая с BMI160: сериализуем с IMU-задачей (Core 1).
    // Брать мьютекс — с ТАЙМАУТОМ, не вечно: если шина клинит (просадка
    // питания сервами посреди чужой транзакции), пропуск одного шага 20 мс
    // незаметен, а вечное ожидание роняло бы всё движение целиком.
    if (pca9685Available) {
        extern SemaphoreHandle_t i2cMutex;
        bool i2cLocked = false;
        if (i2cMutex) {
            i2cLocked = (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE);
        } else {
            i2cLocked = true;  // мьютекса ещё нет (ранний setup) — шина наша
        }
        if (i2cLocked) {
            pwm.setChannelServoPulseDuration((uint16_t)servoId, pulseUs);
            if (i2cMutex) xSemaphoreGive(i2cMutex);
        }
        // else: шаг пропущен, currentAngles ниже всё равно обновится
    }

    // ШАГ 6: сохраняем МАТЕМАТИЧЕСКИЙ угол (без offset и коррекции) —
    // чтобы FK/IK/safety/display работали в единой системе координат.
    currentAngles.angles[servoId] = angle;
}

/**
 * Последовательная установка углов для всех 7 сервоприводов.
 *
 * Пакетный путь интерполятора (шаг 20 мс): математика всех 7 каналов
 * считается без I2C, затем ОДИН take мьютекса на 7 записей (раньше —
 * 7 take/give с таймаутом 100 мс каждый). Не взяли мьютекс — пропускаем
 * все 7 записей разом, currentAngles всё равно обновляются (как раньше
 * при пропуске одиночного шага). Поведение вовне то же.
 *
 * @param angles — структура с МАТЕМАТИЧЕСКИМИ углами всех серво
 */
void ServoController::setAngles(const JointAngles &angles) {
    if (!enabled) return;

    uint16_t pulse[7];
    for (int i = 0; i < 7; i++) {
        pulse[i] = computePulseUs(i, angles.angles[i]);
    }

    if (pca9685Available) {
        extern SemaphoreHandle_t i2cMutex;
        bool i2cLocked = false;
        if (i2cMutex) {
            i2cLocked = (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE);
        } else {
            i2cLocked = true;
        }
        if (i2cLocked) {
            for (int i = 0; i < 7; i++) {
                pwm.setChannelServoPulseDuration((uint16_t)i, pulse[i]);
            }
            if (i2cMutex) xSemaphoreGive(i2cMutex);
        }
    }

    for (int i = 0; i < 7; i++) {
        currentAngles.angles[i] = angles.angles[i];
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
 * 2. Гасит все 7 каналов (duty 0 — постоянный LOW).
 *    При on=0, off=0 сигнал на выходе канала постоянно LOW —
 *    сервопривод получает питание, но управляющий импульс отсутствует.
 *    Серво «разжимается» и удерживает текущую позицию без усилия.
 */
void ServoController::disable() {
    enabled = false;
    // Гасим все 7 каналов: duty 0 — на выходе постоянный LOW,
    // серво «разжимается» и удерживает позицию без усилия.
    if (pca9685Available) {
        for (int i = 0; i < 7; i++) {
            pwm.setChannelDutyCycle((uint16_t)i, 0);
        }
    }
}
