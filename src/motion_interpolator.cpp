/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       :
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-18
 */

#include "motion_interpolator.h"
#include <cmath>

/**
 * @brief Конструктор по умолчанию.
 * Инициализирует все указатели как nullptr, стратегию — INTERP_AUTO,
 * текущие углы — нули. Флаг moving устанавливается в false.
 */
MotionInterpolator::MotionInterpolator()
    : servoCtrl(nullptr), backlash(nullptr), safety(nullptr),
      strategy(INTERP_AUTO), stepCallback(nullptr), moving(false) {
    memset(&currentAngles, 0, sizeof(currentAngles));
}

void MotionInterpolator::setServoController(ServoController *ctrl) {
    servoCtrl = ctrl;
}

void MotionInterpolator::setBacklashCompensator(BacklashCompensator *comp) {
    backlash = comp;
}

void MotionInterpolator::setSafetyChecker(SafetyChecker *s) {
    safety = s;
}

void MotionInterpolator::setStrategy(InterpolationStrategy strat) {
    strategy = strat;
}

void MotionInterpolator::setStepCallback(StepCallback cb) {
    stepCallback = cb;
}

bool MotionInterpolator::isMoving() const {
    return moving;
}

/**
 * @brief Немедленная остановка движения.
 * Устанавливает флаг moving=false. Цикл интерполяции проверяет этот флаг
 * на каждой итерации и прерывается при его сбросе.
 */
void MotionInterpolator::stop() {
    moving = false;
}

/**
 * @brief Движение к целевой позиции с заданной скоростью.
 *
 * Основной метод интерполяции. Выбирает стратегию и выполняет движение.
 * После успешного движения обновляет currentAngles на целевые значения.
 *
 * @param target Целевые углы 7 сервоприводов
 * @param feedrate Скорость подачи (градусов/сек для максимального.delta)
 * @return true если движение выполнено успешно
 */
bool MotionInterpolator::moveTo(const JointAngles &target, float feedrate) {
    if (!servoCtrl) return false;

    moving = true;

    bool result = false;
    switch (strategy) {
        case INTERP_LINEAR:
            result = interpolateLinear(target, feedrate);
            break;
        case INTERP_SEQUENTIAL:
            result = interpolateSequential(target, feedrate);
            break;
        case INTERP_AUTO:
        default:
            result = interpolateAuto(target, feedrate);
            break;
    }

    if (result) {
        currentAngles = target;
    }

    moving = false;
    return result;
}

/**
 * @brief Линейная интерполяция с ease-in-out сглаживанием.
 *
 * Все 7 сервоприводов движутся одновременно и синхронно.
 *
 * Алгоритм:
 * 1. Проверка безопасности траектории (checkTrajectory с 20 шагами)
 * 2. Нахождение максимального изменения угла среди всех сервоприводов
 * 3. Расчет времени движения на основе feedrate и максимального изменения
 * 4. Цикл интерполяции с ease-in-out сглаживанием:
 *    - Проверка флага moving (для экстренной остановки)
 *    - Проверка.serial порта на команду M112 (экстренная остановка)
 *    - Вычисление сглаженного параметра eased = easeInOut(t)
 *    - Выполнение шага executeStep(start, target, eased)
 *    - Задержка STEP_INTERVAL_MS между шагами
 * 5. Финальная позиция (t=1.0) для гарантии точного достижения цели
 *
 * @param target Целевые углы
 * @param feedrate Скорость подачи
 * @return true если движение выполнено успешно
 */
bool MotionInterpolator::interpolateLinear(const JointAngles &target, float feedrate) {
    // Проверка безопасности траектории перед началом движения
    if (safety && safety->isEnabled()) {
        if (!safety->checkTrajectory(currentAngles.angles, target.angles, 20)) {
            return false;
        }
    }

    // Нахождение максимального изменения угла среди всех сервоприводов
    float maxDelta = 0;
    for (int i = 0; i < 7; i++) {
        float delta = fabs(target.angles[i] - currentAngles.angles[i]);
        if (delta > maxDelta) maxDelta = delta;
    }

    // Если изменение меньше порога — движение не требуется
    if (maxDelta < MIN_STEP_DEG) return true;

    // Расчёт времени движения для каждого сервопривода
    // time_i = delta_i / maxAngularSpeed_i
    float servoTime[7] = {0};
    
    for (int i = 0; i < 7; i++) {
        float delta = fabs(target.angles[i] - currentAngles.angles[i]);
        
        if (servoCtrl && delta > 0) {
            float maxSpeed = servoCtrl->getConfig(i).maxAngularSpeed;
            if (maxSpeed > 0) {
                servoTime[i] = delta / maxSpeed;  // секунды
            }
        }
    }

    // Находим максимальное время (самый медленный сервопривод)
    float maxTime = 0;
    for (int i = 0; i < 7; i++) {
        if (servoTime[i] > maxTime) maxTime = servoTime[i];
    }
    
    // Применяем feedrate как множитель (100% = норма, 50% = в 2 раза медленнее)
    if (feedrate > 0 && feedrate != 100.0f) {
        maxTime = maxTime * (100.0f / feedrate);
    }
    
    // Пересчитываем в миллисекунды и количество шагов
    float timeMs = maxTime * 1000.0f;
    int steps = (int)(timeMs / STEP_INTERVAL_MS);
    if (steps < 1) steps = 1;

    // Сохранение начальных углов для интерполяции
    JointAngles start = currentAngles;

    // Цикл интерполяции
    for (int step = 0; step <= steps; step++) {
        // Проверка флага moving для возможности экстренной остановки через stop()
        if (!moving) break;

        // Проверка serial порта на команду M112 (экстренная остановка)
        // Читаем все доступные символы до символа новой строки
        if (Serial.available()) {
            String line = "";
            while (Serial.available()) {
                char c = Serial.read();
                if (c == '\n' || c == '\r') break;
                line += c;
            }
            line.trim();
            line.toUpperCase();
            // Если получена команда M112 — немедленная остановка
            if (line.startsWith("M112")) {
                moving = false;
                return false;
            }
        }

        // Вычисление параметра интерполяции (от 0 до 1)
        float t = (float)step / steps;
        // Применение ease-in-out сглаживания для плавного ускорения/замедления
        float eased = easeInOut(t);

        // Выполнение шага интерполяции (все сервоприводы одновременно)
        executeStep(start, target, eased);
        // Задержка между шагами для соблюдения временных интервалов
        delay((unsigned long)STEP_INTERVAL_MS);
    }

    // Финальная позиция: гарантируем точное достижение целевых углов
    if (moving) {
        executeStep(start, target, 1.0f);
    }

    return moving;
}

/**
 * @brief Последовательная интерполяция (сервоприводы по очереди).
 *
 * Каждый из 7 сервоприводов движется независимо, по очереди.
 * Этот метод безопаснее линейного, так как движение одного сервопривода
 * не вызывает одновременного изменения всех звеньев.
 *
 * Алгоритм для каждого сервопривода i (0-6):
 * 1. Проверка изменения угла (если < MIN_STEP_DEG — пропуск)
 * 2. Расчет времени движения на основе feedrate и изменения угла
 * 3. Цикл интерполяции для данного сервопривода:
 *    - Проверка флага moving
 *    - Проверка serial порта на M112 (экстренная остановка)
 *    - Вычисление угла с ease-in-out сглаживанием
 *    - Применение компенсации люфта (backlash compensation)
 *    - Установка угла через servoCtrl
 *    - Задержка STEP_INTERVAL_MS
 * 4. Финальный угол: точное установление целевого значения
 *    - Обновление направления люфта (backlash->updateDirection)
 *
 * @param target Целевые углы
 * @param feedrate Скорость подачи
 * @return true если движение выполнено успешно
 */
bool MotionInterpolator::interpolateSequential(const JointAngles &target, float feedrate) {
    for (int i = 0; i < 7; i++) {
        if (!moving) break;

        // Вычисление изменения угла для текущего сервопривода
        float delta = target.angles[i] - currentAngles.angles[i];
        // Если изменение меньше порога — пропускаем этот сервопривод
        if (fabs(delta) < MIN_STEP_DEG) continue;

        // Расчет времени движения (мс)
        float timeMs = (fabs(delta) / feedrate) * 1000.0f;
        int steps = (int)(timeMs / STEP_INTERVAL_MS);
        if (steps < 1) steps = 1;

        // Сохранение начального угла для интерполяции
        float startAngle = currentAngles.angles[i];

        // Цикл интерполяции для текущего сервопривода
        for (int step = 0; step <= steps; step++) {
            // Проверка флага moving для экстренной остановки
            if (!moving) break;

            // Проверка serial порта на команду M112 (экстренная остановка)
            if (Serial.available()) {
                String line = "";
                while (Serial.available()) {
                    char c = Serial.read();
                    if (c == '\n' || c == '\r') break;
                    line += c;
                }
                line.trim();
                line.toUpperCase();
                if (line.startsWith("M112")) {
                    moving = false;
                    return false;
                }
            }

            // Вычисление параметра интерполяции и сглаженного значения
            float t = (float)step / steps;
            float eased = easeInOut(t);
            // Линейная интерполяция угла с ease-in-out сглаживанием
            float angle = startAngle + delta * eased;

            // Применение компенсации люфта (если компенсатор установлен)
            if (backlash) {
                angle = backlash->compensate(i, angle);
            }

            // Установка угла на сервопривод
            servoCtrl->setAngle(i, angle);

            // Вызов callback для обновления состояния дисплея
            if (stepCallback) {
                JointAngles stepState = currentAngles;
                stepState.angles[i] = angle;
                stepCallback(stepState);
            }

            // Задержка между шагами
            delay((unsigned long)STEP_INTERVAL_MS);
        }

        // Финальное установление целевого угла
        if (moving) {
            float finalAngle = target.angles[i];
            // Применение компенсации люфта для финального угла
            if (backlash) {
                finalAngle = backlash->compensate(i, finalAngle);
                // Обновление направления люфта для корректной компенсации в будущем
                backlash->updateDirection(i, finalAngle);
            }
            servoCtrl->setAngle(i, finalAngle);
            // Обновление текущего угла в структуре
            currentAngles.angles[i] = target.angles[i];
        }
    }

    return moving;
}

/**
 * @brief Автоматическая стратегия интерполяции.
 *
 * Оптимальный подход: сначала пробует быструю линейную интерполяцию
 * (все сервоприводы одновременно). Если проверка безопасности не пройдена
 * (нарушение границ, коллизия), автоматически переключается на
 * последовательную интерполяцию (медленнее, но безопаснее).
 *
 * Это обеспечивает максимальную скорость при сохранении безопасности.
 * Если линейная интерполяция проходит проверку — используется она.
 * Если нет — безопасность важнее скорости.
 */
bool MotionInterpolator::interpolateAuto(const JointAngles &target, float feedrate) {
    // Сначала пробуем линейную интерполяцию (быстрый путь)
    if (interpolateLinear(target, feedrate)) {
        return true;
    }

    // Если M112 сработал во время линейной — не перезапускаем движение
    if (!moving) return false;

    // Если линейная не прошла по безопасности — переключаемся на последовательную
    return interpolateSequential(target, feedrate);
}

/**
 * @brief Функция сглаживания ease-in-out (квадратичная).
 *
 * Создает плавное ускорение в начале и замедление в конце движения.
 * Формула:
 *   t < 0.5: 2 * t²              (ускорение)
 *   t >= 0.5: 1 - (-2t + 2)² / 2 (замедление)
 *
 * Это квадратичная кривая, обеспечивающая естественное ощущение движения.
 *
 * @param t Время от 0.0 до 1.0
 * @return Сглаженное значение от 0.0 до 1.0
 */
float MotionInterpolator::easeInOut(float t) {
    return t < 0.5f ? 2.0f * t * t : 1.0f - (-2.0f * t + 2.0f) * (-2.0f * t + 2.0f) / 2.0f;
}

/**
 * @brief Выполнение одного шага интерполяции для всех сервоприводов.
 *
 * Для каждого из 7 сервоприводов:
 * 1. Вычисляет промежуточный угол: angle = from + (to - from) * t
 * 2. Применяет компенсацию люфта (если компенсатор установлен)
 * 3. Сохраняет угол в структуре step
 *
 * Затем отправляет все углы на сервоприводы одновременно
 * через servoCtrl->setAngles(step).
 *
 * @param from Начальные углы (текущая позиция)
 * @param to Целевые углы (конечная позиция)
 * @param t Параметр интерполяции (0.0 = from, 1.0 = to)
 */
void MotionInterpolator::executeStep(const JointAngles &from, const JointAngles &to, float t) {
    JointAngles step;

    for (int i = 0; i < 7; i++) {
        // Линейная интерполяция угла для текущего сервопривода
        float angle = from.angles[i] + (to.angles[i] - from.angles[i]) * t;

        // Применение компенсации люфта (если компенсатор установлен)
        if (backlash) {
            angle = backlash->compensate(i, angle);
        }

        step.angles[i] = angle;
    }

    // Отправка всех углов на сервоприводы одновременно
    servoCtrl->setAngles(step);

    // Вызов callback для обновления состояния дисплея (если установлен)
    if (stepCallback) {
        stepCallback(step);
    }
}
