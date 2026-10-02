/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-10-01
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
      ikSolver(nullptr), fkSolver(nullptr),
      strategy(INTERP_AUTO), stepCallback(nullptr), moving(false) {
    memset(&currentAngles, 0, sizeof(currentAngles));
    setMark("NONE", "-");
}

/**
 * @brief Записать метку последнего движения (для M115).
 */
void MotionInterpolator::setMark(const char *mode, const char *note) {
    strncpy(lastMode, mode, sizeof(lastMode) - 1);
    lastMode[sizeof(lastMode) - 1] = '\0';
    strncpy(lastNote, note, sizeof(lastNote) - 1);
    lastNote[sizeof(lastNote) - 1] = '\0';
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

void MotionInterpolator::setIKSolver(IKSolver *ik) {
    ikSolver = ik;
}

void MotionInterpolator::setFKSolver(FKSolver *fk) {
    fkSolver = fk;
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
 * @param feedrate Процент от предельной скорости (0-100, >100 = 100%).
 *               Лимиты серв (maxAngularSpeed) не превышаются никогда.
 * @return true если движение выполнено успешно
 */
bool MotionInterpolator::moveTo(const JointAngles &target, float feedrate) {
    if (!servoCtrl) return false;

    // Зажим цели в лимиты серв (математический диапазон): команда вроде
    // M104 P2=200 больше не уведёт currentAngles в ложь (серво всё равно
    // обрезалось бы в setAngle, а состояние врало бы 200). Единая точка —
    // накрывает G0/M104/G28 сразу, IK-решения уже зажаты в clampAngle.
    JointAngles lim = target;
    for (int i = 0; i < 7; i++) {
        const ServoConfig &cfg = servoCtrl->getConfig(i);
        lim.angles[i] = constrain(lim.angles[i],
                                 (float)cfg.minAngle, (float)cfg.maxAngle);
    }

    moving = true;

    bool result = false;
    switch (strategy) {
        case INTERP_LINEAR:
            result = interpolateLinear(lim, feedrate);
            break;
        case INTERP_SEQUENTIAL:
            result = interpolateSequential(lim, feedrate);
            break;
        case INTERP_AUTO:
        default:
            result = interpolateAuto(lim, feedrate);
            break;
    }

    if (result) {
        currentAngles = lim;
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
            char note[64];
            snprintf(note, sizeof(note), "traj-fail: %.48s",
                     safety->getLastError().c_str());
            setMark("LIN", note);
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
    
    // Feedrate — процент от предельной скорости (100 = полная скорость
    // по лимитам серв, 50 = в 2 раза медленнее). Значения >100 обрезаются
    // до 100: maxAngularSpeed — жёсткий потолок, превышение запрещено.
    // NOTE: исторически сюда приходили мм/с (500/300) — они мапятся на 100%.
    // Честный мм/с по TCP требует длины траектории через FK (TODO).
    float pct = 100.0f;
    if (feedrate > 0.0f && feedrate < 100.0f) pct = feedrate;
    if (pct != 100.0f) {
        maxTime = maxTime * (100.0f / pct);
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
                setMark("LIN", "stopped-M112");
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
        char note[64];
        snprintf(note, sizeof(note), "steps=%d timeMs=%.0f", steps, timeMs);
        setMark("LIN", note);
    } else {
        setMark("LIN", "stopped");
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
 * 2. Расчет времени движения из maxAngularSpeed и изменения угла
 *    (feedrate ≤100 — только замедление, см. interpolateLinear)
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
    int totalSteps = 0;
    float totalMs = 0;
    for (int i = 0; i < 7; i++) {
        if (!moving) break;

        // Вычисление изменения угла для текущего сервопривода
        float delta = target.angles[i] - currentAngles.angles[i];
        // Если изменение меньше порога — пропускаем этот сервопривод
        if (fabs(delta) < MIN_STEP_DEG) continue;

        // Время движения из лимита сервы (maxAngularSpeed — жёсткий потолок),
        // feedrate ≤100 — процент замедления (см. interpolateLinear).
        float ceiling = 180.0f;
        if (servoCtrl) {
            float lim = servoCtrl->getConfig(i).maxAngularSpeed;
            if (lim > 0) ceiling = lim;
        }
        float pct = 100.0f;
        if (feedrate > 0.0f && feedrate < 100.0f) pct = feedrate;
        float timeMs = (fabs(delta) / (ceiling * pct / 100.0f)) * 1000.0f;
        int steps = (int)(timeMs / STEP_INTERVAL_MS);
        if (steps < 1) steps = 1;
        totalSteps += steps;
        totalMs += timeMs;

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
                    setMark("SEQ", "stopped-M112");
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

    if (moving) {
        char note[64];
        snprintf(note, sizeof(note), "steps=%d timeMs=%.0f", totalSteps, totalMs);
        setMark("SEQ", note);
    } else {
        setMark("SEQ", "stopped");
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

    // Запоминаем причину отказа linear: sequential ниже её перезапишет,
    // а для диагностики (M115) важно знать, почему не поехали прямо.
    char linNote[64];
    strncpy(linNote, lastNote, sizeof(linNote) - 1);
    linNote[sizeof(linNote) - 1] = '\0';

    // Если линейная не прошла по безопасности — переключаемся на последовательную
    bool ok = interpolateSequential(target, feedrate);
    if (ok && strncmp(linNote, "traj-fail", 9) == 0) {
        char seqNote[96];
        strncpy(seqNote, lastNote, sizeof(seqNote) - 1);
        seqNote[sizeof(seqNote) - 1] = '\0';
        char note[96];
        snprintf(note, sizeof(note), "%s | fbk %.40s", seqNote, linNote);
        setMark("SEQ", note);
    }
    return ok;
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
 * @param t Параметр интерполяции (0..1)
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

// ============================================================================
// Декартово движение TCP (moveToPose): lerp позиции + slerp ориентации,
// IK-доводка каждого шага от предыдущего (трекинг).
// ============================================================================

static const float CART_PI = 3.14159265f;
static const float CART_JUMP_DEG = 45.0f;  // порог срыва трекинга за шаг

static float cartDeg2Rad(float d) { return d * CART_PI / 180.0f; }
static float cartRad2Deg(float r) { return r * 180.0f / CART_PI; }
static float cartClamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Эйлеры ZYX (градусы) -> R 3x3 row-major. Та же конвенция, что eulerToMatrix:
// R = Rz(yaw) * Ry(pitch) * Rx(roll).
static void cartEulerToR(float roll, float pitch, float yaw, float R[9]) {
    float cr = cos(cartDeg2Rad(roll)),  sr = sin(cartDeg2Rad(roll));
    float cp = cos(cartDeg2Rad(pitch)), sp = sin(cartDeg2Rad(pitch));
    float cy = cos(cartDeg2Rad(yaw)),   sy = sin(cartDeg2Rad(yaw));
    R[0] = cy*cp; R[1] = cy*sp*sr - sy*cr; R[2] = cy*sp*cr + sy*sr;
    R[3] = sy*cp; R[4] = sy*sp*sr + cy*cr; R[5] = sy*sp*cr - cy*sr;
    R[6] = -sp;   R[7] = cp*sr;            R[8] = cp*cr;
}

// R 3x3 -> эйлеры ZYX (градусы).
// Замок кардана: степени сливаются — кладём всё в yaw, roll=0:
//   pitch=+90: yaw = atan2(-R01, R02); pitch=-90: yaw = atan2(-R01, -R02).
static void cartRToEuler(const float R[9], float &roll, float &pitch, float &yaw) {
    float sp = -R[6];
    if (sp > 0.9999f) {
        pitch = 90.0f; roll = 0.0f;
        yaw = cartRad2Deg(atan2(-R[1], R[2]));
    } else if (sp < -0.9999f) {
        pitch = -90.0f; roll = 0.0f;
        yaw = cartRad2Deg(atan2(-R[1], -R[2]));
    } else {
        pitch = cartRad2Deg(asin(sp));
        roll = cartRad2Deg(atan2(R[7], R[8]));
        yaw = cartRad2Deg(atan2(R[3], R[0]));
    }
}

// Максимальная разница суставов двух конфигураций (первые 6, градусы).
static float cartJointDist(const JointAngles &a, const JointAngles &b) {
    float m = 0;
    for (int i = 0; i < 6; i++) {
        float d = fabs(a.angles[i] - b.angles[i]);
        if (d > m) m = d;
    }
    return m;
}

// Подтянуть эйлеры e к prev: варианты ±360 и flip-представление
// (roll+180, 180-pitch, yaw+180) — та же матрица, гладкий трек без snap'ов.
static void cartUnwrapEuler(const float prev[3], float e[3]) {
    float best[3] = {e[0], e[1], e[2]};
    float bestD = 1e30f;
    for (int fr = -1; fr <= 1; fr++) {
        for (int fy = -1; fy <= 1; fy++) {
            float c0[3] = {e[0] + 360.0f * fr, e[1], e[2] + 360.0f * fy};
            float d0 = fabs(c0[0]-prev[0]) + fabs(c0[1]-prev[1]) + fabs(c0[2]-prev[2]);
            if (d0 < bestD) {
                bestD = d0;
                best[0] = c0[0]; best[1] = c0[1]; best[2] = c0[2];
            }
            float c1[3] = {e[0] + 180.0f + 360.0f * fr, 180.0f - e[1],
                           e[2] + 180.0f + 360.0f * fy};
            float d1 = fabs(c1[0]-prev[0]) + fabs(c1[1]-prev[1]) + fabs(c1[2]-prev[2]);
            if (d1 < bestD) {
                bestD = d1;
                best[0] = c1[0]; best[1] = c1[1]; best[2] = c1[2];
            }
        }
    }
    e[0] = best[0]; e[1] = best[1]; e[2] = best[2];
}

/**
 * @brief Вывод готовых углов на сервы (люфт + callback).
 * Как executeStep(), но углы уже посчитаны (IK), интерполировать нечего.
 */
void MotionInterpolator::outputStep(const JointAngles &q) {
    JointAngles out = q;
    if (backlash) {
        for (int i = 0; i < 7; i++) {
            out.angles[i] = backlash->compensate(i, out.angles[i]);
        }
    }
    servoCtrl->setAngles(out);
    if (stepCallback) {
        stepCallback(out);
    }
}

/**
 * @brief Декартово движение к целевой позе TCP.
 *
 * TCP идёт по прямой (lerp позиции + slerp ориентации через ось-угол
 * R0ᵀ·R1), эйлеры разворачиваются к предыдущему шагу (без snap'ов),
 * каждый шаг — IK-доводка от предыдущего решения (трекинг) + safety.
 * Срыв трекинга (скачок > CART_JUMP_DEG) — вторая попытка геометрией
 * (solveAll, ближайший к предыдущему); не вышло — стоп на последнем
 * хорошем шаге, false. Кончик стоит на прямой с точностью IK.
 */
bool MotionInterpolator::moveToPose(const Pose &targetPose, float feedrate) {
    if (!servoCtrl || !ikSolver || !fkSolver) return false;
    moving = true;

    // Бюджет доводки на шаг: трекинг сходится за ~3-15 итераций (двойник),
    // полный бюджет 100 на шаг давал рывки ~6 раз медленнее плана.
    // Допуски те же (1мм/1°). В конце восстанавливаем (дефолт 100/1.0/1.0).
    ikSolver->setNumericParams(25, 1.0f, 1.0f);

    // --- Стартовая поза TCP из текущих углов ---
    float qs[6];
    for (int i = 0; i < 6; i++) qs[i] = currentAngles.angles[i];
    Vec3 p0 = fkSolver->getEndEffector(qs);
    Mat4 T0 = fkSolver->getRotationMatrix(qs, 5);
    float R0[9] = {T0.m[0][0], T0.m[0][1], T0.m[0][2],
                   T0.m[1][0], T0.m[1][1], T0.m[1][2],
                   T0.m[2][0], T0.m[2][1], T0.m[2][2]};
    float R1[9];
    cartEulerToR(targetPose.orientation.x, targetPose.orientation.y,
                 targetPose.orientation.z, R1);

    // --- Ось-угол slerp: Rd = R0ᵀ · R1 ---
    float Rd[9];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0;
            for (int k = 0; k < 3; k++) s += R0[k*3+i] * R1[k*3+j];
            Rd[i*3+j] = s;
        }
    }
    float tr = Rd[0] + Rd[4] + Rd[8];
    float cth = cartClamp((tr - 1.0f) * 0.5f, -1.0f, 1.0f);
    float th = acos(cth);
    float ax[3] = {Rd[7]-Rd[5], Rd[2]-Rd[6], Rd[3]-Rd[1]};
    float sl = sqrt(ax[0]*ax[0] + ax[1]*ax[1] + ax[2]*ax[2]);
    if (sl > 1e-6f) {
        ax[0] /= sl; ax[1] /= sl; ax[2] /= sl;
    } else if (cth < 0) {
        // ~180°: ось вырождена. Направление: столбец max диагонали (Rd+I)/2.
        float xx = (Rd[0]+1.0f)*0.5f, yy = (Rd[4]+1.0f)*0.5f, zz = (Rd[8]+1.0f)*0.5f;
        if (xx >= yy && xx >= zz && xx > 1e-9f) {
            ax[0] = sqrt(xx); ax[1] = (Rd[1]+Rd[3])/(4.0f*ax[0]); ax[2] = (Rd[2]+Rd[6])/(4.0f*ax[0]);
        } else if (yy >= zz && yy > 1e-9f) {
            ax[1] = sqrt(yy); ax[0] = (Rd[1]+Rd[3])/(4.0f*ax[1]); ax[2] = (Rd[5]+Rd[7])/(4.0f*ax[1]);
        } else if (zz > 1e-9f) {
            ax[2] = sqrt(zz); ax[0] = (Rd[2]+Rd[6])/(4.0f*ax[2]); ax[1] = (Rd[5]+Rd[7])/(4.0f*ax[2]);
        } else {
            ax[0] = 1.0f; ax[1] = 0.0f; ax[2] = 0.0f;
        }
    } else {
        th = 0.0f; ax[0] = 1.0f; ax[1] = 0.0f; ax[2] = 0.0f;
    }

    float dx = targetPose.position.x - p0.x;
    float dy = targetPose.position.y - p0.y;
    float dz = targetPose.position.z - p0.z;
    float posDist = sqrt(dx*dx + dy*dy + dz*dz);
    if (posDist < 0.5f && th < cartDeg2Rad(0.5f)) {
        moving = false;
        ikSolver->setNumericParams(100, 1.0f, 1.0f);
        return true;
    }

    // --- Грубая end-IK для оценки времени (как в linear: по joint-дельтам).
    // Берём ближайшую к текущей из {трекинг, геометрия}: чистый refine
    // иногда уходит в дальнюю ветвь и раздувает timeMs в разы (рука потом
    // ползёт еле-еле по ease-in-out, хотя путь рядом).
    JointAngles qEnd = currentAngles;
    ikSolver->refineSolution(targetPose, qEnd, nullptr);
    {
        JointAngles sols[4];
        IKSolutionMeta mm[4];
        int n = ikSolver->solveAll(targetPose, sols, mm);
        float bd = 1e30f;
        int bi = -1;
        for (int k = 0; k < n; k++) {
            float dd = cartJointDist(sols[k], currentAngles);
            if (dd < bd) { bd = dd; bi = k; }
        }
        if (bi >= 0 && cartJointDist(sols[bi], currentAngles) <
                        cartJointDist(qEnd, currentAngles)) {
            qEnd = sols[bi];
        }
    }
    float maxDelta = 0;
    for (int i = 0; i < 7; i++) {
        float dd = (i < 6) ? fabs(qEnd.angles[i] - currentAngles.angles[i])
                           : fabs(targetPose.gripper - currentAngles.angles[6]);
        if (dd > maxDelta) maxDelta = dd;
    }
    if (maxDelta < MIN_STEP_DEG) {
        moving = false;
        ikSolver->setNumericParams(100, 1.0f, 1.0f);
        return true;
    }
    float maxTime = 0;
    for (int i = 0; i < 7; i++) {
        float dd = (i < 6) ? fabs(qEnd.angles[i] - currentAngles.angles[i])
                           : fabs(targetPose.gripper - currentAngles.angles[6]);
        if (servoCtrl && dd > 0) {
            float maxSpeed = servoCtrl->getConfig(i).maxAngularSpeed;
            if (maxSpeed > 0) {
                float tm = dd / maxSpeed;
                if (tm > maxTime) maxTime = tm;
            }
        }
    }
    float pct = 100.0f;
    if (feedrate > 0.0f && feedrate < 100.0f) pct = feedrate;
    if (pct != 100.0f) maxTime = maxTime * (100.0f / pct);
    float timeMs = maxTime * 1000.0f;
    int steps = (int)(timeMs / STEP_INTERVAL_MS);
    if (steps < 1) steps = 1;

    float grip0 = currentAngles.angles[6];
    float grip1 = targetPose.gripper;
    JointAngles qPrev = currentAngles;
    float ePrev[3];
    { float er, ep, ey; cartRToEuler(R0, er, ep, ey); ePrev[0] = er; ePrev[1] = ep; ePrev[2] = ey; }

    // --- Цикл по декартовым шагам ---
    for (int step = 0; step <= steps; step++) {
        if (!moving) break;
        uint32_t stepT0 = millis();

        if (Serial.available()) {
            String line = "";
            while (Serial.available()) {
                char ch = Serial.read();
                if (ch == '\n' || ch == '\r') break;
                line += ch;
            }
            line.trim();
            line.toUpperCase();
            if (line.startsWith("M112")) {
                moving = false;
                setMark("CART", "stopped-M112");
                ikSolver->setNumericParams(100, 1.0f, 1.0f);
                return false;
            }
        }

        float t = (float)step / steps;
        float e = easeInOut(t);
        Vec3 pp = {p0.x + dx * e, p0.y + dy * e, p0.z + dz * e};

        // Rodrigues: R = R0 · Rot(ax, th·e)
        float a = th * e;
        float ca = cos(a), sa = sin(a);
        float x = ax[0], y = ax[1], z = ax[2];
        float Rm[9] = {ca+x*x*(1-ca),   x*y*(1-ca)-z*sa, x*z*(1-ca)+y*sa,
                       y*x*(1-ca)+z*sa, ca+y*y*(1-ca),   y*z*(1-ca)-x*sa,
                       z*x*(1-ca)-y*sa, z*y*(1-ca)+x*sa, ca+z*z*(1-ca)};
        float R[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float s = 0;
                for (int k = 0; k < 3; k++) s += R0[i*3+k] * Rm[k*3+j];
                R[i*3+j] = s;
            }
        }
        float eu[3];
        { float er, ep, ey; cartRToEuler(R, er, ep, ey); eu[0] = er; eu[1] = ep; eu[2] = ey; }
        cartUnwrapEuler(ePrev, eu);
        ePrev[0] = eu[0]; ePrev[1] = eu[1]; ePrev[2] = eu[2];

        Pose ps;
        ps.position = pp;
        ps.orientation.x = eu[0]; ps.orientation.y = eu[1]; ps.orientation.z = eu[2];
        ps.gripper = grip0 + (grip1 - grip0) * e;

        JointAngles qq = qPrev;
        ikSolver->refineSolution(ps, qq, nullptr);
        if (cartJointDist(qq, qPrev) > CART_JUMP_DEG) {
            // Срыв трекинга: вторая попытка геометрией, ближайший к qPrev.
            JointAngles sols[4];
            IKSolutionMeta mm[4];
            int n = ikSolver->solveAll(ps, sols, mm);
            int bi = -1;
            float bd = 1e30f;
            for (int k = 0; k < n; k++) {
                float dd = cartJointDist(sols[k], qPrev);
                if (dd < bd) { bd = dd; bi = k; }
            }
            if (bi >= 0) {
                qq = sols[bi];
                ikSolver->refineSolution(ps, qq, nullptr);
            }
            if (cartJointDist(qq, qPrev) > CART_JUMP_DEG) {
                char note[64];
                snprintf(note, sizeof(note), "ik-lost@%d", step);
                setMark("CART", note);
                moving = false;
                ikSolver->setNumericParams(100, 1.0f, 1.0f);
                return false;
            }
        }
        if (safety && safety->isEnabled() && !safety->checkAllLinks(qq.angles)) {
            char note[64];
            snprintf(note, sizeof(note), "safety@%d: %.40s", step,
                     safety->getLastError().c_str());
            setMark("CART", note);
            moving = false;
            ikSolver->setNumericParams(100, 1.0f, 1.0f);
            return false;
        }

        outputStep(qq);
        qPrev = qq;
        // Честный темп: вычитаем время счёта шага, иначе движение
        // идёт рывками медленнее плана (счёт ~100мс + фикс 20мс).
        {
            uint32_t spent = millis() - stepT0;
            if (spent < (uint32_t)STEP_INTERVAL_MS) {
                delay((unsigned long)(STEP_INTERVAL_MS - spent));
            }
        }
    }

    ikSolver->setNumericParams(100, 1.0f, 1.0f);
    if (moving) {
        currentAngles = qPrev;
        char note[64];
        snprintf(note, sizeof(note), "steps=%d timeMs=%.0f", steps, timeMs);
        setMark("CART", note);
        moving = false;
        return true;
    }
    setMark("CART", "stopped");
    moving = false;
    return false;
}
