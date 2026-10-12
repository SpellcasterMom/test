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

#pragma once

#include "types.h"
#include "servo_controller.h"
#include "backlash_compensator.h"
#include "safety_checker.h"
#include "ik_solver.h"

/**
 * Тип callback-функции, вызываемой на каждом шаге интерполяции.
 * Позволяет внешнему коду (дисплею) получать текущие промежуточные углы
 * в реальном времени, без блокировки мьютекса внутри интерполятора.
 */
typedef void (*StepCallback)(const JointAngles &angles);

/**
 * @brief Класс интерполяции движений для 6-осевого робота-манипулятора.
 *
 * Управляет плавным движением робота между заданными позициями.
 * Поддерживает три стратегии интерполяции:
 *
 * 1. Линейная интерполяция (INTERP_LINEAR):
 *    - Все 7 сервоприводов движутся одновременно
 *    - С ease-in-out сглаживанием (квадратичная кривая)
 *    - Проверка безопасности траектории перед началом движения
 *    - Быстрый способ, но может не пройти проверку безопасности
 *
 * 2. Последовательная интерполяция (INTERP_SEQUENTIAL):
 *    - Сервоприводы двигаются по очереди (по одному)
 *    - Каждый сервопривод движется с ease-in-out сглаживанием
 *    - Компенсация люфта (backlash compensation) для каждого сервопривода
 *    - Медленнее, но безопаснее (меньше риск столкновения)
 *
 * 3. Автоматическая стратегия (INTERP_AUTO):
 *    - Сначала пробует линейную интерполяцию (быстрый путь)
 *    - Если проверка безопасности не пройдена — переключается на последовательную
 *    - Оптимальный баланс скорости и безопасности
 *
 * Все стратегии поддерживают экстренную остановку (M112) во время движения.
 * После успешного движения обновляются текущие углы сочленений.
 */
class MotionInterpolator {
public:
    MotionInterpolator();

    /**
     * @brief Установка контроллера сервоприводов.
     * @param ctrl Указатель на объект ServoController
     */
    void setServoController(ServoController *ctrl);

    /**
     * @brief Установка компенсатора люфта.
     * @param comp Указатель на объект BacklashCompensator
     */
    void setBacklashCompensator(BacklashCompensator *comp);

    /**
     * @brief Установка проверки безопасности.
     * @param safety Указатель на объект SafetyChecker
     */
    void setSafetyChecker(SafetyChecker *safety);

    /**
     * @brief Установка решателя обратной кинематики (для moveToPose).
     * @param ik Указатель на объект IKSolver
     */
    void setIKSolver(IKSolver *ik);

    /**
     * @brief Установка решателя прямой кинематики (для moveToPose).
     * @param fk Указатель на объект FKSolver
     */
    void setFKSolver(FKSolver *fk);

    /**
     * @brief Установка стратегии интерполяции.
     * @param strat Стратегия: INTERP_LINEAR, INTERP_SEQUENTIAL или INTERP_AUTO
     */
    void setStrategy(InterpolationStrategy strat);

    /**
     * @brief Установка callback-функции для вызова на каждом шаге интерполяции.
     * Вызывается в executeStep() и interpolateSequential() после вычисления
     * промежуточных углов. Позволяет обновлять SharedState для дисплея
     * в реальном времени (каждые STEP_INTERVAL_MS ~20мс).
     * @param cb Функция callback или nullptr для отключения
     */
    void setStepCallback(StepCallback cb);

    /**
     * @brief Движение к целевой позиции с заданной подачей.
     * Выполняет интерполяцию от текущей позиции к целевой.
     * Поддерживает экстренную остановку (M112) во время выполнения.
     * @param target Целевые углы 7 сервоприводов
     * @param feedrate Процент от предельной скорости (0-100, >100 = 100%).
     *               Лимиты серв (maxAngularSpeed) не превышаются никогда.
     * @return true если движение завершено успешно
     */
    bool moveTo(const JointAngles &target, float feedrate);

    /**
     * @brief Немедленная остановка движения.
     * Устанавливает флаг moving=false, что прерывает цикл интерполяции.
     */
    void stop();

    /**
     * @brief Декартово движение к целевой позе TCP (для G1).
     * TCP идёт по прямой (lerp позиции + slerp ориентации), на каждом шаге
     * IK-доводка от предыдущего решения (трекинг), safety каждого шага.
     * Кончик стоит на прямой с точностью IK (а не гуляет по дуге joint-интерполяции).
     * При срыве трекинга/safety — остановка на последнем хорошем шаге, false.
     * @param targetPose Целевая поза TCP (позиция + эйлеры ZYX + захват)
     * @param feedrate Процент от предельной скорости (0-100, >100 = 100%).
     * @param blendOut true (G1 ... Q1): в конце НЕ тормозить — следующий G1
     *        продолжит цепочку. Скорость на стыке не рвётся в 0 (трапеция,
     *        см. chainEase), стартовый solveAll следующего отрезка пропускается
     *        (трекинг непрерывен). Без Q — стоп в конце как раньше.
     * @return true если движение завершено успешно
     */
    bool moveToPose(const Pose &targetPose, float feedrate, bool blendOut = false);

    /**
     * @brief Внутри ли цепочки блендинга (предыдущий G1 был с Q1 и доехал).
     * handleG1 по нему пропускает стартовый solveAll/диагностику сидов.
     */
    bool chainActive() const { return chainActive_; }

    /** Разорвать цепочку (M112/отказ/смена режима делают это сами). */
    void clearChain() { chainActive_ = false; }

    // --- Пакет точек (M450/G5-стриминг) ---
    static constexpr int BATCH_MAX = 32;  // точек в пакете (круг 24 влезает)

    /** Открыт ли пакет (между M450 S1 и S0). */
    bool batchOpen() const { return batchOpen_; }

    /** Сколько точек уже накоплено. */
    int batchCount() const { return batchN_; }

    /** Последняя накопленная точка (для continuity-выбора следующей).
     * @return false если пакет пуст (q не тронут). */
    bool batchPrevPt(JointAngles &q) const {
        if (batchN_ <= 0) return false;
        q = batchPts_[batchN_ - 1];
        return true;
    }

    /** M450 S1: открыть пакет (сбросить накопленное). */
    bool batchBegin();

    /** Положить точку в пакет (валидация — на вызывающем, здесь только слот).
     * @param q articular цель (7 углов), segMs длительность ноги (мс),
     *        dPos/dOri плановые расхождения для отчёта M115.
     * @return false если пакет переполнен (состояние не меняется). */
    bool batchAdd(const JointAngles &q, float segMs, float dPos, float dOri);

    /** M450 S0: закрыть и исполнить весь пакет как одну траекторию.
     * Рампы только по краям (первая/последняя нога), стыки без останова.
     * @return true если всё доехало; иначе стоп на последней хорошей точке. */
    bool batchExecute();

    /** M450 S2 / M112: сбросить пакет без движения (идемпотентно). */
    void batchAbort();

    /** Плановые расхождения точки i для отчёта M115 (0 если нет). */
    float batchResPos(int i) const;
    float batchResOri(int i) const;

    /**
     * @brief Проверка, выполняется ли движение в данный момент.
     * @return true если робот движется
     */
    bool isMoving() const;

    /**
     * @brief Синхронизация внутренних текущих углов (после прямых M104 и т.п.).
     * Без неё проверка траектории и расчёт времени идут от устаревшей точки,
     * что даёт ложный фолбэк на sequential и неверную длительность.
     */
    void setCurrentAngles(const JointAngles &a) { currentAngles = a; }

    /**
     * @brief Текущие углы по версии интерполятора (после moveTo/moveToPose).
     */
    JointAngles getCurrentAngles() const { return currentAngles; }

    /** Кто ехал последним: "LIN", "SEQ" или "NONE" (движений ещё не было). */
    const char *getLastMode() const { return lastMode; }

    /** Короткая причина/параметры последнего движения (шаги, время, отказ). */
    const char *getLastNote() const { return lastNote; }

    /** Записать метку последнего движения (для M115). Публичный: обработчики
     * команд тоже фиксируют «не ехали» (отказ до вызова интерполятора). */
    void setMark(const char *mode, const char *note);

private:
    ServoController      *servoCtrl;   // Контроллер сервоприводов
    BacklashCompensator  *backlash;    // Компенсатор люфта
    SafetyChecker        *safety;      // Проверка безопасности
    IKSolver             *ikSolver;    // Решатель IK (для moveToPose)
    FKSolver             *fkSolver;    // Решатель FK (для moveToPose)
    InterpolationStrategy strategy;    // Текущая стратегия интерполяции
    StepCallback         stepCallback; // Callback на каждом шаге (для дисплея)

    JointAngles currentAngles; // Текущие углы сочленений
    bool        moving;        // Флаг выполняющегося движения
    bool        chainActive_;  // Предыдущий G1(Q1) доехал: следующий продолжает без останова.
                               // Сбрасывают: moveTo(), M112/любой отказ, clearChain(), старт.
    // Пакет M450: копятся articular-цели + длительности ног + плановые
    // расхождения. Исполнение — одной траекторией без остановок.
    bool        batchOpen_;
    JointAngles batchPts_[BATCH_MAX];
    float       batchSegMs_[BATCH_MAX];
    float       batchRes_[BATCH_MAX][2];  // [i][0]=dPos мм, [i][1]=dOri град
    int         batchN_;
    char        lastMode[8];   // LIN / SEQ / CART / NONE — кто ехал последним
    char        lastNote[96];  // шаги/время/причина отказа (для M115)

    /**
     * @brief Линейная интерполяция (все сервоприводы одновременно).
     * Проверяет безопасность траектории, затем выполняет плавное движение
     * с ease-in-out сглаживанием. Все сервоприводы движутся синхронно.
     */
    bool interpolateLinear(const JointAngles &target, float feedrate);

    /**
     * @brief Последовательная интерполяция (сервоприводы по очереди).
     * Каждый сервопривод движется независимо с ease-in-out сглаживанием
     * и компенсацией люфта. Медленнее, но безопаснее.
     */
    bool interpolateSequential(const JointAngles &target, float feedrate);

    /**
     * @brief Автоматическая стратегия (пробует линейную, затем последовательную).
     * Сначала пытается линейную интерполяцию (быстрый путь).
     * При неудаче (нарушение безопасности) переключается на последовательную.
     */
    bool interpolateAuto(const JointAngles &target, float feedrate);

    /**
     * @brief Функция сглаживания ease-in-out (квадратичная).
     * Плавно ускоряет и замедляет движение.
     * @param t Время от 0.0 до 1.0
     * @return Сглаженное значение от 0.0 до 1.0
     */
    float easeInOut(float t);

    /**
     * @brief Выполнение одного шага интерполяции.
     * Вычисляет промежуточные углы для всех 7 сервоприводов,
     * применяет компенсацию люфта и отправляет команды.
     * @param from Начальные углы
     * @param to Целевые углы
     * @param t Параметр интерполяции (0..1)
     */
    void executeStep(const JointAngles &from, const JointAngles &to, float t);

    /**
     * @brief Вывод готовых углов (для moveToPose).
     * Люфт + setAngles + callback, интерполировать нечего.
     */
    void outputStep(const JointAngles &q);

    /**
     * @brief Оценка длительности ноги пакета (мс) по joint-дельтам
     * и потолкам maxAngularSpeed (включая гриппер). Та же формула, что
     * оценка времени в moveToPose. Только для темпа, не для точности.
     */
    float legMs(const JointAngles &from, const JointAngles &to, float feedrate);
};
