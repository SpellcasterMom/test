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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

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

    // --- Поток точек M450 (стриминг) ---
    // Планировщик (core0Task) считает IK заранее и кладёт joint-позы
    // в кольцо; исполнитель (своя задача, Core1) дренирует его строгим
    // кадансом 20мс. ШИМ не останавливается на время счёта никогда.
    // Backpressure: кольцо полное → OK хосту придерживается (ждём слота
    // со сниффом M112), а не ERROR. Пустой дрен → hold-and-resume.
    static constexpr int STREAM_DEPTH = 180;  // слотов (~3 чанка, ~7 КБ heap)
    static constexpr int STREAM_CHUNK = 60;   // авто-чанк для учёта/диагностики
    static constexpr int STREAM_KICK = 3;     // старт дрена при глубине >= 3
    static constexpr int CHUNK_ROWS = 8;      // строк таблицы чанков (кольцо)

    /** Состояние исполнителя (пишет исполнитель, читают все). */
    enum StreamState : uint8_t {
        ST_IDLE,     // потока нет
        ST_FILLING,  // копим, дрен не стартовал (глубина < KICK)
        ST_RUNNING,  // дренаж идёт
        ST_HOLDING,  // голодание: стоим, ждём докачки
    };

    /** Слот кольца: спланированная нога (позиции + длительность). */
    struct StreamWp {
        JointAngles q;    // articular-цель ноги
        float segMs;      // длительность ноги, мс
        float dPos;       // плановое расхождение, мм (для M115)
        float dOri;       // плановое расхождение, град (для M115)
        uint32_t seq;     // сквозной номер (учёт чанков/потребления)
    };

    /** Строка учёта авто-чанка (пишет только планировщик). */
    struct ChunkRow {
        uint32_t startSeq;  // сиквенс первой точки
        uint16_t n;         // точек в чанке
        float worstPos;     // худшее плановое dPos, мм
        float worstOri;     // худшее плановое dOri, град
    };

    /** Открыт ли приём (между M450 S1 и S0/S2). */
    bool batchOpen() const { return batchOpen_; }

    /** Глубина кольца сейчас (0 если очереди нет). */
    int streamDepth() const;

    /** Состояние исполнителя (для M115 и гардов). */
    StreamState streamState() const { return streamState_; }

    /** Занят ли исполнитель дреном (RUNNING/HOLDING). */
    bool streamBusy() const {
        return streamState_ == ST_RUNNING || streamState_ == ST_HOLDING;
    }

    /** Последняя спланированная точка (continuity следующей).
     * @return false если в текущем потоке ещё ничего нет. */
    bool batchPrevPt(JointAngles &q) const {
        if (!planTailValid_) return false;
        q = planTail_;
        return true;
    }

    /** M450 S1: открыть поток. Только из IDLE, иначе false. */
    bool batchBegin();

    /** Положить точку (валидация — на вызывающем).
     * Backpressure: кольцо полное → ждём слота (снифф M112, колпак 60с),
     * OK хосту уходит только по факту записи.
     * @return false при аборте/таймауте (поток уже сброшен). */
    bool batchAdd(const JointAngles &q, float feedrate,
                  float dPos, float dOri);

    /** M450 S0: барьер — докачать, исполнить всё, встать. Блокирует
     * вызывающего до конца дрена (со сниффом M112).
     * @return true если всё доехало. */
    bool batchExecute();

    /** M450 S2 / M112: сбросить поток без движения (идемпотентно). */
    void batchAbort();

    /** Сколько чанков учтено (валидных строк, <= CHUNK_ROWS). */
    int chunkCount() const { return chunkN_ < CHUNK_ROWS ? chunkN_ : CHUNK_ROWS; }

    /** Строка чанка i (0..chunkCount-1, от старых к новым).
     * @return false если нет такой. */
    bool chunkRow(int i, ChunkRow &row) const {
        int valid = chunkCount();
        if (i < 0 || i >= valid) return false;
        row = chunks_[(chunkN_ - valid + i) % CHUNK_ROWS];
        return true;
    }

    /** Сколько точек исполнитель потребил (монотонно в пределах потока). */
    uint32_t streamConsumed() const { return execSeq_; }

    /** Ватермарка сброса: сиквенсы >= неё сброшены (для статуса чанков). */
    uint32_t streamDropSeq() const { return dropSeq_; }

    /** Счётчик голоданий / суммарный hold, мс / макс. превышение шага, мс. */
    uint32_t streamStarved() const { return starved_; }
    uint32_t streamHoldsMs() const { return holdsMs_; }
    uint32_t streamMaxStep() const { return maxStepMs_; }

    /** High-water глубины кольца. */
    int streamDepthMax() const { return depthMax_; }

    /** Сколько точек принято в текущем потоке (монотонно до S1). */
    uint32_t streamPlanned() const { return planSeq_; }

    /** Создать задачу исполнителя (setup, один раз). Core1, приоритет 2. */
    void startExecTask();

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
    // Поток M450: очередь — единственный кросс-ядерный канал
    // (планировщик core0Task пишет, исполнитель Core1 читает).
    // Остальное — однописательное: планировщик (core0) пишет счётчики
    // плана и таблицу чанков; исполнитель — consumed/счётчики/состояние
    // (volatile, читает M115 и гарды).
    QueueHandle_t streamQ_;       // слоты StreamWp (SPSC по построению)
    TaskHandle_t  execTask_;      // задача исполнителя (nullptr до старта)
    volatile bool batchOpen_;     // приём точек открыт (S1..S0/S2)
    volatile bool streamClosing_; // S0: докачать, встать, закрыть
    volatile bool streamAbortReq_;  // stop()/ISR: сбросить при первой возможности
    volatile StreamState streamState_;
    volatile uint32_t planSeq_;   // следующий сиквенс планировщика
    volatile uint32_t execSeq_;   // следующий сиквенс исполнителя (=потреблено)
    volatile uint32_t starved_;   // счётчик голоданий
    volatile uint32_t holdsMs_;   // суммарный hold, мс
    volatile uint32_t maxStepMs_;  // макс. превышение шага над 20мс
    volatile int depthMax_;       // high-water глубины
    volatile uint32_t dropSeq_;   // сиквенсы >= этого сброшены (для статуса чанков)
    volatile bool execOk_;        // дрен завершился чисто (читает S0-барьер)
    JointAngles planTail_;        // последняя спланированная точка
    bool        planTailValid_;   // (только планировщик)
    ChunkRow    chunks_[CHUNK_ROWS];  // таблица авто-чанков (только планировщик)
    int         chunkN_;          // строк учтено (<= CHUNK_ROWS, кольцо по модулю)
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
