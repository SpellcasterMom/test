/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-10-05
 * Last modify : 2026-10-05
 */

/*
 * imu_logger.h — опрос BMI160 (гиро+аксель, один чип) и кольцевой
 * журнал последнего движения для хоста.
 * БЕЗ СТОРОННИХ ЛИБ: регистры читаем/пишем напрямую через Wire
 * (спарковская и pololu-либы под RTOS чудили — unbounded циклы,
 * повторный Wire.begin, залипающие статусы). Транспорт — 3 функции
 * с жёсткими таймаутами ниже; всё остальное (фьюжн, кольцо, M119)
 * от транспорта не зависит.
 *   - калибровка нуля гиро при старте (рука стоит в home);
 *   - комплементарный фьюжн наклона imuFuse() (замена на EKF — внутри,
 *     интерфейс не меняется);
 *   - задача Core 1, 200 Гц: burst-чтение, фьюжн, пики, запись в кольцо
 *     пока интерполятор в движении (опрос isMoving, сброс кольца если
 *     между движениями пауза > 1 с — вейпоинты одного G0 не рвут запись);
 *   - метки точек потока: опрос следит за сиквенсом исполнителя
 *     (streamConsumed) — каждый поп метится битом 1, вход в hold — битом 2;
 *   - живой режим M119 S=2: печать сэмплов сразу (только при движении),
 *     гасится повторным S=2 или M112;
 *   - M119: живые значения + пики; M119 S=1 [D=n]: дамп кольца текстом.
 *
 * В кольце — сырые counts (перевод в единицы на хосте по #SCALE-строке
 * дампа). Шина I2C общая с PCA9685 — все чтения под i2cMutex.
 */

#pragma once

#include <stdint.h>
#include "types.h"

class MotionInterpolator;  // только указатель, полный тип — в .cpp

// Масштабы (даташит BMI160):
//   acc ±16g => 16000/32768 = 0.4883 мг/LSB
//   gyro ±250 dps => 250000/32768 = 7.6294 mdps/LSB, ODR 200 Гц
#define IMU_ACC_MG_LSB   0.4883f
#define IMU_GYRO_MDPS_LSB 7.6294f

// BMI160 (один чип, I2C 0x68/0x69 по SDO): CHIP_ID, питание, диапазоны.
#define BMI160_CHIP_ID    0x00  // = 0xD1
#define BMI160_GYRO_DATA  0x0C  // burst 12 Б: гиро XYZ + аксель XYZ, LE
#define BMI160_ACC_CONF   0x40  // ODR/bwp: 0x29 = 200 Гц normal
#define BMI160_ACC_RANGE  0x41  // 0x0C = ±16g
#define BMI160_GYR_CONF   0x42  // ODR/bw: 0x29 = 200 Гц normal
#define BMI160_GYR_RANGE  0x43  // 0x03 = ±250 dps
#define BMI160_CMD        0x7E  // 0x11 acc normal, 0x15 gyro normal

#define IMU_HZ        200
#define IMU_PERIOD_MS (1000 / IMU_HZ)
#define IMU_LOG_N     4096          // сэмплов в кольце (~20 с на 200 Гц: длинные потоки влезают; ~72 КБ статики)
#define IMU_IDLE_GAP_MS 1000        // пауза «кончилось движение» для сброса кольца

struct ImuSample {
    uint32_t t;                       // millis() взятия
    int16_t  ax, ay, az;              // аксель, counts
    int16_t  gx, gy, gz;              // гиро, counts (bias НЕ вычтен — вычитает хост)
    uint16_t flags;                   // бит0 = старт записи, бит1 = вейпоинт,
                                      // бит2 = вход в hold (голодание стрима),
                                      // бит3 = вычисленная траектория (G1-декарт),
                                      //   нет — заданные суставы (G0/M104/G28)
};  // 18 байт (было 16 — метки того стоят)

class ImuLogger {
public:
    ImuLogger();

    // mask — биты scanBMI160(): 0x01 гиро, 0x02 аксель. Wire уже поднят.
    // Отсутствующие чипы молча пропускаются.
    void begin(uint8_t mask);
    bool ready() const { return hasGyro_ && hasAcc_; }

    // Блокирующая калибровка нуля гиро (рука стоит!): первые 20 сэмплов
    // мимо (переходный), дальше среднее n. Вызывать из setup в CALIB.
    void calibrateGyro(uint16_t n = 400);

    void setInterpolator(MotionInterpolator *m) { interp_ = m; }
    void startTask();  // Core 1, опрос 200 Гц

    // --- Данные для M119 (вызывать из обработчика, Core 0) ---
    bool hasGyro() const { return hasGyro_; }
    bool hasAcc() const { return hasAcc_; }
    void getBiasDps(float &bx, float &by, float &bz) const;
    void getTilt(float &rollDeg, float &pitchDeg) const;
    void getLast(int16_t out[6]) const;
    void getPeaks(float &accG, float &jerkGS) const;
    uint32_t sampleCount() const;
    bool isRecording() const;
    uint32_t markCount() const { return marks_; }
    void setHold(bool h);  // заморозить запись на время DUMP (рвёт tearing)
    void dump(uint8_t decim);  // печать кольца в Serial, старое → новое
    void setLive(bool on) { live_ = on; }  // живой режим M119 S=2 (тоггл/S2/M112)
    bool isLive() const { return live_; }
    uint32_t busFails() const { return busFails_; }
    uint32_t busRecovers() const { return recovers_; }

private:
    static void taskTrampoline(void *arg);
    void poll();                       // один тик 5 мс
    void push(const ImuSample &s);
    void reinitChips();                // переинит регистров после просадки
    void recoverBus();                 // Wire.end/begin + reinitChips

    // Комплементарный фьюжн наклона (deg). Замена на EKF — только здесь.
    void imuFuse(float gxDps, float gyDps, float axMg, float ayMg, float azMg);

    // --- Железо: только адреса, никаких объектов либ ---
    uint8_t imuAddr_ = 0;  // 0x68/0x69 (0 = нет чипа)
    bool hasGyro_ = false;
    bool hasAcc_ = false;  // один чип: всегда вместе с гиро

    float biasGx_ = 0, biasGy_ = 0, biasGz_ = 0;  // counts
    float tiltR_ = 0, tiltP_ = 0;                 // deg, фьюжн
    bool fuseInit_ = false;
    int16_t last_[6] = {0};
    float peakAccG_ = 0, peakJerkGS_ = 0;
    float prevAccMg_[3] = {0};
    bool prevAccValid_ = false;

    MotionInterpolator *interp_ = nullptr;
    uint32_t busFails_ = 0;   // подряд идущие битые транзакции
    uint32_t recovers_ = 0;   // сколько раз поднимали шину
    static ImuSample ring_[IMU_LOG_N];
    volatile uint32_t head_ = 0;    // куда пишем следующим
    volatile uint32_t count_ = 0;   // сколько валидных (<= N)
    volatile bool recording_ = false;
    volatile bool hold_ = false;
    volatile uint32_t marks_ = 0;  // меток в текущей записи
    bool wasMoving_ = false;
    bool segComputed_ = false;  // вид текущего сегмента: true = декартов план
    bool pendingMark_ = false;  // следующий сэмпл пометить хопом
    bool pendingIsStart_ = true;  // true = метка старта (бит0), false = вейпоинт (бит1)
    bool pendingHold_ = false;  // следующий сэмпл пометить входом в hold (бит2)
    bool live_ = false;  // живой режим M119 S=2: печатать сэмплы сразу (только при движении)
    uint32_t lastSeq_ = 0;  // последний виденный сиквенс исполнителя
    bool seqInit_ = false;  // трекинг сиквенсов заведён
    bool wasHolding_ = false;  // прошлое состояние hold (фронт для метки)
    uint32_t lastEndMs_ = 0;
};
