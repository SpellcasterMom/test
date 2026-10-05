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
 * imu_logger.h — опрос GY-80 (гиро L3G4200D + аксель ADXL345) и кольцевой
 * журнал последнего движения для хоста.
 *
 * Состав:
 *   - драйвер по настоящему API либ: L3G::init/enableDefault/read (+g),
 *     ADXL345::powerOn/setRangeSetting/set_bw/readAccel;
 *   - калибровка нуля гиро при старте (рука стоит в home);
 *   - комплементарный фьюжн наклона imuFuse() (замена на EKF — внутри,
 *     интерфейс не меняется);
 *   - задача Core 1, 200 Гц: burst-чтение, фьюжн, пики, запись в кольцо
 *     пока интерполятор в движении (опрос isMoving, сброс кольца если
 *     между движениями пауза > 1 с — вейпоинты одного G0 не рвут запись);
 *   - M119: живые значения + пики; M119 S=1 [D=n]: дамп кольца текстом.
 *
 * В кольце — сырые counts (перевод в единицы на хосте по #SCALE-строке
 * дампа). Шина I2C общая с PCA9685 — все чтения под i2cMutex.
 */

#pragma once

#include <stdint.h>
#include "types.h"

class MotionInterpolator;  // только указатель, полный тип — в .cpp

// Масштабы (даташиты; сверены с исходниками либ):
//   ADXL345: range 16g + full-res => 3.902 мг/LSB
//   L3G4200D: enableDefault => 250 dps => 8.75 mdps/LSB, ODR 200 Гц
#define IMU_ACC_MG_LSB   3.902f
#define IMU_GYRO_MDPS_LSB 8.75f

#define IMU_HZ        200
#define IMU_PERIOD_MS (1000 / IMU_HZ)
#define IMU_LOG_N     1024          // сэмплов в кольце (~5 с на 200 Гц)
#define IMU_IDLE_GAP_MS 1000        // пауза «кончилось движение» для сброса кольца

struct ImuSample {
    uint32_t t;                       // millis() взятия
    int16_t  ax, ay, az;              // аксель, counts
    int16_t  gx, gy, gz;              // гиро, counts (bias НЕ вычтен — вычитает хост)
};  // ровно 16 байт

class ImuLogger {
public:
    ImuLogger();

    // mask — биты scanGY80(): 0x01 гиро, 0x02 аксель. Wire уже поднят.
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
    void setHold(bool h);  // заморозить запись на время DUMP (рвёт tearing)
    void dump(uint8_t decim);  // печать кольца в Serial, старое → новое
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

    // --- Железо (либы живут здесь, наружу не торчат) ---
    void *gyro_ = nullptr;  // L3G* (void чтобы не тащить L3G.h в заголовок)
    void *acc_ = nullptr;   // ADXL345*
    bool hasGyro_ = false;
    bool hasAcc_ = false;

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
    bool wasMoving_ = false;
    uint32_t lastEndMs_ = 0;
};
