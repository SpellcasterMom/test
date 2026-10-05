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
 * imu_logger.cpp — см. imu_logger.h.
 * API либ — один в один с их исходниками:
 *   L3G (pololu/l3g-arduino): init(device_4200D, sa0_auto) сам находит
 *     0x68/0x69 по WHO_AM_I==0xD3; enableDefault() => 250 dps
 *     (8.75 mdps/LSB), ODR 200 Гц; read() в g.x/y/z; setTimeout() bounds read.
 *   ADXL345 (sparkfun): адрес зашит 0x53; powerOn(); setRangeSetting(16);
 *     setFullResBit(true) => 3.902 мг/LSB (без него 10-бит, 31 мг/LSB!);
 *     set_bw(BW_200); readAccel(&x,&y,&z).
 */

#include "imu_logger.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <L3G.h>
#include <SparkFun_ADXL345.h>
#include "motion_interpolator.h"

extern SemaphoreHandle_t i2cMutex;  // создаётся в main.cpp setup()

ImuSample ImuLogger::ring_[IMU_LOG_N];

ImuLogger::ImuLogger() {}

static bool imuTake(uint32_t ms) {
    if (!i2cMutex) return true;
    return xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(ms)) == pdTRUE;
}

static void imuGive() {
    if (i2cMutex) xSemaphoreGive(i2cMutex);
}

void ImuLogger::begin(uint8_t mask) {
    // Wire уже поднят (servoCtrl.init). Вызывать из setup, задач ещё нет —
    // мьютекс не нужен, но take/give с NULL-хендлом безопасны и здесь.
    if (mask & 0x01) {
        L3G *g = new L3G();
        g->setTimeout(10);  // read() дольше 10 мс не висит никогда
        if (g->init(L3G::device_4200D, L3G::sa0_auto)) {
            g->enableDefault();
            gyro_ = g;
            hasGyro_ = true;
            Serial.println("IMU: gyro L3G4200D OK");
        } else {
            delete g;
            Serial.println("IMU: gyro not found (mask said present)");
        }
    }
    if (mask & 0x02) {
        ADXL345 *a = new ADXL345();  // I2C, адрес 0x53 зашит в либе
        a->powerOn();                // Wire.begin() внутри — пины дефолтные 21/22, наши же
        a->setRangeSetting(16);
        a->setFullResBit(true);      // иначе 10-бит и 31 мг/LSB!
        a->set_bw(ADXL345_BW_200);    // ODR 200 Гц под наш опрос
        acc_ = a;
        hasAcc_ = true;
        Serial.println("IMU: acc ADXL345 OK (16g, full-res, 200Hz)");
    }
}

void ImuLogger::calibrateGyro(uint16_t n) {
    if (!hasGyro_) return;
    // Рука стоит (CALIB поза home + отстойка — делает вызывающий код).
    // Первые 20 сэмплов мимо: переходный после powerOn/enableDefault.
    L3G *g = (L3G *)gyro_;
    long sx = 0, sy = 0, sz = 0;
    for (uint16_t i = 0; i < (uint16_t)(n + 20); i++) {
        if (!imuTake(100)) continue;
        g->read();
        bool to = g->timeoutOccurred();
        imuGive();
        if (to) continue;
        if (i < 20) { delay(IMU_PERIOD_MS); continue; }
        sx += g->g.x; sy += g->g.y; sz += g->g.z;
        delay(IMU_PERIOD_MS);
    }
    biasGx_ = (float)sx / n;
    biasGy_ = (float)sy / n;
    biasGz_ = (float)sz / n;
    Serial.printf("IMU: gyro bias counts x=%.1f y=%.1f z=%.1f\n",
                  biasGx_, biasGy_, biasGz_);
}

void ImuLogger::imuFuse(float gxDps, float gyDps,
                        float axMg, float ayMg, float azMg) {
    // Наклон по акселю (deg): roll вокруг X, pitch вокруг Y.
    float rollAcc = atan2f(ayMg, azMg) * 57.2958f;
    float pitchAcc = atan2f(-axMg, sqrtf(ayMg * ayMg + azMg * azMg)) * 57.2958f;
    const float dt = IMU_PERIOD_MS / 1000.0f;
    const float A = 0.99f;  // tau ~0.5 с: гиро ведёт, аксель правит дрейф
    if (!fuseInit_) {
        tiltR_ = rollAcc;
        tiltP_ = pitchAcc;
        fuseInit_ = true;
        return;
    }
    tiltR_ = A * (tiltR_ + gxDps * dt) + (1 - A) * rollAcc;
    tiltP_ = A * (tiltP_ + gyDps * dt) + (1 - A) * pitchAcc;
}

void ImuLogger::push(const ImuSample &s) {
    ring_[head_ % IMU_LOG_N] = s;
    head_++;
    if (count_ < IMU_LOG_N) count_++;
}

void ImuLogger::poll() {
    if (!hasGyro_ && !hasAcc_) return;
    // Запись идёт пока движется интерполятор. Сброс кольца — только если
    // между движениями пауза > 1 с: вейпоинты одного G0 идут вплотную
    // и запись не рвут.
    bool mv = interp_ && interp_->isMoving();
    uint32_t now = millis();
    if (mv && (!wasMoving_ && (now - lastEndMs_ > IMU_IDLE_GAP_MS))) {
        head_ = 0;
        count_ = 0;
        peakAccG_ = 0;
        peakJerkGS_ = 0;
        prevAccValid_ = false;
    }
    recording_ = mv;
    if (!mv && wasMoving_) lastEndMs_ = now;
    wasMoving_ = mv;
    if (!mv || hold_) return;

    if (!imuTake(50)) return;  // шина занята сервами — сэмпл пропускаем
    int ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
    if (hasAcc_) {
        int x, y, z;
        ((ADXL345 *)acc_)->readAccel(&x, &y, &z);
        ax = x; ay = y; az = z;
    }
    if (hasGyro_) {
        L3G *g = (L3G *)gyro_;
        g->read();
        if (!g->timeoutOccurred()) {
            gx = g->g.x; gy = g->g.y; gz = g->g.z;
        }
    }
    imuGive();

    ImuSample s;
    s.t = now;
    s.ax = (int16_t)ax; s.ay = (int16_t)ay; s.az = (int16_t)az;
    s.gx = gx; s.gy = gy; s.gz = gz;
    last_[0] = s.ax; last_[1] = s.ay; last_[2] = s.az;
    last_[3] = s.gx; last_[4] = s.gy; last_[5] = s.gz;

    // Фьюжн + пики (float, 200 Гц — ESP32 не замечает).
    float axMg = ax * IMU_ACC_MG_LSB;
    float ayMg = ay * IMU_ACC_MG_LSB;
    float azMg = az * IMU_ACC_MG_LSB;
    float gxDps = (gx - biasGx_) * IMU_GYRO_MDPS_LSB / 1000.0f;
    float gyDps = (gy - biasGy_) * IMU_GYRO_MDPS_LSB / 1000.0f;
    imuFuse(gxDps, gyDps, axMg, ayMg, azMg);
    float accG = sqrtf(axMg * axMg + ayMg * ayMg + azMg * azMg) / 1000.0f;
    if (accG > peakAccG_) peakAccG_ = accG;
    if (prevAccValid_) {
        float dx = axMg - prevAccMg_[0];
        float dy = ayMg - prevAccMg_[1];
        float dz = azMg - prevAccMg_[2];
        float jerk = sqrtf(dx * dx + dy * dy + dz * dz) / 1000.0f
                     / (IMU_PERIOD_MS / 1000.0f);
        if (jerk > peakJerkGS_) peakJerkGS_ = jerk;
    }
    prevAccMg_[0] = axMg; prevAccMg_[1] = ayMg; prevAccMg_[2] = azMg;
    prevAccValid_ = true;

    push(s);
}

void ImuLogger::taskTrampoline(void *arg) {
    ImuLogger *self = (ImuLogger *)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        self->poll();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

void ImuLogger::startTask() {
    if (!hasGyro_ && !hasAcc_) return;
    xTaskCreatePinnedToCore(taskTrampoline, "imu_log", 4096,
                            this, 2, NULL, 1);  // Core 1, рядом с дисплеем
}

void ImuLogger::getBiasDps(float &bx, float &by, float &bz) const {
    bx = biasGx_ * IMU_GYRO_MDPS_LSB / 1000.0f;
    by = biasGy_ * IMU_GYRO_MDPS_LSB / 1000.0f;
    bz = biasGz_ * IMU_GYRO_MDPS_LSB / 1000.0f;
}

void ImuLogger::getTilt(float &rollDeg, float &pitchDeg) const {
    rollDeg = tiltR_;
    pitchDeg = tiltP_;
}

void ImuLogger::getLast(int16_t out[6]) const {
    for (int i = 0; i < 6; i++) out[i] = last_[i];
}

void ImuLogger::getPeaks(float &accG, float &jerkGS) const {
    accG = peakAccG_;
    jerkGS = peakJerkGS_;
}

uint32_t ImuLogger::sampleCount() const { return count_; }
bool ImuLogger::isRecording() const { return recording_; }
void ImuLogger::setHold(bool h) { hold_ = h; }

void ImuLogger::dump(uint8_t decim) {
    if (decim < 1) decim = 1;
    Serial.printf("#SCALE acc_mg_lsb=%.3f gyro_mdps_lsb=%.2f\n",
                  IMU_ACC_MG_LSB, IMU_GYRO_MDPS_LSB);
    float bx, by, bz;
    getBiasDps(bx, by, bz);
    Serial.printf("#BIAS dps gx=%.2f gy=%.2f gz=%.2f\n", bx, by, bz);
    uint32_t n = count_;
    Serial.printf("#N=%lu\n", (unsigned long)n);
    uint32_t start = (head_ >= n) ? (head_ - n) : 0;
    for (uint32_t i = 0; i < n; i += decim) {
        const ImuSample &s = ring_[(start + i) % IMU_LOG_N];
        Serial.printf("#IMU t=%lu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n",
                      (unsigned long)s.t,
                      (int)s.ax, (int)s.ay, (int)s.az,
                      (int)s.gx, (int)s.gy, (int)s.gz);
    }
    Serial.println("OK");
}
