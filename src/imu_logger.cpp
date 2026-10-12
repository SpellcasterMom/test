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
 * Транспорт — голые регистры через Wire, без сторонних либ:
 *   L3G4200D: WHO_AM_I 0x0F (=0xD3); CTRL_REG4 0x23 = 0x00 (250 dps);
 *     CTRL_REG1 0x20 = 0x6F (ODR 200 Гц, все оси); чтение с OUT_X_L 0x28
 *     + автоинкремент (бит 7); порядок LE.
 *   ADXL345: DEVID 0x00 (=0xE5); POWER_CTL 0x2D: 0/16/8;
 *     DATA_FORMAT 0x31 = 0x0B (full-res + 16g); BW_RATE 0x2C = 0x0C;
 *     чтение с DATAX0 0x32 + multi-byte бит (0x40); порядок LE.
 */

#include "imu_logger.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
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

// Один регистр напрямую, без либ (у либ внутри местами повторный
// Wire.begin(), который вешает шину — см. begin()).
static void imuWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Чтение одного регистра (-1 при transact-ошибке).
static int imuReadReg(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(addr, (uint8_t)1) != 1) return -1;
    unsigned long t0 = millis();
    while (!Wire.available()) {
        if (millis() - t0 > 5) return -1;
    }
    return Wire.read();
}

// Пакетное чтение 3 осей LE (autoMask: 0x80 гиро / 0x40 аксель-MB).
// false = шина/чип не ответили в срок (никаких unbounded циклов).
static bool imuRead6(uint8_t addr, uint8_t reg, uint8_t autoMask,
                     int16_t out[3]) {
    Wire.beginTransmission(addr);
    Wire.write(reg | autoMask);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, (uint8_t)6) != 6) {
        while (Wire.available()) Wire.read();  // подчистить хвост
        return false;
    }
    uint8_t b[6];
    unsigned long t0 = millis();
    for (int i = 0; i < 6; i++) {
        while (!Wire.available()) {
            if (millis() - t0 > 5) return false;
        }
        b[i] = Wire.read();
    }
    for (int i = 0; i < 3; i++) {
        out[i] = (int16_t)((b[2 * i + 1] << 8) | b[2 * i]);
    }
    return true;
}

void ImuLogger::begin(uint8_t mask) {
    // Wire уже поднят (servoCtrl.init). Вызывать из setup, задач ещё нет.
    if (mask & 0x01) {
        // Гиро: ищем на 0x69/0x68 по WHO_AM_I, инитим парой записей.
        uint8_t addr = 0;
        if (imuReadReg(0x69, 0x0F) == 0xD3) addr = 0x69;
        else if (imuReadReg(0x68, 0x0F) == 0xD3) addr = 0x68;
        if (addr) {
            imuWriteReg(addr, 0x23, 0x00);  // CTRL_REG4: 250 dps
            imuWriteReg(addr, 0x20, 0x6F);  // CTRL_REG1: ODR 200 Гц, все оси
            gyroAddr_ = addr;
            hasGyro_ = true;
            Serial.println("IMU: gyro L3G4200D OK");
        } else {
            Serial.println("IMU: gyro not found (mask said present)");
        }
    }
    if (mask & 0x02) {
        // Аксель: DEVID обязан быть 0xE5, дальше те же 5 записей.
        if (imuReadReg(0x53, 0x00) == 0xE5) {
            imuWriteReg(0x53, 0x2D, 0);   // wakeup
            imuWriteReg(0x53, 0x2D, 16);  // auto-sleep
            imuWriteReg(0x53, 0x2D, 8);   // measure
            // DATA_FORMAT: FULL_RES(0x08) | range 16g(0x03) => 3.902 мг/LSB
            imuWriteReg(0x53, 0x31, 0x0B);
            imuWriteReg(0x53, 0x2C, 0x0C);  // BW_RATE: ODR 200 Гц
            hasAcc_ = true;
            Serial.println("IMU: acc ADXL345 OK (16g, full-res, 200Hz)");
        } else {
            Serial.println("IMU: acc not found (mask said present)");
        }
    }
}

void ImuLogger::calibrateGyro(uint16_t n) {
    if (!hasGyro_) return;
    // Рука стоит (CALIB поза home + отстойка — делает вызывающий код).
    // Первые 20 сэмплов мимо: переходный после включения.
    long sx = 0, sy = 0, sz = 0;
    int16_t v[3];
    for (uint16_t i = 0; i < (uint16_t)(n + 20); i++) {
        bool ok = imuRead6(gyroAddr_, 0x28, 0x80, v);
        if (!ok) { delay(IMU_PERIOD_MS); continue; }
        if (i < 20) { delay(IMU_PERIOD_MS); continue; }
        sx += v[0]; sy += v[1]; sz += v[2];
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

// Переинит регистров после просадки/рыва шины (чип мог сброситься в дефолт).
// Только прямые записи, никакого Wire.begin() — шиной владеет вызывающий
// (держать i2cMutex!).
void ImuLogger::reinitChips() {
    if (hasGyro_) {
        imuWriteReg(gyroAddr_, 0x23, 0x00);  // 250 dps
        imuWriteReg(gyroAddr_, 0x20, 0x6F);  // ODR 200 Гц, все оси
    }
    if (hasAcc_) {
        imuWriteReg(0x53, 0x2D, 0);
        imuWriteReg(0x53, 0x2D, 16);
        imuWriteReg(0x53, 0x2D, 8);
        imuWriteReg(0x53, 0x31, 0x0B);
        imuWriteReg(0x53, 0x2C, 0x0C);
    }
}

// Жёсткое восстановление шины: периферия I2C реинитится, чипы — следом.
// Вызывать только держа i2cMutex (все остальные молчат в это время).
void ImuLogger::recoverBus() {
    Wire.end();
    delay(5);
    Wire.begin(21, 22);
    Wire.setClock(400000);  // см. servo init: провода 5 см, голый транспорт
    Wire.setTimeout(20);
    reinitChips();
    busFails_ = 0;
    recovers_++;
    // Печатаем первые разы и дальше каждую 40-ю (иначе спам при дохлой шине).
    if (recovers_ <= 3 || (recovers_ % 40) == 0) {
        Serial.printf("IMU: bus recovered #%lu\n", (unsigned long)recovers_);
    }
}

void ImuLogger::poll() {
    if (!hasGyro_ && !hasAcc_) return;
    // Запись идёт пока движется интерполятор. Сброс кольца — только если
    // между движениями пауза > 1 с: вейпоинты одного G0 идут вплотную
    // и запись не рвут.
    bool mv = interp_ && interp_->isMoving();
    uint32_t now = millis();
    if (mv && !wasMoving_) {
        if (now - lastEndMs_ > IMU_IDLE_GAP_MS) {
            // Новая команда: сброс кольца, первый сэмпл с меткой старта.
            head_ = 0;
            count_ = 0;
            marks_ = 0;
            peakAccG_ = 0;
            peakJerkGS_ = 0;
            prevAccValid_ = false;
            pendingMark_ = true;
            pendingIsStart_ = true;
        } else {
            // Хоп вейпоинта внутри одного G0 (пауза миллисекунды):
            // кольцо не рвём, следующий сэмпл метим.
            pendingMark_ = true;
            pendingIsStart_ = false;
        }
    }
    recording_ = mv;
    if (!mv && wasMoving_) lastEndMs_ = now;
    wasMoving_ = mv;
    // Метки потока M450: следим за сиквенсом исполнителя (тот же Core 1 —
    // гонки нет, в худшем случае метка съедет на один сэмпл 5мс).
    // Каждый поп — бит1; вход в hold — бит2 (отличаем простой от езды).
    // Вне потока сиквенс стоит — меток нет. Откат сиквенса (новый поток) —
    // просто перезахват, стартовая метка придёт переходом выше.
    if (interp_) {
        uint32_t sq = interp_->streamConsumed();
        if (!seqInit_ || sq < lastSeq_) {
            lastSeq_ = sq;
            seqInit_ = true;
        } else if (sq != lastSeq_) {
            lastSeq_ = sq;
            if (mv) { pendingMark_ = true; pendingIsStart_ = false; }
        }
        bool holding = (interp_->streamState() == MotionInterpolator::ST_HOLDING);
        if (holding && !wasHolding_ && mv) pendingHold_ = true;
        wasHolding_ = holding;
    }
    if (!mv || hold_) return;

    if (!imuTake(50)) return;  // шина занята сервами — сэмпл пропускаем
    int16_t axyz[3] = {0, 0, 0};
    int16_t gxyz[3] = {0, 0, 0};
    bool burstOk = true;
    if (hasAcc_) {
        if (!imuRead6(0x53, 0x32, 0x40, axyz)) burstOk = false;
    }
    if (hasGyro_) {
        if (!imuRead6(gyroAddr_, 0x28, 0x80, gxyz)) burstOk = false;
    }
    if ((hasGyro_ || hasAcc_) && !burstOk) {
        // Дребезг Dupont / просадка: чиним сами, молча до 100 подряд.
        if (++busFails_ > 100) recoverBus();  // ~0.5 с дохлой шины
    } else {
        busFails_ = 0;
    }
    imuGive();

    ImuSample s;
    s.t = now;
    s.ax = axyz[0]; s.ay = axyz[1]; s.az = axyz[2];
    s.gx = gxyz[0]; s.gy = gxyz[1]; s.gz = gxyz[2];
    s.flags = 0;
    if (pendingMark_) {
        s.flags = pendingIsStart_ ? 0x01 : 0x02;
        marks_++;
        pendingMark_ = false;
    }
    if (pendingHold_) {
        s.flags |= 0x04;
        marks_++;
        pendingHold_ = false;
    }
    last_[0] = s.ax; last_[1] = s.ay; last_[2] = s.az;
    last_[3] = s.gx; last_[4] = s.gy; last_[5] = s.gz;

    // Фьюжн + пики (float, 200 Гц — ESP32 не замечает).
    float axMg = axyz[0] * IMU_ACC_MG_LSB;
    float ayMg = axyz[1] * IMU_ACC_MG_LSB;
    float azMg = axyz[2] * IMU_ACC_MG_LSB;
    float gxDps = (gxyz[0] - biasGx_) * IMU_GYRO_MDPS_LSB / 1000.0f;
    float gyDps = (gxyz[1] - biasGy_) * IMU_GYRO_MDPS_LSB / 1000.0f;
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
    // Живой режим M119 S=2: та же строка, что в дампе, — сразу в Serial.
    // Сюда доходят лишь при mv && !hold_ (гейт выше), т.е. пишем строго
    // при движении. Поток Serial делим с core0Task, printf атомарен.
    if (live_) {
        Serial.printf("#IMU t=%lu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d f=%u\n",
                      (unsigned long)s.t,
                      (int)s.ax, (int)s.ay, (int)s.az,
                      (int)s.gx, (int)s.gy, (int)s.gz,
                      (unsigned)s.flags);
    }
}

void ImuLogger::taskTrampoline(void *arg) {
    ImuLogger *self = (ImuLogger *)arg;
    for (;;) {
        self->poll();
        // Дохлая шина: опрашивать на 200 Гц бессмысленно (одни таймауты) —
        // откатываемся на 10 Гц, чтобы не жечь время и не спамить.
        // Метки времени всё равно ставятся на каждый сэмпл, джиттер не важен.
        if (self->busFails_ > 100) {
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
        }
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
        Serial.printf("#IMU t=%lu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d f=%u\n",
                      (unsigned long)s.t,
                      (int)s.ax, (int)s.ay, (int)s.az,
                      (int)s.gx, (int)s.gy, (int)s.gz,
                      (unsigned)s.flags);
    }
    Serial.println("OK");
}
