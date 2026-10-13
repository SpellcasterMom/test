/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-10-13
 * Last modify : 2026-10-13
 */

/**
 * @file i2c_bus.h
 * @brief Единый подъём шины I2C (пины/частота/таймаут — из config.h).
 *
 * Было продублировано в ServoController::init() и ImuLogger::recoverBus().
 * Пользователи:
 * - servoCtrl.init() — стартовый подъём;
 * - imuLogger.recoverBus() — ремонт (перед вызовом Wire.end() + delay(5),
 *   сам вызов — только держа i2cMutex: все остальные в это время молчат).
 */

#pragma once

#include <Wire.h>
#include "config.h"

inline void bringUpI2CBus() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeout(I2C_TIMEOUT_MS);
}
