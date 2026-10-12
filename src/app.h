/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * app.h — общий хаб рефакторинга main.cpp (план: 5 модулей по веткам).
 *
 * Здесь ТОЛЬКО extern-объявления общего состояния (см. ниже). Определения
 * живут в main.cpp и никуда не едут. Логики в этом файле нет вообще.
 * Новые модули (display_ui, kin_diag, handlers_*) включают этот хаб
 * вместо того, чтобы тащить main.cpp целиком.
 *
 * Правила веток распила:
 * - перенос — построчный, без переименований и смен сигнатур;
 * - static у переехавших хелперов снимается только если их зовут снаружи;
 * - SharedState/displayMutex/diagLast едут ВМЕСТЕ со своими модулями
 *   (ветки 1-2), здесь их нет осознанно.
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "gcode_parser.h"
#include "fk_solver.h"
#include "ik_solver.h"
#include "servo_controller.h"
#include "motion_interpolator.h"
#include "backlash_compensator.h"
#include "safety_checker.h"
#include "gripper_controller.h"
#include "jog_pendant.h"
#include "detour_planner.h"
#include "eeprom_storage.h"
#include "imu_logger.h"

// ===== Глобальные объекты (определены в main.cpp) =====
extern GCodeParser          parser;
extern FKSolver             fkSolver;
extern IKSolver             ikSolver;
extern ServoController      servoCtrl;
extern MotionInterpolator   interpolator;
extern BacklashCompensator  backlash;
extern SafetyChecker        safety;
extern GripperController    gripper;
extern JogPendant           pendant;
extern DetourPlanner        detour;
extern ImuLogger            imuLogger;
extern SemaphoreHandle_t    i2cMutex;
extern EEPROMStorage        storage;

// ===== Глобальное состояние (определено в main.cpp) =====
extern ManipulatorConfig    config;
extern JointAngles          homeAngles;
extern JointAngles          currentAngles;
extern bool                 absoluteMode;
extern InterpolationStrategy interpStrategy;

// ===== FreeRTOS: задачи и мьютексы (определены в main.cpp) =====
extern TaskHandle_t core0TaskHandle;
extern TaskHandle_t core1TaskHandle;
extern SemaphoreHandle_t angleMutex;
