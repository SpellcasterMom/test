/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * handlers_motion.h — обработчики движения (G0/G1/G28/G90/G91/M104/M108).
 *
 * Ветка ref-handlers-motion: построчный перенос из main.cpp, логики ноль.
 * Общее состояние — через app.h / display_ui.h / kin_diag.h.
 */

#pragma once

#include <Arduino.h>
#include "types.h"

/**
 * G0 — перемещение (быстрое).
 */
void handleG0(GCodeCommand &cmd);

/**
 * G1 — декартово движение TCP по прямой (трекинг / блендинг Q / пакет M450).
 */
void handleG1(GCodeCommand &cmd);

/**
 * G28 — перемещение в домашнюю позицию (home), H1 — парковка.
 */
void handleG28(GCodeCommand &cmd);

/**
 * G90 — абсолютная система координат.
 */
void handleG90(GCodeCommand &cmd);

/**
 * G91 — относительная система координат.
 */
void handleG91(GCodeCommand &cmd);

/**
 * M104 — плавный заезд серв (углы).
 */
void handleM104(GCodeCommand &cmd);

/**
 * M108 — захват в процентах (P7).
 */
void handleM108(GCodeCommand &cmd);
