/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * handlers_system.h — системные обработчики и отчёты.
 *
 * Ветка ref-handlers-system: построчный перенос из main.cpp, логики ноль.
 * M119/M105/M114 (отчёты), M112/M111, M400/M410–M417/M420/M425–M426/M430,
 * M450 (поток), M500/M501/M502, handleHelp, printServoAngles/printPosition/
 * printStatus. Общее состояние — через app.h / display_ui.h / kin_diag.h.
 */

#pragma once

#include <Arduino.h>
#include "types.h"

/**
 * M119 — телеметрия IMU (GY-80): живые значения, пики, дамп кольца.
 */
void handleM119(GCodeCommand &cmd);

/**
 * M105 — вывод текущих углов сервоприводов.
 */
void handleM105(GCodeCommand &cmd);

/**
 * M114 — вывод текущей позиции энд-эффектора (FK).
 */
void handleM114(GCodeCommand &cmd);

/**
 * M112 — аварийная остановка.
 */
void handleM112(GCodeCommand &cmd);

/**
 * M111 — сохранение текущих углов как домашней позиции.
 */
void handleM111(GCodeCommand &cmd);

/**
 * M400 — люфт сервы (°).
 */
void handleM400(GCodeCommand &cmd);

/**
 * M410 — добавить запретную зону.
 */
void handleM410(GCodeCommand &cmd);

/**
 * M411 — установка минимальной высоты стола.
 */
void handleM411(GCodeCommand &cmd);

/**
 * M412 — установка максимального радиуса дотягивания.
 */
void handleM412(GCodeCommand &cmd);

/**
 * M413 — установка минимального радиуса дотягивания.
 */
void handleM413(GCodeCommand &cmd);

/**
 * M414 — установка длины пальцев захвата.
 */
void handleM414(GCodeCommand &cmd);

/**
 * M415 — отключение проверки безопасности.
 */
void handleM415(GCodeCommand &cmd);

/**
 * M416 — включение проверки безопасности.
 */
void handleM416(GCodeCommand &cmd);

/**
 * M417 — вывод списка запретных зон.
 */
void handleM417(GCodeCommand &cmd);

/**
 * M420 — установка минимальной высоты Z для звена.
 */
void handleM420(GCodeCommand &cmd);

/**
 * M425 — отключение проверки безопасности (алиас).
 */
void handleM425(GCodeCommand &cmd);

/**
 * M426 — включение проверки безопасности (алиас).
 */
void handleM426(GCodeCommand &cmd);

/**
 * M430 — выбор стратегии интерполяции движения.
 */
void handleM430(GCodeCommand &cmd);

/**
 * M450 — пакет/поток точек (открыть/исполнить/сбросить).
 */
void handleM450(GCodeCommand &cmd);

/**
 * M500 — сохранение конфигурации в EEPROM.
 */
void handleM500(GCodeCommand &cmd);

/**
 * M501 — загрузка конфигурации из EEPROM.
 */
void handleM501(GCodeCommand &cmd);

/**
 * M502 — загрузка конфигурации по умолчанию (заводские настройки).
 */
void handleM502(GCodeCommand &cmd);

/**
 * handleHelp (?) — вывод справки по всем доступным командам.
 */
void handleHelp(GCodeCommand &cmd);

/**
 * printServoAngles() — вывод текущих углов всех 7 сервоприводов.
 */
void printServoAngles();

/**
 * printPosition() — вывод текущей позиции энд-эффектора (FK).
 */
void printPosition();

/**
 * printStatus() — вывод полного статуса манипулятора.
 */
void printStatus();
