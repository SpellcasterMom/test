/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * handlers_system.cpp — системные обработчики (перенос из main.cpp 1:1).
 */

#include "handlers_system.h"
#include "app.h"
#include "display_ui.h"
#include "kin_diag.h"
#include "config.h"
#include <cmath>
#include <cstring>

/**
 * M119 — телеметрия IMU (GY-80): живые значения, пики, дамп кольца.
 *   M119          — живые: наклон, последние counts, bias, пики, счётчик
 *   M119 S=1      — дамп кольца текстом (#IMU строки + OK в конце)
 *   M119 S=1 D=4  — дамп каждого 4-го сэмпла (быстрее)
 */
void handleM119(GCodeCommand &cmd) {
    if (!imuLogger.hasGyro() && !imuLogger.hasAcc()) {
        Serial.println("IMU: none (GY-80 not found)");
        Serial.println("OK");
        return;
    }
    bool dump = cmd.hasParam['S' - 'A'] && cmd.params['S' - 'A'] != 0.0f;
    if (!dump) {
        float tr, tp;
        imuLogger.getTilt(tr, tp);
        int16_t last[6];
        imuLogger.getLast(last);
        float bx, by, bz;
        imuLogger.getBiasDps(bx, by, bz);
        float pkA, pkJ;
        imuLogger.getPeaks(pkA, pkJ);
        Serial.printf("IMU: tilt R=%.1f P=%.1f\n", tr, tp);
        Serial.printf("IMU: last ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n",
                      last[0], last[1], last[2], last[3], last[4], last[5]);
        Serial.printf("IMU: bias dps gx=%.2f gy=%.2f gz=%.2f\n", bx, by, bz);
        Serial.printf("IMU: peak acc=%.2fg jerk=%.1fg/s log n=%lu marks=%lu rec=%d\n",
                      pkA, pkJ, (unsigned long)imuLogger.sampleCount(),
                      (unsigned long)imuLogger.markCount(),
                      imuLogger.isRecording() ? 1 : 0);
        Serial.printf("IMU: bus fails=%lu recovers=%lu\n",
                      (unsigned long)imuLogger.busFails(),
                      (unsigned long)imuLogger.busRecovers());
        Serial.println("OK");
        return;
    }
    uint8_t decim = 1;
    if (cmd.hasParam['D' - 'A'] && cmd.params['D' - 'A'] >= 1.0f)
        decim = (uint8_t)cmd.params['D' - 'A'];
    imuLogger.setHold(true);   // заморозить кольцо на время печати
    imuLogger.dump(decim);
    imuLogger.setHold(false);
}

void handleM105(GCodeCommand &cmd) {
    printServoAngles();
}
// handleM108 — уехал в handlers_motion.cpp (ветка ref-handlers-motion).

/**
 * M114 — вывод текущей позиции энд-эффектора (FK).
 */
void handleM114(GCodeCommand &cmd) {
    float q[6];
    for (int i = 0; i < 6; i++) q[i] = currentAngles.angles[i];

    // Позиции всех 7 сочленений
    Vec3 joints[7];
    fkSolver.getAllJointPositions(q, joints);
    for (int i = 0; i < 7; i++) {
        Serial.printf("J%d: X=%.1f Y=%.1f Z=%.1f\n", i,
                      joints[i].x, joints[i].y, joints[i].z);
    }

    // Направление пальца
    Vec3 dir = fkSolver.getFingerDirection(q);
    Serial.printf("FingerDir: X=%.3f Y=%.3f Z=%.3f\n", dir.x, dir.y, dir.z);

    // TCP
    Vec3 tcp = fkSolver.getEndEffector(q);
    Serial.printf("TCP: X=%.1f Y=%.1f Z=%.1f\n", tcp.x, tcp.y, tcp.z);
}

// handleM115 — уехал в kin_diag.cpp (ветка ref-kin-diag).

/**
 * M112 — аварийная остановка.
 */
void handleM112(GCodeCommand &cmd) {
    interpolator.stop();
    servoCtrl.disable();
    Serial.println("OK: Emergency stop");
}

/**
 * M111 — сохранение текущих углов как домашней позиции.
 */
void handleM111(GCodeCommand &cmd) {
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    for (int i = 0; i < 7; i++) {
        homeAngles.angles[i] = currentAngles.angles[i];
    }
    xSemaphoreGive(angleMutex);
    Serial.println("OK: Home saved");
}

/**
 * M400 — установка компенсации люфта.
 */
void handleM400(GCodeCommand &cmd) {
    float bl[7] = {};
    for (int i = 0; i < 7; i++) {
        if (cmd.servoOverride[i]) {
            bl[i] = cmd.servoAngles.angles[i];
        }
    }
    backlash.setAll(bl);
    Serial.println("OK: Backlash set");
}

/**
 * M410 — добавление запретной зоны.
 */
void handleM410(GCodeCommand &cmd) {
    float x1 = cmd.pose.position.x;
    float x2 = cmd.pose.position.y;
    float y1 = cmd.pose.position.z;
    float y2 = cmd.pose.orientation.x;
    float z1 = cmd.pose.orientation.y;
    float z2 = cmd.pose.orientation.z;
    if (x2 == 0 && y2 == 0 && z2 == 0) {
        x1 = -100; x2 = 100; y1 = -100; y2 = 100; z1 = 0; z2 = 50;
    }
    safety.addForbiddenZone(x1, x2, y1, y2, z1, z2);
    Serial.println("OK: Forbidden zone added");
}

/**
 * M411 — установка минимальной высоты стола.
 */
void handleM411(GCodeCommand &cmd) {
    float zMin = cmd.pose.position.z;
    if (zMin == 0) zMin = 50.0f;
    safety.setTableZMin(zMin);
    Serial.println("OK: Table Z min = " + String(zMin, 1));
}

/**
 * M412 — установка максимального радиуса дотягивания.
 */
void handleM412(GCodeCommand &cmd) {
    float r = cmd.hasParam['R' - 'A'] ? cmd.params['R' - 'A'] : 395.0f;
    safety.setMaxReach(r);
    Serial.println("OK: Max reach = " + String(r, 1));
}

/**
 * M413 — установка минимального радиуса дотягивания.
 */
void handleM413(GCodeCommand &cmd) {
    float r = cmd.hasParam['R' - 'A'] ? cmd.params['R' - 'A'] : 50.0f;
    safety.setMinReach(r);
    Serial.println("OK: Min reach = " + String(r, 1));
}

/**
 * M414 — установка длины пальцев захвата.
 *
 * ВАЖНО: длина пальца передаётся и в FK, и в IK, и в safety.
 * IK вычитает её из целевой позиции TCP, чтобы solvePosition()
 * решал задачу для ЗАПЯСТЬЯ. Без этого G0 X100 Y100 Z100 даёт
 * ошибку ~20 мм.
 */
void handleM414(GCodeCommand &cmd) {
    float len = cmd.hasParam['L' - 'A'] ? cmd.params['L' - 'A'] : 50.0f;
    fkSolver.setFingerLength(len);
    ikSolver.setFingerLength(len);   // ← добавить: IK тоже должен знать длину пальца
    safety.setFingerLength(len);
    config.fingerLength = len;       // ← в config, иначе M500 сохранит старое,
                                     // а после ребута палец слетит на EEPROM-значение
    Serial.println("OK: Finger length = " + String(len, 1) + " (M500 to save)");
}

/**
 * M415 — отключение проверки безопасности.
 */
void handleM415(GCodeCommand &cmd) {
    safety.enable(false);
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.safetyEnabled = false;
    xSemaphoreGive(displayMutex);
    Serial.println("OK: Safety disabled");
}

/**
 * M416 — включение проверки безопасности.
 */
void handleM416(GCodeCommand &cmd) {
    safety.enable(true);
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.safetyEnabled = true;
    xSemaphoreGive(displayMutex);
    Serial.println("OK: Safety enabled");
}

/**
 * M417 — вывод списка запретных зон.
 */
void handleM417(GCodeCommand &cmd) {
    int count = safety.getZoneCount();
    Serial.println("OK: Forbidden zones count = " + String(count));
    for (int i = 0; i < count; i++) {
        float x1, x2, y1, y2, z1, z2;
        bool active;
        if (safety.getForbiddenZone(i, x1, x2, y1, y2, z1, z2, active)) {
            Serial.println("Zone[" + String(i) + "]: X[" + String(x1, 0) + "-" + String(x2, 0) +
                           "] Y[" + String(y1, 0) + "-" + String(y2, 0) +
                           "] Z[" + String(z1, 0) + "-" + String(z2, 0) +
                           "] active=" + String(active ? "true" : "false"));
        }
    }
}

/**
 * M420 — установка минимальной высоты Z для звена.
 */
void handleM420(GCodeCommand &cmd) {
    int link = cmd.hasParam['L' - 'A'] ? (int)cmd.params['L' - 'A'] : 0;
    float zMin = cmd.pose.position.z;
    safety.setLinkZMin(link, zMin);
    Serial.println("OK: Link " + String(link) + " Z min = " + String(zMin, 1));
}

/**
 * M425 — отключение проверки безопасности (дублирует M415).
 */
void handleM425(GCodeCommand &cmd) {
    safety.enable(false);
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.safetyEnabled = false;
    xSemaphoreGive(displayMutex);
    Serial.println("OK: Safety disabled");
}

/**
 * M426 — включение проверки безопасности (дублирует M416).
 */
void handleM426(GCodeCommand &cmd) {
    safety.enable(true);
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.safetyEnabled = true;
    xSemaphoreGive(displayMutex);
    Serial.println("OK: Safety enabled");
}

/**
 * M430 — выбор стратегии интерполяции движения.
 */
void handleM430(GCodeCommand &cmd) {
    int s = cmd.hasParam['S' - 'A'] ? (int)cmd.params['S' - 'A'] : 3;
    interpStrategy = (InterpolationStrategy)s;
    interpolator.setStrategy(interpStrategy);
    Serial.println("OK: Strategy = " + String(s));
}

/**
 * M450 — пакет точек: планирование отдельно, исполнение отдельно.
 *   S=1 — открыть (следующие G1 копятся: OK = принято+провалидировано);
 *   S=0 — закрыть и исполнить всё одной траекторией без остановок;
 *   S=2 — сбросить без движения (идемпотентно).
 * Ворота каждой точки — те же 10мм/15° (отказ = ERROR + сброс пакета).
 * G0/M104/G28/M108 при открытом пакете запрещены (сначала закрой).
 */
void handleM450(GCodeCommand &cmd) {
    int s = cmd.hasParam['S' - 'A'] ? (int)cmd.params['S' - 'A'] : 0;
    if (s == 1) {
        if (!interpolator.batchBegin()) {
            Serial.println("ERROR: stream busy (S0/S2 first)");
            return;
        }
        Serial.println("OK: stream open (180 max, backpressure by late OK)");
    } else if (s == 0) {
        if (!interpolator.batchOpen()) {
            Serial.println("ERROR: no open stream (M450 S1 first)");
            return;
        }
        if (interpolator.streamPlanned() == 0) {
            interpolator.batchAbort();
            Serial.println("ERROR: empty stream, flushed");
            return;
        }
        uint32_t n = interpolator.streamPlanned();
        if (interpolator.batchExecute()) {
            xSemaphoreTake(angleMutex, portMAX_DELAY);
            JointAngles done = interpolator.getCurrentAngles();
            currentAngles = done;
            xSemaphoreGive(angleMutex);
            xSemaphoreTake(displayMutex, portMAX_DELAY);
            for (int i = 0; i < 7; i++) {
                sharedState.servoAngles[i] = done.angles[i];
            }
            float q[6];
            for (int i = 0; i < 6; i++) q[i] = done.angles[i];
            sharedState.tcpPosition = fkSolver.getEndEffector(q);
            sharedState.gripperPercent = done.angles[6] / config.servos[6].maxAngle * 100.0f;
            sharedState.isMoving = false;
            sharedState.hasError = false;
            sharedState.commandCount++;
            xSemaphoreGive(displayMutex);
            Serial.printf("OK: stream done (%lu pts)\n", (unsigned long)n);
        } else {
            xSemaphoreTake(angleMutex, portMAX_DELAY);
            JointAngles done = interpolator.getCurrentAngles();
            currentAngles = done;
            xSemaphoreGive(angleMutex);
            xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
            sharedState.hasError = true;
            snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Stream failed");
            xSemaphoreGive(displayMutex);
            Serial.println("ERROR: stream failed (see M115 INTERP)");
        }
    } else if (s == 2) {
        interpolator.batchAbort();
        Serial.println("OK: batch aborted");
    } else {
        Serial.println("ERROR: M450 S=0/1/2");
    }
}

/**
 * M500 — сохранение конфигурации в EEPROM.
 */
void handleM500(GCodeCommand &cmd) {
    storage.saveConfig(config, homeAngles.angles, interpStrategy);
    Serial.println("OK: Saved to EEPROM");
}

/**
 * M501 — загрузка конфигурации из EEPROM.
 *
 * После загрузки config обновляем зависимые модули:
 *   - servoCtrl — конфигурация сервоприводов
 *   - ikSolver  — лимиты углов и длина пальца (для TCP → запястье)
 *   - fkSolver  — длина пальца (для расчёта TCP)
 *   - safety    — длина пальца (для проверок безопасности)
 */
void handleM501(GCodeCommand &cmd) {
    if (storage.loadConfig(config, homeAngles.angles)) {
        servoCtrl.setConfig(config.servos);
        ikSolver.setServoLimits(config.servos);
        ikSolver.setFingerLength(config.fingerLength);   // ← добавить
        fkSolver.setFingerLength(config.fingerLength);
        safety.setFingerLength(config.fingerLength);
        Serial.println("OK: Loaded from EEPROM");
    } else {
        Serial.println("ERROR: Invalid EEPROM data");
    }
}

/**
 * M502 — загрузка конфигурации по умолчанию (заводские настройки).
 *
 * После сброса config обновляем зависимые модули (аналогично M501).
 */
void handleM502(GCodeCommand &cmd) {
    config = DEFAULT_CONFIG;
    for (int i = 0; i < 7; i++) {
        homeAngles.angles[i] = DEFAULT_HOME_ANGLES[i];
    }
    servoCtrl.setConfig(config.servos);
    ikSolver.setServoLimits(config.servos);
    ikSolver.setFingerLength(config.fingerLength);   // ← добавить
    fkSolver.setFingerLength(config.fingerLength);
    safety.setFingerLength(config.fingerLength);
    // Заводские зоны (включая конструктив) — в рантайм safety.
    safety.clearForbiddenZones();
    for (int i = 0; i < config.forbiddenZoneCount && i < MAX_FORBIDDEN_ZONES; i++) {
        const ForbiddenZone &z = config.forbiddenZones[i];
        if (z.active) {
            safety.addForbiddenZone(z.xMin, z.xMax, z.yMin, z.yMax, z.zMin, z.zMax,
                                    z.skipLinks);
        }
    }
    Serial.println("OK: Factory defaults loaded");
}

/**
 * handleHelp (?) — вывод справки по всем доступным командам.
 */
void handleHelp(GCodeCommand &cmd) {
    Serial.println("=== 6-DOF Robot Arm Commands ===");
    Serial.print("FW version ");
    Serial.print(FW_VERSION_STR);
    Serial.print(" (");
    Serial.print(FW_VERSION);
    Serial.print("), HW version ");
    Serial.println(HW_VERSION_STR);
    Serial.println("G0 X.. Y.. Z.. A.. B.. C.. J.. - Move (IK/Direct/Mixed, def A-90=tool+Y)");
    Serial.println("G1 X.. Y.. Z.. F..            - Cartesian straight line (TCP holds)");
    Serial.println("G28 [H1] [F..]                - Home/park (F15 = тихо)");
    Serial.println("G90                           - Absolute mode");
    Serial.println("G91                           - Relative mode");
    Serial.println("M104 P1=90 P2=0 ..            - Set servo angles directly");
    Serial.println("M105                          - Report servo angles");
    Serial.println("M108 S=50                     - Set gripper (percent 0-100)");
    Serial.println("M119 [S=1] [D=4]               - IMU live / dump ring log");
    Serial.println("M114                          - Report position (FK)");
    Serial.println("M115 [S=1]                    - Kinematics diag (target/sols/actual)");
    Serial.println("M112                          - Emergency stop");
    Serial.println("M111                          - Save current as HOME");
    Serial.println("M400 P1=2.0 P2=1.5 ..         - Set backlash compensation");
    Serial.println("M410 X Y Z A B C              - Add forbidden zone");
    Serial.println("M411 Z=50                     - Set table Z min");
    Serial.println("M412 R=395                    - Set max reach (default 395mm)");
    Serial.println("M413 R=50                     - Set min reach");
    Serial.println("M414 L=50                     - Set gripper finger length (mm)");
    Serial.println("M415 / M425                   - Disable safety");
    Serial.println("M416 / M426                   - Enable safety");
    Serial.println("M417                          - List forbidden zones");
    Serial.println("M420 L1 Z=30                  - Set link Z min");
    Serial.println("M430 S=3                      - Strategy (0=lin,1=seq,2=grp,3=auto)");
    Serial.println("M500                          - Save config to EEPROM");
    Serial.println("M501                          - Load config from EEPROM");
    Serial.println("M502                          - Load factory defaults");
    Serial.println("?                             - This help");
}

// ====================================================================
// Вспомогательные функции
// ====================================================================

/**
 * printServoAngles() — вывод текущих углов всех 7 сервоприводов.
 */
void printServoAngles() {
    Serial.print("Angles:");
    for (int i = 0; i < 7; i++) {
        Serial.print(" P");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(currentAngles.angles[i], 1);
    }
    Serial.println();
}

/**
 * printPosition() — вывод текущей позиции энд-эффектора (FK).
 */
void printPosition() {
    float q[6];
    for (int i = 0; i < 6; i++) {
        q[i] = currentAngles.angles[i];
    }

    Vec3 pos = fkSolver.getEndEffector(q);
    Serial.print("Position: X=");
    Serial.print(pos.x, 1);
    Serial.print(" Y=");
    Serial.print(pos.y, 1);
    Serial.print(" Z=");
    Serial.println(pos.z, 1);
}

/**
 * printStatus() — вывод полного статуса манипулятора.
 */
void printStatus() {
    Serial.println("=== Robot Arm Status ===");
    Serial.print("Mode: ");
    Serial.println(absoluteMode ? "Absolute" : "Relative");
    Serial.print("Safety: ");
    Serial.println(safety.isEnabled() ? "ON" : "OFF");
    Serial.print("Strategy: ");
    Serial.println(interpStrategy);
    Serial.print("Home set: ");
    Serial.println(homeAngles.angles[0] != 0 ? "YES" : "DEFAULT");
    Serial.print("EEPROM: ");
    Serial.println(storage.isConfigValid() ? "VALID" : "EMPTY");
    printServoAngles();
    printPosition();
}
