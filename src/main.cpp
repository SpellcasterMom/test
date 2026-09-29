/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       :
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-25
 *
 * Основной файл контроллера 6-осевого робота-манипулятора на ESP32.
 *
 * =====================================================================
 * Подключение проводов (Pinout):
 * =====================================================================
 *
 * PCA9685 (сервоприводы) — I2C:
 *   SDA  → GPIO 21
 *   SCL  → GPIO 22
 *   VCC  → 5V (внешнее питание сервоприводов!)
 *   GND  → GND
 *   V+   → адаптер питания сервоприводов (отдельный от ESP32!) 3-12V 10A
 *
 * ST7789 1.54" (дисплей) — SPI (VSPI):
 *   MOSI/SDA → GPIO 23
 *   SCK/SCL  → GPIO 18
 *   CS   → GPIO 5
 *   DC   → GPIO 2
 *   RST  → GPIO 4
 *   VCC  → 3.3V
 *   GND  → GND
 *
 * Serial (USB-to-Serial):
 *   TX   → GPIO 1  (или пин UART TX)
 *   RX   → GPIO 3  (или пин UART RX)
 *   Скорость: 115200 бод
 *
 * =====================================================================
 * Распиновка сервоприводов (PCA9685 каналы):
 * =====================================================================
 *   Канал 0  → Joint 1 (Base)        — серва 20кг DS3218 4.8-6.8V
 *   Канал 1  → Joint 2 (Shoulder)    — серва 70кг BLS-HV70MG 4.8-8.4V
 *   Канал 2  → Joint 3 (Elbow)       — серва 70кг BLS-HV70MG 4.8-8.4V
 *   Канал 3  → Joint 4 (Wrist Pitch) — серва 2.5кг MG90S 4.8V
 *   Канал 4  → Joint 5 (Wrist Roll)  — серва 2.5кг MG90S 4.8V
 *   Канал 5  → Joint 6 (Wrist Yaw)   — серва 2.5кг MG90S 4.8V
 *   Канал 6  → Gripper               — серва 2.5кг MG90S 4.8V
 *
 * =====================================================================
 * Архитектура FreeRTOS на двух ядрах:
 * =====================================================================
 *   Core 0 — Serial + Motion:
 *     Чтение Serial, парсинг G-code, IK/FK, safety, интерполяция,
 *     прямое управление сервоприводами, обновление sharedState.
 *
 *   Core 1 — Display:
 *     Чтение sharedState (каждые 200 мс), обновление ST7789 дисплея.
 *
 * Доступ к общим данным (sharedState, currentAngles) защищён мьютексами.
 *
 * =====================================================================
 * Компоновка дисплея (ST7789 240×240):
 * =====================================================================
 *
 *   ┌─────────────────────────────────────┐  y=0
 *   │ TCP POS.                            │  заголовок
 *   ├─────────────────────────────────────┤  y=16
 *   │ X:   415mm                          │
 *   │ Y:     0mm                          │
 *   │ Z:   160mm                          │
 *   ├─────────────────────────────────────┤  y=72
 *   │ JOINTS                              │  заголовок
 *   ├─────────────────────────────────────┤  y=88
 *   │ P1:    0     P4:    0               │
 *   │ P2:  -40     P5:    0               │
 *   │ P3:    0     P6:    0               │
 *   │ Gripper:   50%                      │
 *   ├─────────────────────────────────────┤  y=176
 *   │ READY                          123  │  статус + счётчик
 *   │ Safe:ON     PCA:OFF            ABS  │  safety + PCA + режим
 *   │ G:G28                               │  последняя команда
 *   └─────────────────────────────────────┘  y=240
 *
 * =====================================================================
 * ДЕМОНСТРАЦИЯ: Endpoint Control — 8 позиций
 * =====================================================================
 *
 * G28
 * G0 X200 Y50 Z250 A10 B-20 C5 F300
 * G0 X180 Y-80 Z150 A-15 B10 C-20 F300
 * G0 X-50 Y-150 Z200 A25 B0 C30 F300
 * G0 X-100 Y120 Z300 A-10 B-30 C-15 F300
 * G0 X100 Y0 Z100 A0 B20 C0 F250
 * G0 P7=20 F200
 * G0 X50 Y-200 Z280 A30 B-25 C40 F300
 * G0 P7=90 F200
 * G0 X150 Y180 Z180 A-20 B15 C-35 F300
 * G0 X250 Y0 Z200 A0 B-40 C0 F250
 * G28
 * M114
 * M105
 */

#include <Arduino.h>
// #include <TFT_eSPI.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <cmath>
#include "config.h"
#include "types.h"
#include "commands.h"
#include "gcode_parser.h"
#include "fk_solver.h"
#include "ik_solver.h"
#include "servo_controller.h"
#include "motion_interpolator.h"
#include "backlash_compensator.h"
#include "safety_checker.h"
#include "gripper_controller.h"
#include "eeprom_storage.h"

// Пины SPI для ST7789
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4

// ===== Глобальные объекты =====

// Парсер G-code: преобразует текстовую команду в структуру GCodeCommand
GCodeParser          parser;

// Прямая кинематика (FK): вычисляет положение конца манипулятора
// по углам суставов. Используется для отчёта позиции (M114).
FKSolver             fkSolver;

// Обратная кинематика (IK): вычисляет углы суставов по целевой
// позиции/ориентации энд-эффектора. Вызывается в обработчиках G0/G1.
IKSolver             ikSolver;

// Контроллер сервоприводов (PCA9685): управляет 7 каналами PWM,
// задаёт углы сервоприводов и отключает их.
ServoController      servoCtrl;

// Интерполятор движений: плавно перемещает сервоприводы из текущего
// положения в целевое с заданной подачей (feedrate).
MotionInterpolator   interpolator;

// Компенсатор люфта: хранит поправки на люфт каждого сервопривода
// (в градусах), корректирует целевые углы перед отправкой.
BacklashCompensator  backlash;

// Проверка безопасности: проверяет запретные зоны, ограничения
// по радиусу дотягивания, минимальной/максимальной высоте стола
// и длине пальцев захвата. При нарушении — запрещает движение.
SafetyChecker        safety;

// Контроллер захвата: управляет мотором захвата (сервопривод №7),
// открывает/закрывает «пальцы».
GripperController    gripper;

// Хранилище EEPROM: сохранение и загрузка конфигурации
EEPROMStorage        storage;

// ===== Глобальное состояние =====
ManipulatorConfig    config = DEFAULT_CONFIG;
JointAngles          homeAngles;
JointAngles          currentAngles;
bool                 absoluteMode = true;
InterpolationStrategy interpStrategy = INTERP_AUTO;

// ===== FreeRTOS: задачи и мьютексы =====
TaskHandle_t core0TaskHandle = NULL;
TaskHandle_t core1TaskHandle = NULL;
SemaphoreHandle_t angleMutex = NULL;

// ===== Дисплей ST7789 =====
// Экран 1.54" 240×240, SPI (VSPI), пины: MOSI=23, SCK=18, CS=5, DC=2, RST=4
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// ===== Структура общего состояния для дисплея =====
struct SharedState {
    Vec3        tcpPosition;      // Текущая позиция TCP (из FK)
    float       servoAngles[7];   // Углы 7 сервоприводов (градусы)
    float       gripperPercent;   // Процент открытия захвата (0-100%)
    bool        isMoving;         // Флаг движения
    bool        safetyEnabled;    // Состояние безопасности
    bool        pcaReady;         // PCA9685 найден на I2C
    bool        hasError;         // Есть ошибка
    char        lastError[64];    // Текст последней ошибки
    char        lastCommand[24];  // Последняя полученная команда
    uint32_t    commandCount;     // Счётчик выполненных команд
};

SharedState    sharedState;
SemaphoreHandle_t displayMutex = NULL;

// ===== Предварительные объявления функций =====
void core0Task(void *pvParameters);
void core1Task(void *pvParameters);
void printPosition();
void printServoAngles();
void printStatus();
void displayInit();
void displayUpdate();
void onInterpolationStep(const JointAngles &angles);

// ====================================================================
// Журнал последнего движения для M115 (разбор кинематики)
// ====================================================================
// Заполняется в handleG0 (G0/G1), handleM104, handleG28.
// Читается в handleM115. Всё выполняется в задаче core0,
// поэтому дополнительный мьютекс не нужен.
struct DiagSol {
    JointAngles q;          // углы кандидата (6 суставов + захват)
    Vec3 joints[7];         // FK-цепочка J0..J6
    Vec3 tcp;               // FK TCP
    Vec3 dir;               // FK направление пальца
    float dPos = 0;         // |TCP - TARGET| (мм)
    float dOri = 0;         // угол между dir и заданным toolDir (град)
    bool safe = false;      // прошёл safety.checkAllLinks
    char safeErr[48];       // причина отказа safety
    int iters = 0;          // итераций DLS-уточнения
    bool converged = false; // DLS сошлось по допускам
    float seedPos = 0;      // невязка позиции у seed'а (мм)
    float seedOri = 0;      // невязка ориентации у seed'а (град)
    float dbgDq0 = -1;      // норма первого dq (рад)
    float dbgN1 = -1;       // норма ошибки после первого полного шага
    DiagSol() { memset(safeErr, 0, sizeof(safeErr)); }
};

struct DiagLastMove {
    bool valid = false;
    char cmdText[48];       // исходный текст команды
    char kind[8];           // IK / DIRECT / MIXED / HOME / PARK
    Pose target;            // заданная поза (для IK/MIXED)
    bool hasPos = false;
    bool hasOri = false;
    float feedrate = 0;
    bool anglesOnly = false;// DIRECT/HOME/PARK: цель — углы, а не TCP
    int solCount = 0;
    DiagSol sols[4];
    int chosen = -1;        // индекс выбранного решения
    char failReason[64];    // причина отказа (пусто = отказа не было)
    DiagLastMove() {
        memset(cmdText, 0, sizeof(cmdText));
        memset(kind, 0, sizeof(kind));
        memset(failReason, 0, sizeof(failReason));
    }
};
DiagLastMove diagLast;

// Направление инструмента (ось Z) из углов Эйлера ZYX — та же формула,
// что IKSolver::eulerToMatrix, третий столбец.
static Vec3 diagToolDir(const Vec3 &o) {
    float cr = cos(o.x * DEG_TO_RAD), sr = sin(o.x * DEG_TO_RAD);
    float cp = cos(o.y * DEG_TO_RAD), sp = sin(o.y * DEG_TO_RAD);
    float cy = cos(o.z * DEG_TO_RAD), sy = sin(o.z * DEG_TO_RAD);
    Vec3 d;
    d.x = cy*sp*cr + sy*sr;
    d.y = sy*sp*cr - cy*sr;
    d.z = cp*cr;
    return d;
}

// Угол между двумя единичными векторами (градусы).
static float diagOriErr(const Vec3 &a, const Vec3 &b) {
    float dot = a.x*b.x + a.y*b.y + a.z*b.z;
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    return acos(dot) * RAD_TO_DEG;
}

// Посчитать FK + ошибки + safety для кандидата и положить в DiagSol.
static void diagEvalSol(DiagSol &s, const JointAngles &q, const Pose &target) {
    s.q = q;
    float qq[6];
    for (int i = 0; i < 6; i++) qq[i] = q.angles[i];
    fkSolver.getAllJointPositions(qq, s.joints);
    s.tcp = fkSolver.getEndEffector(qq);
    s.dir = fkSolver.getFingerDirection(qq);
    float dx = s.tcp.x - target.position.x;
    float dy = s.tcp.y - target.position.y;
    float dz = s.tcp.z - target.position.z;
    s.dPos = sqrt(dx*dx + dy*dy + dz*dz);
    s.dOri = diagOriErr(s.dir, diagToolDir(target.orientation));
    if (!safety.isEnabled()) {
        s.safe = true;
        s.safeErr[0] = '\0';
    } else if (safety.checkAllLinks(qq)) {
        s.safe = true;
        s.safeErr[0] = '\0';
    } else {
        s.safe = false;
        strncpy(s.safeErr, safety.getLastError().c_str(), sizeof(s.safeErr) - 1);
        s.safeErr[sizeof(s.safeErr) - 1] = '\0';
    }
}

// Начать новую запись журнала: текст команды берём из sharedState.
static void diagBegin(const char *kind, float feedrate) {
    diagLast = DiagLastMove();
    diagLast.valid = true;
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    strncpy(diagLast.cmdText, sharedState.lastCommand, sizeof(diagLast.cmdText) - 1);
    xSemaphoreGive(displayMutex);
    strncpy(diagLast.kind, kind, sizeof(diagLast.kind) - 1);
    diagLast.feedrate = feedrate;
}

// Печать цепочки суставов J0..J6.
static void diagPrintJoints(const Vec3 joints[7]) {
    for (int i = 0; i < 7; i++) {
        Serial.printf("  J%d: X=%.1f Y=%.1f Z=%.1f\n", i,
                      joints[i].x, joints[i].y, joints[i].z);
    }
}

// ====================================================================
// Обработчики G-code команд
// ====================================================================

/**
 * G0 — перемещение (быстрое).
 */
void handleG0(GCodeCommand &cmd) {
    cmd.determineMode();

    // Сброс ошибки при новой команде
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    // Эффективная поза: без A/B/C берём дефолт «палец вперёд +Y»
    // (DEFAULT_ORI_*). M115 покажет её же с пометкой (orient=default).
    Pose effPose = cmd.pose;
    if (!cmd.hasOrientation) {
        effPose.orientation.x = DEFAULT_ORI_A;
        effPose.orientation.y = DEFAULT_ORI_B;
        effPose.orientation.z = DEFAULT_ORI_C;
    }

    JointAngles target;

    switch (cmd.mode) {
        case GCodeCommand::IK_MODE: {
            // Обратная кинематика: позиция + ориентация → углы суставов
            // Ищем безопасное решение среди всех вариантов (elbow up/down)
            diagBegin("IK", cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE);
            diagLast.target = effPose;
            diagLast.hasPos = cmd.hasPosition;
            diagLast.hasOri = cmd.hasOrientation;
            JointAngles solutions[4];
            IKSolutionMeta metas[4];
            int solCount = ikSolver.solveAll(effPose, solutions, metas);

            if (solCount == 0) {
                strncpy(diagLast.failReason, "IK solution not found",
                        sizeof(diagLast.failReason) - 1);
                xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
                sharedState.hasError = true;
                snprintf(sharedState.lastError, sizeof(sharedState.lastError), "IK not found");
                xSemaphoreGive(displayMutex);
                Serial.println("ERROR: IK solution not found");
                return;
            }

            // Оцениваем каждое решение (FK + ошибки + safety) для M115
            for (int i = 0; i < solCount && diagLast.solCount < 4; i++) {
                if (cmd.hasGripper) {
                    solutions[i].angles[6] = cmd.pose.gripper;
                }
                diagEvalSol(diagLast.sols[diagLast.solCount], solutions[i], effPose);
                diagLast.sols[diagLast.solCount].iters = metas[i].iters;
                diagLast.sols[diagLast.solCount].converged = metas[i].converged;
                diagLast.sols[diagLast.solCount].seedPos = metas[i].seedPos;
                diagLast.sols[diagLast.solCount].seedOri = metas[i].seedOri;
                diagLast.sols[diagLast.solCount].dbgDq0 = metas[i].dbgDq0;
                diagLast.sols[diagLast.solCount].dbgN1 = metas[i].dbgN1;
                diagLast.solCount++;
            }

            // Выбираем безопасное решение с минимальной невязкой:
            // score = dPos[мм] + 1.0·dOri[град] (вес настраивается).
            // «Первый безопасный» систематически брал худший вариант
            // (M115: [0] 2.7мм/23.8° вместо [1] 2.2мм/0.0°).
            int best = -1;
            float bestScore = 0;
            for (int i = 0; i < diagLast.solCount; i++) {
                if (!diagLast.sols[i].safe) continue;
                float score = diagLast.sols[i].dPos + diagLast.sols[i].dOri;
                if (best < 0 || score < bestScore) {
                    best = i;
                    bestScore = score;
                }
            }
            bool foundSafe = (best >= 0);
            if (foundSafe) {
                target = diagLast.sols[best].q;
                diagLast.chosen = best;
            }

            if (!foundSafe) {
                strncpy(diagLast.failReason, "No safe IK solution",
                        sizeof(diagLast.failReason) - 1);
                xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
                sharedState.hasError = true;
                snprintf(sharedState.lastError, sizeof(sharedState.lastError), "No safe IK");
                xSemaphoreGive(displayMutex);
                Serial.println("ERROR: No safe IK solution");
                return;
            }
            break;
        }
        case GCodeCommand::DIRECT_MODE: {
            target = cmd.servoAngles;
            diagBegin("DIRECT", 0);
            diagLast.anglesOnly = true;
            diagEvalSol(diagLast.sols[0], target, cmd.pose);
            diagLast.solCount = 1;
            diagLast.chosen = 0;
            break;
        }
        case GCodeCommand::MIXED_MODE: {
            diagBegin("MIXED", cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE);
            diagLast.target = effPose;
            diagLast.hasPos = cmd.hasPosition;
            diagLast.hasOri = cmd.hasOrientation;
            target = currentAngles;  // страховка: если solve() не найдёт решение,
                                     // остаёмся на месте, а не едем в мусор
            bool ikOk = true;
            if (cmd.hasPosition || cmd.hasOrientation) {
                ikOk = ikSolver.solve(effPose, target);
            }
            for (int i = 0; i < 7; i++) {
                if (cmd.servoOverride[i]) {
                    target.angles[i] = cmd.servoAngles.angles[i];
                }
            }
            if (!ikOk && (cmd.hasPosition || cmd.hasOrientation)) {
                strncpy(diagLast.failReason, "IK solve failed (MIXED moved anyway)",
                        sizeof(diagLast.failReason) - 1);
            }
            diagEvalSol(diagLast.sols[0], target, effPose);
            diagLast.solCount = 1;
            diagLast.chosen = 0;
            // solve() выше уже довёл решение; забираем meta для M115
            // (доводка копии — железо не двигаем, дёшево: уже сошлось)
            {
                JointAngles tq = target;
                IKSolutionMeta mm;
                ikSolver.refineSolution(effPose, tq, &mm);
                diagLast.sols[0].iters = mm.iters;
                diagLast.sols[0].converged = mm.converged;
                diagLast.sols[0].seedPos = mm.seedPos;
                diagLast.sols[0].seedOri = mm.seedOri;
                diagLast.sols[0].dbgDq0 = mm.dbgDq0;
                diagLast.sols[0].dbgN1 = mm.dbgN1;
            }
            break;
        }
    }

    // Проверка безопасности
    if (safety.isEnabled()) {
        if (!safety.checkAllLinks(target.angles)) {
            strncpy(diagLast.failReason, safety.getLastError().c_str(),
                    sizeof(diagLast.failReason) - 1);
            xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
            sharedState.hasError = true;
            snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Safety");
            xSemaphoreGive(displayMutex);
            Serial.print("ERROR: Safety violation: ");
            Serial.println(safety.getLastError());
            return;
        }
    }

    float feedrate = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
    // Старт траектории и времени — из лучшего известного положения.
    // Без этого проверка идёт от устаревшей точки интерполятора
    // (после ребута — нули = недостижимый вылет 415мм) и даёт ложный
    // фолбэк на sequential с шага 0.
    interpolator.setCurrentAngles(currentAngles);
    if (interpolator.moveTo(target, feedrate)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        for (int i = 0; i < 7; i++) {
            sharedState.servoAngles[i] = target.angles[i];
        }
        float q[6];
        for (int i = 0; i < 6; i++) q[i] = target.angles[i];
        sharedState.tcpPosition = fkSolver.getEndEffector(q);
        sharedState.gripperPercent = target.angles[6] / config.servos[6].maxAngle * 100.0f;
        sharedState.isMoving = false;
        sharedState.hasError = false;
        sharedState.commandCount++;
        xSemaphoreGive(displayMutex);

        Serial.println("OK");
    } else {
        strncpy(diagLast.failReason, "Move failed (interpolator)",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: Move failed");
    }
}

/**
 * G1 — перемещение с подачей (аналогично G0).
 */
void handleG1(GCodeCommand &cmd) {
    handleG0(cmd);
}

/**
 * G28 — перемещение в домашнюю позицию (home).
 */
void handleG28(GCodeCommand &cmd) {
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    JointAngles target;
    bool park = cmd.hasParam['H' - 'A'] && (int)cmd.params['H' - 'A'] == 1;

    if (park) {
        target.setAll(0, -60, -90, 0, 0, 0, 0);
    } else {
        for (int i = 0; i < 7; i++) {
            target.angles[i] = homeAngles.angles[i];
        }
    }

    diagBegin(park ? "PARK" : "HOME", 300.0f);
    diagLast.anglesOnly = true;
    diagEvalSol(diagLast.sols[0], target, cmd.pose);
    diagLast.solCount = 1;
    diagLast.chosen = 0;

    // Старт — из лучшего известного положения (см. handleG0)
    interpolator.setCurrentAngles(currentAngles);
    if (interpolator.moveTo(target, 300.0f)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        for (int i = 0; i < 7; i++) {
            sharedState.servoAngles[i] = target.angles[i];
        }
        float q[6];
        for (int i = 0; i < 6; i++) q[i] = target.angles[i];
        sharedState.tcpPosition = fkSolver.getEndEffector(q);
        sharedState.gripperPercent = target.angles[6] / config.servos[6].maxAngle * 100.0f;
        sharedState.isMoving = false;
        sharedState.commandCount++;
        xSemaphoreGive(displayMutex);

        Serial.println(park ? "OK: Parked" : "OK: Home");
    } else {
        strncpy(diagLast.failReason, park ? "Park failed" : "Home move failed",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Park failed");
        xSemaphoreGive(displayMutex);
        Serial.println(park ? "ERROR: Park failed" : "ERROR: Home move failed");
    }
}

/**
 * G90 — абсолютный режим координат.
 */
void handleG90(GCodeCommand &cmd) {
    absoluteMode = true;
    Serial.println("OK: Absolute mode");
}

/**
 * G91 — относительный режим координат.
 */
void handleG91(GCodeCommand &cmd) {
    absoluteMode = false;
    Serial.println("OK: Relative mode");
}

/**
 * M104 — прямая установка углов сервоприводов (без интерполяции).
 */
void handleM104(GCodeCommand &cmd) {
    for (int i = 0; i < 7; i++) {
        if (cmd.servoOverride[i]) {
            servoCtrl.setAngle(i, cmd.servoAngles.angles[i]);
            xSemaphoreTake(angleMutex, portMAX_DELAY);
            currentAngles.angles[i] = cmd.servoAngles.angles[i];
            xSemaphoreGive(angleMutex);
        }
    }
    // Синхронизируем интерполятор: иначе следующий G0 посчитает
    // траекторию и время от устаревшей точки (ложный фолбэк на SEQ).
    interpolator.setCurrentAngles(currentAngles);
    diagBegin("DIRECT", 0);
    diagLast.anglesOnly = true;
    diagEvalSol(diagLast.sols[0], currentAngles, cmd.pose);
    diagLast.solCount = 1;
    diagLast.chosen = 0;
    Serial.println("OK");
}

/**
 * M105 — вывод текущих углов сервоприводов.
 */
void handleM105(GCodeCommand &cmd) {
    printServoAngles();
}

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

/**
 * M115 — разбор кинематики: куда шли (цель), кандидаты IK,
 * выбранное решение, куда пришли (факт), невязка.
 * Параметр S=1 — полные цепочки J0..J6 для всех кандидатов.
 */
void handleM115(GCodeCommand &cmd) {
    bool full = cmd.hasParam['S' - 'A'] && (int)cmd.params['S' - 'A'] == 1;
    Serial.println("=== KIN-DIAG ===");
    if (!diagLast.valid) {
        Serial.println("No moves yet");
        return;
    }
    Serial.printf("CMD: %s\n", diagLast.cmdText);
    Serial.printf("KIND: %s FEED: %.0f SAFETY: %s FINGER: %.1f MODE: %s\n",
                  diagLast.kind, diagLast.feedrate,
                  safety.isEnabled() ? "ON" : "OFF",
                  config.fingerLength,
                  absoluteMode ? "ABS" : "REL");
    Serial.printf("STRAT: %d INTERP: %s %s\n", (int)interpStrategy,
                  interpolator.getLastMode(), interpolator.getLastNote());
    Vec3 toolDir = diagToolDir(diagLast.target.orientation);
    if (!diagLast.anglesOnly) {
        Serial.printf("TARGET TCP: X=%.1f Y=%.1f Z=%.1f A=%.1f B=%.1f C=%.1f%s\n",
                      diagLast.target.position.x, diagLast.target.position.y,
                      diagLast.target.position.z, diagLast.target.orientation.x,
                      diagLast.target.orientation.y, diagLast.target.orientation.z,
                      diagLast.hasOri ? "" : " (orient=default)");
        Serial.printf("TARGET wrist: X=%.1f Y=%.1f Z=%.1f toolDir: %.3f %.3f %.3f\n",
                      diagLast.target.position.x - toolDir.x * config.fingerLength,
                      diagLast.target.position.y - toolDir.y * config.fingerLength,
                      diagLast.target.position.z - toolDir.z * config.fingerLength,
                      toolDir.x, toolDir.y, toolDir.z);
    } else {
        Serial.print("TARGET angles:");
        for (int i = 0; i < 7; i++) {
            Serial.printf(" P%d=%.1f", i + 1, diagLast.sols[0].q.angles[i]);
        }
        Serial.println();
    }
    Serial.printf("SOLS: %d\n", diagLast.solCount);
    for (int i = 0; i < diagLast.solCount; i++) {
        DiagSol &s = diagLast.sols[i];
        // Метки углов, сидящих на механических лимитах
        char clamp[32]; clamp[0] = '\0';
        for (int j = 0; j < 6; j++) {
            if (fabs(s.q.angles[j] - config.servos[j].minAngle) < 0.5f ||
                fabs(s.q.angles[j] - config.servos[j].maxAngle) < 0.5f) {
                char tmp[8];
                snprintf(tmp, sizeof(tmp), "P%d ", j + 1);
                strncat(clamp, tmp, sizeof(clamp) - strlen(clamp) - 1);
            }
        }
        Serial.printf("[%d] q:", i);
        for (int j = 0; j < 6; j++) Serial.printf(" P%d=%.1f", j + 1, s.q.angles[j]);
        Serial.printf("\n    TCP: X=%.1f Y=%.1f Z=%.1f", s.tcp.x, s.tcp.y, s.tcp.z);
        if (!diagLast.anglesOnly) {
            Serial.printf(" dPos=%.1fmm dOri=%.1fdeg", s.dPos, s.dOri);
        }
        Serial.printf("\n    %s", s.safe ? "SAFE:OK" : "SAFE:FAIL");
        if (!s.safe) Serial.printf("(%s)", s.safeErr);
        Serial.printf(" CLAMP:%s", clamp[0] ? clamp : "-");
        if (!diagLast.anglesOnly) {
            Serial.printf(" NLS:%s/%dit", s.converged ? "ok" : "fail", s.iters);
            if (!s.converged) Serial.printf(" seed=%.1fmm/%.1fdeg", s.seedPos, s.seedOri);
            Serial.printf(" dq0=%.3f n1=%.1f", s.dbgDq0, s.dbgN1);
        }
        Serial.println();
        if (full || i == diagLast.chosen) {
            diagPrintJoints(s.joints);
        }
    }
    if (diagLast.chosen >= 0) {
        const char *how = "direct";
        if (strcmp(diagLast.kind, "IK") == 0) how = "min-residual";
        else if (strcmp(diagLast.kind, "MIXED") == 0) how = "single";
        Serial.printf("CHOSEN: [%d] (%s)\n", diagLast.chosen, how);
    } else {
        Serial.printf("FAIL: %s\n", diagLast.failReason);
    }
    // Фактическое положение
    float q[6];
    for (int i = 0; i < 6; i++) q[i] = currentAngles.angles[i];
    Vec3 joints[7];
    fkSolver.getAllJointPositions(q, joints);
    Vec3 dir = fkSolver.getFingerDirection(q);
    Vec3 atcp = fkSolver.getEndEffector(q);
    Serial.print("ACTUAL q:");
    for (int i = 0; i < 7; i++) Serial.printf(" P%d=%.1f", i + 1, currentAngles.angles[i]);
    Serial.println();
    diagPrintJoints(joints);
    Serial.printf("  DIR: X=%.3f Y=%.3f Z=%.3f\n", dir.x, dir.y, dir.z);
    Serial.printf("  TCP: X=%.1f Y=%.1f Z=%.1f\n", atcp.x, atcp.y, atcp.z);
    if (!diagLast.anglesOnly) {
        float dx = atcp.x - diagLast.target.position.x;
        float dy = atcp.y - diagLast.target.position.y;
        float dz = atcp.z - diagLast.target.position.z;
        float dist = sqrt(dx*dx + dy*dy + dz*dz);
        Serial.printf("RESIDUAL: dX=%.1f dY=%.1f dZ=%.1f dist=%.1fmm dOri=%.1fdeg%s\n",
                      dx, dy, dz, dist, diagOriErr(dir, toolDir),
                      diagLast.hasOri ? "" : " (vs default orient)");
    } else {
        Serial.println("RESIDUAL: n/a (angles mode)");
    }
}

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
    Serial.println("OK: Finger length = " + String(len, 1));
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
    Serial.println("OK: Factory defaults loaded");
}

/**
 * handleHelp (?) — вывод справки по всем доступным командам.
 */
void handleHelp(GCodeCommand &cmd) {
    Serial.println("=== 6-DOF Robot Arm Commands ===");
    Serial.println("G0 X.. Y.. Z.. A.. B.. C.. J.. - Move (IK/Direct/Mixed, def A-90=tool+Y)");
    Serial.println("G1 X.. Y.. Z.. F.. P1=.. P7=.. - Move with feedrate (same defaults)");
    Serial.println("G28                           - Home all axes");
    Serial.println("G28 H1                        - Park (fold into compact position)");
    Serial.println("G90                           - Absolute mode");
    Serial.println("G91                           - Relative mode");
    Serial.println("M104 P1=90 P2=0 ..            - Set servo angles directly");
    Serial.println("M105                          - Report servo angles");
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

// ====================================================================
// Core 0: чтение Serial + управление движением
// ====================================================================
void core0Task(void *pvParameters) {
    String buffer = "";

    while (true) {
        while (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (buffer.length() > 0) {
                    // Эхо команды в Serial
                    Serial.print("> ");
                    Serial.println(buffer);

                    // Сохраняем последнюю команду в sharedState для дисплея
                    xSemaphoreTake(displayMutex, portMAX_DELAY);
                    strncpy(sharedState.lastCommand, buffer.c_str(),
                            sizeof(sharedState.lastCommand) - 1);
                    sharedState.lastCommand[sizeof(sharedState.lastCommand) - 1] = '\0';
                    xSemaphoreGive(displayMutex);

                    // Парсинг G-code → структура GCodeCommand
                    GCodeCommand cmd = parser.parse(buffer);

                    if (cmd.type == CMD_ERROR) {
                        Serial.print("ERROR: ");
                        Serial.println(cmd.errorMsg);
                    }

                    buffer = "";
                }
            } else {
                buffer += c;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ====================================================================
// Core 1: дисплей
// ====================================================================
void core1Task(void *pvParameters) {
    Serial.println("[Core1] Display task started");
    while (true) {
        displayUpdate();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ====================================================================
// setup() — инициализация
// ====================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Инициализация дисплея ПЕРЕД I2C/PCA9685
    displayInit();

    Serial.println("=== 6-DOF Robot Arm Controller ===");
    Serial.println("ESP32 @ 240 MHz, PCA9685, 7 servos");

    // Создание мьютексов ДО первого использования
    angleMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();

    // Инициализация PCA9685
    servoCtrl.init();
    servoCtrl.setConfig(config.servos);

    // Сохраняем состояние PCA9685 в SharedState
    xSemaphoreTake(displayMutex, portMAX_DELAY);
    sharedState.pcaReady = servoCtrl.isPcaAvailable();
    xSemaphoreGive(displayMutex);

    if (servoCtrl.isPcaAvailable()) {
        Serial.println("PCA9685: OK (servos active)");
    } else {
        Serial.println("PCA9685: NOT FOUND (debug mode)");
    }

    // Связывание интерполятора с зависимостями
    interpolator.setServoController(&servoCtrl);
    interpolator.setBacklashCompensator(&backlash);
    interpolator.setSafetyChecker(&safety);
    interpolator.setStrategy(interpStrategy);

    gripper.setServoController(&servoCtrl);
    storage.begin();

    // Загрузка конфигурации из EEPROM
    if (storage.isConfigValid()) {
        if (storage.loadConfig(config, homeAngles.angles)) {
            servoCtrl.setConfig(config.servos);
            ikSolver.setServoLimits(config.servos);
            ikSolver.setFingerLength(config.fingerLength);   // ← добавить
            fkSolver.setFingerLength(config.fingerLength);
            safety.setFingerLength(config.fingerLength);
            Serial.println("Config loaded from EEPROM");
        }
    } else {
        for (int i = 0; i < 7; i++) {
            homeAngles.angles[i] = DEFAULT_HOME_ANGLES[i];
        }
        ikSolver.setFingerLength(config.fingerLength);   // ← добавить
        fkSolver.setFingerLength(config.fingerLength);
        safety.setFingerLength(config.fingerLength);
        Serial.println("Using default config");
    }

    // Стартовая поза после ребута: лучшего знания о железе нет
    // (сервы без обратной связи), поэтому считаем, что рука запаркована
    // дома (так уже считает дисплей). Нули здесь — худший вариант:
    // это недостижимый вылет 415мм, и первая же проверка траектории
    // валится на шаге 0 с ложным фолбэком на sequential.
    // После ребута всё равно рекомендуется G28.
    for (int i = 0; i < 7; i++) {
        currentAngles.angles[i] = homeAngles.angles[i];
    }
    interpolator.setCurrentAngles(currentAngles);

    interpolator.setStepCallback(onInterpolationStep);

    // Инициализация SharedState
    memset(&sharedState, 0, sizeof(sharedState));
    sharedState.safetyEnabled = safety.isEnabled();
    sharedState.pcaReady = servoCtrl.isPcaAvailable();
    sharedState.tcpPosition = fkSolver.getEndEffector(homeAngles.angles);
    for (int i = 0; i < 7; i++) {
        sharedState.servoAngles[i] = homeAngles.angles[i];
    }
    strncpy(sharedState.lastCommand, "-", sizeof(sharedState.lastCommand) - 1);

    // Создание задач
    xTaskCreatePinnedToCore(core0Task, "Core0_Motion",  8192, NULL, 1, &core0TaskHandle, 0);
    xTaskCreatePinnedToCore(core1Task, "Core1_Display", 8192, NULL, 1, &core1TaskHandle, 1);

    Serial.println("Ready. Send G-code commands.");
    Serial.println("Type '?' for help");
}

// ====================================================================
// Callback интерполятора
// ====================================================================
void onInterpolationStep(const JointAngles &angles) {
    if (!displayMutex) return;

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < 7; i++) {
            sharedState.servoAngles[i] = angles.angles[i];
        }
        float q[6];
        for (int i = 0; i < 6; i++) q[i] = angles.angles[i];
        sharedState.tcpPosition = fkSolver.getEndEffector(q);
        sharedState.gripperPercent = angles.angles[6] / config.servos[6].maxAngle * 100.0f;
        sharedState.isMoving = true;
        xSemaphoreGive(displayMutex);
    }
}

// ====================================================================
// Дисплей: цвета, зоны
// ====================================================================

// Цвета
#define COLOR_ZONE_HEADER_BG  0x000F   // navy (тёмно-синий)
#define COLOR_ZONE_HEADER_FG  ST77XX_CYAN
#define COLOR_FIELD_LABEL     ST77XX_WHITE
#define COLOR_TCP             ST77XX_GREEN
#define COLOR_JOINT           ST77XX_YELLOW
#define COLOR_GRIPPER         0xFDA0   // оранжевый
#define COLOR_READY           ST77XX_GREEN
#define COLOR_MOVING          ST77XX_YELLOW
#define COLOR_ERROR           ST77XX_RED
#define COLOR_STATUS_LABEL    ST77XX_WHITE
#define COLOR_STATUS_VALUE    ST77XX_CYAN

// Шрифт: setTextSize(2) = 16px высота строки
#define FONT_HEIGHT  16

// Координаты зон (Y начала)
#define ZONE_TCP_Y      0
#define ZONE_TCP_H      72
#define ZONE_JOINTS_Y   72
#define ZONE_JOINTS_H   104
#define ZONE_STATUS_Y   176
#define ZONE_STATUS_H   64

#define HEADER_H  16

// ====================================================================
// Дисплей: вспомогательные функции
// ====================================================================

/**
 * drawZoneHeader() — рисует цветной заголовок зоны.
 */
static void drawZoneHeader(int y, const char *title) {
    tft.fillRect(0, y, 240, HEADER_H, COLOR_ZONE_HEADER_BG);
    tft.setCursor(4, y + 2);
    tft.setTextColor(COLOR_ZONE_HEADER_FG, COLOR_ZONE_HEADER_BG);
    tft.print(title);
}

/**
 * drawField() — рисует строку: метка (xLabel) + число (xValue).
 * Формат числа "%4.0f%s " — целое, фиксированное поле 4 знака.
 * lastValue — антифилкер (не перерисовывать, если изменение < 0.5).
 */
static float lastTCP[6]    = {99999, 99999, 99999, 99999, 99999, 99999};
static float lastJoints[7] = {99999, 99999, 99999, 99999, 99999, 99999, 99999};
static float lastGrip      = -1;

static void drawField(int xLabel, int xValue, int y,
                      const char *label, float value, float &lastVal,
                      uint16_t color, const char *unit, bool forceRedraw = false) {
    if (!forceRedraw && fabs(value - lastVal) < 0.5f) return;
    lastVal = value;

    tft.setTextSize(2);

    // Метка слева
    tft.setTextColor(COLOR_FIELD_LABEL, ST77XX_BLACK);
    tft.setCursor(xLabel, y);
    tft.print(label);

    // Число — фиксированное поле, знак "-" не сдвигает цифры
    char buf[16];
    snprintf(buf, sizeof(buf), "%4.0f%s ", value, unit);
    tft.setTextColor(color, ST77XX_BLACK);
    tft.setCursor(xValue, y);
    tft.print(buf);
}

// ====================================================================
// Дисплей: displayInit() и displayUpdate()
// ====================================================================

/**
 * displayInit() — инициализация ST7789 дисплея.
 */
void displayInit() {
    tft.init(240, 240);
    tft.setSPISpeed(40000000);   // 40 МГц
    tft.setRotation(2);           // поворот 180°
    tft.fillScreen(ST77XX_BLACK);

    // Заставка: "6DOF" + "Initializing..."
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(3);
    tft.setCursor(40, 80);
    tft.println("  6DOF");
    tft.setTextSize(2);
    tft.setCursor(20, 120);
    tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    tft.println(" Initializing...");
    delay(2000);
    tft.fillScreen(ST77XX_BLACK);

    tft.setTextSize(2);

    // Заголовки зон
    drawZoneHeader(ZONE_TCP_Y,    "TCP POS.");
    drawZoneHeader(ZONE_JOINTS_Y, "JOINTS");

    // Инициализация last-значений
    for (int i = 0; i < 6; i++) lastTCP[i] = 99999;
    for (int i = 0; i < 7; i++) lastJoints[i] = 99999;
    lastGrip = -1;
}

/**
 * displayUpdate() — обновление данных на экране.
 * Вызывается из Core1 каждые 200 мс.
 */
void displayUpdate() {
    SharedState state;

    // Копирование SharedState под мьютексом
    if (displayMutex == NULL) return;
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    state = sharedState;
    xSemaphoreGive(displayMutex);

    tft.startWrite();

    // === TCP Position Zone (3 строки, +2 px от заголовка) ===
    drawField(4, 30, ZONE_TCP_Y + HEADER_H + 2,  "X:", state.tcpPosition.x, lastTCP[0], COLOR_TCP, "mm");
    drawField(4, 30, ZONE_TCP_Y + HEADER_H + 18, "Y:", state.tcpPosition.y, lastTCP[1], COLOR_TCP, "mm");
    drawField(4, 30, ZONE_TCP_Y + HEADER_H + 34, "Z:", state.tcpPosition.z, lastTCP[2], COLOR_TCP, "mm");

    // === Joints Zone (4 строки × 2 колонки, +2 px от заголовка) ===
    drawField(4,   40,  ZONE_JOINTS_Y + HEADER_H + 2,  "P1:", state.servoAngles[0], lastJoints[0], COLOR_JOINT, "");
    drawField(124, 160, ZONE_JOINTS_Y + HEADER_H + 2,  "P4:", state.servoAngles[3], lastJoints[3], COLOR_JOINT, "");

    drawField(4,   40,  ZONE_JOINTS_Y + HEADER_H + 18, "P2:", state.servoAngles[1], lastJoints[1], COLOR_JOINT, "");
    drawField(124, 160, ZONE_JOINTS_Y + HEADER_H + 18, "P5:", state.servoAngles[4], lastJoints[4], COLOR_JOINT, "");

    drawField(4,   40,  ZONE_JOINTS_Y + HEADER_H + 34, "P3:", state.servoAngles[2], lastJoints[2], COLOR_JOINT, "");
    drawField(124, 160, ZONE_JOINTS_Y + HEADER_H + 34, "P6:", state.servoAngles[5], lastJoints[5], COLOR_JOINT, "");

    drawField(4, 100, ZONE_JOINTS_Y + HEADER_H + 50, "Gripper:", state.gripperPercent, lastGrip, COLOR_GRIPPER, "%");

    // === Status Bar (3 строки) ===
    int sy = ZONE_STATUS_Y + 8;

    // --- Строка 1: READY / MOVING / ERROR + счётчик справа ---
    tft.setTextSize(2);
    tft.setCursor(4, sy);
    if (state.hasError) {
        tft.setTextColor(COLOR_ERROR, ST77XX_BLACK);
        tft.print("! ERROR  ");
    } else if (state.isMoving) {
        tft.setTextColor(COLOR_MOVING, ST77XX_BLACK);
        tft.print("> MOVING ");
    } else {
        tft.setTextColor(COLOR_READY, ST77XX_BLACK);
        tft.print("READY    ");
    }

    // Счётчик прижат вправо
    char cntBuf[8];
    snprintf(cntBuf, sizeof(cntBuf), "%4u ", (unsigned)state.commandCount);
    int cntW = strlen(cntBuf) * 6 * 2;
    tft.setCursor(236 - cntW, sy);
    tft.setTextColor(COLOR_STATUS_VALUE, ST77XX_BLACK);
    tft.print(cntBuf);

    // --- Строка 2: Safe + PCA + ABS/REL ---
    sy += 16;
    tft.setCursor(4, sy);
    tft.setTextColor(COLOR_STATUS_LABEL, ST77XX_BLACK);
    tft.print("Safe:");
    tft.setTextColor(state.safetyEnabled ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
    tft.print(state.safetyEnabled ? "ON " : "OFF");

    tft.setCursor(98, sy);
    tft.setTextColor(COLOR_STATUS_LABEL, ST77XX_BLACK);
    tft.print("PCA:");
    tft.setTextColor(state.pcaReady ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
    tft.print(state.pcaReady ? "ON " : "OFF");

    const char *modeStr = absoluteMode ? "ABS " : "REL ";
    int modeW = 4 * 6 * 2;
    tft.setCursor(236 - modeW, sy);
    tft.setTextColor(COLOR_STATUS_VALUE, ST77XX_BLACK);
    tft.print(modeStr);

    // --- Строка 3: последняя команда ---
    // Формат: "G:" + команда (до 17 символов) + хвостовые пробелы
    // до фиксированной ширины 20 символов = 240 px при size=2.
    sy += 16;
    tft.setCursor(4, sy);
    tft.setTextColor(COLOR_STATUS_VALUE, ST77XX_BLACK);

    char cmdBuf[32];
    snprintf(cmdBuf, sizeof(cmdBuf), "G:%-17.17s", state.lastCommand);
    // Добиваем пробелами до 20 символов (240 px)
    int cmdW = strlen(cmdBuf) * 6 * 2;
    while (cmdW < 20 * 12 && strlen(cmdBuf) < sizeof(cmdBuf) - 1) {
        strcat(cmdBuf, " ");
        cmdW = strlen(cmdBuf) * 6 * 2;
    }
    tft.print(cmdBuf);

    tft.endWrite();
}


// ====================================================================
// loop() — пуста (вся работа делается задачами FreeRTOS)
// ====================================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}