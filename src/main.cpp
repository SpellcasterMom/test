/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-10-07
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
 *   Скорость: SERIAL_BAUD (921600 бод, см. config.h)
 *
 * Ручной пульт (штурвал EC11 + кнопки, все INPUT_PULLUP):
 *   ENC_A (штурвал) → GPIO 32
 *   ENC_B (штурвал) → GPIO 33
 *   NEXT (выбор P1..P7) → GPIO 25 (активный LOW)
 *   E-STOP → GPIO 26 (активный LOW, прерывание,
 *     только стоп движения, сервы под током)
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
 *   │ TCP POS.              FW: 8(1.6)     │  заголовок + версия справа
 *   ├─────────────────────────────────────┤  y=16
 *   │ X:   415mm    Roll:        90  │
 *   │ Y:     0mm    Pitch:        0  │
 *   │ Z:   160mm    Yaw:         90  │
 *   ├─────────────────────────────────────┤  y=72
 *   │ JOINTS                              │  заголовок
 *   ├─────────────────────────────────────┤  y=88
 *   │ P1:    0     P4:    0               │
 *   │ P2:  -40     P5:    0               │
 *   │ P3:    0     P6:    0               │
 *   │ Gripper:   50%                      │
 *   ├─────────────────────────────────────┤  y=176
 *   │ READY           ABS            123  │  статус + режим + счётчик
 *   │ Safe:ON     PCA:●     GYR:●          │  режимы текстом; кружки: PCA зел/красн, GYR зел=гиро+аксель, жёлт=только гиро, красн=нет гиро
 *   │ G28                                 │  последняя команда
 *   └─────────────────────────────────────┘  y=240
 *
 * =====================================================================
 * ДЕМОНСТРАЦИЯ: Endpoint Control — TCP=(200,200,200) стоит,
 * инструмент идёт +X -> вниз (12 шагов G1, раннер demo_endpoint.py;
 * просчитаны в двойнике: dPos<=1мм на всех шагах).
 * =====================================================================
 *
 * G90
 * G1 X200 Y200 Z200 A90.0 B-0.0 C90.0 F300
 * G1 X200 Y200 Z200 A96.7 B5.9 C96.7 F300
 * G1 X200 Y200 Z200 A104.1 B11.0 C104.1 F300
 * G1 X200 Y200 Z200 A112.2 B15.0 C112.2 F300
 * G1 X200 Y200 Z200 A121.0 B17.8 C121.0 F300
 * G1 X200 Y200 Z200 A130.3 B19.3 C130.3 F300
 * G1 X200 Y200 Z200 A139.7 B19.3 C139.7 F300
 * G1 X200 Y200 Z200 A149.0 B17.8 C149.0 F300
 * G1 X200 Y200 Z200 A157.8 B15.0 C157.8 F300
 * G1 X200 Y200 Z200 A165.9 B11.0 C165.9 F300
 * G1 X200 Y200 Z200 A173.3 B5.9 C173.3 F300
 * G1 X200 Y200 Z200 A-180.0 B-0.0 C-180.0 F300
 */

#include <Arduino.h>
// #include <TFT_eSPI.h>
#include <LovyanGFX.hpp>
#include <SPI.h>
#include <Wire.h>
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
#include "jog_pendant.h"
#include "detour_planner.h"
#include "eeprom_storage.h"
#include "imu_logger.h"

// Пины SPI для ST7789 заданы в классе LGFX ниже (VSPI).
// Отдельные #define не нужны.

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
JogPendant           pendant;   // ручной пульт: штурвал + NEXT + E-STOP
DetourPlanner        detour;    // объезд запретных зон: план вейпоинтов
ImuLogger            imuLogger; // GY-80: опрос 200 Гц + кольцо последнего движения

// Мьютекс общей шины I2C (PCA9685 на Core 0 + GY-80 на Core 1).
// Создаётся первым в setup; брать только вокруг коротких транзакций.
SemaphoreHandle_t    i2cMutex = NULL;

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

// ===== Дисплей ST7789 (LovyanGFX, DMA) =====
// Экран 1.54" 240×240. Пины заданы здесь (VSPI-родные: MOSI=23, SCLK=18).
// Ручки под конкретную партию стекла (если картинка сдвинута/инвертирована):
//   offset_rotation / invert / rgb_order ниже.
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI      _bus_instance;
public:
    LGFX(void) {
        {
            auto cfg = _bus_instance.config();
            // Хост: SPI2_HOST (HSPI). Макрос VSPI (=3) spi_bus_initialize
            // отвергает (invalid host_id) — берём IDF-enum напрямую.
            // Пины заданы ниже явно, матрица дотянет их до любого хоста.
            cfg.spi_host   = SPI2_HOST;
            cfg.spi_mode   = 0;
            cfg.freq_write = 40000000;
            cfg.freq_read  = 16000000;
            cfg.pin_sclk   = 18;
            cfg.pin_mosi   = 23;
            cfg.pin_miso   = -1;
            cfg.pin_dc     = 2;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs         = 5;
            cfg.pin_rst        = 4;
            cfg.panel_width    = 240;
            cfg.panel_height   = 240;
            cfg.offset_rotation = 0;
            cfg.readable       = false;
            cfg.invert         = true;    // ST7789: без инверсии цвета перевёрнуты
            cfg.rgb_order      = false;
            _panel_instance.config(cfg);
        }
        setPanel(&_panel_instance);
    }
};
LGFX tft;

// ===== Структура общего состояния для дисплея =====
struct SharedState {
    Vec3        tcpPosition;      // Текущая позиция TCP (из FK)
    float       servoAngles[7];   // Углы 7 сервоприводов (градусы)
    float       gripperPercent;   // Процент открытия захвата (0-100%)
    bool        isMoving;         // Флаг движения
    bool        isCalibrating;    // Идёт калибровка IMU (дисплей CALIB)
    bool        safetyEnabled;    // Состояние безопасности
    bool        pcaReady;         // PCA9685 найден на I2C
    uint8_t     imuState;         // GYR: 0=нет гиро (красный), 1=гиро без акселя (жёлтый), 2=гиро+аксель (зелёный)
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
    float minZ = 0;         // минимальная Z цепочки J1..J6 (для высотного штрафа)
    bool safe = false;      // прошёл safety.checkAllLinks
    char safeErr[48];       // причина отказа safety
    int iters = 0;          // итераций DLS-уточнения
    bool converged = false; // DLS сошлось по допускам
    float seedPos = 0;      // расхождение позиции у seed'а (мм)
    float seedOri = 0;      // расхождение ориентации у seed'а (град)
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
    Vec3 detourPts[8];      // TCP вейпоинтов планировщика объезда
    int detourCount = 0;    // сколько их (0 = прямо/нет плана)
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
    Mat4 T06dummy;
    fkSolver.getPose(qq, s.joints, s.tcp, s.dir, T06dummy);
    (void)T06dummy;
    float dx = s.tcp.x - target.position.x;
    float dy = s.tcp.y - target.position.y;
    float dz = s.tcp.z - target.position.z;
    s.dPos = sqrt(dx*dx + dy*dy + dz*dz);
    s.dOri = diagOriErr(s.dir, diagToolDir(target.orientation));
    // Минимальная высота цепочки J2..J6 (без J0/J1: база и плечо всегда
    // высоко — сигнал «утопленника» несут локоть и ниже).
    // Высотный штраф в выборе предпочитает elbow-up: утопленник с локтем
    // на z=17 проигрывает высокой ветви, вверху штраф нулевой.
    s.minZ = s.joints[2].z;
    for (int i = 3; i <= 6; i++) {
        if (s.joints[i].z < s.minZ) s.minZ = s.joints[i].z;
    }
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
    // Пакет M450 копит только G1: движение тут разорвало бы траекторию.
    if (interpolator.batchOpen()) {
        Serial.println("ERROR: batch open - close it first (M450 S0)");
        return;
    }
    cmd.determineMode();

    // Сброс ошибки при новой команде
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    // Эффективная поза: без A/B/C берём дефолт «палец вперёд +X»
    // (DEFAULT_ORI_*). M115 покажет её же с пометкой (orient=default).
    Pose effPose = cmd.pose;
    if (!cmd.hasOrientation) {
        effPose.orientation.x = DEFAULT_ORI_A;
        effPose.orientation.y = DEFAULT_ORI_B;
        effPose.orientation.z = DEFAULT_ORI_C;
    }

    // Поосевая семантика координат (чинило молчаливый G91):
    //   - отсутствующая ось = держать текущую (а не 0!);
    //   - в REL (G91) заданная ось = текущая TCP + дельта.
    // Без этого G91 игнорировался (ехали в абсолют), а частичный
    // G0 X20 в ABS уводил Y,Z в ноль. A/B/C в REL — абсолютные.
    {
        float cq[6];
        for (int i = 0; i < 6; i++) cq[i] = currentAngles.angles[i];
        Vec3 curTcp = fkSolver.getEndEffector(cq);
        if (!absoluteMode) {
            effPose.position.x = cmd.hasX ? curTcp.x + cmd.pose.position.x : curTcp.x;
            effPose.position.y = cmd.hasY ? curTcp.y + cmd.pose.position.y : curTcp.y;
            effPose.position.z = cmd.hasZ ? curTcp.z + cmd.pose.position.z : curTcp.z;
        } else {
            if (!cmd.hasX) effPose.position.x = curTcp.x;
            if (!cmd.hasY) effPose.position.y = curTcp.y;
            if (!cmd.hasZ) effPose.position.z = curTcp.z;
        }
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
                interpolator.setMark("NONE", "no motion (IK refused)");
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

            // Кандидат от текущей позы (трекинг при стриминге мелких шагов):
            // DLS из current сходится за ~2 итерации (двойник: 40/40 из ±10°).
            // Общий выбор min-residual сам предпочтёт его, когда он хорош.
            // Слот в diagLast.sols есть (максимум 4, solveAll даёт ≤3).
            if (diagLast.solCount < 4 && solCount > 0) {
                JointAngles curSeed = currentAngles;
                IKSolutionMeta curMeta;
                ikSolver.refineSolution(effPose, curSeed, &curMeta);
                if (cmd.hasGripper) {
                    curSeed.angles[6] = cmd.pose.gripper;
                }
                diagEvalSol(diagLast.sols[diagLast.solCount], curSeed, effPose);
                diagLast.sols[diagLast.solCount].iters = curMeta.iters;
                diagLast.sols[diagLast.solCount].converged = curMeta.converged;
                diagLast.sols[diagLast.solCount].seedPos = curMeta.seedPos;
                diagLast.sols[diagLast.solCount].seedOri = curMeta.seedOri;
                diagLast.sols[diagLast.solCount].dbgDq0 = curMeta.dbgDq0;
                diagLast.sols[diagLast.solCount].dbgN1 = curMeta.dbgN1;
                diagLast.solCount++;
            }

            // Выбираем безопасное решение с минимальной расхождением:
            // score = dPos[мм] + 1.0·dOri[град] + CONT_W·jump[град от текущей].
            // Чистый min-residual выбирал ветвь в 250° ради 0.3 мм
            // (двойник: круг) — резкие перебросы без необходимости. CONT_W=0.02:
            // 50° прыжок окупается лишь при +1мм/+1° точности.
            // «Первый безопасный» систематически брал худший вариант
            // (M115: [0] 2.7мм/23.8° вместо [1] 2.2мм/0.0°).
            int best = -1;
            float bestScore = 0;
            // Порог высотного штрафа: ниже — предпочитаем верхние ветви.
            // Срезка min(15): в честно-низкой работе все ветви внизу,
            // штраф у всех максимальный и одинаковый — решает расхождение.
            const float HEIGHT_Z_REF = 150.0f;   // мм
            const float HEIGHT_W = 0.15f;        // штрафа за мм ниже порога
            const float HEIGHT_MAX = 15.0f;      // потолок штрафа
            for (int i = 0; i < diagLast.solCount; i++) {
                if (!diagLast.sols[i].safe) continue;
                float jd = 0;
                for (int j = 0; j < 6; j++) {
                    float d = fabs(diagLast.sols[i].q.angles[j] - currentAngles.angles[j]);
                    if (d > jd) jd = d;
                }
                float hPen = diagLast.sols[i].minZ < HEIGHT_Z_REF
                    ? HEIGHT_W * (HEIGHT_Z_REF - diagLast.sols[i].minZ) : 0.0f;
                if (hPen > HEIGHT_MAX) hPen = HEIGHT_MAX;
                float score = diagLast.sols[i].dPos + diagLast.sols[i].dOri + 0.02f * jd + hPen;
                if (best < 0 || score < bestScore) {
                    best = i;
                    bestScore = score;
                }
            }
            bool foundSafe = (best >= 0);
            // Порог точности: лучший безопасный кандидат с расхождением хуже
            // порога — это уже не заданное движение (напр. 127 мм мимо из-за
            // отбраковки точных, но опасных). Отказываем вместо прыжка в мусор.
            // Рабочие ходы идут с dPos<1мм/dOri в единицы градусов (двойник+железо).
            const float IK_POS_TOL = 10.0f;   // мм
            const float IK_ORI_TOL = 15.0f;   // град
            if (foundSafe &&
                (diagLast.sols[best].dPos > IK_POS_TOL ||
                 diagLast.sols[best].dOri > IK_ORI_TOL)) {
                foundSafe = false;
                char tmp[96];
                snprintf(tmp, sizeof(tmp),
                         "No accurate safe IK (best dPos=%.1f dOri=%.1f)",
                         diagLast.sols[best].dPos, diagLast.sols[best].dOri);
                strncpy(diagLast.failReason, tmp, sizeof(diagLast.failReason) - 1);
            }
            if (foundSafe) {
                target = diagLast.sols[best].q;
                diagLast.chosen = best;
            }

            if (!foundSafe) {
                // Не затираем причину, уже записанную порогом точности выше
                // ("No accurate safe IK..."), — иначе диагностика врёт.
                if (diagLast.failReason[0] == '\0') {
                    strncpy(diagLast.failReason, "No safe IK solution",
                            sizeof(diagLast.failReason) - 1);
                }
                interpolator.setMark("NONE", "no motion (refused)");
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
    // Объезд препятствий: планировщик даёт цепочку joint-вейпоинтов
    // (последний = target), едем каждый обычным moveTo. Сам интерполятор
    // не тронут. DIRECT-режиму поза ни к чему — только RRT (nullptr).
    float qs0[6];
    for (int i = 0; i < 6; i++) qs0[i] = currentAngles.angles[i];
    const Pose *pp = (cmd.mode == GCodeCommand::DIRECT_MODE) ? nullptr : &effPose;
    JointAngles wps[8];
    int nwp = detour.plan(qs0, target, pp, wps, 8);
    // TCP вейпоинтов в журнал (видны в M115, даже если движение сорвётся).
    diagLast.detourCount = 0;
    for (int w = 0; w < nwp && w < 8; w++) {
        float qw[6];
        for (int i = 0; i < 6; i++) qw[i] = wps[w].angles[i];
        diagLast.detourPts[diagLast.detourCount++] = fkSolver.getEndEffector(qw);
    }
    bool moved = false;
    if (nwp <= 0) {
        strncpy(diagLast.failReason, "No path: obstacle blocks all routes",
                sizeof(diagLast.failReason) - 1);
    } else {
        moved = true;
        for (int w = 0; w < nwp && moved; w++) {
            if (!interpolator.moveTo(wps[w], feedrate)) moved = false;
        }
        if (!moved) {
            strncpy(diagLast.failReason, "Move failed on detour leg",
                    sizeof(diagLast.failReason) - 1);
        }
    }
    if (moved) {
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
        // Не затираем конкретную причину (nopath / detour leg),
        // записанную выше.
        if (diagLast.failReason[0] == '\0') {
            strncpy(diagLast.failReason, "Move failed (interpolator)",
                    sizeof(diagLast.failReason) - 1);
        }
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: Move failed");
    }
}

/**
 * G1 — декартово движение (прямая TCP).
 * Чистый IK-режим: TCP идёт по прямой (lerp + slerp), IK на каждом шаге.
 * DIRECT/MIXED — как G0 (joint-интерполяция).
 */
void handleG1(GCodeCommand &cmd) {
    cmd.determineMode();
    if (cmd.mode != GCodeCommand::IK_MODE) {
        handleG0(cmd);
        return;
    }

    // Сброс ошибки при новой команде
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    // Эффективная поза: как в G0 (дефолт ориентации + поосевая семантика).
    Pose effPose = cmd.pose;
    if (!cmd.hasOrientation) {
        effPose.orientation.x = DEFAULT_ORI_A;
        effPose.orientation.y = DEFAULT_ORI_B;
        effPose.orientation.z = DEFAULT_ORI_C;
    }
    {
        float cq[6];
        for (int i = 0; i < 6; i++) cq[i] = currentAngles.angles[i];
        Vec3 curTcp = fkSolver.getEndEffector(cq);
        if (!absoluteMode) {
            effPose.position.x = cmd.hasX ? curTcp.x + cmd.pose.position.x : curTcp.x;
            effPose.position.y = cmd.hasY ? curTcp.y + cmd.pose.position.y : curTcp.y;
            effPose.position.z = cmd.hasZ ? curTcp.z + cmd.pose.position.z : curTcp.z;
        } else {
            if (!cmd.hasX) effPose.position.x = curTcp.x;
            if (!cmd.hasY) effPose.position.y = curTcp.y;
            if (!cmd.hasZ) effPose.position.z = curTcp.z;
        }
    }
    // Захват: как в G0 (без J — в 0).
    effPose.gripper = cmd.hasGripper ? cmd.pose.gripper : 0;

    // Валидация конца пути (те же ворота, что G0): solveAll + min-residual
    // + порог 10мм/15°. Сам путь считает moveToPose.
    // В цепочке блендинга (прошлый G1 был с Q1 и доехал) стартовый рестарт
    // пропускаем: трекинг от qPrev непрерывен, ворота проверяются ПОСЛЕ
    // движения пост-гейтом ниже. Диагностика сидов в цепочке не пишется
    // (останова нет — мерить нечего).
    const float IK_POS_TOL = 10.0f;
    const float IK_ORI_TOL = 15.0f;
    bool chainIn = interpolator.chainActive();
    bool blendOut = cmd.hasParam['Q' - 'A'] && cmd.params['Q' - 'A'] != 0.0f;
    bool inBatch = interpolator.batchOpen();
    // Опор непрерывности для min-residual: в пакете — последняя накопленная
    // точка (currentAngles стоит: движения нет), иначе — текущая поза.
    // Вне пакета копия равна currentAngles — поведение бит-в-бит как раньше.
    JointAngles contRef = currentAngles;
    if (inBatch) interpolator.batchPrevPt(contRef);
    // Выборка для пакета (заполняется внутри валидации ниже).
    JointAngles batchSel;
    float batchRdp = 0.0f, batchRdo = 0.0f;
    diagBegin("IK-CART", cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE);
    diagLast.target = effPose;
    diagLast.hasPos = cmd.hasPosition;
    diagLast.hasOri = cmd.hasOrientation;
    if (!chainIn) {
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

    int best = -1;
    float bestScore = 0;
    // Порог высотного штрафа: ниже — предпочитаем верхние ветви.
    // Срезка min(15): в честно-низкой работе все ветви внизу,
    // штраф у всех максимальный и одинаковый — решает расхождение.
    const float HEIGHT_Z_REF = 150.0f;   // мм
    const float HEIGHT_W = 0.15f;        // штрафа за мм ниже порога
    const float HEIGHT_MAX = 15.0f;      // потолок штрафа
    for (int i = 0; i < diagLast.solCount; i++) {
        if (!diagLast.sols[i].safe) continue;
        float jd = 0;
        for (int j = 0; j < 6; j++) {
            float d = fabs(diagLast.sols[i].q.angles[j] - contRef.angles[j]);
            if (d > jd) jd = d;
        }
        float hPen = diagLast.sols[i].minZ < HEIGHT_Z_REF
            ? HEIGHT_W * (HEIGHT_Z_REF - diagLast.sols[i].minZ) : 0.0f;
        if (hPen > HEIGHT_MAX) hPen = HEIGHT_MAX;
        float score = diagLast.sols[i].dPos + diagLast.sols[i].dOri + 0.02f * jd + hPen;
        if (best < 0 || score < bestScore) {
            best = i;
            bestScore = score;
        }
    }
    bool foundSafe = (best >= 0);
    if (foundSafe &&
        (diagLast.sols[best].dPos > IK_POS_TOL ||
         diagLast.sols[best].dOri > IK_ORI_TOL)) {
        foundSafe = false;
        char tmp[96];
        snprintf(tmp, sizeof(tmp),
                 "No accurate safe IK (best dPos=%.1f dOri=%.1f)",
                 diagLast.sols[best].dPos, diagLast.sols[best].dOri);
        strncpy(diagLast.failReason, tmp, sizeof(diagLast.failReason) - 1);
    }
    if (!foundSafe) {
        if (diagLast.failReason[0] == '\0') {
            strncpy(diagLast.failReason, "No safe IK solution",
                    sizeof(diagLast.failReason) - 1);
        }
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "No safe IK");
        xSemaphoreGive(displayMutex);
        Serial.print("ERROR: ");
        Serial.println(diagLast.failReason);
        return;
    }
    diagLast.chosen = best;
        // Пакет: запоминаем выбранное для накопления ниже (цепочка и пакет
        // взаимоисключают друг друга: сюда в цепочке не попадаем).
        if (inBatch) {
            batchSel = solutions[best];
            batchRdp = diagLast.sols[best].dPos;
            batchRdo = diagLast.sols[best].dOri;
        }
    }  // if (!chainIn): в цепочке стартовый solveAll/диагностика пропущены

    // Пакет M450: точка прошла те же ворота — копим вместо движения.
    // Движения нет, поэтому OK здесь = «принято и провалидировано».
    // chainIn здесь невозможен (цепочка и пакет взаимоисключают друг друга),
    // но нули в буфер страшнее лишней проверки: страхуемся явно.
    if (inBatch && chainIn) {
        interpolator.clearChain();
        interpolator.batchAbort();
        Serial.println("ERROR: chain meets batch (internal), both reset");
        return;
    }
    if (inBatch) {
        float feed = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
        if (!interpolator.batchAdd(batchSel, feed, batchRdp, batchRdo)) {
            interpolator.batchAbort();
            xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
            sharedState.hasError = true;
            snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Batch full");
            xSemaphoreGive(displayMutex);
            Serial.println("ERROR: batch full (64 max), flushed - send in chunks");
            return;
        }
        Serial.printf("OK: buffered %d/%d\n",
                      interpolator.batchCount(), MotionInterpolator::BATCH_MAX);
        return;
    }

    // Декартово исполнение: TCP по прямой, IK на каждом шаге.
    float feedrate = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
    interpolator.setCurrentAngles(currentAngles);
    if (interpolator.moveToPose(effPose, feedrate, blendOut)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        // currentAngles уже обновлены внутри moveToPose; забираем итог.
        JointAngles done = interpolator.getCurrentAngles();
        currentAngles = done;
        xSemaphoreGive(angleMutex);

        // CHOSEN для декарта: не min-residual (движение трекингом от текущей,
        // а не прыжок в выбранное), а ближайший безопасный кандидат к факту.
        {
            int near = -1;
            float nearD = 0;
            for (int i = 0; i < diagLast.solCount; i++) {
                if (!diagLast.sols[i].safe) continue;
                float dd = 0;
                for (int j = 0; j < 6; j++) {
                    float d = fabs(diagLast.sols[i].q.angles[j] - done.angles[j]);
                    if (d > dd) dd = d;
                }
                if (near < 0 || dd < nearD) { near = i; nearD = dd; }
            }
            if (near >= 0) diagLast.chosen = near;
        }

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

        // Пост-гейт цепочки: пред-валидации не было — проверяем факт.
        // currentAngles уже = done (честная позиция), цепочку рвём.
        if (chainIn) {
            DiagSol gate;
            diagEvalSol(gate, done, effPose);
            if (gate.dPos > IK_POS_TOL || gate.dOri > IK_ORI_TOL) {
                interpolator.clearChain();
                char tmp[96];
                snprintf(tmp, sizeof(tmp),
                         "chain residual too big (dPos=%.1f dOri=%.1f)",
                         gate.dPos, gate.dOri);
                xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
                sharedState.hasError = true;
                snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Chain residual");
                xSemaphoreGive(displayMutex);
                Serial.print("ERROR: ");
                Serial.println(tmp);
                return;
            }
        }

        Serial.println("OK");
    } else {
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Cart move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: Cartesian move failed (see M115 INTERP)");
    }
}

/**
 * G28 — перемещение в домашнюю позицию (home).
 */
void handleG28(GCodeCommand &cmd) {
    if (interpolator.batchOpen()) {
        Serial.println("ERROR: batch open - close it first (M450 S0)");
        return;
    }
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    JointAngles target;
    bool park = cmd.hasParam['H' - 'A'] && (int)cmd.params['H' - 'A'] == 1;

    if (park) {
        // Компактная сложенная поза (единый источник — DEFAULT_PARK_ANGLES,
        // точные углы даст владелец). Старое (0,-60,-90) вело локоть ПОД СТОЛ!
        for (int i = 0; i < 7; i++) {
            target.angles[i] = DEFAULT_PARK_ANGLES[i];
        }
    } else {
        for (int i = 0; i < 7; i++) {
            target.angles[i] = homeAngles.angles[i];
        }
    }

    diagBegin(park ? "PARK" : "HOME", 300.0f);
    diagLast.anglesOnly = true;

    // Не едем вслепую: цель паркинга/дома обязана проходить safety.
    // При выключенной безопасности checkAllLinks всегда true.
    if (safety.isEnabled() && !safety.checkAllLinks(target.angles)) {
        strncpy(diagLast.failReason, safety.getLastError().c_str(),
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Bad home/park");
        xSemaphoreGive(displayMutex);
        Serial.print("ERROR: Home/park target unsafe: ");
        Serial.println(safety.getLastError());
        interpolator.setMark("NONE", "no motion (refused)");
        return;
    }
    diagEvalSol(diagLast.sols[0], target, cmd.pose);
    diagLast.solCount = 1;
    diagLast.chosen = 0;

    // Старт — из лучшего известного положения (см. handleG0).
    // Подача: из F (G28 F15 — тихо), иначе 300 как раньше.
    interpolator.setCurrentAngles(currentAngles);
    float homingFeed = cmd.hasFeedrate ? cmd.feedrate : 300.0f;
    if (interpolator.moveTo(target, homingFeed)) {
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
 * M104 — установка углов сервоприводов ЧЕРЕЗ интерполятор.
 * Едет плавно (ease-in-out), с лимитами угловых скоростей и проверкой
 * safety — как G0, только цель задана углами, а не позой TCP.
 * Неуказанные сервы остаются на месте. Параметр F — скорость (%), по
 * умолчанию DEFAULT_FEEDRATE.
 */
void handleM104(GCodeCommand &cmd) {
    if (interpolator.batchOpen()) {
        Serial.println("ERROR: batch open - close it first (M450 S0)");
        return;
    }
    JointAngles target = currentAngles;
    for (int i = 0; i < 7; i++) {
        if (cmd.servoOverride[i]) {
            target.angles[i] = cmd.servoAngles.angles[i];
        }
    }
    float feedrate = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
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

        diagBegin("DIRECT", 0);
        diagLast.anglesOnly = true;
        diagEvalSol(diagLast.sols[0], target, cmd.pose);
        diagLast.solCount = 1;
        diagLast.chosen = 0;
        Serial.println("OK");
    } else {
        strncpy(diagLast.failReason, "Move failed (interpolator/safety)",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "M104 move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: M104 move failed (safety/interpolator)");
    }
}

/**
 * M105 — вывод текущих углов сервоприводов.
 */
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

/**
 * M108 — захват в процентах: M108 S=50 (0=закрыт, 100=открыт).
 * P7 живёт в math-шкале 0..100, поэтому процент = угол 1:1.
 * Едет только сустав 6 через тот же interpolator, что M104.
 */
void handleM108(GCodeCommand &cmd) {
    if (interpolator.batchOpen()) {
        Serial.println("ERROR: batch open - close it first (M450 S0)");
        return;
    }
    const int SI = 'S' - 'A';
    if (!cmd.hasParam[SI]) {
        strncpy(diagLast.failReason, "M108 requires S (0-100)",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "M108 needs S=0..100");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: M108 requires S (0-100)");
        return;
    }
    float pct = cmd.params[SI];
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    JointAngles target = currentAngles;
    target.angles[6] = pct;
    float feedrate = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
    interpolator.setCurrentAngles(currentAngles);
    if (interpolator.moveTo(target, feedrate)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.servoAngles[6] = target.angles[6];
        sharedState.gripperPercent = pct;
        sharedState.isMoving = false;
        sharedState.hasError = false;
        sharedState.commandCount++;
        xSemaphoreGive(displayMutex);

        diagBegin("DIRECT", 0);
        diagLast.anglesOnly = true;
        diagEvalSol(diagLast.sols[0], target, cmd.pose);
        diagLast.solCount = 1;
        diagLast.chosen = 0;
        Serial.printf("OK: Gripper %.0f%%\n", pct);
    } else {
        strncpy(diagLast.failReason, "Move failed (interpolator/safety)",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "M108 move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: M108 move failed (safety/interpolator)");
    }
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
 * выбранное решение, куда пришли (факт), расхождение.
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
    Serial.printf("KIND: %s FEED: %.0f SAFETY: %s FINGER: %.1f MODE: %s FW:%s\n",
                  diagLast.kind, diagLast.feedrate,
                  safety.isEnabled() ? "ON" : "OFF",
                  config.fingerLength,
                  absoluteMode ? "ABS" : "REL",
                  FW_VERSION_STR);
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
        Serial.printf(" CLAMP:%s minZ=%.0f", clamp[0] ? clamp : "-", s.minZ);
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
        else if (strcmp(diagLast.kind, "IK-CART") == 0) how = "cart-tracked";
        else if (strcmp(diagLast.kind, "MIXED") == 0) how = "single";
        Serial.printf("CHOSEN: [%d] (%s)\n", diagLast.chosen, how);
    } else {
        Serial.printf("FAIL: %s\n", diagLast.failReason);
    }
    // Вейпоинты планировщика объезда (TCP-точки пути).
    if (diagLast.detourCount > 0) {
        Serial.printf("WPS: %d\n", diagLast.detourCount);
        for (int i = 0; i < diagLast.detourCount; i++) {
            Serial.printf("  [%d] X=%.1f Y=%.1f Z=%.1f\n", i,
                          diagLast.detourPts[i].x,
                          diagLast.detourPts[i].y,
                          diagLast.detourPts[i].z);
        }
    }
    // Пакет M450: плановые расхождения точек (замерены при накоплении,
    // рантайм детерминирован — перемерять нечего). Пусто = пакета не было.
    {
        int bn = interpolator.batchCount();
        if (bn > 0) {
            Serial.printf("BATCH: %d pts\n", bn);
            for (int i = 0; i < bn; i++) {
                Serial.printf("  [%d] dPos=%.1fmm dOri=%.1fdeg\n", i,
                              interpolator.batchResPos(i),
                              interpolator.batchResOri(i));
            }
        }
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
        if (interpolator.batchOpen()) {
            Serial.println("ERROR: batch already open (M450 S0/S2 first)");
            return;
        }
        interpolator.batchBegin();
        Serial.println("OK: batch open (64 max)");
    } else if (s == 0) {
        if (!interpolator.batchOpen()) {
            Serial.println("ERROR: no open batch (M450 S1 first)");
            return;
        }
        if (interpolator.batchCount() == 0) {
            interpolator.batchAbort();
            Serial.println("ERROR: empty batch, flushed");
            return;
        }
        int n = interpolator.batchCount();
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
            Serial.printf("OK: batch done (%d pts)\n", n);
        } else {
            xSemaphoreTake(angleMutex, portMAX_DELAY);
            JointAngles done = interpolator.getCurrentAngles();
            currentAngles = done;
            xSemaphoreGive(angleMutex);
            xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
            sharedState.hasError = true;
            snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Batch failed");
            xSemaphoreGive(displayMutex);
            Serial.println("ERROR: batch failed (see M115 INTERP)");
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
// pendantTask — опрос ручного пульта (штурвал + кнопка NEXT).
// E-STOP идёт мимо задачи (прерывание → interpolator.stop()).
// ====================================================================
void pendantTask(void *pvParameters) {
    (void)pvParameters;
    while (true) {
        pendant.poll();
        vTaskDelay(pdMS_TO_TICKS(20));
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
// GY-80 на общей I2C (SDA=21/SCL=22, Wire уже поднят servoCtrl.init()).
// Возвращает битовую маску найденных чипов: бит0=гиро L3G4200D (0x68/0x69 —
// зависит от запайки SDO), бит1=аксель ADXL345 (0x53),
// бит2=магнит HMC5883L (0x1E), бит3=баро BMP180 (0x77).
// Рабочий минимум для наклона и guard'а — гиро+аксель (маска 0x03);
// магнит нужен только для 9-DOF yaw, баро руке не нужно вовсе.
static int gy80Mask = 0;

static bool i2cPing(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// Состояние GYR-кружка по маске scanGY80():
// 2 = гиро+аксель (зелёный), 1 = только гиро (жёлтый), 0 = нет гиро (красный).
// Магнит для кружка не нужен (только для будущего 9-DOF yaw).
static uint8_t gy80State() {
    if (!(gy80Mask & 0x01)) return 0;
    return (gy80Mask & 0x02) ? 2 : 1;
}

static int scanGY80() {
    int mask = 0;
    if (i2cPing(0x68) || i2cPing(0x69)) mask |= 0x01;  // гиро
    if (i2cPing(0x53)) mask |= 0x02;                   // аксель
    if (i2cPing(0x1E)) mask |= 0x04;                   // магнит
    if (i2cPing(0x77)) mask |= 0x08;                   // баро
    return mask;
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    // Инициализация дисплея ПЕРЕД I2C/PCA9685
    displayInit();

    Serial.println("=== 6-DOF Robot Arm Controller ===");
    Serial.println("ESP32 @ 240 MHz, PCA9685, 7 servos");

    // Создание мьютексов ДО первого использования
    angleMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();  // шина Wire: сервы + GY-80

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

    // Поиск GY-80 (железо воткнуто или нет — драйвер позже, см. TODO).
    // Кружок: зелёный = гиро+аксель, жёлтый = только гиро, красный = нет гиро.
    gy80Mask = scanGY80();
    xSemaphoreTake(displayMutex, portMAX_DELAY);
    sharedState.imuState = gy80State();
    xSemaphoreGive(displayMutex);
    Serial.printf("GY-80: gyro=%c acc=%c mag=%c baro=%c\n",
                  (gy80Mask & 0x01) ? '+' : '-',
                  (gy80Mask & 0x02) ? '+' : '-',
                  (gy80Mask & 0x04) ? '+' : '-',
                  (gy80Mask & 0x08) ? '+' : '-');

    // IMU-логгер: драйвер по маске найденного + указатель на интерполятор
    // (запись идёт пока isMoving). Задача стартует после калибровки ниже.
    imuLogger.begin(gy80Mask);
    imuLogger.setInterpolator(&interpolator);

    // Связывание интерполятора с зависимостями
    interpolator.setServoController(&servoCtrl);
    interpolator.setBacklashCompensator(&backlash);
    interpolator.setSafetyChecker(&safety);
    interpolator.setIKSolver(&ikSolver);
    interpolator.setFKSolver(&fkSolver);
    interpolator.setStrategy(interpStrategy);
    detour.setIKSolver(&ikSolver);
    detour.setFKSolver(&fkSolver);
    detour.setSafetyChecker(&safety);
    detour.setServoController(&servoCtrl);

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

    // Версия прошивки в EEPROM: залита новая (или EEPROM пуст/старого
    // формата — тогда getStoredFwVersion()=0) → заводские + сохранить + ребут.
    // Иначе старые DH/палец/лимиты/home из EEPROM тихо ломали бы новую логику.
    // Home при сбросе = все сервы в 0 (DEFAULT_HOME_ANGLES).
    {
        uint32_t storedVer = storage.getStoredFwVersion();
        if (storedVer != FW_VERSION) {
            Serial.printf("FW version %lu -> %lu: factory defaults + save + reboot\n",
                          (unsigned long)storedVer, (unsigned long)FW_VERSION);
            config = DEFAULT_CONFIG;
            for (int i = 0; i < 7; i++) {
                homeAngles.angles[i] = DEFAULT_HOME_ANGLES[i];
            }
            servoCtrl.setConfig(config.servos);
            ikSolver.setServoLimits(config.servos);
            ikSolver.setFingerLength(config.fingerLength);
            fkSolver.setFingerLength(config.fingerLength);
            safety.setFingerLength(config.fingerLength);
            storage.saveConfig(config, homeAngles.angles, interpStrategy);
            delay(200);
            ESP.restart();
        }
        Serial.printf("FW version %lu (%s) OK\n",
                      (unsigned long)FW_VERSION, FW_VERSION_STR);
    }

    // Запретные зоны: конфиг -> рантайм safety, плюс конструктив.
    // Зоны живут только в рантайме: переносим сохранённые, затем проверяем
    // конструктив колонны (DEFAULT_CONSTRUCT_ZONE): нет такой — добавляем
    // и сохраняем в EEPROM (чтобы была сразу забита и переживала ребут).
    safety.clearForbiddenZones();
    for (int i = 0; i < config.forbiddenZoneCount && i < MAX_FORBIDDEN_ZONES; i++) {
        const ForbiddenZone &z = config.forbiddenZones[i];
        if (z.active) {
            safety.addForbiddenZone(z.xMin, z.xMax, z.yMin, z.yMax, z.zMin, z.zMax,
                                    z.skipLinks);
        }
    }
    {
        const ForbiddenZone &cz = DEFAULT_CONSTRUCT_ZONE;
        bool hasConstruct = false;
        for (int i = 0; i < safety.getZoneCount(); i++) {
            float x1, x2, y1, y2, z1, z2;
            bool act = false;
            if (safety.getForbiddenZone(i, x1, x2, y1, y2, z1, z2, act) &&
                fabs(x1 - cz.xMin) < 1.0f && fabs(x2 - cz.xMax) < 1.0f &&
                fabs(y1 - cz.yMin) < 1.0f && fabs(y2 - cz.yMax) < 1.0f &&
                fabs(z1 - cz.zMin) < 1.0f && fabs(z2 - cz.zMax) < 1.0f) {
                hasConstruct = true;
                break;
            }
        }
        if (!hasConstruct) {
            Serial.println("Construct zone missing: adding [-65,65]x[-65,65]x[0,180] + save");
            safety.addForbiddenZone(cz.xMin, cz.xMax, cz.yMin, cz.yMax, cz.zMin, cz.zMax);
            if (config.forbiddenZoneCount < MAX_FORBIDDEN_ZONES) {
                config.forbiddenZones[config.forbiddenZoneCount] = cz;
                config.forbiddenZoneCount++;
                storage.saveConfig(config, homeAngles.angles, interpStrategy);
            }
        }
    }

    // Проверка сохранённого дома: битый home (напр. старые нули =
    // недостижимый вылет 415мм) ломал бы первую проверку траектории.
    // Откатываемся на дефолт с предупреждением (в EEPROM не пишем).
    if (!safety.checkAllLinks(homeAngles.angles)) {
        Serial.print("WARNING: stored HOME unsafe (");
        Serial.print(safety.getLastError());
        Serial.println("), using defaults");
        for (int i = 0; i < 7; i++) {
            homeAngles.angles[i] = DEFAULT_HOME_ANGLES[i];
        }
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

    // Ручной пульт: штурвал + NEXT + E-STOP (задача, опрос 20 мс).
    // При JOG_ENABLED=false — только лог, прерывания не вешаются.
    pendant.begin(&interpolator, &currentAngles, angleMutex);
    xTaskCreatePinnedToCore(pendantTask, "pendant", 4096, NULL, 2, NULL, 1);
    Serial.println(JOG_ENABLED ? "Jog pendant: ON (pins 32/33/25), E-STOP armed (pin 26)"
                               : "Jog pendant: manual OFF, E-STOP armed (pin 26)");
    Serial.println(JOG_ENABLED ? "Jog pendant: ON (pins 32/33/25/26)"
                               : "Jog pendant: OFF");

    // Инициализация SharedState
    memset(&sharedState, 0, sizeof(sharedState));
    sharedState.safetyEnabled = safety.isEnabled();
    sharedState.pcaReady = servoCtrl.isPcaAvailable();
    sharedState.imuState = gy80State();
    sharedState.tcpPosition = fkSolver.getEndEffector(homeAngles.angles);
    for (int i = 0; i < 7; i++) {
        sharedState.servoAngles[i] = homeAngles.angles[i];
    }
    strncpy(sharedState.lastCommand, "-", sizeof(sharedState.lastCommand) - 1);

    // Стартовый заезд в home — ВСЕГДА (иначе софт врёт нули, а железо
    // стоит где попало, и первый G28 даёт прыжок). Едем штатным G28:
    // там safety-пречек цели, синк стейта и диагностика.
    // Конвенция: ночью рука в парке — стартуем из DEFAULT_PARK_ANGLES,
    // едем тихо (F15). Нарушил конвенцию (не запарковался) — заезд всё
    // равно будет, но стартовая точка враньё: рука должна стоять свободно!
    {
        for (int i = 0; i < 7; i++) {
            currentAngles.angles[i] = DEFAULT_PARK_ANGLES[i];
        }
        GCodeCommand cmd28;  // пустая = без H1, едем в homeAngles
        cmd28.hasFeedrate = true;
        cmd28.feedrate = 15.0f;  // тихо: 15% скорости
        handleG28(cmd28);
    }
    // Принудительный refresh home-PWM на ВСЕ 7 каналов: интерполятор мог
    // пропустить нулевые дельты (sequential делает continue), а обратной
    // связи у серв нет — запястье/гриппер могли стоять где угодно и так
    // бы там и остались. Прямая запись home без условий движения.
    // Гейт тот же, что у G28 выше: цель обязана быть safe.
    {
        bool homeSafe = !safety.isEnabled() || safety.checkAllLinks(homeAngles.angles);
        if (homeSafe) {
            servoCtrl.setAngles(homeAngles);
            Serial.println("OK: Home PWM refreshed on all 7 servos");
        } else {
            Serial.print("ERROR: Home unsafe, PWM refresh skipped: ");
            Serial.println(safety.getLastError());
        }
    }
    // Калибровка нуля гиро — только если IMU есть. Для bias нужна лишь
    // неподвижность (рука уже стоит в home), а не сама поза.
    // Рука должна стоять смирно!
    if (imuLogger.ready()) {
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.isCalibrating = true;
        xSemaphoreGive(displayMutex);
        // Задача дисплея ещё не создана — рисуем CALIB напрямую.
        // Литералы вместо ZONE_STATUS_Y/COLOR_MOVING: их #define идут
        // ниже по файлу (176 / TFT_YELLOW), setup выше — компилятор иначе
        // не видит. Значения сверены с дефайнами, не магические.
        tft.setTextSize(2);
        tft.setCursor(4, 176 + 8);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.print(">> CALIB");
        Serial.println("IMU CAL: стоим в home, не трогать руку (отстойка + 2с нуля)...");
        delay(1500);  // отстойка механики: звон MG90S + выборка люфта
        imuLogger.calibrateGyro();
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.isCalibrating = false;
        xSemaphoreGive(displayMutex);
        Serial.println("IMU CAL: done");
    }
    imuLogger.startTask();  // опрос 200 Гц на Core 1 (после калибровки!)

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
#define COLOR_ZONE_HEADER_FG  TFT_CYAN
#define COLOR_FIELD_LABEL     TFT_WHITE
#define COLOR_TCP             TFT_GREEN
#define COLOR_JOINT           TFT_YELLOW
#define COLOR_GRIPPER         0xFDA0   // оранжевый
#define COLOR_READY           TFT_GREEN
#define COLOR_MOVING          TFT_YELLOW
#define COLOR_ERROR           TFT_RED
#define COLOR_STATUS_LABEL    TFT_WHITE
#define COLOR_STATUS_VALUE    TFT_CYAN

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
static int8_t lastModeAbs  = -1;   // -1 = неизвестно, 0 = ABS, 1 = REL
static int8_t lastStatus   = -1;   // -1 = неизвестно, 0 = READY, 1 = MOVING, 2 = ERROR, 3 = CALIB
static char lastCmdShown[24] = ""; // последняя выведенная команда (анти-мерцание)

static void drawField(int xLabel, int xValue, int y,
                      const char *label, float value, float &lastVal,
                      uint16_t color, const char *unit, bool forceRedraw = false) {
    if (!forceRedraw && fabs(value - lastVal) < 0.5f) return;
    lastVal = value;

    tft.setTextSize(2);

    // Метка слева
    tft.setTextColor(COLOR_FIELD_LABEL, TFT_BLACK);
    tft.setCursor(xLabel, y);
    tft.print(label);

    // Число — фиксированное поле, знак "-" не сдвигает цифры.
    // Без хвостового пробела: он съедал зазор до правой колонки
    // (все поля и так фиксированной ширины, чистить нечего).
    char buf[16];
    snprintf(buf, sizeof(buf), "%4.0f%s", value, unit);
    tft.setTextColor(color, TFT_BLACK);
    tft.setCursor(xValue, y);
    tft.print(buf);
}

/**
 * drawFieldR() — как drawField, но число прижато ВПРАВО к xRight.
 * Формат "%4.0f" всегда даёт ровно 4 символа (наши диапазоны ±180/0-100),
 * поэтому курсор = xRight - 48, хвостов не остаётся.
 */
static void drawFieldR(int xLabel, int xRight, int y,
                       const char *label, float value, float &lastVal,
                       uint16_t color) {
    if (fabs(value - lastVal) < 0.5f) return;
    lastVal = value;

    tft.setTextSize(2);

    tft.setTextColor(COLOR_FIELD_LABEL, TFT_BLACK);
    tft.setCursor(xLabel, y);
    tft.print(label);

    char buf[16];
    snprintf(buf, sizeof(buf), "%4.0f", value);
    tft.setTextColor(color, TFT_BLACK);
    tft.setCursor(xRight - 48, y);
    tft.print(buf);
}

/**
 * toolEulerZYX() — ориентация инструмента (R06) в эйлеры ZYX, градусы.
 * Та же конвенция, что G-code A/B/C (R = Rz·Ry·Rx).
 */
static void toolEulerZYX(const float q[6], float &a, float &b, float &c) {
    Mat4 R = fkSolver.getRotationMatrix(q, 5);
    float sp = -R.m[2][0];
    if (sp > 0.9999f) {
        b = 90.0f; a = 0.0f;
        c = atan2(-R.m[0][1], R.m[0][2]) * 180.0f / 3.14159265f;
    } else if (sp < -0.9999f) {
        b = -90.0f; a = 0.0f;
        c = atan2(-R.m[0][1], -R.m[0][2]) * 180.0f / 3.14159265f;
    } else {
        b = asin(sp) * 180.0f / 3.14159265f;
        a = atan2(R.m[2][1], R.m[2][2]) * 180.0f / 3.14159265f;
        c = atan2(R.m[1][0], R.m[0][0]) * 180.0f / 3.14159265f;
    }
}

// ====================================================================
// Дисплей: displayInit() и displayUpdate()
// ====================================================================

/**
 * displayInit() — инициализация ST7789 дисплея.
 */
void displayInit() {
    tft.init();
    tft.setRotation(0);           // было 180° (Adafruit), здесь вверх ногами — ставим 0°
    tft.fillScreen(TFT_BLACK);

    // Заставка: "6DOF" + "Initializing..."
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(40, 80);
    tft.println("  6DOF");
    tft.setTextSize(2);
    tft.setCursor(20, 120);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.println(" Initializing...");
    delay(2000);
    tft.fillScreen(TFT_BLACK);

    tft.setTextSize(2);

    // Заголовки зон
    drawZoneHeader(ZONE_TCP_Y,    "TCP POS.");
    drawZoneHeader(ZONE_JOINTS_Y, "JOINTS");

    // Версия прошивки в строке заголовка TCP (прижата вправо).
    // Рисуем один раз здесь (константа): в displayUpdate не трогаем —
    // так мерцания нет по построению. Раскладка (кегль 2 = 12 px/символ):
    // "TCP POS." 4..100, "FW: 8(1.6)" 10 символов = 120 px → x=116..236.
    {
        char fwBuf[16];
        snprintf(fwBuf, sizeof(fwBuf), "FW: %lu(%s)",
                 (unsigned long)FW_VERSION, FW_VERSION_STR);
        tft.setTextSize(2);
        tft.setTextColor(COLOR_ZONE_HEADER_FG, COLOR_ZONE_HEADER_BG);
        tft.setCursor(240 - 4 - (int)strlen(fwBuf) * 12, ZONE_TCP_Y + 2);
        tft.print(fwBuf);
    }

    // Инициализация last-значений
    for (int i = 0; i < 6; i++) lastTCP[i] = 99999;
    for (int i = 0; i < 7; i++) lastJoints[i] = 99999;
    lastGrip = -1;
    lastModeAbs = -1;
    lastStatus = -1;
    lastCmdShown[0] = '\0';
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

    // === TCP Position Zone (3 строки × 2 колонки: слева XYZ, справа Roll/Pitch/Yaw) ===
    // Раскладка по пикселям (шрифт 12px/знак): левые значения до x=114
    // ("-500mm"), метки с x=116 ("Pitch:" до 188), значения 188..236.
    // Правая колонка ужата, иначе не влезает в 240.
    float qq[6];
    for (int i = 0; i < 6; i++) qq[i] = state.servoAngles[i];
    float ea, eb, ec;
    toolEulerZYX(qq, ea, eb, ec);
    drawField(4, 30, ZONE_TCP_Y + HEADER_H + 2,  "X:", state.tcpPosition.x, lastTCP[0], COLOR_TCP, "mm");
    drawFieldR(116, 236, ZONE_TCP_Y + HEADER_H + 2,  "Roll:", ea, lastTCP[3], COLOR_TCP);
    drawField(4, 30, ZONE_TCP_Y + HEADER_H + 18, "Y:", state.tcpPosition.y, lastTCP[1], COLOR_TCP, "mm");
    drawFieldR(116, 236, ZONE_TCP_Y + HEADER_H + 18, "Pitch:", eb, lastTCP[4], COLOR_TCP);
    drawField(4, 30, ZONE_TCP_Y + HEADER_H + 34, "Z:", state.tcpPosition.z, lastTCP[2], COLOR_TCP, "mm");
    drawFieldR(116, 236, ZONE_TCP_Y + HEADER_H + 34, "Yaw:", ec, lastTCP[5], COLOR_TCP);

    // === Joints Zone (4 строки × 2 колонки, +2 px от заголовка) ===
    drawField(4,   40,  ZONE_JOINTS_Y + HEADER_H + 2,  "P1:", state.servoAngles[0], lastJoints[0], COLOR_JOINT, "");
    drawFieldR(124, 232, ZONE_JOINTS_Y + HEADER_H + 2,  "P4:", state.servoAngles[3], lastJoints[3], COLOR_JOINT);

    drawField(4,   40,  ZONE_JOINTS_Y + HEADER_H + 18, "P2:", state.servoAngles[1], lastJoints[1], COLOR_JOINT, "");
    drawFieldR(124, 232, ZONE_JOINTS_Y + HEADER_H + 18, "P5:", state.servoAngles[4], lastJoints[4], COLOR_JOINT);

    drawField(4,   40,  ZONE_JOINTS_Y + HEADER_H + 34, "P3:", state.servoAngles[2], lastJoints[2], COLOR_JOINT, "");
    drawFieldR(124, 232, ZONE_JOINTS_Y + HEADER_H + 34, "P6:", state.servoAngles[5], lastJoints[5], COLOR_JOINT);

    drawField(4, 100, ZONE_JOINTS_Y + HEADER_H + 50, "Gripper:", state.gripperPercent, lastGrip, COLOR_GRIPPER, "%");

    // === Status Bar (3 строки) ===
    int sy = ZONE_STATUS_Y + 8;

    // --- Строка 1: READY / MOVING / ERROR / CALIB + режим + счётчик справа ---
    // Статус — только при смене (безусловная перерисовка даёт мерцание
    // и затирает соседей). Поле статуса ровно 8 символов (x=4..100),
    // ABS начинается с x=110 — пересечений нет. CALIB (8 симв.) как MOVING.
    int8_t st = state.hasError ? 2 : (state.isCalibrating ? 3 : (state.isMoving ? 1 : 0));
    if (st != lastStatus) {
        lastStatus = st;
        tft.setTextSize(2);
        tft.setCursor(4, sy);
        if (st == 2) {
            tft.setTextColor(COLOR_ERROR, TFT_BLACK);
            tft.print("! ERROR ");
        } else if (st == 3) {
            tft.setTextColor(COLOR_MOVING, TFT_BLACK);
            tft.print(">> CALIB");
        } else if (st == 1) {
            tft.setTextColor(COLOR_MOVING, TFT_BLACK);
            tft.print("> MOVING");
        } else {
            tft.setTextColor(COLOR_READY, TFT_BLACK);
            tft.print("READY   ");
        }
    }

    // Счётчик прижат вправо
    tft.setTextSize(2);
    char cntBuf[8];
    snprintf(cntBuf, sizeof(cntBuf), "%4u ", (unsigned)state.commandCount);
    int cntW = strlen(cntBuf) * 6 * 2;
    tft.setCursor(236 - cntW, sy);
    tft.setTextColor(COLOR_STATUS_VALUE, TFT_BLACK);
    tft.print(cntBuf);

    // Режим ABS/REL — над PCA (колонка x=110).
    // Рисуем только при смене: безусловная перерисовка каждые 200 мс
    // даёт видимое мерцание буквы.
    {
        int8_t m = absoluteMode ? 0 : 1;
        if (m != lastModeAbs) {
            lastModeAbs = m;
            tft.setCursor(110, sy);
            tft.setTextColor(COLOR_STATUS_VALUE, TFT_BLACK);
            tft.print(m == 0 ? "ABS" : "REL");
        }
    }

    // --- Строка 2: Safe + PCA + ABS/REL ---
    sy += 16;
    tft.setCursor(4, sy);
    tft.setTextColor(COLOR_STATUS_LABEL, TFT_BLACK);
    tft.print("Safe:");
    tft.setTextColor(state.safetyEnabled ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(state.safetyEnabled ? "ON " : "OFF");

    // --- Строка 2: Safe + PCA-кружок + GYR-кружок ---
    // Кружок = наличие железа; GYR трёхцветный: зелёный = гиро+аксель,
    // жёлтый = только гиро, красный = нет гиро. Текст не пишем.
    // Перерисовка поверх тем же центром/радиусом полностью перекрывает
    // старый кружок — мерцания нет. Раскладка: Safe 4..100, PCA 110..170,
    // GYR 180..237 (кегль 2 = 12 px/символ, кружок r=4).
    tft.setCursor(110, sy);
    tft.setTextColor(COLOR_STATUS_LABEL, TFT_BLACK);
    tft.print("PCA:");
    tft.fillCircle(166, sy + 8, 4, state.pcaReady ? TFT_GREEN : TFT_RED);

    tft.setCursor(180, sy);
    tft.setTextColor(COLOR_STATUS_LABEL, TFT_BLACK);
    tft.print("GYR:");
    uint16_t gyrColor = (state.imuState >= 2) ? TFT_GREEN
                      : (state.imuState == 1) ? TFT_YELLOW : TFT_RED;
    tft.fillCircle(233, sy + 8, 4, gyrColor);

    // --- Строка 3: последняя команда (как есть, до 20 символов) ---
    // Добиваем пробелами до фиксированной ширины 20 символов = 240 px при size=2.
    sy += 16;
    tft.setCursor(4, sy);
    tft.setTextColor(COLOR_STATUS_VALUE, TFT_BLACK);

    char cmdBuf[32];
    snprintf(cmdBuf, sizeof(cmdBuf), "%-20.20s", state.lastCommand);
    // Команду тоже только при смене (та же причина — мерцание).
    if (strncmp(cmdBuf, lastCmdShown, sizeof(lastCmdShown)) != 0) {
        strncpy(lastCmdShown, cmdBuf, sizeof(lastCmdShown) - 1);
        lastCmdShown[sizeof(lastCmdShown) - 1] = '\0';
    // Добиваем пробелами до 20 символов (240 px)
    int cmdW = strlen(cmdBuf) * 6 * 2;
    while (cmdW < 20 * 12 && strlen(cmdBuf) < sizeof(cmdBuf) - 1) {
        strcat(cmdBuf, " ");
        cmdW = strlen(cmdBuf) * 6 * 2;
    }
    tft.print(cmdBuf);
    }

    tft.endWrite();
}


// ====================================================================
// loop() — пуста (вся работа делается задачами FreeRTOS)
// ====================================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}