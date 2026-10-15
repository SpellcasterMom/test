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
 * Подключение проводов (Pinout; цифры — дефолты из config.h,
 * блок «Пины и шины» — разводка правится только там):
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
#include "display_ui.h"  // ветка ref-display-ui: LGFX/SharedState/дисплей
#include "kin_diag.h"    // ветка ref-kin-diag: журнал M115 + handleM115

// Хаб общего состояния для будущих модулей распила (ветки ref-*):
// только extern-объявления, дублирует определения ниже. Логики нет.
#include "app.h"

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
ImuLogger            imuLogger; // BMI160: опрос 200 Гц + кольцо последнего движения

// Мьютекс общей шины I2C (PCA9685 на Core 0 + BMI160 на Core 1).
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

// Дисплей (LGFX/tft/SharedState/displayMutex) и его функции живут в
// display_ui.{h,cpp} (ветка ref-display-ui). Здесь — только инклуд.

// ===== Предварительные объявления функций =====
void core0Task(void *pvParameters);
void core1Task(void *pvParameters);

// Журнал M115 (DiagSol/DiagLastMove/diagLast + helpers + handleM115)
// уехал в kin_diag.{h,cpp} (ветка ref-kin-diag). Здесь — только инклуд.

// ====================================================================
// Обработчики G-code команд
// ====================================================================

// M119/M105/M114/M112/M111/M400/M410-M417/M420/M425-M426/M430/M450/M500-M502/help/print* — уехали в handlers_system.cpp (ветка ref-handlers-system).

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
// BMI160 на общей I2C (пины из config.h, Wire уже поднят servoCtrl.init()).
// TODO(bmi160): маска — один чип (бит0=гиро, бит1=аксель, CHIP_ID 0xD1
// на 0x68/0x69); магнит/баро уходят. Пока скан старый, GY-80-стиль:
// бит0=гиро L3G4200D (0x68/0x69 — зависит от запайки SDO),
// бит1=аксель ADXL345 (0x53), бит2=магнит HMC5883L (0x1E), бит3=баро BMP180 (0x77).
// Рабочий минимум для наклона и guard'а — гиро+аксель (маска 0x03);
// магнит нужен только для 9-DOF yaw, баро руке не нужно вовсе.
static int bmi160Mask = 0;

static bool i2cPing(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// Состояние GYR-кружка по маске scanBMI160():
// 2 = гиро+аксель (зелёный), 1 = только гиро (жёлтый), 0 = нет гиро (красный).
// Магнит для кружка не нужен (только для будущего 9-DOF yaw).
static uint8_t bmi160State() {
    if (!(bmi160Mask & 0x01)) return 0;
    return (bmi160Mask & 0x02) ? 2 : 1;
}

static int scanBMI160() {
    int mask = 0;
    if (i2cPing(0x68) || i2cPing(0x69)) mask |= 0x01;  // гиро
    if (i2cPing(0x53)) mask |= 0x02;                   // аксель
    if (i2cPing(0x1E)) mask |= 0x04;                   // магнит
    if (i2cPing(0x77)) mask |= 0x08;                   // баро
    return mask;
}

// ====================================================================
// Фазы setup() (ветка ref-setup-split): тот же код построчно, только
// нарезанный. Порядок вызовов не менялся вообще.
// ====================================================================

// Фаза 1: Serial/дисплей/мьютексы/железо/связи модулей.
static void setupHardware() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);

    // Инициализация дисплея ПЕРЕД I2C/PCA9685
    displayInit();

    Serial.println("=== 6-DOF Robot Arm Controller ===");
    Serial.println("ESP32 @ 240 MHz, PCA9685, 7 servos");

    // Создание мьютексов ДО первого использования
    angleMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();  // шина Wire: сервы + BMI160

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

    // Поиск BMI160 (железо воткнуто или нет — драйвер позже, см. TODO).
    // Кружок: зелёный = гиро+аксель, жёлтый = только гиро, красный = нет гиро.
    bmi160Mask = scanBMI160();
    xSemaphoreTake(displayMutex, portMAX_DELAY);
    sharedState.imuState = bmi160State();
    xSemaphoreGive(displayMutex);
    Serial.printf("BMI160: gyro=%c acc=%c mag=%c baro=%c\n",
                  (bmi160Mask & 0x01) ? '+' : '-',
                  (bmi160Mask & 0x02) ? '+' : '-',
                  (bmi160Mask & 0x04) ? '+' : '-',
                  (bmi160Mask & 0x08) ? '+' : '-');

    // IMU-логгер: драйвер по маске найденного + указатель на интерполятор
    // (запись идёт пока isMoving). Задача стартует после калибровки ниже.
    imuLogger.begin(bmi160Mask);
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
}

// Форварды фазников (определения ниже, порядок в файле не меняли).
static void setupConfig();
static void setupHome();
static void setupCalibAndTasks();

void setup() {
    setupHardware();      // Serial/дисплей/мьютексы/железо/связи модулей
    setupConfig();        // EEPROM + FW-миграция + зоны + проверка home
    setupHome();          // стартовая поза + sharedState + заезд в home + refresh
    setupCalibAndTasks(); // калибровка IMU + задачи + Ready
}

// Фаза 2: EEPROM + FW-миграция + зоны + проверка сохранённого home.
static void setupConfig() {
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
}

// Фаза 3: стартовая поза + sharedState + заезд в home + refresh PWM.
static void setupHome() {
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
    // Core0: на Core1 живут исполнитель потока + IMU, пульту там не место
    // (E-STOP всё равно через прерывание, от ядра не зависит).
    pendant.begin(&interpolator, &currentAngles, angleMutex);
    xTaskCreatePinnedToCore(pendantTask, "pendant", 4096, NULL, 2, NULL, 0);
    Serial.println(JOG_ENABLED ? "Jog pendant: ON (pins 32/33/25), E-STOP armed (pin 26)"
                               : "Jog pendant: manual OFF, E-STOP armed (pin 26)");
    Serial.println(JOG_ENABLED ? "Jog pendant: ON (pins 32/33/25/26)"
                               : "Jog pendant: OFF");

    // Инициализация SharedState
    memset(&sharedState, 0, sizeof(sharedState));
    sharedState.safetyEnabled = safety.isEnabled();
    sharedState.pcaReady = servoCtrl.isPcaAvailable();
    sharedState.imuState = bmi160State();
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
}

// Фаза 4: калибровка IMU + задачи + Ready.
static void setupCalibAndTasks() {
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

    // Создание задач.
    // Раскладка ядер: Core1 = исполнитель потока + IMU (движение и гиро
    // рядом, как договаривались); всё остальное — Core0. Дисплей уехал
    // с Core1, чтобы 200мс-отрисовки не дёргали каданс исполнителя.
    interpolator.startExecTask();  // stream_exec, Core1, приор. 2 (= IMU)
    xTaskCreatePinnedToCore(core0Task, "Core0_Motion",  8192, NULL, 1, &core0TaskHandle, 0);
    xTaskCreatePinnedToCore(core1Task, "Core0_Display", 8192, NULL, 1, &core1TaskHandle, 0);

    Serial.println("Ready. Send G-code commands.");
    Serial.println("Type '?' for help");
}

// ====================================================================
// Callback интерполятора — уехал в display_ui.cpp (ветка ref-display-ui).
// ====================================================================

// ====================================================================
// Дисплей: цвета, зоны — уехали в display_ui.cpp (ветка ref-display-ui).
// ====================================================================

// ====================================================================
// loop() — пуста (вся работа делается задачами FreeRTOS)


// ====================================================================
// loop() — пуста (вся работа делается задачами FreeRTOS)
// ====================================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}