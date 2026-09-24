/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       :
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-18
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
 *   V+   → адаптер питания сервоприводов (отдельный от ESP32!)
 *
 * ST7789 1.54" (дисплей) — SPI (VSPI):
 *   MOSI → GPIO 23
 *   SCK  → GPIO 18
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
 *   Канал 0  → Joint 1 (Base)        — серва 20кг
 *   Канал 1  → Joint 2 (Shoulder)    — серва 70кг
 *   Канал 2  → Joint 3 (Elbow)       — серва 70кг
 *   Канал 3  → Joint 4 (Wrist Pitch) — MG90S
 *   Канал 4  → Joint 5 (Wrist Roll)  — MG90S
 *   Канал 5  → Joint 6 (Wrist Yaw)   — MG90S
 *   Канал 6  → Gripper               — MG90S
 *
 * =====================================================================
 * Архитектура FreeRTOS на двух ядрах:
 * =====================================================================
 *   Core 0 — Serial + Motion:
 *     Чтение Serial, парсинг G-code, IK/FK, safety, интерполяция,
 *     прямое управление сервоприводами, обновление sharedState.
 *
 *   Core 1 — Display:
 *     Чтение sharedState (каждые200мс), обновление ST7789 дисплея.
 *     Отображает: TCP позицию, углы сервоприводов, захват, статус.
 *
 * Доступ к общим данным (sharedState, currentAngles) защищён мьютексами,
 * что исключает гонки между задачами Core0 и Core1.
 *
 * Протокол общения: отправить G/M-команду → дождаться ответа OK/ERROR.
 * Поддерживает: прямое управление сервоприводами, ИК-решение, смешанный режим,
 * компенсацию люфта, проверку безопасности (запретные зоны, ограничения
 * по длине/высоте), сохранение/загрузку конфигурации из EEPROM.
 *
 * =====================================================================
 * ДЕМОНСТРАЦИЯ: Endpoint Control — 8 позиций
 * =====================================================================
 * Отправляйте команды пошагово: вставляйте одну строку,
 * ждите ответ OK, затем следующую.
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
#include <TFT_eSPI.h>
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

// ===== Глобальные объекты =====
// Все объекты создаются глобально для доступа из задач FreeRTOS
// и обработчиков команд. Жизненный цикл = всё время работы контроллера.

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
// Поддерживает несколько стратегий: линейная, последовательная,
// групповая и автоматический выбор.
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
// (углы home, параметры сервоприводов и т.д.)
EEPROMStorage        storage;

// ===== Глобальное состояние =====
// Текущая конфигурация манипулятора (углы сервоприводов, ограничения,
// длина пальцев и т.д.). Загружается из EEPROM или берётся из DEFAULT_CONFIG.
ManipulatorConfig    config = DEFAULT_CONFIG;

// Углы «домашней» позиции (home). Используются при вызове G28.
// Могут быть сохранены в EEPROM через M500, загружены через M501.
JointAngles          homeAngles;

// Текущие углы всех 7 сервоприводов (operational state).
// Защищены мьютексом angleMutex, так как читаются из разных задач.
JointAngles          currentAngles;

// Режим координат: true = абсолютный (G90), false = относительный (G91).
// Влияет на обработку целевых координат в G0/G1.
bool                 absoluteMode = true;

// Стратегия интерполяции движения.
// INTERP_AUTO = автоматический выбор на основе текущей задачи.
// INTERP_LINEAR = линейная, INTERP_SEQUENTIAL = последовательная,
// INTERP_GROUPED = групповая.
InterpolationStrategy interpStrategy = INTERP_AUTO;

// ===== FreeRTOS: задачи и мьютекс =====
// Дескрипторы задач: используются для управления задачами (создание,
// удаление, приостановка) из основного потока.
TaskHandle_t core0TaskHandle = NULL;
TaskHandle_t core1TaskHandle = NULL;

// Мьютекс (mutual exclusion) для защиты доступа к currentAngles.
// Захватывается через xSemaphoreTake / освобождается через xSemaphoreGive.
// portMAX_DELAY означает бесконечное ожидание, если мьютекс занят.
SemaphoreHandle_t angleMutex = NULL;

// ===== Дисплей ST7789 =====
// Экран 1.54" 240×240, SPI (VSPI), пины: MOSI=23, SCK=18, CS=5, DC=2, RST=4
TFT_eSPI tft = TFT_eSPI();

// ===== Структура общего состояния для дисплея =====
// Заполняется Core0 (callback интерполятора + обработчики команд)
// Читается Core1 (задача обновления дисплея).
// Защищена мьютексом displayMutex.
struct SharedState {
    Vec3        tcpPosition;      // Текущая позиция TCP (из FK)
    float       servoAngles[7];   // Углы 7 сервоприводов (градусы)
    float       gripperPercent;   // Процент открытия захвата (0-100%)
    bool        isMoving;         // Флаг движения
    bool        safetyEnabled;    // Состояние безопасности
    bool        hasError;         // Есть ошибка
    char        lastError[64];    // Текст последней ошибки
    uint32_t    commandCount;     // Счётчик выполненных команд
};

SharedState    sharedState;       // Общее состояние (заполняется Core0)
SemaphoreHandle_t displayMutex = NULL;  // Мьютекс для SharedState

// ===== Предварительные объявления функций =====
void core0Task(void *pvParameters);  // Задача Core 0: Serial + движение
void core1Task(void *pvParameters);  // Задача Core 1: дисплей / фоновые задачи
void printPosition();                // Вывод текущей позиции (FK) в Serial
void printServoAngles();             // Вывод текущих углов сервоприводов в Serial
void printStatus();                  // Вывод полного статуса манипулятора

// ===== Дисплей: функции отрисовки =====
void displayInit();                  // Инициализация ST7789 дисплея
void displayUpdate();                // Обновление данных на экране (вызывается Core1)
void onInterpolationStep(const JointAngles &angles);  // Callback интерполятора

// ====================================================================
// Обработчики G-code команд
// ====================================================================

/**
 * G0 — перемещение (быстрое).
 * Поддерживает три режима:
 *   IK_MODE       — задаётся позиция (X, Y, Z) и ориентация (A, B, C),
//                    вычисляются углы через обратную кинематику.
 *   DIRECT_MODE   — задаются углы сервоприводов (P1..P7) напрямую.
 *   MIXED_MODE    — комбинация: IK + ручное перекрытие отдельных сервоприводов.
 *
 * После вычисления целевых углов выполняется проверка безопасности.
 * Если проверка пройдена — запускается интерполяция движения.
 * Мьютекс angleMutex захватывается при обновлении currentAngles.
 */
void handleG0(GCodeCommand &cmd) {
    // Определяет режим работы команды (IK / Direct / Mixed)
    cmd.determineMode();

    // Сброс ошибки при новой команде
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    JointAngles target;

    switch (cmd.mode) {
        case GCodeCommand::IK_MODE: {
            // Обратная кинематика: позиция + ориентация → углы суставов
            // Ищем безопасное решение среди всех вариантов (elbow up/down)
            JointAngles solutions[4];
            int solCount = ikSolver.solveAll(cmd.pose, solutions);
            
            if (solCount == 0) {
                xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
                sharedState.hasError = true;
                snprintf(sharedState.lastError, sizeof(sharedState.lastError), "IK not found");
                xSemaphoreGive(displayMutex);
                Serial.println("ERROR: IK solution not found");
                return;
            }
            
            // Выбираем первое безопасное решение
            bool foundSafe = false;
            for (int i = 0; i < solCount; i++) {
                target = solutions[i];
                // Устанавливаем захват
                if (cmd.hasGripper) {
                    target.angles[6] = cmd.pose.gripper;
                }
                // Проверяем безопасность
                if (!safety.isEnabled() || safety.checkAllLinks(target.angles)) {
                    foundSafe = true;
                    break;
                }
            }
            
            if (!foundSafe) {
                xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
                sharedState.hasError = true;
                snprintf(sharedState.lastError, sizeof(sharedState.lastError), "No safe IK solution");
                xSemaphoreGive(displayMutex);
                Serial.println("ERROR: No safe IK solution (all violate safety)");
                return;
            }
            break;
        }
        case GCodeCommand::DIRECT_MODE: {
            // Прямое управление: целевые углы заданы пользователем
            target = cmd.servoAngles;
            break;
        }
        case GCodeCommand::MIXED_MODE: {
            // Смешанный режим: сначала IK, затем перекрытие отдельных сервоприводов
            if (cmd.hasPosition || cmd.hasOrientation) {
                ikSolver.solve(cmd.pose, target);
            } else {
                // Если позиция/ориентация не заданы — берём текущие углы
                target = currentAngles;
            }
            // Перекрытие сервоприводов, для которых заданы P-параметры
            for (int i = 0; i < 7; i++) {
                if (cmd.servoOverride[i]) {
                    target.angles[i] = cmd.servoAngles.angles[i];
                }
            }
            break;
        }
    }

    // Проверка безопасности: запретные зоны, ограничения по длине/высоте
    if (safety.isEnabled()) {
        if (!safety.checkAllLinks(target.angles)) {
            xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
            sharedState.hasError = true;
            snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Safety");
            xSemaphoreGive(displayMutex);
            Serial.print("ERROR: Safety violation: ");
            Serial.println(safety.getLastError());
            return;
        }
    }

    // Запуск движения: интерполятор плавно перемещает сервоприводы
    float feedrate = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
    if (interpolator.moveTo(target, feedrate)) {
        // Защита мьютексом: обновляем currentAngles
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        // Обновление SharedState для дисплея
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
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: Move failed");
    }
}

/**
 * G1 — перемещение с подачей (аналогично G0).
 * Фактически вызывает handleG0, так как feedrate уже обрабатывается
 * внутри handleG0 на основе cmd.hasFeedrate / cmd.feedrate.
 */
void handleG1(GCodeCommand &cmd) {
    handleG0(cmd);  // Same as G0 but with feedrate
}

/**
 * G28 — перемещение в домашнюю позицию (home).
 * Если задан параметр H1 (cmd.hasParam['H' - 'A'] == 1 и значение == 1),
 * выполняется компактная складка (park): плечо вниз, локоть согнут,
 * запястье вертикально, захват закрыт.
 * Иначе — перемещение в сохранённую домашнюю позицию (homeAngles).
 */
void handleG28(GCodeCommand &cmd) {
    // Сброс ошибки при новой команде
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    JointAngles target;
    bool park = cmd.hasParam['H' - 'A'] && (int)cmd.params['H' - 'A'] == 1;

    if (park) {
        // Компактная складка (park position):
        //   P1=0   — база по центру
        //   P2=-60 — плечо вниз
        //   P3=-90 — локоть сильно согнут
        //   P4=0   — запястье (pitch) нейтрально
        //   P5=0   — запястье (roll) нейтрально
        //   P6=0   — запястье (yaw) нейтрально
        //   P7=0   — захват закрыт
        target.setAll(0, -60, -90, 0, 0, 0, 0);
    } else {
        // Перемещение в сохранённую домашнюю позицию
        for (int i = 0; i < 7; i++) {
            target.angles[i] = homeAngles.angles[i];
        }
    }

    // Движение с фиксированной подачей 300 мм/с
    if (interpolator.moveTo(target, 300.0f)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        // Обновление SharedState для дисплея
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
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Park failed");
        xSemaphoreGive(displayMutex);
        Serial.println(park ? "ERROR: Park failed" : "ERROR: Home move failed");
    }
}

/**
 * G90 — абсолютный режим координат.
 * Все последующие координаты в G0/G1 интерпретируются как абсолютные
 * (относительно начала координат манипулятора).
 */
void handleG90(GCodeCommand &cmd) {
    absoluteMode = true;
    Serial.println("OK: Absolute mode");
}

/**
 * G91 — относительный режим координат.
 * Все последующие координаты в G0/G1 интерпретируются как приращения
 * (относительно текущей позиции).
 */
void handleG91(GCodeCommand &cmd) {
    absoluteMode = false;
    Serial.println("OK: Relative mode");
}

/**
 * M104 — прямая установка углов сервоприводов (без интерполяции).
 * Для каждого сервопривода P1..P7, у которого задан параметр,
 * устанавливается угол напрямую через servoCtrl.setAngle().
 * Мьютекс защищает currentAngles.
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
    Serial.println("OK");
}

/**
 * M105 — вывод текущих углов сервоприводов.
 * Формат: Angles: P1=0.0 P2=0.0 ... P7=0.0
 */
void handleM105(GCodeCommand &cmd) {
    printServoAngles();
}

/**
 * M114 — вывод текущей позиции энд-эффектора (прямая кинематика).
 * Формат: Position: X=0.0 Y=0.0 Z=0.0
 */
void handleM114(GCodeCommand &cmd) {
    printPosition();
}

/**
 * M112 — аварийная остановка.
 * Немедленно останавливает интерполяцию и отключает все сервоприводы.
 * Используется в экстренных ситуациях (столкновение, сбой и т.д.).
 */
void handleM112(GCodeCommand &cmd) {
    interpolator.stop();
    servoCtrl.disable();
    Serial.println("OK: Emergency stop");
}

/**
 * M111 — сохранение текущих углов как домашней позиции.
 * Копирует currentAngles в homeAngles.
 * Мьютекс защищает доступ к currentAngles.
 */
void handleM111(GCodeCommand &cmd) {
    // Save current angles as home
    xSemaphoreTake(angleMutex, portMAX_DELAY);
    for (int i = 0; i < 7; i++) {
        homeAngles.angles[i] = currentAngles.angles[i];
    }
    xSemaphoreGive(angleMutex);
    Serial.println("OK: Home saved");
}

/**
 * M400 — установка компенсации люфта.
 * Задаёт поправки на люфт для каждого сервопривода (в градусах).
 * Значения передаются через параметры P1..P7.
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
 * Параметры: X, Y, Z (мин. значения) + A, B, C (макс. значения).
 * Если координаты ограничения не заданы — используются значения по умолчанию:
 *   X: -100..100, Y: -100..100, Z: 0..50
 */
void handleM410(GCodeCommand &cmd) {
    float x1 = cmd.pose.position.x;
    float x2 = cmd.pose.position.y;
    float y1 = cmd.pose.position.z;
    float y2 = cmd.pose.orientation.x;
    float z1 = cmd.pose.orientation.y;
    float z2 = cmd.pose.orientation.z;
    if (x2 == 0 && y2 == 0 && z2 == 0) {
        // Значения по умолчанию, если ограничения не заданы
        x1 = -100; x2 = 100; y1 = -100; y2 = 100; z1 = 0; z2 = 50;
    }
    safety.addForbiddenZone(x1, x2, y1, y2, z1, z2);
    Serial.println("OK: Forbidden zone added");
}

/**
 * M411 — установка минимальной высоты стола (Z_min).
 * Предотвращает столкновение манипулятора с поверхностью стола.
 * По умолчанию: 50 мм.
 */
void handleM411(GCodeCommand &cmd) {
    float zMin = cmd.pose.position.z;
    if (zMin == 0) zMin = 50.0f;
    safety.setTableZMin(zMin);
    Serial.println("OK: Table Z min = " + String(zMin, 1));
}

/**
 * M412 — установка максимального радиуса дотягивания (max reach).
 * Ограничивает максимальное расстояние энд-эффектора от базы.
 * По умолчанию: 395 мм.
 */
void handleM412(GCodeCommand &cmd) {
    float r = cmd.hasParam['R' - 'A'] ? cmd.params['R' - 'A'] : 395.0f;
    safety.setMaxReach(r);
    Serial.println("OK: Max reach = " + String(r, 1));
}

/**
 * M413 — установка минимального радиуса дотягивания (min reach).
 * Ограничивает минимальное расстояние энд-эффектора от базы.
 * По умолчанию: 50 мм.
 */
void handleM413(GCodeCommand &cmd) {
    float r = cmd.hasParam['R' - 'A'] ? cmd.params['R' - 'A'] : 50.0f;
    safety.setMinReach(r);
    Serial.println("OK: Min reach = " + String(r, 1));
}

/**
 * M414 — установка длины пальцев захвата (finger length).
 * Используется в FK-решении и проверке безопасности.
 * По умолчанию: 50 мм.
 */
void handleM414(GCodeCommand &cmd) {
    float len = cmd.hasParam['L' - 'A'] ? cmd.params['L' - 'A'] : 50.0f;
    fkSolver.setFingerLength(len);
    safety.setFingerLength(len);
    Serial.println("OK: Finger length = " + String(len, 1));
}

/**
 * M415 — отключение проверки безопасности.
 * После этой команды проверки запретных зон, ограничений по
 * длине/высоте и длине пальцев НЕ выполняются.
 * Используйте с осторожностью!
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
 * Все ограничения снова активны.
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
 * Выводит количество и параметры всех запретных зон.
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
 * M420 — установка минимальной высоты Z для конкретного звена (link).
 * Параметр L задаёт номер звена (по умолчанию 0 = база).
 * Параметр Z задаёт минимальную высоту.
 */
void handleM420(GCodeCommand &cmd) {
    int link = cmd.hasParam['L' - 'A'] ? (int)cmd.params['L' - 'A'] : 0;
    float zMin = cmd.pose.position.z;
    safety.setLinkZMin(link, zMin);
    Serial.println("OK: Link " + String(link) + " Z min = " + String(zMin, 1));
}

/**
 * M425 — отключение проверки безопасности (дублирует M415).
 * Оставлено для обратной совместимости.
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
 * Оставлено для обратной совместимости.
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
 * Параметр S задаёт стратегию:
 *   0 = INTERP_LINEAR    — линейная интерполяция углов
 *   1 = INTERP_SEQUENTIAL — последовательное перемещение сервоприводов
 *   2 = INTERP_GROUPED    — групповое перемещение
 *   3 = INTERP_AUTO       — автоматический выбор (по умолчанию)
 */
void handleM430(GCodeCommand &cmd) {
    int s = cmd.hasParam['S' - 'A'] ? (int)cmd.params['S' - 'A'] : 3;
    interpStrategy = (InterpolationStrategy)s;
    interpolator.setStrategy(interpStrategy);
    Serial.println("OK: Strategy = " + String(s));
}

/**
 * M500 — сохранение конфигурации в EEPROM.
 * Сохраняет текущую конфигурацию (config) и домашние углы (homeAngles).
 * Сохраняется при выключении питания и загружается при включении.
 */
void handleM500(GCodeCommand &cmd) {
    storage.saveConfig(config, homeAngles.angles, interpStrategy);
    Serial.println("OK: Saved to EEPROM");
}

/**
 * M501 — загрузка конфигурации из EEPROM.
 * Если данные валидны — загружает config и homeAngles,
 * обновляет конфигурацию всех модулей (servoCtrl, ikSolver, fkSolver, safety).
 */
void handleM501(GCodeCommand &cmd) {
    if (storage.loadConfig(config, homeAngles.angles)) {
        servoCtrl.setConfig(config.servos);
        ikSolver.setServoLimits(config.servos);
        fkSolver.setFingerLength(config.fingerLength);
        safety.setFingerLength(config.fingerLength);
        Serial.println("OK: Loaded from EEPROM");
    } else {
        Serial.println("ERROR: Invalid EEPROM data");
    }
}

/**
 * M502 — загрузка конфигурации по умолчанию (заводские настройки).
 * Сбрасывает config, homeAngles и обновляет все модули.
 * EEPROM НЕ перезаписывается — команда только сбрасывает текущее состояние.
 */
void handleM502(GCodeCommand &cmd) {
    config = DEFAULT_CONFIG;
    for (int i = 0; i < 7; i++) {
        homeAngles.angles[i] = DEFAULT_HOME_ANGLES[i];
    }
    servoCtrl.setConfig(config.servos);
    ikSolver.setServoLimits(config.servos);
    fkSolver.setFingerLength(config.fingerLength);
    safety.setFingerLength(config.fingerLength);
    Serial.println("OK: Factory defaults loaded");
}

/**
 * handleHelp (?) — вывод справки по всем доступным командам.
 * Формат: список всех G/M-команд с кратким описанием параметров.
 */
void handleHelp(GCodeCommand &cmd) {
    Serial.println("=== 6-DOF Robot Arm Commands ===");
    Serial.println("G0 X.. Y.. Z.. A.. B.. C.. J.. - Move (IK/Direct/Mixed)");
    Serial.println("G1 X.. Y.. Z.. F.. P1=.. P7=.. - Move with feedrate");
    Serial.println("G28                           - Home all axes");
    Serial.println("G28 H1                        - Park (fold into compact position)");
    Serial.println("G90                           - Absolute mode");
    Serial.println("G91                           - Relative mode");
    Serial.println("M104 P1=90 P2=0 ..            - Set servo angles directly");
    Serial.println("M105                          - Report servo angles");
    Serial.println("M114                          - Report position (FK)");
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
// Вспомогательные функции (helper functions)
// ====================================================================

/**
 * printServoAngles() — вывод текущих углов всех 7 сервоприводов.
 * Формат: "Angles: P1=0.0 P2=0.0 ... P7=0.0"
 * Используется обработчиком M105.
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
 * printPosition() — вычисление и вывод текущей позиции энд-эффектора
 * с помощью прямой кинематики (FK-solver).
 * Формат: "Position: X=0.0 Y=0.0 Z=0.0"
 * Используется обработчиком M114.
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
 * Включает: текущий режим координат, состояние безопасности,
 * стратегию интерполяции, домашнюю позицию, статус EEPROM,
 * а также текущие углы и позицию.
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
// Core 0: чтение Serial + управление движением (главная задача)
// ====================================================================
/**
 * Задача Core0_Motion — выполняется на ядре Core 0.
 *
 * Отвечает за:
 *   - Посимвольное чтение данных из Serial (буферизация строки)
 *   - Парсинг G-code при получении символа '\n' или '\r'
 *   - Выполнение команд (moveTo, установка углов и т.д.)
 *
 * Запускается на Core 0, чтобы не блокировать Core 1.
 * Движение сервоприводов (moveTo) тоже выполняется здесь —
 * это исключает межъядерные конфликты при обновлении PCA9685.
 *
 * Задержка vTaskDelay(5 мс) предотвращает потребление 100% CPU.
 */
void core0Task(void *pvParameters) {
    String buffer = "";

    while (true) {
        // Посимвольное чтение из Serial на Core 0
        while (Serial.available()) {
            char c = Serial.read();
            // Конец строки: парсим и выполняем команду
            if (c == '\n' || c == '\r') {
                if (buffer.length() > 0) {
                    // Эхо полученной команды в порт
                    Serial.print("> ");
                    Serial.println(buffer);

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

        // Пауза 5 мс: даёт время другим задачам (Core 1, FreeRTOS scheduler)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ====================================================================
// Core 1: фоновые задачи (дисплей, индикаторы и т.д.)
// ====================================================================
/**
 * Задача Core1_Serial — выполняется на ядре Core 1.
 *
 * В данный момент пуста (зарезервирована) — предназначена для:
 *   - Управления SPI-дисплеем (отображение углов, позиции, статуса)
 *   - Мигания светодиодами индикатора
 *   - Обработки кнопок
 *   - Любых фоновых задач, не требующих доступа к Serial
 *
 * Задержка vTaskDelay(100 мс) — достаточно для обновления дисплея
 * или обработки UI.
 */
void core1Task(void *pvParameters) {
    // Инициализация дисплея на Core 1 (SPI отдельный поток)
    displayInit();

    while (true) {
        // Обновление дисплея каждые200мс
        displayUpdate();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ====================================================================
// setup() — последовательность инициализации при старте контроллера
// ====================================================================
/**
 * setup() — вызывается один раз при загрузке ESP32.
 *
 * Последовательность:
 *   1. Инициализация Serial (115200 бод) для отладочного вывода
 *   2. Инициализация контроллера сервоприводов (PCA9685)
 *   3. Настройка интерполятора (связь с servoCtrl, backlash, safety)
 *   4. Инициализация контроллера захвата
 *   5. Инициализация EEPROM-хранилища
 *   6. Загрузка конфигурации из EEPROM (или использование значений по умолчанию)
 *   7. Создание мьютекса angleMutex для защиты currentAngles
 *   8. Создание задач FreeRTOS и привязка к ядрам:
 *        - Core 0 → core0Task (Serial + движение)
 *        - Core 1 → core1Task (фоновые задачи)
 *   9. Вывод приветственного сообщения и инструкций
 *
 * Функция loop() пуста — вся работа делается задачами FreeRTOS.
 */
void setup() {
    Serial.begin(115200);
    delay(1000);

    // === Инициализация дисплея ПЕРЕД I2C/PCA9685 ===
    // Если I2C зависнет — дисплей уже работает и покажет статус
    displayInit();
    
    // Вывод "6DOF" на экран (большие буквы)
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(3);  // Крупный шрифт
    tft.setCursor(40, 80);
    tft.println("  6DOF");
    tft.setTextSize(1);  // Возвращаем обычный размер
    tft.setCursor(20, 120);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.println("Initializing...");

    Serial.println("=== 6-DOF Robot Arm Controller ===");
    Serial.println("ESP32 @ 240 MHz, PCA9685, 7 servos");

    // Инициализация модулей
    servoCtrl.init();                   // PCA9685, I2C, ШИМ-каналы
    servoCtrl.setConfig(config.servos); // Установка ограничений сервоприводов

    // Связывание интерполятора с зависимостями
    interpolator.setServoController(&servoCtrl);
    interpolator.setBacklashCompensator(&backlash);
    interpolator.setSafetyChecker(&safety);
    interpolator.setStrategy(interpStrategy);

    gripper.setServoController(&servoCtrl);

    storage.begin();  // Инициализация EEPROM

    // Загрузка конфигурации из EEPROM (или значений по умолчанию)
    if (storage.isConfigValid()) {
        if (storage.loadConfig(config, homeAngles.angles)) {
            servoCtrl.setConfig(config.servos);
            ikSolver.setServoLimits(config.servos);
            fkSolver.setFingerLength(config.fingerLength);
            safety.setFingerLength(config.fingerLength);
            Serial.println("Config loaded from EEPROM");
        }
    } else {
        // EEPROM пуста — используем конфигурацию по умолчанию
        for (int i = 0; i < 7; i++) {
            homeAngles.angles[i] = DEFAULT_HOME_ANGLES[i];
        }
        fkSolver.setFingerLength(config.fingerLength);
        safety.setFingerLength(config.fingerLength);
        Serial.println("Using default config");
    }

    // Создание мьютекса для защиты currentAngles от гонок между задачами
    angleMutex = xSemaphoreCreateMutex();

    // Создание мьютекса для защиты SharedState от гонок между задачами
    displayMutex = xSemaphoreCreateMutex();

    // Регистрация callback интерполятора для обновления SharedState
    // в реальном времени (вызывается на каждом шаге ~20мс)
    interpolator.setStepCallback(onInterpolationStep);

    // Инициализация SharedState начальными значениями
    memset(&sharedState, 0, sizeof(sharedState));
    sharedState.safetyEnabled = safety.isEnabled();
    sharedState.tcpPosition = fkSolver.getEndEffector(homeAngles.angles);
    for (int i = 0; i < 7; i++) {
        sharedState.servoAngles[i] = homeAngles.angles[i];
    }

    // Создание задачи Core 0: чтение Serial + управление движением
    // Стек: 8192 байт, приоритет: 1, привязка к ядру 0
    xTaskCreatePinnedToCore(
        core0Task,
        "Core0_Motion",
        8192,
        NULL,
        1,
        &core0TaskHandle,
        0  // Core 0
    );

    // Создание задачи Core 1: дисплей ST7789
    // Стек: 8192 байт (TFT_eSPI требует больше памяти), приоритет: 1, привязка к ядру 1
    xTaskCreatePinnedToCore(
        core1Task,
        "Core1_Display",
        8192,
        NULL,
        1,
        &core1TaskHandle,
        1  // Core 1
    );

    Serial.println("Ready. Send G-code commands.");
    Serial.println("Protocol: send command, wait for OK/ERROR before next.");
    Serial.println("Type '?' for help");
}

// ====================================================================
// Дисплей: callback интерполятора
// ====================================================================
/**
 * onInterpolationStep() — вызывается на каждом шаге интерполяции (~20мс).
 * Обновляет SharedState текущими промежуточными углами и TCP-позицией.
 * Вызывается из Core0 (внутри interpolator.moveTo), защищается displayMutex.
 */
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
// Дисплей: константы и координаты зон (ST7789 240×240)
// ====================================================================

// Цвета
#define COLOR_ZONE_HEADER_BG  TFT_NAVY
#define COLOR_ZONE_HEADER_FG  TFT_CYAN
#define COLOR_FIELD_LABEL     TFT_WHITE
#define COLOR_TCP             TFT_GREEN
#define COLOR_JOINT           TFT_YELLOW
#define COLOR_GRIPPER         0xFDA0  // оранжевый
#define COLOR_READY           TFT_GREEN
#define COLOR_MOVING          TFT_YELLOW
#define COLOR_ERROR           TFT_RED

// Шрифт: Font 2 (16px высота строки)
#define FONT_HEIGHT  16
#define ZONE_GAP     4   // отступ между зонами

// Координаты зон (Y начала)
#define ZONE_TCP_Y      0
#define ZONE_TCP_H      56     // 12 header + 3×14 строки
#define ZONE_JOINTS_Y   60
#define ZONE_JOINTS_H   92     // 12 header + 6×14 строки
#define ZONE_GRIPPER_Y  156
#define ZONE_GRIPPER_H  36     // 12 header + 14 строка + 12 bar
#define ZONE_STATUS_Y   196
#define ZONE_STATUS_H   44     // 2 строки ×14 + отступы

// Поля для zone header: высота 12px
#define HEADER_H  12

// ====================================================================
// Дисплей: вспомогательные функции отрисовки
// ====================================================================

/**
 * drawZoneHeader() — рисует цветной заголовок зоны.
 * Фон: COLOR_ZONE_HEADER_BG, текст: COLOR_ZONE_HEADER_FG.
 */
static void drawZoneHeader(int y, const char *title) {
    tft.fillRect(0, y, 240, HEADER_H, COLOR_ZONE_HEADER_BG);
    tft.setCursor(4, y + 2);
    tft.setTextColor(COLOR_ZONE_HEADER_FG, COLOR_ZONE_HEADER_BG);
    tft.print(title);
}

/**
 * drawField() — рисует поле: название влево, число вправо.
 * Число выровнено по правому краю (240 - padding).
 * lastValue хранится для очистки предыдущего значения (antiflicker).
 */
static float lastTCP[3] = {9999, 9999, 9999};
static float lastJoints[7] = {9999, 9999, 9999, 9999, 9999, 9999, 9999};
static float lastGrip = -1;

static void drawField(int y, const char *label, float value, float &lastVal,
                      uint16_t color, const char *unit, bool forceRedraw = false) {
    if (!forceRedraw && fabs(value - lastVal) < 0.05f) return;
    lastVal = value;

    // Название влево (белый)
    tft.setTextColor(COLOR_FIELD_LABEL, TFT_BLACK);
    tft.setCursor(4, y);
    tft.print(label);

    // Очистка старого значения (чёрный прямоугольник справа)
    tft.fillRect(140, y, 100, FONT_HEIGHT, TFT_BLACK);

    // Новое число вправо
    char buf[16];
    snprintf(buf, sizeof(buf), "%6.1f%s", value, unit);
    int textW = tft.textWidth(buf);
    tft.setTextColor(color, TFT_BLACK);
    tft.setCursor(236 - textW, y);
    tft.print(buf);
}

// ====================================================================
// Дисплей: displayInit() и displayUpdate()
// ====================================================================

/**
 * displayInit() — инициализация ST7789 дисплея.
 * Устанавливает поворот, очищает экран чёрным, рисует заголовки зон.
 * Вызывается один раз из Core1 при старте.
 */
void displayInit() {
    tft.init();
    tft.setRotation(0);        // Квадратный — поворот не важен
    tft.fillScreen(TFT_BLACK);
    tft.setTextFont(2);        // Шрифт 2 (16px)

    // Начальная отрисовка заголовков зон и пустых полей
    drawZoneHeader(ZONE_TCP_Y, "TCP POSITION");
    drawZoneHeader(ZONE_JOINTS_Y, "JOINTS");
    drawZoneHeader(ZONE_GRIPPER_Y, "GRIPPER");

    // Рамка статус-бара
    tft.drawRect(0, ZONE_STATUS_Y, 240, ZONE_STATUS_H, 0x4208);

    // Инициализация last-значений для antiflicker
    for (int i = 0; i < 3; i++) lastTCP[i] = 9999;
    for (int i = 0; i < 7; i++) lastJoints[i] = 9999;
    lastGrip = -1;
}

/**
 * displayUpdate() — обновление данных на экране.
 * Читает SharedState через displayMutex, рисует только изменившиеся значения.
 * Вызывается из Core1 каждые 200мс.
 */
void displayUpdate() {
    SharedState state;
    bool moving;
    bool safety;
    bool error;
    uint32_t cmdCount;

    // Копирование SharedState под мьютексом
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    state = sharedState;
    xSemaphoreGive(displayMutex);

    // === TCP Position Zone ===
    drawField(ZONE_TCP_Y + HEADER_H + 0,  " X:", state.tcpPosition.x,
              lastTCP[0], COLOR_TCP, "mm");
    drawField(ZONE_TCP_Y + HEADER_H + 14, " Y:", state.tcpPosition.y,
              lastTCP[1], COLOR_TCP, "mm");
    drawField(ZONE_TCP_Y + HEADER_H + 28, " Z:", state.tcpPosition.z,
              lastTCP[2], COLOR_TCP, "mm");

    // === Joints Zone ===
    const char *jointNames[] = {
        " Base:", " Shoulder:", " Elbow:", " Wrist P:",
        " Wrist R:", " Wrist Y:", " Grip:"
    };
    for (int i = 0; i < 6; i++) {
        drawField(ZONE_JOINTS_Y + HEADER_H + i * 14,
                  jointNames[i], state.servoAngles[i],
                  lastJoints[i], COLOR_JOINT, "°");
    }

    // === Gripper Zone ===
    drawField(ZONE_GRIPPER_Y + HEADER_H, " Position:",
              state.gripperPercent, lastGrip, COLOR_GRIPPER, "%");

    // === Status Bar ===
    moving = state.isMoving;
    safety = state.safetyEnabled;
    error = state.hasError;
    cmdCount = state.commandCount;

    // Левая часть: статус (READY / MOVING / ERROR)
    tft.setCursor(4, ZONE_STATUS_Y + 4);
    if (error) {
        tft.setTextColor(COLOR_ERROR, TFT_BLACK);
        tft.print("! ERROR  ");
    } else if (moving) {
        tft.setTextColor(COLOR_MOVING, TFT_BLACK);
        tft.print("> MOVING ");
    } else {
        tft.setTextColor(COLOR_READY, TFT_BLACK);
        tft.print("  READY  ");
    }

    // Правая часть: Safety + команда
    tft.setCursor(4, ZONE_STATUS_Y + 20);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("Safety:");
    tft.setTextColor(safety ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(safety ? " ON" : "OFF");

    tft.setCursor(130, ZONE_STATUS_Y + 20);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("Cmd:");
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.print(cmdCount);

    // Нижняя строка: версия
    tft.setCursor(4, ZONE_STATUS_Y + 34);
    tft.setTextColor(0x8410, TFT_BLACK);  // серый
    tft.print("6DOF v1.0");
}

// ====================================================================
// loop() — пуста (вся работа делается задачами FreeRTOS)
// ====================================================================
/**
 * loop() — стандартная функция Arduino.
 * Пуста, так как вся логика выполняется в задачах FreeRTOS
 * (core0Task на Core 0, core1Task на Core 1).
 * Просто ждёт, чтобы не потреблять 100% CPU.
 */
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
