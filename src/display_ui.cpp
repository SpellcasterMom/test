/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * display_ui.cpp — реализация дисплея (перенос из main.cpp 1:1).
 */

#include "display_ui.h"
#include "app.h"
#include "config.h"
#include <cmath>
#include <cstring>

LGFX tft;

SharedState    sharedState;
SemaphoreHandle_t displayMutex = NULL;

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
 * Множитель DEG_PER_RAD — общий из types.h.
 */
static void toolEulerZYX(const float q[6], float &a, float &b, float &c) {
    Mat4 R = fkSolver.getRotationMatrix(q, 5);
    float sp = -R.m[2][0];
    if (sp > 0.9999f) {
        b = 90.0f; a = 0.0f;
        c = atan2(-R.m[0][1], R.m[0][2]) * DEG_PER_RAD;
    } else if (sp < -0.9999f) {
        b = -90.0f; a = 0.0f;
        c = atan2(-R.m[0][1], -R.m[0][2]) * DEG_PER_RAD;
    } else {
        b = asin(sp) * DEG_PER_RAD;
        a = atan2(R.m[2][1], R.m[2][2]) * DEG_PER_RAD;
        c = atan2(R.m[1][0], R.m[0][0]) * DEG_PER_RAD;
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
    drawField(4,   40, ZONE_JOINTS_Y + HEADER_H + 2,  "P1:", state.servoAngles[0], lastJoints[0], COLOR_JOINT, "");
    drawFieldR(124, 232, ZONE_JOINTS_Y + HEADER_H + 2,  "P4:", state.servoAngles[3], lastJoints[3], COLOR_JOINT);

    drawField(4,   40, ZONE_JOINTS_Y + HEADER_H + 18, "P2:", state.servoAngles[1], lastJoints[1], COLOR_JOINT, "");
    drawFieldR(124, 232, ZONE_JOINTS_Y + HEADER_H + 18, "P5:", state.servoAngles[4], lastJoints[4], COLOR_JOINT);

    drawField(4,   40, ZONE_JOINTS_Y + HEADER_H + 34, "P3:", state.servoAngles[2], lastJoints[2], COLOR_JOINT, "");
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
    // Рисуем только при смене: безусловная перерисовка каждые 200мс
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
    // Кружок = наличие железа; GYR двухцветный (один чип BMI160:
    // гиро+аксель всегда вместе): зелёный = чип найден, красный = нет гиро.
    // Текст не пишем.
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
    uint16_t gyrColor = (state.imuState != 0) ? TFT_GREEN : TFT_RED;
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
