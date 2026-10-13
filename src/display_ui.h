/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * display_ui.h — дисплей ST7789 + общее состояние для него.
 *
 * Ветка ref-display-ui: построчный перенос из main.cpp, логики ноль.
 * SharedState + инстанс + displayMutex живут ЗДЕСЬ (а не в app.h):
 * пишет их кто угодно (хендлеры — в следующих ветках через этот заголовок),
 * читает только displayUpdate. Остальное состояние — в app.h.
 */

#pragma once

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include "types.h"
#include "config.h"

// ===== Дисплей ST7789 (LovyanGFX, DMA) =====
// Экран 1.54" 240×240. Пины — из config.h (блок «Пины и шины»);
// здесь только ручки под конкретную партию стекла:
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
            cfg.pin_sclk   = DISP_SPI_SCLK_PIN;
            cfg.pin_mosi   = DISP_SPI_MOSI_PIN;
            cfg.pin_miso   = -1;
            cfg.pin_dc     = DISP_PIN_DC;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs         = DISP_PIN_CS;
            cfg.pin_rst        = DISP_PIN_RST;
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

extern LGFX tft;

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

extern SharedState    sharedState;
extern SemaphoreHandle_t displayMutex;

/**
 * displayInit() — инициализация ST7789 дисплея.
 */
void displayInit();

/**
 * displayUpdate() — обновление данных на экране.
 * Вызывается из Core1 каждые 200 мс.
 */
void displayUpdate();

/**
 * onInterpolationStep() — callback интерполятора.
 */
void onInterpolationStep(const JointAngles &angles);
