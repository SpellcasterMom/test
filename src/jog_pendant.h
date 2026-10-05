/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Almamater   : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-10-01
 * Last modify : 2026-10-01
 */

/*
 * jog_pendant.h — ручной пульт: штурвал (квадратурный энкодер) + кнопка
 * выбора сустава + кнопка аварийного стопа. Всё на пинах ESP32.
 *
 * Органы:
 *   - штурвал EC11 (A/B, квадратура): крутит ВЫБРАННЫЙ канал;
 *   - BTN_NEXT: короткое нажатие = следующий канал P1..P7 (захват тоже);
 *   - BTN_ESTOP: отдельный пин, просто кнопка. Прерывание =
 *     interpolator.stop() (ТОЛЬКО стоп движения, сервы остаются
 *     под током — как просили, НЕ как M112). Вооружён всегда,
 *     даже при JOG_ENABLED=false.
 *
 * Движение идёт через MotionInterpolator::moveTo: ease-in-out, лимиты
 * угловых скоростей, зажим в лимиты серв, safety. Шаг: 1°/детент для
 * P1–P6, 2%/детент для P7 (захват 0–100%).
 *
 * Потоки: ISR только копит детенты/стоп-флаг; движение — из задачи
 * (pendantTask в main.cpp, опрос каждые 20 мс). В ISR ни Serial, ни delay.
 * Входы с INPUT_PULLUP: неподключённый пульт = стабильная HIGH, ложных
 * срабатываний нет.
 */

#pragma once

#include <Arduino.h>
#include "types.h"   // JointAngles

// ====================================================================
// Конфиг пульта (менять здесь)
// ====================================================================
#ifndef JOG_ENABLED
#define JOG_ENABLED true        // false = пульт отключён (нулевой эффект)
#endif
#ifndef JOG_PIN_ENC_A
#define JOG_PIN_ENC_A 32        // штурвал A (нужен pullup — не 34-39!)
#endif
#ifndef JOG_PIN_ENC_B
#define JOG_PIN_ENC_B 33        // штурвал B
#endif
#ifndef JOG_PIN_NEXT
#define JOG_PIN_NEXT 25         // кнопка выбора канала (активный LOW)
#endif
#ifndef JOG_PIN_ESTOP
#define JOG_PIN_ESTOP 26        // кнопка стоп (активный LOW, прерывание)
#endif
#ifndef JOG_STEP_DEG
#define JOG_STEP_DEG 1.0f       // градусов на детент (P1–P6)
#endif
#ifndef JOG_GRIP_STEP
#define JOG_GRIP_STEP 2.0f      // процентов на детент (P7, 0–100%)
#endif
#ifndef JOG_COUNTS_PER_DETENT
#define JOG_COUNTS_PER_DETENT 2 // фронтов Gray-кода на детент EC11
#endif
#ifndef JOG_FEEDRATE
#define JOG_FEEDRATE 20.0f      // подача джога (%), лимиты серв сверху
#endif

class MotionInterpolator;   // fwd: полный тип нужен только в .cpp

class JogPendant {
public:
    JogPendant();

    /** Привязка к интерполятору/состоянию + навешивание прерываний. */
    void begin(MotionInterpolator *interp, JointAngles *cur, SemaphoreHandle_t mtx);

    /**
     * Опрос из задачи (каждые ~20 мс):
     * кнопка NEXT (debounce) + потребление накопленных детентов движением.
     * Блокирует задачу на время moveTo (мелкие шаги — десятки мс).
     */
    void poll();

    /** Выбранный канал 0-6 (P1..P7). */
    int selected() const { return sel_; }
    /** Был удар по E-STOP (флаг снимается чтением через poll). */
    bool estopHit() const { return estopHit_; }

private:
    MotionInterpolator *interp_;
    JointAngles        *cur_;
    SemaphoreHandle_t   mtx_;
    int                 sel_;        // 0..6
    volatile long       acc_;        // накопленные фронты Gray-кода (ISR)
    volatile bool       estopISR_;   // флаг из ISR E-STOP
    bool                estopHit_;   // уже залогированный удар
    uint8_t             prevQ_;      // прошлое состояние A/B
    bool                nextPrev_;   // прошлое состояние BTN_NEXT
    uint32_t            nextT_;      // время последнего переключения

    static JogPendant *instance_;
    static void IRAM_ATTR isrEnc();
    static void IRAM_ATTR isrEstop();
};
