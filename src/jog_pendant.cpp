/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-10-01
 * Last modify : 2026-10-01
 */

/*
 * jog_pendant.cpp — реализация ручного пульта.
 * Квадратура — таблицей Gray-переходов 4x, E-STOP — прямым стопом.
 */

#include "jog_pendant.h"
#include "motion_interpolator.h"

// Таблица направлений: индекс = (prev<<2)|cur, значение = -1/0/+1.
static const int8_t QDIR[16] = {
     0, -1, +1, 0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1, 0,
};

JogPendant *JogPendant::instance_ = nullptr;

JogPendant::JogPendant()
    : interp_(nullptr), cur_(nullptr), mtx_(NULL), sel_(0),
      acc_(0), estopISR_(false), estopHit_(false),
      prevQ_(0), nextPrev_(true), nextT_(0) {
}

void JogPendant::begin(MotionInterpolator *interp, JointAngles *cur,
                       SemaphoreHandle_t mtx) {
    interp_ = interp;
    cur_ = cur;
    mtx_ = mtx;
    instance_ = this;
    if (!JOG_ENABLED) return;
    pinMode(JOG_PIN_ENC_A, INPUT_PULLUP);
    pinMode(JOG_PIN_ENC_B, INPUT_PULLUP);
    pinMode(JOG_PIN_NEXT, INPUT_PULLUP);
    pinMode(JOG_PIN_ESTOP, INPUT_PULLUP);
    prevQ_ = (digitalRead(JOG_PIN_ENC_A) << 1) | digitalRead(JOG_PIN_ENC_B);
    attachInterrupt(digitalPinToInterrupt(JOG_PIN_ENC_A), isrEnc, CHANGE);
    attachInterrupt(digitalPinToInterrupt(JOG_PIN_ENC_B), isrEnc, CHANGE);
    attachInterrupt(digitalPinToInterrupt(JOG_PIN_ESTOP), isrEstop, FALLING);
}

void IRAM_ATTR JogPendant::isrEnc() {
    JogPendant *p = instance_;
    if (!p) return;
    uint8_t s = (digitalRead(JOG_PIN_ENC_A) << 1) | digitalRead(JOG_PIN_ENC_B);
    p->acc_ += QDIR[(p->prevQ_ << 2) | s];
    p->prevQ_ = s;
}

void IRAM_ATTR JogPendant::isrEstop() {
    JogPendant *p = instance_;
    if (!p || !p->interp_) return;
    p->interp_->stop();      // только стоп движения, сервы под током
    p->estopISR_ = true;
}

void JogPendant::poll() {
    if (!JOG_ENABLED || !interp_ || !cur_) return;

    if (estopISR_) {
        estopISR_ = false;
        estopHit_ = true;
        acc_ = 0;   // набранное за время стопа сбрасываем
        Serial.println("JOG: E-STOP (motion halted, servos powered)");
    }

    // Кнопка NEXT: листаем P1..P7 по кругу (debounce 200 мс).
    bool nb = digitalRead(JOG_PIN_NEXT);
    uint32_t now = millis();
    if (!nb && nextPrev_ && (now - nextT_) > 200) {
        nextT_ = now;
        sel_ = (sel_ + 1) % 7;
        Serial.print("JOG: channel P");
        Serial.println(sel_ + 1);
    }
    nextPrev_ = nb;

    // Детенты -> движение. Недобор до детента остаётся в acc_ (копим).
    long detents;
    noInterrupts();
    detents = acc_ / JOG_COUNTS_PER_DETENT;
    acc_ = acc_ % JOG_COUNTS_PER_DETENT;
    interrupts();
    if (detents == 0) return;
    if (detents == 0) return;

    float step = (sel_ == 6) ? JOG_GRIP_STEP : JOG_STEP_DEG;
    JointAngles target;
    if (mtx_) xSemaphoreTake(mtx_, portMAX_DELAY);
    target = *cur_;
    if (mtx_) xSemaphoreGive(mtx_);
    target.angles[sel_] += (float)detents * step;
    // Зажим в лимиты и safety — внутри moveTo (единая точка).
    interp_->moveTo(target, JOG_FEEDRATE);
}
