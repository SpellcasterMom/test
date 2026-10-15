/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-10-01
 */

/*
 * gripper_controller.cpp — Реализация контроллера захвата
 *
 * Управление захватом роботической манипуляции. Захват приводится в движение
 * сервоприводом с индексом 6. Положение задаётся в процентах:
 *   0%   — захват полностью закрыт (клеммы сведены)
 *   50%  — среднее положение (начальное по умолчанию)
 *   100% — захват полностью открыт (клеммы разведены)
 *
 * Контроллер делегирует физическое управление сервоприводом объекту
 * ServoController, вызывая setAngle() с соответствующим индексом.
 */

#include "gripper_controller.h"

/**
 * GripperController — Конструктор по умолчанию.
 *
 * Инициализация:
 *   servoCtrl = nullptr — контроллер сервоприводов не привязан (нужно вызвать setServoController)
 *   currentPosition = 50.0f — начальная позиция захвата 50% (среднее положение)
 *   gripperServo = 6 — индекс сервопривода захвата (6-й сервопривод в системе)
 */
GripperController::GripperController()
    : servoCtrl(nullptr), currentPosition(50.0f), gripperServo(6) {}

/**
 * setServoController — Привязка контроллера сервоприводов.
 * Все последующие вызовы setPosition()/open()/close() будут использовать
 * этот объект для физического управления сервоприводом захвата.
 *
 * @param ctrl — указатель на инициализированный ServoController
 */
void GripperController::setServoController(ServoController *ctrl) {
    servoCtrl = ctrl;
}

/**
 * setPosition — Установка позиции захвата в процентах.
 *
 * Алгоритм:
 *   1. Ограничение входного значения диапазоном [0.0, 100.0] через constrain().
 *   2. Сохранение ограниченного значения в currentPosition.
 *   3. Если контроллер сервоприводов привязан (servoCtrl != nullptr),
 *      вызов servoCtrl->setAngle(gripperServo, currentPosition) для
 *      физического перемещения сервопривода.
 *
 * @param percent — позиция захвата: 0% (закрыт) .. 100% (открыт)
 */
void GripperController::setPosition(float percent) {
    currentPosition = constrain(percent, 0.0f, 100.0f);

    if (servoCtrl) {
        servoCtrl->setAngle(gripperServo, gripperPctToAngle(currentPosition));
    }
}

/**
 * getPosition — Получение текущей позиции захвата.
 * @return текущая позиция в процентах (0.0 - 100.0)
 */
float GripperController::getPosition() const {
    return currentPosition;
}

/**
 * open — Полное открытие захвата.
 * Устанавливает позицию 100.0% (максимальное раскрытие клемм).
 */
void GripperController::open() {
    setPosition(100.0f);
}

/**
 * close — Полное закрытие захвата.
 * Устанавливает позицию 0.0% (максимальное сжатие клемм).
 */
void GripperController::close() {
    setPosition(0.0f);
}
