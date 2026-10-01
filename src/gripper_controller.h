/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       : УТС/Б-23-1-О
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-18
 */

/*
 * gripper_controller.h — Заголовочный файл контроллера захвата
 *
 * Контроллер захвата управляет сервоприводом захвата (6-й сервопривод)
 * и предоставляет простой интерфейс для управления его положением.
 *
 * Основные принципы:
 *  1. Позиция захвата задаётся в процентах от 0% до 100%:
 *     - 0% — захват полностью закрыт (максимальное сжатие)
 *     - 50% — захват в среднем положении (по умолчанию при инициализации)
 *     - 100% — захват полностью открыт (максимальное раскрытие)
 *
 *  2. Контроллер делегирует управление сервоприводом объекту ServoController,
 *     вызывая метод setAngle() с индексом серво 6.
 *
 *  3. Методы open() и close() — convenience-обёртки для setPosition(100)
 *     и setPosition(0) соответственно.
 *
 * Использование:
 *   GripperController gripper;
 *   gripper.setServoController(&servoCtrl);
 *   gripper.setPosition(75.0f);  // открыть на 75%
 *   gripper.open();              // полностью открыть
 *   gripper.close();             // полностью закрыть
 */

#pragma once

#include "servo_controller.h"

class GripperController {
public:
    /**
     * GripperController — Конструктор по умолчанию.
     * Инициализирует контроллер с начальным положением 50% (среднее)
     * и индексом сервопривода 6 (шестой сервопривод — захват).
     */
    GripperController();

    /**
     * setServoController — Установка указателя на контроллер сервоприводов.
     * Все операции управления захватом делегируются этому объекту.
     * @param ctrl — указатель на инициализированный ServoController
     */
    void setServoController(ServoController *ctrl);

    /**
     * setPosition — Установка позиции захвата в процентах.
     *
     * Значение автоматически ограничивается диапазоном [0, 100]
     * с помощью constrain(). После установки значения вызывается
     * servoCtrl->setAngle(gripperServo, currentPosition) для
     * физического перемещения сервопривода захвата.
     *
     * @param percent — позиция захвата от 0% (закрыт) до 100% (открыт)
     */
    void setPosition(float percent);

    /**
     * getPosition — Получение текущей позиции захвата в процентах.
     * @return текущая позиция от 0% до 100%
     */
    float getPosition() const;

    /**
     * open — Полное открытие захвата (setPosition(100.0f)).
     */
    void open();

    /**
     * close — Полное закрытие захвата (setPosition(0.0f)).
     */
    void close();

private:
    ServoController *servoCtrl;  // Указатель на контроллер сервоприводов
    float currentPosition;       // Текущая позиция захвата в % (0-100)
    int   gripperServo;          // Индекс сервопривода захвата (6)
};
