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
 * detour_planner.h — планировщик объезда запретных зон (ТОЛЬКО план,
 * исполнение — обычным moveTo по вейпоинтам из вызывающего кода).
 * Интерполятор НЕ трогаем.
 *
 * Прототип проверен в tools/twin/detour.py на тех же правилах:
 *   прямо (IK + проверка ломаной) → грани коробов + середина (IK) →
 *   RRT в joint-space → сглаживание срезами.
 * Возврат: вейпоинты joint-конфигураций, последний = цель;
 * 0 = пути нет (как раньше — отказ вместо прыжка в мусор).
 * Без зон (count==0): всегда 1 (сама цель) — поведение как moveTo.
 */

#pragma once

#include "types.h"
#include "ik_solver.h"
#include "fk_solver.h"
#include "safety_checker.h"
#include "servo_controller.h"

class DetourPlanner {
public:
    DetourPlanner();

    void setIKSolver(IKSolver *ik);
    void setFKSolver(FKSolver *fk);
    void setSafetyChecker(SafetyChecker *safety);
    void setServoController(ServoController *ctrl);

    /**
     * Построить цепочку вейпоинтов qstart -> target.
     * @param qstart  стартовые углы (6 суставов)
     * @param target  целевые углы (уже проверены вызывающим кодом)
     * @param pose    целевая поза TCP (для IK граневых точек) или nullptr
     *                (тогда только RRT без граневой фазы)
     * @param wps     выходной массив вейпоинтов (joint-конфиги)
     * @param cap     вместимость wps (рекомендуется 8)
     * @return число вейпоинтов (последний = target), 0 = пути нет
     */
    int plan(const float qstart[6], const JointAngles &target,
             const Pose *pose, JointAngles *wps, int cap);

private:
    IKSolver        *ikSolver = nullptr;
    FKSolver        *fkSolver = nullptr;
    SafetyChecker   *safety = nullptr;
    ServoController *servoCtrl = nullptr;
};
