/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * kin_diag.h — журнал последнего движения для M115 (разбор кинематики).
 *
 * Ветка ref-kin-diag: построчный перенос из main.cpp, логики ноль.
 * Заполняется в handleG0 (G0/G1), handleM104, handleG28.
 * Читается в handleM115. Всё выполняется в задаче core0,
 * поэтому дополнительный мьютекс не нужен.
 */

#pragma once

#include <Arduino.h>
#include "types.h"

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

extern DiagLastMove diagLast;

/**
 * Посчитать FK + ошибки + safety для кандидата и положить в DiagSol.
 */
void diagEvalSol(DiagSol &s, const JointAngles &q, const Pose &target);

/**
 * Начать новую запись журнала: текст команды берём из sharedState.
 */
void diagBegin(const char *kind, float feedrate);

/**
 * Печать цепочки суставов J0..J6.
 */
void diagPrintJoints(const Vec3 joints[7]);

/**
 * M115 — разбор кинематики: куда шли (цель), кандидаты IK,
 * выбранное решение, куда пришли (факт), расхождение.
 * Параметр S=1 — полные цепочки J0..J6 для всех кандидатов.
 */
void handleM115(GCodeCommand &cmd);
