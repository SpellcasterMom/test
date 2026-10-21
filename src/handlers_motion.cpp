/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * handlers_motion.cpp — обработчики движения (перенос из main.cpp 1:1).
 */

#include "handlers_motion.h"
#include "app.h"
#include "display_ui.h"
#include "kin_diag.h"
#include "config.h"
#include <cmath>
#include <cstring>

/**
 * G0 — перемещение (быстрое).
 */
void handleG0(GCodeCommand &cmd) {
    if (interpolator.batchOpen() || interpolator.streamBusy()) {
        Serial.println("ERROR: batch open - close it first (M450 S0)");
        return;
    }
    cmd.determineMode();

    // Сброс ошибки при новой команде
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    // Эффективная поза: без A/B/C берём дефолт «палец вперёд +X»
    // (DEFAULT_ORI_*). M115 покажет её же с пометкой (orient=default).
    Pose effPose = cmd.pose;
    if (!cmd.hasOrientation) {
        effPose.orientation.x = DEFAULT_ORI_A;
        effPose.orientation.y = DEFAULT_ORI_B;
        effPose.orientation.z = DEFAULT_ORI_C;
    }

    // Поосевая семантика координат (чинило молчаливый G91):
    //   - отсутствующая ось = держать текущую (а не 0!);
    //   - в REL (G91) заданная ось = текущая TCP + дельта.
    // Без этого G91 игнорировался (ехали в абсолют), а частичный
    // G0 X20 в ABS уводил Y,Z в ноль. A/B/C в REL — абсолютные.
    {
        float cq[6];
        for (int i = 0; i < 6; i++) cq[i] = currentAngles.angles[i];
        Vec3 curTcp = fkSolver.getEndEffector(cq);
        if (!absoluteMode) {
            effPose.position.x = cmd.hasX ? curTcp.x + cmd.pose.position.x : curTcp.x;
            effPose.position.y = cmd.hasY ? curTcp.y + cmd.pose.position.y : curTcp.y;
            effPose.position.z = cmd.hasZ ? curTcp.z + cmd.pose.position.z : curTcp.z;
        } else {
            if (!cmd.hasX) effPose.position.x = curTcp.x;
            if (!cmd.hasY) effPose.position.y = curTcp.y;
            if (!cmd.hasZ) effPose.position.z = curTcp.z;
        }
    }

    JointAngles target;

    switch (cmd.mode) {
        case GCodeCommand::IK_MODE: {
            // Обратная кинематика: позиция + ориентация → углы суставов
            // Ищем безопасное решение среди всех вариантов (elbow up/down)
            diagBegin("IK", cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE);
            diagLast.target = effPose;
            diagLast.hasPos = cmd.hasPosition;
            diagLast.hasOri = cmd.hasOrientation;
            JointAngles solutions[4];
            IKSolutionMeta metas[4];
            int solCount = ikSolver.solveAll(effPose, solutions, metas);

            if (solCount == 0) {
                strncpy(diagLast.failReason, "IK solution not found",
                        sizeof(diagLast.failReason) - 1);
                interpolator.setMark("NONE", "no motion (IK refused)");
                xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
                sharedState.hasError = true;
                snprintf(sharedState.lastError, sizeof(sharedState.lastError), "IK not found");
                xSemaphoreGive(displayMutex);
                Serial.println("ERROR: IK solution not found");
                return;
            }

            // Оцениваем каждое решение (FK + ошибки + safety) для M115
            for (int i = 0; i < solCount && diagLast.solCount < 4; i++) {
                if (cmd.hasGripper) {
                    solutions[i].angles[6] = gripperPctToAngle(cmd.pose.gripper);
                }
                diagEvalSol(diagLast.sols[diagLast.solCount], solutions[i], effPose);
                diagLast.sols[diagLast.solCount].iters = metas[i].iters;
                diagLast.sols[diagLast.solCount].converged = metas[i].converged;
                diagLast.sols[diagLast.solCount].seedPos = metas[i].seedPos;
                diagLast.sols[diagLast.solCount].seedOri = metas[i].seedOri;
                diagLast.sols[diagLast.solCount].dbgDq0 = metas[i].dbgDq0;
                diagLast.sols[diagLast.solCount].dbgN1 = metas[i].dbgN1;
                diagLast.solCount++;
            }

            // Кандидат от текущей позы (трекинг при стриминге мелких шагов):
            // DLS из current сходится за ~2 итерации (двойник: 40/40 из ±10°).
            // Общий выбор min-residual сам предпочтёт его, когда он хорош.
            // Слот в diagLast.sols есть (максимум 4, solveAll даёт ≤3).
            if (diagLast.solCount < 4 && solCount > 0) {
                JointAngles curSeed = currentAngles;
                IKSolutionMeta curMeta;
                ikSolver.refineSolution(effPose, curSeed, &curMeta);
                if (cmd.hasGripper) {
                    curSeed.angles[6] = gripperPctToAngle(cmd.pose.gripper);
                }
                diagEvalSol(diagLast.sols[diagLast.solCount], curSeed, effPose);
                diagLast.sols[diagLast.solCount].iters = curMeta.iters;
                diagLast.sols[diagLast.solCount].converged = curMeta.converged;
                diagLast.sols[diagLast.solCount].seedPos = curMeta.seedPos;
                diagLast.sols[diagLast.solCount].seedOri = curMeta.seedOri;
                diagLast.sols[diagLast.solCount].dbgDq0 = curMeta.dbgDq0;
                diagLast.sols[diagLast.solCount].dbgN1 = curMeta.dbgN1;
                diagLast.solCount++;
            }

            // Выбираем безопасное решение с минимальной расхождением:
            // score = dPos[мм] + 1.0·dOri[град] + CONT_W·jump[град от текущей].
            // Чистый min-residual выбирал ветвь в 250° ради 0.3 мм
            // (двойник: круг) — резкие перебросы без необходимости. CONT_W=0.02:
            // 50° прыжок окупается лишь при +1мм/+1° точности.
            // «Первый безопасный» систематически брал худший вариант
            // (M115: [0] 2.7мм/23.8° вместо [1] 2.2мм/0.0°).
            int best = -1;
            float bestScore = 0;
            // Порог высотного штрафа: ниже — предпочитаем верхние ветви.
            // Срезка min(15): в честно-низкой работе все ветви внизу,
            // штраф у всех максимальный и одинаковый — решает расхождение.
            const float HEIGHT_Z_REF = 150.0f;   // мм
            const float HEIGHT_W = 0.15f;        // штрафа за мм ниже порога
            const float HEIGHT_MAX = 15.0f;      // потолок штрафа
            for (int i = 0; i < diagLast.solCount; i++) {
                if (!diagLast.sols[i].safe) continue;
                float jd = 0;
                for (int j = 0; j < 6; j++) {
                    float d = fabs(diagLast.sols[i].q.angles[j] - currentAngles.angles[j]);
                    if (d > jd) jd = d;
                }
                float hPen = diagLast.sols[i].minZ < HEIGHT_Z_REF
                    ? HEIGHT_W * (HEIGHT_Z_REF - diagLast.sols[i].minZ) : 0.0f;
                if (hPen > HEIGHT_MAX) hPen = HEIGHT_MAX;
                float score = diagLast.sols[i].dPos + diagLast.sols[i].dOri + 0.02f * jd + hPen;
                if (best < 0 || score < bestScore) {
                    best = i;
                    bestScore = score;
                }
            }
            bool foundSafe = (best >= 0);
            // Порог точности: лучший безопасный кандидат с расхождением хуже
            // порога — это уже не заданное движение (напр. 127 мм мимо из-за
            // отбраковки точных, но опасных). Отказываем вместо прыжка в мусор.
            // Рабочие ходы идут с dPos<1мм/dOri в единицы градусов (двойник+железо).
            const float IK_POS_TOL = 10.0f;   // мм
            const float IK_ORI_TOL = 15.0f;   // град
            if (foundSafe &&
                (diagLast.sols[best].dPos > IK_POS_TOL ||
                 diagLast.sols[best].dOri > IK_ORI_TOL)) {
                foundSafe = false;
                char tmp[96];
                snprintf(tmp, sizeof(tmp),
                         "No accurate safe IK (best dPos=%.1f dOri=%.1f)",
                         diagLast.sols[best].dPos, diagLast.sols[best].dOri);
                strncpy(diagLast.failReason, tmp, sizeof(diagLast.failReason) - 1);
            }
            if (foundSafe) {
                target = diagLast.sols[best].q;
                diagLast.chosen = best;
            }

            if (!foundSafe) {
                // Не затираем причину, уже записанную порогом точности выше
                // ("No accurate safe IK..."), — иначе диагностика врёт.
                if (diagLast.failReason[0] == '\0') {
                    strncpy(diagLast.failReason, "No safe IK solution",
                            sizeof(diagLast.failReason) - 1);
                }
                interpolator.setMark("NONE", "no motion (refused)");
                xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
                sharedState.hasError = true;
                snprintf(sharedState.lastError, sizeof(sharedState.lastError), "No safe IK");
                xSemaphoreGive(displayMutex);
                Serial.println("ERROR: No safe IK solution");
                return;
            }
            break;
        }
        case GCodeCommand::DIRECT_MODE: {
            target = cmd.servoAngles;
            diagBegin("DIRECT", 0);
            diagLast.anglesOnly = true;
            diagEvalSol(diagLast.sols[0], target, cmd.pose);
            diagLast.solCount = 1;
            diagLast.chosen = 0;
            break;
        }
        case GCodeCommand::MIXED_MODE: {
            diagBegin("MIXED", cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE);
            diagLast.target = effPose;
            diagLast.hasPos = cmd.hasPosition;
            diagLast.hasOri = cmd.hasOrientation;
            target = currentAngles;  // страховка: если solve() не найдёт решение,
                                     // остаёмся на месте, а не едем в мусор
            bool ikOk = true;
            if (cmd.hasPosition || cmd.hasOrientation) {
                ikOk = ikSolver.solve(effPose, target);
            }
            for (int i = 0; i < 7; i++) {
                if (cmd.servoOverride[i]) {
                    target.angles[i] = cmd.servoAngles.angles[i];
                }
            }
            if (!ikOk && (cmd.hasPosition || cmd.hasOrientation)) {
                strncpy(diagLast.failReason, "IK solve failed (MIXED moved anyway)",
                        sizeof(diagLast.failReason) - 1);
            }
            diagEvalSol(diagLast.sols[0], target, effPose);
            diagLast.solCount = 1;
            diagLast.chosen = 0;
            // solve() выше уже довёл решение; забираем meta для M115
            // (доводка копии — железо не двигаем, дёшево: уже сошлось)
            {
                JointAngles tq = target;
                IKSolutionMeta mm;
                ikSolver.refineSolution(effPose, tq, &mm);
                diagLast.sols[0].iters = mm.iters;
                diagLast.sols[0].converged = mm.converged;
                diagLast.sols[0].seedPos = mm.seedPos;
                diagLast.sols[0].seedOri = mm.seedOri;
                diagLast.sols[0].dbgDq0 = mm.dbgDq0;
                diagLast.sols[0].dbgN1 = mm.dbgN1;
            }
            break;
        }
    }

    // Проверка безопасности
    if (safety.isEnabled()) {
        if (!safety.checkAllLinks(target.angles)) {
            strncpy(diagLast.failReason, safety.getLastError().c_str(),
                    sizeof(diagLast.failReason) - 1);
            xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
            sharedState.hasError = true;
            snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Safety");
            xSemaphoreGive(displayMutex);
            Serial.print("ERROR: Safety violation: ");
            Serial.println(safety.getLastError());
            return;
        }
    }

    float feedrate = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
    // Старт траектории и времени — из лучшего известного положения.
    // Без этого проверка идёт от устаревшей точки интерполятора
    // (после ребута — нули = недостижимый вылет 415мм) и даёт ложный
    // фолбэк на sequential с шага 0.
    interpolator.setCurrentAngles(currentAngles);
    // Объезд препятствий: планировщик даёт цепочку joint-вейпоинтов
    // (последний = target), едем каждый обычным moveTo. Сам интерполятор
    // не тронут. DIRECT-режиму поза ни к чему — только RRT (nullptr).
    float qs0[6];
    for (int i = 0; i < 6; i++) qs0[i] = currentAngles.angles[i];
    const Pose *pp = (cmd.mode == GCodeCommand::DIRECT_MODE) ? nullptr : &effPose;
    JointAngles wps[8];
    int nwp = detour.plan(qs0, target, pp, wps, 8);
    // TCP вейпоинтов в журнал (видны в M115, даже если движение сорвётся).
    diagLast.detourCount = 0;
    for (int w = 0; w < nwp && w < 8; w++) {
        float qw[6];
        for (int i = 0; i < 6; i++) qw[i] = wps[w].angles[i];
        diagLast.detourPts[diagLast.detourCount++] = fkSolver.getEndEffector(qw);
    }
    bool moved = false;
    if (nwp <= 0) {
        strncpy(diagLast.failReason, "No path: obstacle blocks all routes",
                sizeof(diagLast.failReason) - 1);
    } else {
        moved = true;
        for (int w = 0; w < nwp && moved; w++) {
            if (!interpolator.moveTo(wps[w], feedrate)) moved = false;
        }
        if (!moved) {
            strncpy(diagLast.failReason, "Move failed on detour leg",
                    sizeof(diagLast.failReason) - 1);
        }
    }
    if (moved) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        for (int i = 0; i < 7; i++) {
            sharedState.servoAngles[i] = target.angles[i];
        }
        float q[6];
        for (int i = 0; i < 6; i++) q[i] = target.angles[i];
        sharedState.tcpPosition = fkSolver.getEndEffector(q);
        sharedState.gripperPercent = gripperAngleToPct(target.angles[6]);
        sharedState.isMoving = false;
        sharedState.hasError = false;
        sharedState.commandCount++;
        xSemaphoreGive(displayMutex);

        Serial.println("OK");
    } else {
        // Не затираем конкретную причину (nopath / detour leg),
        // записанную выше.
        if (diagLast.failReason[0] == '\0') {
            strncpy(diagLast.failReason, "Move failed (interpolator)",
                    sizeof(diagLast.failReason) - 1);
        }
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: Move failed");
    }
}

/**
 * G1 — декартово движение (прямая TCP).
 * Чистый IK-режим: TCP идёт по прямой (lerp + slerp), IK на каждом шаге.
 * DIRECT/MIXED — как G0 (joint-интерполяция).
 */
void handleG1(GCodeCommand &cmd) {
    cmd.determineMode();
    if (cmd.mode != GCodeCommand::IK_MODE) {
        handleG0(cmd);
        return;
    }

    // Сброс ошибки при новой команде
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    // Эффективная поза: как в G0 (дефолт ориентации + поосевая семантика).
    Pose effPose = cmd.pose;
    if (!cmd.hasOrientation) {
        effPose.orientation.x = DEFAULT_ORI_A;
        effPose.orientation.y = DEFAULT_ORI_B;
        effPose.orientation.z = DEFAULT_ORI_C;
    }
    {
        float cq[6];
        for (int i = 0; i < 6; i++) cq[i] = currentAngles.angles[i];
        Vec3 curTcp = fkSolver.getEndEffector(cq);
        if (!absoluteMode) {
            effPose.position.x = cmd.hasX ? curTcp.x + cmd.pose.position.x : curTcp.x;
            effPose.position.y = cmd.hasY ? curTcp.y + cmd.pose.position.y : curTcp.y;
            effPose.position.z = cmd.hasZ ? curTcp.z + cmd.pose.position.z : curTcp.z;
        } else {
            if (!cmd.hasX) effPose.position.x = curTcp.x;
            if (!cmd.hasY) effPose.position.y = curTcp.y;
            if (!cmd.hasZ) effPose.position.z = curTcp.z;
        }
    }
    // Захват: как в G0 (без J — в 0).
    effPose.gripper = cmd.hasGripper ? cmd.pose.gripper : 0;

    // Валидация конца пути (те же ворота, что G0): solveAll + min-residual
    // + порог 10мм/15°. Сам путь считает moveToPose.
    // В цепочке блендинга (прошлый G1 был с Q1 и доехал) стартовый рестарт
    // пропускаем: трекинг от qPrev непрерывен, ворота проверяются ПОСЛЕ
    // движения пост-гейтом ниже. Диагностика сидов в цепочке не пишется
    // (останова нет — мерить нечего).
    const float IK_POS_TOL = 10.0f;
    const float IK_ORI_TOL = 15.0f;
    bool chainIn = interpolator.chainActive();
    bool blendOut = cmd.hasParam['Q' - 'A'] && cmd.params['Q' - 'A'] != 0.0f;
    bool inBatch = interpolator.batchOpen();
    // Опор непрерывности для min-residual: в пакете — последняя накопленная
    // точка (currentAngles стоит: движения нет), иначе — текущая поза.
    // Вне пакета копия равна currentAngles — поведение бит-в-бит как раньше.
    JointAngles contRef = currentAngles;
    if (inBatch) interpolator.batchPrevPt(contRef);
    // Выборка для пакета (заполняется внутри валидации ниже).
    JointAngles batchSel;
    float batchRdp = 0.0f, batchRdo = 0.0f;
    diagBegin("IK-CART", cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE);
    diagLast.target = effPose;
    diagLast.hasPos = cmd.hasPosition;
    diagLast.hasOri = cmd.hasOrientation;
    if (!chainIn) {
    JointAngles solutions[4];
    IKSolutionMeta metas[4];
    int solCount = ikSolver.solveAll(effPose, solutions, metas);

    if (solCount == 0) {
        strncpy(diagLast.failReason, "IK solution not found",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "IK not found");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: IK solution not found");
        return;
    }

    for (int i = 0; i < solCount && diagLast.solCount < 4; i++) {
        if (cmd.hasGripper) {
            solutions[i].angles[6] = gripperPctToAngle(cmd.pose.gripper);
        }
        diagEvalSol(diagLast.sols[diagLast.solCount], solutions[i], effPose);
        diagLast.sols[diagLast.solCount].iters = metas[i].iters;
        diagLast.sols[diagLast.solCount].converged = metas[i].converged;
        diagLast.sols[diagLast.solCount].seedPos = metas[i].seedPos;
        diagLast.sols[diagLast.solCount].seedOri = metas[i].seedOri;
        diagLast.sols[diagLast.solCount].dbgDq0 = metas[i].dbgDq0;
        diagLast.sols[diagLast.solCount].dbgN1 = metas[i].dbgN1;
        diagLast.solCount++;
    }

    int best = -1;
    float bestScore = 0;
    // Порог высотного штрафа: ниже — предпочитаем верхние ветви.
    // Срезка min(15): в честно-низкой работе все ветви внизу,
    // штраф у всех максимальный и одинаковый — решает расхождение.
    const float HEIGHT_Z_REF = 150.0f;   // мм
    const float HEIGHT_W = 0.15f;        // штрафа за мм ниже порога
    const float HEIGHT_MAX = 15.0f;      // потолок штрафа
    for (int i = 0; i < diagLast.solCount; i++) {
        if (!diagLast.sols[i].safe) continue;
        float jd = 0;
        for (int j = 0; j < 6; j++) {
            float d = fabs(diagLast.sols[i].q.angles[j] - contRef.angles[j]);
            if (d > jd) jd = d;
        }
        float hPen = diagLast.sols[i].minZ < HEIGHT_Z_REF
            ? HEIGHT_W * (HEIGHT_Z_REF - diagLast.sols[i].minZ) : 0.0f;
        if (hPen > HEIGHT_MAX) hPen = HEIGHT_MAX;
        float score = diagLast.sols[i].dPos + diagLast.sols[i].dOri + 0.02f * jd + hPen;
        if (best < 0 || score < bestScore) {
            best = i;
            bestScore = score;
        }
    }
    bool foundSafe = (best >= 0);
    if (foundSafe &&
        (diagLast.sols[best].dPos > IK_POS_TOL ||
         diagLast.sols[best].dOri > IK_ORI_TOL)) {
        foundSafe = false;
        char tmp[96];
        snprintf(tmp, sizeof(tmp),
                 "No accurate safe IK (best dPos=%.1f dOri=%.1f)",
                 diagLast.sols[best].dPos, diagLast.sols[best].dOri);
        strncpy(diagLast.failReason, tmp, sizeof(diagLast.failReason) - 1);
    }
    if (!foundSafe) {
        if (diagLast.failReason[0] == '\0') {
            strncpy(diagLast.failReason, "No safe IK solution",
                    sizeof(diagLast.failReason) - 1);
        }
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "No safe IK");
        xSemaphoreGive(displayMutex);
        Serial.print("ERROR: ");
        Serial.println(diagLast.failReason);
        return;
    }
    diagLast.chosen = best;
        // Пакет: запоминаем выбранное для накопления ниже (цепочка и пакет
        // взаимоисключают друг друга: сюда в цепочке не попадаем).
        if (inBatch) {
            batchSel = solutions[best];
            batchRdp = diagLast.sols[best].dPos;
            batchRdo = diagLast.sols[best].dOri;
        }
    }  // if (!chainIn): в цепочке стартовый solveAll/диагностика пропущены

    // Пакет M450: точка прошла те же ворота — копим вместо движения.
    // Движения нет, поэтому OK здесь = «принято и провалидировано».
    // chainIn здесь невозможен (цепочка и пакет взаимоисключают друг друга),
    // но нули в буфер страшнее лишней проверки: страхуемся явно.
    if (inBatch && chainIn) {
        interpolator.clearChain();
        interpolator.batchAbort();
        Serial.println("ERROR: chain meets batch (internal), both reset");
        return;
    }
    if (inBatch) {
        float feed = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
        if (!interpolator.batchAdd(batchSel, feed, batchRdp, batchRdo)) {
            interpolator.batchAbort();
            xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
            sharedState.hasError = true;
            snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Stream stalled");
            xSemaphoreGive(displayMutex);
            Serial.println("ERROR: stream stalled/aborted, flushed");
            return;
        }
        Serial.printf("OK: buffered (depth=%d/%d)\n",
                      interpolator.streamDepth(), MotionInterpolator::STREAM_DEPTH);
        return;
    }

    // Декартово исполнение: TCP по прямой, IK на каждом шаге.
    float feedrate = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
    interpolator.setCurrentAngles(currentAngles);
    if (interpolator.moveToPose(effPose, feedrate, blendOut)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        // currentAngles уже обновлены внутри moveToPose; забираем итог.
        JointAngles done = interpolator.getCurrentAngles();
        currentAngles = done;
        xSemaphoreGive(angleMutex);

        // CHOSEN для декарта: не min-residual (движение трекингом от текущей,
        // а не прыжок в выбранное), а ближайший безопасный кандидат к факту.
        {
            int near = -1;
            float nearD = 0;
            for (int i = 0; i < diagLast.solCount; i++) {
                if (!diagLast.sols[i].safe) continue;
                float dd = 0;
                for (int j = 0; j < 6; j++) {
                    float d = fabs(diagLast.sols[i].q.angles[j] - done.angles[j]);
                    if (d > dd) dd = d;
                }
                if (near < 0 || dd < nearD) { near = i; nearD = dd; }
            }
            if (near >= 0) diagLast.chosen = near;
        }

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        for (int i = 0; i < 7; i++) {
            sharedState.servoAngles[i] = done.angles[i];
        }
        float q[6];
        for (int i = 0; i < 6; i++) q[i] = done.angles[i];
        sharedState.tcpPosition = fkSolver.getEndEffector(q);
        sharedState.gripperPercent = gripperAngleToPct(done.angles[6]);
        sharedState.isMoving = false;
        sharedState.hasError = false;
        sharedState.commandCount++;
        xSemaphoreGive(displayMutex);

        // Пост-гейт цепочки: пред-валидации не было — проверяем факт.
        // currentAngles уже = done (честная позиция), цепочку рвём.
        if (chainIn) {
            DiagSol gate;
            diagEvalSol(gate, done, effPose);
            if (gate.dPos > IK_POS_TOL || gate.dOri > IK_ORI_TOL) {
                interpolator.clearChain();
                char tmp[96];
                snprintf(tmp, sizeof(tmp),
                         "chain residual too big (dPos=%.1f dOri=%.1f)",
                         gate.dPos, gate.dOri);
                xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
                sharedState.hasError = true;
                snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Chain residual");
                xSemaphoreGive(displayMutex);
                Serial.print("ERROR: ");
                Serial.println(tmp);
                return;
            }
        }

        Serial.println("OK");
    } else {
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Cart move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: Cartesian move failed (see M115 INTERP)");
    }
}

/**
 * G28 — перемещение в домашнюю позицию (home).
 */
void handleG28(GCodeCommand &cmd) {
    if (interpolator.batchOpen() || interpolator.streamBusy()) {
        Serial.println("ERROR: batch open - close it first (M450 S0)");
        return;
    }
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    sharedState.hasError = false;
    xSemaphoreGive(displayMutex);

    JointAngles target;
    bool park = cmd.hasParam['H' - 'A'] && (int)cmd.params['H' - 'A'] == 1;

    if (park) {
        // Компактная сложенная поза (единый источник — DEFAULT_PARK_ANGLES,
        // точные углы даст владелец). Старое (0,-60,-90) вело локоть ПОД СТОЛ!
        for (int i = 0; i < 7; i++) {
            target.angles[i] = DEFAULT_PARK_ANGLES[i];
        }
    } else {
        for (int i = 0; i < 7; i++) {
            target.angles[i] = homeAngles.angles[i];
        }
    }

    diagBegin(park ? "PARK" : "HOME", 300.0f);
    diagLast.anglesOnly = true;

    // Не едем вслепую: цель паркинга/дома обязана проходить safety.
    // При выключенной безопасности checkAllLinks всегда true.
    if (safety.isEnabled() && !safety.checkAllLinks(target.angles)) {
        strncpy(diagLast.failReason, safety.getLastError().c_str(),
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Bad home/park");
        xSemaphoreGive(displayMutex);
        Serial.print("ERROR: Home/park target unsafe: ");
        Serial.println(safety.getLastError());
        interpolator.setMark("NONE", "no motion (refused)");
        return;
    }
    diagEvalSol(diagLast.sols[0], target, cmd.pose);
    diagLast.solCount = 1;
    diagLast.chosen = 0;

    // Старт — из лучшего известного положения (см. handleG0).
    // Подача: из F (G28 F15 — тихо), иначе 300 как раньше.
    interpolator.setCurrentAngles(currentAngles);
    float homingFeed = cmd.hasFeedrate ? cmd.feedrate : 300.0f;
    if (interpolator.moveTo(target, homingFeed)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        for (int i = 0; i < 7; i++) {
            sharedState.servoAngles[i] = target.angles[i];
        }
        float q[6];
        for (int i = 0; i < 6; i++) q[i] = target.angles[i];
        sharedState.tcpPosition = fkSolver.getEndEffector(q);
        sharedState.gripperPercent = gripperAngleToPct(target.angles[6]);
        sharedState.isMoving = false;
        sharedState.commandCount++;
        xSemaphoreGive(displayMutex);

        Serial.println(park ? "OK: Parked" : "OK: Home");
    } else {
        strncpy(diagLast.failReason, park ? "Park failed" : "Home move failed",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "Park failed");
        xSemaphoreGive(displayMutex);
        Serial.println(park ? "ERROR: Park failed" : "ERROR: Home move failed");
    }
}

/**
 * G90 — абсолютный режим координат.
 */
void handleG90(GCodeCommand &cmd) {
    absoluteMode = true;
    Serial.println("OK: Absolute mode");
}

/**
 * G91 — относительный режим координат.
 */
void handleG91(GCodeCommand &cmd) {
    absoluteMode = false;
    Serial.println("OK: Relative mode");
}

/**
 * M104 — установка углов сервоприводов ЧЕРЕЗ интерполятор.
 * Едет плавно (ease-in-out), с лимитами угловых скоростей и проверкой
 * safety — как G0, только цель задана углами, а не позой TCP.
 * Неуказанные сервы остаются на месте. Параметр F — скорость (%), по
 * умолчанию DEFAULT_FEEDRATE.
 */
void handleM104(GCodeCommand &cmd) {
    if (interpolator.batchOpen() || interpolator.streamBusy()) {
        Serial.println("ERROR: batch open - close it first (M450 S0)");
        return;
    }
    JointAngles target = currentAngles;
    for (int i = 0; i < 7; i++) {
        if (cmd.servoOverride[i]) {
            target.angles[i] = cmd.servoAngles.angles[i];
        }
    }
    float feedrate = cmd.hasFeedrate ? cmd.feedrate : DEFAULT_FEEDRATE;
    interpolator.setCurrentAngles(currentAngles);
    if (interpolator.moveTo(target, feedrate)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        for (int i = 0; i < 7; i++) {
            sharedState.servoAngles[i] = target.angles[i];
        }
        float q[6];
        for (int i = 0; i < 6; i++) q[i] = target.angles[i];
        sharedState.tcpPosition = fkSolver.getEndEffector(q);
        sharedState.gripperPercent = gripperAngleToPct(target.angles[6]);
        sharedState.isMoving = false;
        sharedState.hasError = false;
        sharedState.commandCount++;
        xSemaphoreGive(displayMutex);

        diagBegin("DIRECT", 0);
        diagLast.anglesOnly = true;
        diagEvalSol(diagLast.sols[0], target, cmd.pose);
        diagLast.solCount = 1;
        diagLast.chosen = 0;
        Serial.println("OK");
    } else {
        strncpy(diagLast.failReason, "Move failed (interpolator/safety)",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "M104 move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: M104 move failed (safety/interpolator)");
    }
}

/**
 * M108 — захват в процентах: M108 S=50 (0=закрыт, 100=открыт).
 * P7 живёт в градусах сервы (закрыт 7°, открыт 20°): процент маппится.
 * Опциональный T — время движения в секундах (0.5–3.0).
 * Едет только сустав 6 через тот же interpolator, что M104.
 */
void handleM108(GCodeCommand &cmd) {
    if (interpolator.batchOpen() || interpolator.streamBusy()) {
        Serial.println("ERROR: batch open - close it first (M450 S0)");
        return;
    }
    const int SI = 'S' - 'A';
    if (!cmd.hasParam[SI]) {
        strncpy(diagLast.failReason, "M108 requires S (0-100)",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "M108 needs S=0..100");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: M108 requires S (0-100)");
        return;
    }
    float pct = cmd.params[SI];
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    // Длительность движения, опционально: M108 S=50 T=2 (секунды)
    float seconds = 0.0f;
    const int TI = 'T' - 'A';
    if (cmd.hasParam[TI]) {
        seconds = cmd.params[TI];
        if (seconds < 0.5f) seconds = 0.5f;
        if (seconds > 3.0f) seconds = 3.0f;
    }

    JointAngles target = currentAngles;
    target.angles[6] = gripperPctToAngle(pct);

    float feedrate = DEFAULT_FEEDRATE;
    if (seconds > 0.0f) {
        // Время full-speed по максимуму серв (мс), потом масштабируем до целевых секунд
        float fullMs = interpolator.legMs(currentAngles, target, 100.0f);
        if (fullMs > 0.0f) {
            feedrate = 100.0f * fullMs / (seconds * 1000.0f);
            if (feedrate < 1.0f) feedrate = 1.0f;
            if (feedrate > 100.0f) feedrate = 100.0f;
        }
    } else if (cmd.hasFeedrate) {
        feedrate = cmd.feedrate;
    }

    interpolator.setCurrentAngles(currentAngles);
    if (interpolator.moveTo(target, feedrate)) {
        xSemaphoreTake(angleMutex, portMAX_DELAY);
        currentAngles = target;
        xSemaphoreGive(angleMutex);

        xSemaphoreTake(displayMutex, portMAX_DELAY);
        sharedState.servoAngles[6] = target.angles[6];
        sharedState.gripperPercent = pct;
        sharedState.isMoving = false;
        sharedState.hasError = false;
        sharedState.commandCount++;
        xSemaphoreGive(displayMutex);

        diagBegin("DIRECT", 0);
        diagLast.anglesOnly = true;
        diagEvalSol(diagLast.sols[0], target, cmd.pose);
        diagLast.solCount = 1;
        diagLast.chosen = 0;
        Serial.printf("OK: Gripper %.0f%%", pct);
        if (seconds > 0.0f) Serial.printf(" t=%.1fs", seconds);
        Serial.println();
    } else {
        strncpy(diagLast.failReason, "Move failed (interpolator/safety)",
                sizeof(diagLast.failReason) - 1);
        xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
        sharedState.hasError = true;
        snprintf(sharedState.lastError, sizeof(sharedState.lastError), "M108 move failed");
        xSemaphoreGive(displayMutex);
        Serial.println("ERROR: M108 move failed (interpolator/safety)");
    }
}
