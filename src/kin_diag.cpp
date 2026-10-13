/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Date        : 2026-10-12
 *
 * kin_diag.cpp — журнал последнего движения для M115 (перенос из main.cpp 1:1).
 */

#include "kin_diag.h"
#include "app.h"
#include "display_ui.h"
#include "config.h"
#include <cmath>
#include <cstring>

DiagLastMove diagLast;

// Направление инструмента (ось Z) из углов Эйлера ZYX — та же формула,
// что IKSolver::eulerToMatrix, третий столбец.
static Vec3 diagToolDir(const Vec3 &o) {
    float cr = cos(o.x * DEG_TO_RAD), sr = sin(o.x * DEG_TO_RAD);
    float cp = cos(o.y * DEG_TO_RAD), sp = sin(o.y * DEG_TO_RAD);
    float cy = cos(o.z * DEG_TO_RAD), sy = sin(o.z * DEG_TO_RAD);
    Vec3 d;
    d.x = cy*sp*cr + sy*sr;
    d.y = sy*sp*cr - cy*sr;
    d.z = cp*cr;
    return d;
}

// Угол между двумя единичными векторами (градусы).
static float diagOriErr(const Vec3 &a, const Vec3 &b) {
    float dot = a.x*b.x + a.y*b.y + a.z*b.z;
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    return acos(dot) * RAD_TO_DEG;
}

// Посчитать FK + ошибки + safety для кандидата и положить в DiagSol.
void diagEvalSol(DiagSol &s, const JointAngles &q, const Pose &target) {
    s.q = q;
    float qq[6];
    for (int i = 0; i < 6; i++) qq[i] = q.angles[i];
    Mat4 T06dummy;
    fkSolver.getPose(qq, s.joints, s.tcp, s.dir, T06dummy);
    (void)T06dummy;
    float dx = s.tcp.x - target.position.x;
    float dy = s.tcp.y - target.position.y;
    float dz = s.tcp.z - target.position.z;
    s.dPos = sqrt(dx*dx + dy*dy + dz*dz);
    s.dOri = diagOriErr(s.dir, diagToolDir(target.orientation));
    // Минимальная высота цепочки J2..J6 (без J0/J1: база и плечо всегда
    // высоко — сигнал «утопленника» несут локоть и ниже).
    // Высотный штраф в выборе предпочитает elbow-up: утопленник с локтем
    // на z=17 проигрывает высокой ветви, вверху штраф нулевой.
    s.minZ = s.joints[2].z;
    for (int i = 3; i <= 6; i++) {
        if (s.joints[i].z < s.minZ) s.minZ = s.joints[i].z;
    }
    if (!safety.isEnabled()) {
        s.safe = true;
        s.safeErr[0] = '\0';
    } else if (safety.checkAllLinks(qq)) {
        s.safe = true;
        s.safeErr[0] = '\0';
    } else {
        s.safe = false;
        strncpy(s.safeErr, safety.getLastError().c_str(), sizeof(s.safeErr) - 1);
        s.safeErr[sizeof(s.safeErr) - 1] = '\0';
    }
}

// Начать новую запись журнала: текст команды берём из sharedState.
void diagBegin(const char *kind, float feedrate) {
    diagLast = DiagLastMove();
    diagLast.valid = true;
    xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50));
    strncpy(diagLast.cmdText, sharedState.lastCommand, sizeof(diagLast.cmdText) - 1);
    xSemaphoreGive(displayMutex);
    strncpy(diagLast.kind, kind, sizeof(diagLast.kind) - 1);
    diagLast.feedrate = feedrate;
}

// Печать цепочки суставов J0..J6.
void diagPrintJoints(const Vec3 joints[7]) {
    for (int i = 0; i < 7; i++) {
        Serial.printf("  J%d: X=%.1f Y=%.1f Z=%.1f\n", i,
                      joints[i].x, joints[i].y, joints[i].z);
    }
}

/**
 * M115 — разбор кинематики: куда шли (цель), кандидаты IK,
 * выбранное решение, куда пришли (факт), расхождение.
 * Параметр S=1 — полные цепочки J0..J6 для всех кандидатов.
 */
void handleM115(GCodeCommand &cmd) {
    bool full = cmd.hasParam['S' - 'A'] && (int)cmd.params['S' - 'A'] == 1;
    Serial.println("=== KIN-DIAG ===");
    if (!diagLast.valid) {
        Serial.println("No moves yet");
        return;
    }
    Serial.printf("CMD: %s\n", diagLast.cmdText);
    Serial.printf("KIND: %s FEED: %.0f SAFETY: %s FINGER: %.1f MODE: %s FW:%s\n",
                  diagLast.kind, diagLast.feedrate,
                  safety.isEnabled() ? "ON" : "OFF",
                  config.fingerLength,
                  absoluteMode ? "ABS" : "REL",
                  FW_VERSION_STR);
    Serial.printf("STRAT: %d INTERP: %s %s\n", (int)interpStrategy,
                  interpolator.getLastMode(), interpolator.getLastNote());
    Vec3 toolDir = diagToolDir(diagLast.target.orientation);
    if (!diagLast.anglesOnly) {
        Serial.printf("TARGET TCP: X=%.1f Y=%.1f Z=%.1f A=%.1f B=%.1f C=%.1f%s\n",
                      diagLast.target.position.x, diagLast.target.position.y,
                      diagLast.target.position.z, diagLast.target.orientation.x,
                      diagLast.target.orientation.y, diagLast.target.orientation.z,
                      diagLast.hasOri ? "" : " (orient=default)");
        Serial.printf("TARGET wrist: X=%.1f Y=%.1f Z=%.1f toolDir: %.3f %.3f %.3f\n",
                      diagLast.target.position.x - toolDir.x * config.fingerLength,
                      diagLast.target.position.y - toolDir.y * config.fingerLength,
                      diagLast.target.position.z - toolDir.z * config.fingerLength,
                      toolDir.x, toolDir.y, toolDir.z);
    } else {
        Serial.print("TARGET angles:");
        for (int i = 0; i < 7; i++) {
            Serial.printf(" P%d=%.1f", i + 1, diagLast.sols[0].q.angles[i]);
        }
        Serial.println();
    }
    Serial.printf("SOLS: %d\n", diagLast.solCount);
    for (int i = 0; i < diagLast.solCount; i++) {
        DiagSol &s = diagLast.sols[i];
        // Метки углов, сидящих на механических лимитах
        char clamp[32]; clamp[0] = '\0';
        for (int j = 0; j < 6; j++) {
            if (fabs(s.q.angles[j] - config.servos[j].minAngle) < 0.5f ||
                fabs(s.q.angles[j] - config.servos[j].maxAngle) < 0.5f) {
                char tmp[8];
                snprintf(tmp, sizeof(tmp), "P%d ", j + 1);
                strncat(clamp, tmp, sizeof(clamp) - strlen(clamp) - 1);
            }
        }
        Serial.printf("[%d] q:", i);
        for (int j = 0; j < 6; j++) Serial.printf(" P%d=%.1f", j + 1, s.q.angles[j]);
        Serial.printf("\n    TCP: X=%.1f Y=%.1f Z=%.1f", s.tcp.x, s.tcp.y, s.tcp.z);
        if (!diagLast.anglesOnly) {
            Serial.printf(" dPos=%.1fmm dOri=%.1fdeg", s.dPos, s.dOri);
        }
        Serial.printf("\n    %s", s.safe ? "SAFE:OK" : "SAFE:FAIL");
        if (!s.safe) Serial.printf("(%s)", s.safeErr);
        Serial.printf(" CLAMP:%s minZ=%.0f", clamp[0] ? clamp : "-", s.minZ);
        if (!diagLast.anglesOnly) {
            Serial.printf(" NLS:%s/%dit", s.converged ? "ok" : "fail", s.iters);
            if (!s.converged) Serial.printf(" seed=%.1fmm/%.1fdeg", s.seedPos, s.seedOri);
            Serial.printf(" dq0=%.3f n1=%.1f", s.dbgDq0, s.dbgN1);
        }
        Serial.println();
        if (full || i == diagLast.chosen) {
            diagPrintJoints(s.joints);
        }
    }
    if (diagLast.chosen >= 0) {
        const char *how = "direct";
        if (strcmp(diagLast.kind, "IK") == 0) how = "min-residual";
        else if (strcmp(diagLast.kind, "IK-CART") == 0) how = "cart-tracked";
        else if (strcmp(diagLast.kind, "MIXED") == 0) how = "single";
        Serial.printf("CHOSEN: [%d] (%s)\n", diagLast.chosen, how);
    } else {
        Serial.printf("FAIL: %s\n", diagLast.failReason);
    }
    // Вейпоинты планировщика объезда (TCP-точки пути).
    if (diagLast.detourCount > 0) {
        Serial.printf("WPS: %d\n", diagLast.detourCount);
        for (int i = 0; i < diagLast.detourCount; i++) {
            Serial.printf("  [%d] X=%.1f Y=%.1f Z=%.1f\n", i,
                          diagLast.detourPts[i].x,
                          diagLast.detourPts[i].y,
                          diagLast.detourPts[i].z);
        }
    }
    // Поток M450: виталы кольца + таблица авто-чанков. Пусто = потока не было.
    {
        MotionInterpolator::StreamState st = interpolator.streamState();
        const char *stName = "?";
        if (st == MotionInterpolator::ST_IDLE) stName = "IDLE";
        else if (st == MotionInterpolator::ST_FILLING) stName = "FILLING";
        else if (st == MotionInterpolator::ST_RUNNING) stName = "RUNNING";
        else if (st == MotionInterpolator::ST_HOLDING) stName = "HOLDING";
        Serial.printf("STREAM: state=%s depth=%d/%d maxDepth=%d starved=%lu holdsMs=%lu maxStep=%lu\n",
                      stName, interpolator.streamDepth(), MotionInterpolator::STREAM_DEPTH,
                      interpolator.streamDepthMax(),
                      (unsigned long)interpolator.streamStarved(),
                      (unsigned long)interpolator.streamHoldsMs(),
                      (unsigned long)interpolator.streamMaxStep());
        int cn = interpolator.chunkCount();
        if (cn > 0) {
            uint32_t consumed = interpolator.streamConsumed();
            uint32_t drop = interpolator.streamDropSeq();
            Serial.printf("CHUNKS: %d\n", cn);
            for (int i = 0; i < cn; i++) {
                MotionInterpolator::ChunkRow row;
                if (!interpolator.chunkRow(i, row)) break;
                const char *rst = "active";
                if (row.startSeq + row.n <= consumed) rst = "done";
                else if (drop > 0 && row.startSeq >= drop) rst = "dropped";
                Serial.printf("  [%d] n=%d worst=%.1fmm/%.1fdeg %s\n", i,
                              row.n, row.worstPos, row.worstOri, rst);
            }
        }
    }
    // Фактическое положение
    float q[6];
    for (int i = 0; i < 6; i++) q[i] = currentAngles.angles[i];
    Vec3 joints[7];
    fkSolver.getAllJointPositions(q, joints);
    Vec3 dir = fkSolver.getFingerDirection(q);
    Vec3 atcp = fkSolver.getEndEffector(q);
    Serial.print("ACTUAL q:");
    for (int i = 0; i < 7; i++) Serial.printf(" P%d=%.1f", i + 1, currentAngles.angles[i]);
    Serial.println();
    diagPrintJoints(joints);
    Serial.printf("  DIR: X=%.3f Y=%.3f Z=%.3f\n", dir.x, dir.y, dir.z);
    Serial.printf("  TCP: X=%.1f Y=%.1f Z=%.1f\n", atcp.x, atcp.y, atcp.z);
    if (!diagLast.anglesOnly) {
        float dx = atcp.x - diagLast.target.position.x;
        float dy = atcp.y - diagLast.target.position.y;
        float dz = atcp.z - diagLast.target.position.z;
        float dist = sqrt(dx*dx + dy*dy + dz*dz);
        Serial.printf("RESIDUAL: dX=%.1f dY=%.1f dZ=%.1f dist=%.1fmm dOri=%.1fdeg%s\n",
                      dx, dy, dz, dist, diagOriErr(dir, toolDir),
                      diagLast.hasOri ? "" : " (vs default orient)");
    } else {
        Serial.println("RESIDUAL: n/a (angles mode)");
    }
}
