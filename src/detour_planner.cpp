/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-10-01
 * Last modify : 2026-10-01
 */

/*
 * detour_planner.cpp — реализация планировщика объезда.
 * Зеркало tools/twin/detour.py: те же этапы, те же пороги.
 */

#include "detour_planner.h"
#include <cmath>

namespace {
const float DET_CLEAR = 30.0f;     // запас вокруг короба для via-точек
const float DET_VIA_DPOS = 5.0f;    // допуск позиции via-точки (мм)
const int DET_RRT_NODES = 120;      // потолок узлов RRT
const float DET_RRT_STEP = 30.0f;   // шаг роста RRT (градусы)
const int DET_RRT_EDGE = 10;        // сэмплов проверки ребра RRT

float detNodes[DET_RRT_NODES][7];
uint8_t detParent[DET_RRT_NODES];
uint32_t detRngState = 12345;

float detRand01() {
    detRngState = detRngState * 1664525u + 1013904223u;
    return (float)(detRngState >> 8) / 16777216.0f;
}

float detDist6Sq(const float a[6], const float b[6]) {
    float s = 0;
    for (int i = 0; i < 6; i++) {
        float d = a[i] - b[i];
        s += d * d;
    }
    return s;
}
}  // namespace

DetourPlanner::DetourPlanner()
    : ikSolver(nullptr), fkSolver(nullptr), safety(nullptr), servoCtrl(nullptr) {
}

void DetourPlanner::setIKSolver(IKSolver *ik) { ikSolver = ik; }
void DetourPlanner::setFKSolver(FKSolver *fk) { fkSolver = fk; }
void DetourPlanner::setSafetyChecker(SafetyChecker *s) { safety = s; }
void DetourPlanner::setServoController(ServoController *ctrl) { servoCtrl = ctrl; }

int DetourPlanner::plan(const float qstart[6], const JointAngles &target,
                        const Pose *pose, JointAngles *wps, int cap) {
    if (!safety || !servoCtrl || cap <= 0) return 0;
    float qg[6];
    for (int i = 0; i < 6; i++) qg[i] = target.angles[i];

    // Без safety — как moveTo без проверок: одна цель.
    if (!safety->isEnabled()) {
        wps[0] = target;
        return 1;
    }
    // Без зон объезжать нечего: прямой ход (проверка — в moveTo при исполнении).
    if (safety->getZoneCount() == 0) {
        wps[0] = target;
        return 1;
    }

    // --- Фаза 1: вейпоинты у граней коробов (нужна поза TCP для IK) ---
    struct ViaQ { float q[6]; };
    ViaQ vias[8];
    int viaN = 0;
    if (pose && ikSolver && fkSolver) {
        float qsf[6];
        for (int i = 0; i < 6; i++) qsf[i] = qstart[i];
        Vec3 tcp0 = fkSolver->getEndEffector(qsf);
        int nz = safety->getZoneCount();
        for (int zi = 0; zi < nz && viaN < 8; zi++) {
            float x1, x2, y1, y2, z1, z2;
            bool act = false;
            if (!safety->getForbiddenZone(zi, x1, x2, y1, y2, z1, z2, act) || !act) continue;
            bool hit = false;
            for (int s = 0; s <= 20; s++) {
                float t = (float)s / 20.0f;
                float px = tcp0.x + (pose->position.x - tcp0.x) * t;
                float py = tcp0.y + (pose->position.y - tcp0.y) * t;
                float pz = tcp0.z + (pose->position.z - tcp0.z) * t;
                if (px >= x1 && px <= x2 && py >= y1 && py <= y2 && pz >= z1 && pz <= z2) {
                    hit = true;
                    break;
                }
            }
            if (!hit) continue;
            float cx = (x1 + x2) * 0.5f, cy = (y1 + y2) * 0.5f, cz = (z1 + z2) * 0.5f;
            float cand[7][3] = {
                {x1 - DET_CLEAR, cy, cz}, {x2 + DET_CLEAR, cy, cz},
                {cx, y1 - DET_CLEAR, cz}, {cx, y2 + DET_CLEAR, cz},
                {cx, cy, z1 - DET_CLEAR}, {cx, cy, z2 + DET_CLEAR},
                {(tcp0.x + pose->position.x) * 0.5f,
                 (tcp0.y + pose->position.y) * 0.5f,
                 (tcp0.z + pose->position.z) * 0.5f},
            };
            for (int c = 0; c < 7 && viaN < 8; c++) {
                bool inBox = false;
                for (int zj = 0; zj < safety->getZoneCount(); zj++) {
                    float a1, a2, b1, b2, c1, c2;
                    bool aa = false;
                    if (!safety->getForbiddenZone(zj, a1, a2, b1, b2, c1, c2, aa) || !aa) continue;
                    if (cand[c][0] >= a1 && cand[c][0] <= a2 &&
                        cand[c][1] >= b1 && cand[c][1] <= b2 &&
                        cand[c][2] >= c1 && cand[c][2] <= c2) {
                        inBox = true;
                        break;
                    }
                }
                if (inBox) continue;
                Pose vp;
                vp.position.x = cand[c][0]; vp.position.y = cand[c][1]; vp.position.z = cand[c][2];
                vp.orientation = pose->orientation;
                vp.gripper = pose->gripper;
                JointAngles sols[4];
                IKSolutionMeta metas[4];
                int n = ikSolver->solveAll(vp, sols, metas);
                int bi = -1;
                float bd = 1e30f;
                for (int k = 0; k < n; k++) {
                    float qq[6];
                    for (int i = 0; i < 6; i++) qq[i] = sols[k].angles[i];
                    Vec3 tcp = fkSolver->getEndEffector(qq);
                    float dx = tcp.x - vp.position.x;
                    float dy = tcp.y - vp.position.y;
                    float dz = tcp.z - vp.position.z;
                    float dp = sqrt(dx*dx + dy*dy + dz*dz);
                    if (dp > DET_VIA_DPOS) continue;
                    float q6[6];
                    for (int i = 0; i < 6; i++) q6[i] = sols[k].angles[i];
                    if (!safety->checkAllLinks(q6)) continue;
                    if (dp < bd) { bd = dp; bi = k; }
                }
                if (bi < 0) continue;
                float qvia[6];
                for (int i = 0; i < 6; i++) qvia[i] = sols[bi].angles[i];
                if (!safety->checkTrajectory(qstart, qvia, DET_RRT_EDGE)) continue;
                for (int i = 0; i < 6; i++) vias[viaN].q[i] = qvia[i];
                viaN++;
            }
        }
    }

    // --- Фаза 2: RRT в joint-space (старты: via..., затем current) ---
    // Быстрый путь: прямая старт->цель свободна — сразу она (как twin,
    // без RRT-танцев). Это 95% движений.
    if (safety->checkTrajectory(qstart, qg, 20)) {
        wps[0] = target;
        return 1;
    }
    // Идём стартами по очереди; первый давший цепочку — исполнение.
    // Цепочка пишется сразу в wps (включая цель), возврат — её длина.
    for (int vs = -1; vs < viaN; vs++) {
        float qst[6];
        if (vs < 0) {
            for (int i = 0; i < 6; i++) qst[i] = qstart[i];
        } else {
            for (int i = 0; i < 6; i++) qst[i] = vias[vs].q[i];
        }
        // RRT qst -> qg
        float lim[6][2];
        for (int i = 0; i < 6; i++) {
            lim[i][0] = (float)servoCtrl->getConfig(i).minAngle;
            lim[i][1] = (float)servoCtrl->getConfig(i).maxAngle;
        }
        for (int i = 0; i < 6; i++) detNodes[0][i] = qst[i];
        detParent[0] = 0;
        int count = 1;
        int goalIdx = -1;
        for (int it = 0; it < DET_RRT_NODES - 1; it++) {
            float qr[6];
            // goal bias 30%: тянемся к цели
            bool toGoal = detRand01() < 0.3f;
            if (toGoal) {
                for (int i = 0; i < 6; i++) qr[i] = qg[i];
            } else {
                for (int i = 0; i < 6; i++) {
                    qr[i] = lim[i][0] + detRand01() * (lim[i][1] - lim[i][0]);
                }
            }
            int bni = 0;
            float bnd2 = 1e30f;
            for (int i = 0; i < count; i++) {
                float d2 = detDist6Sq(detNodes[i], qr);
                if (d2 < bnd2) { bnd2 = d2; bni = i; }
            }
            float bnd = sqrt(bnd2);
            float qn[6];
            if (bnd < 1e-9f) {
                for (int i = 0; i < 6; i++) qn[i] = detNodes[bni][i];
            } else {
                float k = DET_RRT_STEP / bnd;
                if (k > 1.0f) k = 1.0f;
                for (int i = 0; i < 6; i++) {
                    float v = detNodes[bni][i] + (qr[i] - detNodes[bni][i]) * k;
                    if (v < lim[i][0]) v = lim[i][0];
                    if (v > lim[i][1]) v = lim[i][1];
                    qn[i] = v;
                }
            }
            float qnb[6], qbb[6];
            for (int i = 0; i < 6; i++) { qnb[i] = detNodes[bni][i]; qbb[i] = qn[i]; }
            if (!safety->checkAllLinks(qn)) continue;
            if (!safety->checkTrajectory(qnb, qbb, DET_RRT_EDGE)) continue;
            for (int i = 0; i < 6; i++) detNodes[count][i] = qn[i];
            detParent[count] = (uint8_t)bni;
            count++;
            if (it % 4 == 3) {
                float qgo[6];
                for (int i = 0; i < 6; i++) qgo[i] = qg[i];
                if (safety->checkAllLinks(qgo) && safety->checkTrajectory(qn, qgo, DET_RRT_EDGE)) {
                    for (int i = 0; i < 6; i++) detNodes[count][i] = qgo[i];
                    detParent[count] = (uint8_t)(count - 1);
                    goalIdx = count;
                    count++;
                    break;
                }
            }
        }
        if (goalIdx < 0) {
            // Финальная попытка: цель из ближайшего узла.
            // Тут только сравнение — квадрата достаточно, корень не нужен.
            int bni = 0;
            float bnd2 = 1e30f;
            for (int i = 0; i < count; i++) {
                float d2 = detDist6Sq(detNodes[i], qg);
                if (d2 < bnd2) { bnd2 = d2; bni = i; }
            }
            float qgo[6], qbb[6];
            for (int i = 0; i < 6; i++) { qgo[i] = qg[i]; qbb[i] = detNodes[bni][i]; }
            if (safety->checkAllLinks(qgo) && safety->checkTrajectory(qbb, qgo, DET_RRT_EDGE)) {
                for (int i = 0; i < 6; i++) detNodes[count][i] = qgo[i];
                detParent[count] = (uint8_t)bni;
                goalIdx = count;
                count++;
            } else {
                continue;
            }
        }
        // Цепочка + жадкое сглаживание срезами.
        int tmp[DET_RRT_NODES];
        int tn = 0;
        int idx = goalIdx;
        while (true) {
            tmp[tn++] = idx;
            if (detParent[idx] == idx) break;
            idx = detParent[idx];
        }
        int rev[DET_RRT_NODES];
        for (int i = 0; i < tn; i++) rev[i] = tmp[tn - 1 - i];
        int sm[DET_RRT_NODES];
        sm[0] = rev[0];
        int sn = 1;
        int i = 0;
        while (i < tn - 1) {
            int j = tn - 1;
            float qa[6], qb[6];
            for (int k = 0; k < 6; k++) { qa[k] = detNodes[rev[i]][k]; qb[k] = detNodes[rev[j]][k]; }
            while (j > i + 1 && !safety->checkTrajectory(qa, qb, DET_RRT_EDGE)) {
                j--;
                for (int k = 0; k < 6; k++) { qa[k] = detNodes[rev[i]][k]; qb[k] = detNodes[rev[j]][k]; }
            }
            sm[sn++] = rev[j];
            i = j;
        }
        // Пишем вейпоинты: пропуск старта (мы уже там), цель — из target.
        // Хвост цепочки — всегда цель (проверена выше).
        // Защита от усечения: если цепочка не влезает в cap целиком —
        // пропускаем старт (оборванная цепочка без цели хуже отказа).
        int need = (vs >= 0 ? 1 : 0) + (sn - 1) + 1;
        if (need > cap) continue;
        int wn = 0;
        if (vs >= 0) {
            // Первый хоп: старт -> via (ребро уже проверено в фазе 1).
            for (int k = 0; k < 6; k++) wps[wn].angles[k] = vias[vs].q[k];
            wps[wn].angles[6] = target.angles[6];
            wn++;
            if (wn >= cap) return wn;
        }
        for (int h = 1; h < sn && wn < cap; h++) {
            int ni = sm[h];
            for (int k = 0; k < 6; k++) wps[wn].angles[k] = detNodes[ni][k];
            wps[wn].angles[6] = target.angles[6];
            wn++;
        }
        // Финал — точно target (все проверки уже были на нём в вызывающем коде).
        if (wn < cap) {
            wps[wn] = target;
            wn++;
        }
        return wn;
    }
    return 0;
}
