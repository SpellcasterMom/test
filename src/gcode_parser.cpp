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

#include "gcode_parser.h"
#include "commands.h"
#include <cstring>
#include <cstdlib>

GCodeCommand GCodeParser::parse(String line) {
    GCodeCommand cmd;
    line.trim();
    line.toUpperCase();

    const char *s = line.c_str();
    size_t len = 0;
    while (s[len] && s[len] != ' ') len++;

    if (len == 0) {
        cmd.type = CMD_EMPTY;
        return cmd;
    }

    // Имя команды — первый токен в стековый буфер (без substring-аллокаций).
    // Длинный мусор обрезаем: для сравнения хватает, в ошибку попадёт обрезанным.
    char cmdName[32];
    size_t cn = (len < sizeof(cmdName) - 1) ? len : sizeof(cmdName) - 1;
    memcpy(cmdName, s, cn);
    cmdName[cn] = '\0';

    // Точное совпадение токена с таблицей (как String== раньше).
    for (int i = 0; i < COMMAND_COUNT; i++) {
        if (strcmp(cmdName, COMMANDS[i].name) == 0) {
            cmd.type = CMD_VALID;
            cmd.name = COMMANDS[i].name;

            // Parse parameters
            parseParams(cmd, s, COMMANDS[i]);

            // Call handler
            COMMANDS[i].handler(cmd);

            return cmd;
        }
    }

    cmd.type = CMD_ERROR;
    cmd.errorMsg = "Unknown command: " + String(cmdName);
    return cmd;
}

/**
 * Однопроходный разбор параметров: идём по токенам строки один раз,
 * буква значима в НАЧАЛЕ токена или сразу за разобранным числом
 * (слитные "X100Y50", "L1Z=30" — как ели indexOf раньше).
 * Чужие слова целиком пропускаются (чинит класс багов «X ловит MAX»,
 * когда indexOf находил букву внутри чужого слова/комментария).
 * Совместимость со старым поведением токен-в-токен:
 *   - значение = текст сразу за буквой, один опциональный '=' пропускается,
 *     число — strtof (как String::toFloat): "X1=0"→1.0, "Z0=50"→0.0;
 *   - голая буква без значения даёт 0.0 с флагами (старый substring()==""→0.0),
 *     кроме голого "P"/"P<n>" без '=' — они игнорировались и игнорируются;
 *   - P<n>=val — только при P(index=-1) в таблице команды;
 *   - побеждает ПЕРВОЕ вхождение (старый indexOf брал первое в строке).
 * Аллокаций нет: работаем указателями по исходному буферу.
 */
void GCodeParser::parseParams(GCodeCommand &cmd, const char *line, const CommandDef &def) {
    // Маска букв из таблицы команды + флаг P-спецрежима.
    bool letters[26] = {};
    bool pSpecial = false;
    for (int i = 0; i < def.paramCount; i++) {
        char l = def.params[i].letter;
        if (l >= 'A' && l <= 'Z') letters[l - 'A'] = true;
        if (l == 'P' && def.params[i].index == -1) pSpecial = true;
    }

    bool seen[26] = {};
    bool seenServo[7] = {};

    // Пропустить первый токен (имя команды): буквы из него не параметры.
    // (В таблице нет команд/параметров на G/M, так что это только страховка
    // от класса багов «буква внутри чужого слова».)
    const char *p = line;
    while (*p && *p != ' ') p++;

    while (*p) {
        while (*p == ' ') p++;
        if (!*p) break;
        const char *t = p;
        while (*p && *p != ' ') p++;
        // Токен [t, p): цепочка БукваЗначение внутри (для слитных "X100Y50").
        const char *q = t;
        while (q < p) {
            char L = *q;
            if (L < 'A' || L > 'Z') break;  // хвост значения/мусор — токен разобран
            // P<n>=val — прямой ввод углов (только при P(index=-1) в таблице).
            if (L == 'P' && pSpecial) {
                const char *d = q + 1;
                int n = 0, nd = 0;
                while (d < p && *d >= '0' && *d <= '9') {
                    n = n * 10 + (*d - '0');
                    d++;
                    nd++;
                }
                if (nd == 0 || d >= p || *d != '=') {
                    q = (nd == 0) ? q + 1 : d;  // "P"/"P<n>" без '=' — мимо, смотрим дальше
                    continue;
                }
                char *e = nullptr;
                float val = (d + 1 < p) ? strtof(d + 1, &e) : 0.0f;
                q = e ? e : p;  // e==d+1 (числа нет) — продолжаем с этого места
                if (n >= 1 && n <= 7 && !seenServo[n - 1]) {
                    cmd.servoOverride[n - 1] = true;
                    cmd.servoAngles.angles[n - 1] = val;
                    seenServo[n - 1] = true;
                }
                continue;
            }
            int li = L - 'A';
            if (!letters[li]) break;  // чужое слово — токен целиком пропускаем (фикс MAX)
            if (seen[li]) { q++; continue; }  // дубли: первое вхождение побеждает
            // Обычная буква: значение сразу за ней, один '=' пропускается.
            const char *v = q + 1;
            if (v < p && *v == '=') v++;
            char *e = nullptr;
            float val = (v < p) ? strtof(v, &e) : 0.0f;
            setParamValue(cmd, L, 0, val);
            seen[li] = true;
            q = e ? e : p;  // e==v (числа нет, "XY100") — продолжаем с буквы
        }
    }
}

void GCodeParser::setParamValue(GCodeCommand &cmd, char letter, int index, float val) {
    switch (letter) {
        case 'X': cmd.pose.position.x = val;    cmd.hasPosition = true; cmd.hasX = true; break;
        case 'Y': cmd.pose.position.y = val;    cmd.hasPosition = true; cmd.hasY = true; break;
        case 'Z': cmd.pose.position.z = val;    cmd.hasPosition = true; cmd.hasZ = true; break;
        case 'A': cmd.pose.orientation.x = val; cmd.hasOrientation = true; break;
        case 'B': cmd.pose.orientation.y = val; cmd.hasOrientation = true; break;
        case 'C': cmd.pose.orientation.z = val; cmd.hasOrientation = true; break;
        case 'F': cmd.feedrate = val;            cmd.hasFeedrate = true; break;
        case 'J': cmd.pose.gripper = val;       cmd.hasGripper = true; break;
        default: {
            // Store in generic params (L, R, S, etc.)
            int idx = letter - 'A';
            if (idx >= 0 && idx < 26) {
                cmd.params[idx] = val;
                cmd.hasParam[idx] = true;
            }
            break;
        }
    }
}