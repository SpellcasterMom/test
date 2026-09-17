/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       :
 * Mentor      : Lipko I.Y.
 * Date        :
 * Last modify :
 */

#include "gcode_parser.h"
#include "commands.h"

GCodeCommand GCodeParser::parse(String line) {
    GCodeCommand cmd;
    line.trim();
    line.toUpperCase();

    if (line.length() == 0) {
        cmd.type = CMD_EMPTY;
        return cmd;
    }

    // Extract command name (first token)
    int spaceIdx = line.indexOf(' ');
    String cmdName = (spaceIdx > 0) ? line.substring(0, spaceIdx) : line;

    // Find in command table
    for (int i = 0; i < COMMAND_COUNT; i++) {
        if (cmdName == COMMANDS[i].name) {
            cmd.type = CMD_VALID;
            cmd.name = COMMANDS[i].name;

            // Parse parameters
            parseParams(cmd, line, COMMANDS[i]);

            // Call handler
            COMMANDS[i].handler(cmd);

            return cmd;
        }
    }

    cmd.type = CMD_ERROR;
    cmd.errorMsg = "Unknown command: " + cmdName;
    return cmd;
}

void GCodeParser::parseParams(GCodeCommand &cmd, String line, const CommandDef &def) {
    for (int i = 0; i < def.paramCount; i++) {
        ParamDef p = def.params[i];

        if (p.letter == 'P' && p.index == -1) {
            // Special: P1..P7 direct servo angles
            for (int n = 1; n <= 7; n++) {
                String key = "P" + String(n) + "=";
                int idx = line.indexOf(key);
                if (idx >= 0) {
                    float val = line.substring(idx + key.length()).toFloat();
                    cmd.servoOverride[n - 1] = true;
                    cmd.servoAngles.angles[n - 1] = val;
                }
            }
        } else if (p.letter == 'L' || p.letter == 'S' || p.letter == 'R') {
            // Integer or special params: L1, S0, R300
            String key = String(p.letter) + "=";
            int idx = line.indexOf(key);
            if (idx >= 0) {
                float val = line.substring(idx + key.length()).toFloat();
                setParamValue(cmd, p.letter, p.index, val);
            } else {
                // Try without '=' for M420 L1 Z=30 style
                String key2 = " " + String(p.letter);
                idx = line.indexOf(key2);
                if (idx >= 0) {
                    int start = idx + key2.length();
                    int end = line.indexOf(' ', start);
                    if (end < 0) end = line.length();
                    float val = line.substring(start, end).toFloat();
                    setParamValue(cmd, p.letter, p.index, val);
                }
            }
        } else {
            // Standard float params: X, Y, Z, A, B, C, F, J, Z0
            String key = String(p.letter);
            int idx = line.indexOf(key);
            if (idx >= 0) {
                int start = idx + key.length();
                // Skip '=' if present
                if (start < (int)line.length() && line[start] == '=') start++;
                int end = line.indexOf(' ', start);
                if (end < 0) end = line.length();
                float val = line.substring(start, end).toFloat();
                setParamValue(cmd, p.letter, p.index, val);
            }
        }
    }
}

void GCodeParser::setParamValue(GCodeCommand &cmd, char letter, int index, float val) {
    switch (letter) {
        case 'X': cmd.pose.position.x = val;    cmd.hasPosition = true; break;
        case 'Y': cmd.pose.position.y = val;    cmd.hasPosition = true; break;
        case 'Z': cmd.pose.position.z = val;    cmd.hasPosition = true; break;
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
