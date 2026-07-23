// SPDX-License-Identifier: Apache-2.0
//
// WO16: THL tunables, live serial tuning, control law, status. See
// mkh_thl.h for what lives here vs. sketch.cpp.
//
// Live tuning path: "editor page or existing config path - CC's choice,
// cheapest wins" (order's own wording). The settings page is already a
// fully packed 5-row, 240px layout (see sketch.cpp's own SPAGE_* layout
// comments from WO13) - fitting 4 more numeric rows there means a whole
// new sub-page, not a cheap addition. A small serial command reader is
// genuinely runtime-editable (no reboot, no recompile) and fits the
// bench workflow every WO so far has actually used: PM and CC both
// watching/typing over the same USB serial connection already in use
// for the 1Hz readout and every capture in this campaign. Commands:
// "thl deadband <deg>", "thl kp <pct/deg>", "thl floor <pct>",
// "thl ceiling <pct>" - one per line, case-insensitive, logged whether
// accepted or rejected. Values reset to the compiled defaults on power
// cycle - not written to the config file; promote a found-good value to
// the compiled default in a follow-up commit once tuning is done.

#include "mkh_thl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Arduino.h>

#include "uni_log.h"

static float s_deadbandDeg = MKH_THL_DEFAULT_DEADBAND_DEG;
static float s_kpPctPerDeg = MKH_THL_DEFAULT_KP_PCT_PER_DEG;
static float s_floorPct = MKH_THL_DEFAULT_FLOOR_PCT;
static float s_ceilingPct = MKH_THL_DEFAULT_CEILING_PCT;
static float s_cancelThresholdPct = MKH_THL_DEFAULT_CANCEL_THRESHOLD_PCT;

static mkh_thl_state_t s_statusState = MKH_THL_STATE_OFF;
static float s_statusErrorDeg = 0.0f;
static float s_statusOutputPct = 0.0f;

#define THL_TUNE_LINE_MAX 47
static char s_tuneLineBuf[THL_TUNE_LINE_MAX + 1];
static size_t s_tuneLineLen = 0;

float mkh_thl_deadband_deg(void) {
    return s_deadbandDeg;
}
float mkh_thl_kp_pct_per_deg(void) {
    return s_kpPctPerDeg;
}
float mkh_thl_floor_pct(void) {
    return s_floorPct;
}
float mkh_thl_ceiling_pct(void) {
    return s_ceilingPct;
}
float mkh_thl_cancel_threshold_pct(void) {
    return s_cancelThresholdPct;
}

// Parses one already-collected line (no trailing newline) of the form
// "thl <param> <value>". Returns true and applies the value if it
// matched and parsed cleanly; false (with a logged reason) otherwise.
static bool applyTuneLine(char* line) {
    char* saveptr = NULL;
    char* cmd = strtok_r(line, " \t", &saveptr);
    if (!cmd || strcasecmp(cmd, "thl") != 0)
        return false;  // not a THL command - silently ignored, could be operator chatter/other tooling

    char* param = strtok_r(NULL, " \t", &saveptr);
    char* valueStr = strtok_r(NULL, " \t", &saveptr);
    if (!param || !valueStr) {
        logi("MK1 THL: tune command missing param/value - usage: thl <deadband|kp|floor|ceiling|cancel> <value>\n");
        return false;
    }

    char* endPtr = NULL;
    float value = strtof(valueStr, &endPtr);
    if (endPtr == valueStr || value < 0.0f) {
        logi("MK1 THL: tune command bad value '%s' (must be a non-negative number)\n", valueStr);
        return false;
    }

    if (strcasecmp(param, "deadband") == 0) {
        s_deadbandDeg = value;
        logi("MK1 THL: deadband set to %.2f deg\n", s_deadbandDeg);
    } else if (strcasecmp(param, "kp") == 0) {
        s_kpPctPerDeg = value;
        logi("MK1 THL: Kp set to %.2f %%/deg\n", s_kpPctPerDeg);
    } else if (strcasecmp(param, "floor") == 0) {
        s_floorPct = value;
        logi("MK1 THL: floor set to %.2f %%\n", s_floorPct);
    } else if (strcasecmp(param, "ceiling") == 0) {
        s_ceilingPct = value;
        logi("MK1 THL: ceiling set to %.2f %%\n", s_ceilingPct);
    } else if (strcasecmp(param, "cancel") == 0) {
        s_cancelThresholdPct = value;
        logi("MK1 THL: cancel threshold set to %.2f %%\n", s_cancelThresholdPct);
    } else {
        logi("MK1 THL: unknown tune param '%s' (expected deadband|kp|floor|ceiling|cancel)\n", param);
        return false;
    }
    return true;
}

void mkh_thl_poll_serial_tuning(void) {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n') {
            if (s_tuneLineLen > 0) {
                s_tuneLineBuf[s_tuneLineLen] = '\0';
                applyTuneLine(s_tuneLineBuf);
                s_tuneLineLen = 0;
            }
            continue;
        }
        if (s_tuneLineLen < THL_TUNE_LINE_MAX) {
            s_tuneLineBuf[s_tuneLineLen++] = c;
        }  // else: line too long - silently truncated, next '\n' still flushes what we have
    }
}

float mkh_thl_compute_output_pct(float referenceDeg, float headingDeg, float* outErrorDeg) {
    float error = referenceDeg - headingDeg;  // CW-from-above positive, per the WO16 ruling
    if (outErrorDeg)
        *outErrorDeg = error;

    if (fabsf(error) < s_deadbandDeg)
        return 0.0f;

    float outputPct = s_kpPctPerDeg * error;

    if (outputPct > s_ceilingPct)
        outputPct = s_ceilingPct;
    else if (outputPct < -s_ceilingPct)
        outputPct = -s_ceilingPct;

    if (outputPct > 0.0f && outputPct < s_floorPct)
        outputPct = s_floorPct;
    else if (outputPct < 0.0f && outputPct > -s_floorPct)
        outputPct = -s_floorPct;

    return outputPct;
}

void mkh_thl_set_status(mkh_thl_state_t state, float errorDeg, float outputPct) {
    s_statusState = state;
    s_statusErrorDeg = errorDeg;
    s_statusOutputPct = outputPct;
}

mkh_thl_state_t mkh_thl_status_state(void) {
    return s_statusState;
}
float mkh_thl_status_error_deg(void) {
    return s_statusErrorDeg;
}
float mkh_thl_status_output_pct(void) {
    return s_statusOutputPct;
}

#define THL_STATUS_LOG_INTERVAL_MS 1000
static uint32_t s_lastStatusLogMs = 0;

void mkh_thl_log_status_1hz(void) {
    uint32_t now = millis();
    if (now - s_lastStatusLogMs < THL_STATUS_LOG_INTERVAL_MS)
        return;
    s_lastStatusLogMs = now;

    switch (s_statusState) {
        case MKH_THL_STATE_ENGAGED:
            logi("MK1 THL: state=ENGAGED error=%.2f deg output=%.1f %%\n", s_statusErrorDeg, s_statusOutputPct);
            break;
        case MKH_THL_STATE_CALIBRATING_BLOCKED:
            logi("MK1 THL: state=CALIBRATING\n");
            break;
        default:
            logi("MK1 THL: state=OFF\n");
            break;
    }
}
