// SPDX-License-Identifier: Apache-2.0
//
// /mk1config.txt parser - see mkh_config.h for scope and the failure
// posture (any parse problem on a line skips that line and keeps the
// affected port's compiled-in default; nothing here can abort boot).

#include "mkh_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "mkh_ports.h"
#include "uni_log.h"

// Table-driven input tokens: add a new row (and enum value in
// mkh_config.h) to support a new input source. Nothing else in the
// parser needs to change.
typedef struct {
    const char* name;
    mkh_input_source_t value;
} mkh_input_token_t;

static const mkh_input_token_t kInputTokens[] = {
    {"LSNS", MKH_INPUT_LSNS},
    {"RSNS", MKH_INPUT_RSNS},
};
#define NUM_INPUT_TOKENS (sizeof(kInputTokens) / sizeof(kInputTokens[0]))

static mkh_port_config_t s_table[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS];
static bool s_source_is_file = false;
static int s_skipped_lines = 0;

void mkh_config_set_defaults(void) {
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
            s_table[d][p].input = MKH_INPUT_NONE;
            s_table[d][p].invert = false;
            s_table[d][p].max_percent = 100;
            s_table[d][p].from_file = false;
        }
    }
    // Reproduces v0.7.0 exactly: left stick -> device 0 port A, right
    // stick -> device 1 port A.
    s_table[0][MKH_PORT_A].input = MKH_INPUT_LSNS;
    s_table[1][MKH_PORT_A].input = MKH_INPUT_RSNS;
    s_skipped_lines = 0;
}

int mkh_config_get_skipped_line_count(void) {
    return s_skipped_lines;
}

void mkh_config_note_source(bool from_file) {
    s_source_is_file = from_file;
}

bool mkh_config_source_is_file(void) {
    return s_source_is_file;
}

const mkh_port_config_t* mkh_config_get(int device_id, int port) {
    if (device_id < 0 || device_id >= MKH_MK6_NUM_DEVICES)
        return NULL;
    if (port < 0 || port >= MKH_MK6_NUM_CHANNELS)
        return NULL;
    return &s_table[device_id][port];
}

static const char* input_token_name(mkh_input_source_t input) {
    for (size_t i = 0; i < NUM_INPUT_TOKENS; i++) {
        if (kInputTokens[i].value == input)
            return kInputTokens[i].name;
    }
    return "none";
}

static const char* skip_spaces(const char* s) {
    while (*s == ' ' || *s == '\t')
        s++;
    return s;
}

// In-place right-trim of spaces/tabs/CR/LF (handles CRLF-terminated
// files, e.g. edited on Windows).
static void rstrip(char* s) {
    size_t len = strlen(s);
    while (len > 0) {
        char c = s[len - 1];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            s[--len] = '\0';
        } else {
            break;
        }
    }
}

void mkh_config_parse_line(const char* raw_line) {
    char line[128];
    strncpy(line, raw_line, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    rstrip(line);

    const char* trimmed = skip_spaces(line);
    if (*trimmed == '\0' || *trimmed == '#') {
        return;  // blank or comment - not an error
    }

    char* eq = strchr(line, '=');
    if (!eq) {
        loge("MK1 Config: malformed line (no '='), skipped: \"%s\"\n", trimmed);
        s_skipped_lines++;
        return;
    }
    *eq = '\0';
    char* key = (char*)skip_spaces(line);
    rstrip(key);
    char* value = (char*)skip_spaces(eq + 1);

    int hub_num = 0;
    char port_letter = 0;
    if (sscanf(key, "HUB%d_PORT_%c", &hub_num, &port_letter) != 2) {
        loge("MK1 Config: unrecognized key \"%s\", line skipped\n", key);
        s_skipped_lines++;
        return;
    }

    int device_id = mkh_device_protocol_index(hub_num);
    if (device_id < 0) {
        loge("MK1 Config: HUB%d out of range (1-%d), line skipped\n", hub_num, MKH_MK6_NUM_DEVICES);
        s_skipped_lines++;
        return;
    }

    int port = mkh_port_from_letter(port_letter);
    if (port < 0) {
        loge("MK1 Config: HUB%d_PORT_%c invalid port letter, line skipped\n", hub_num, port_letter);
        s_skipped_lines++;
        return;
    }

    char value_buf[96];
    strncpy(value_buf, value, sizeof(value_buf) - 1);
    value_buf[sizeof(value_buf) - 1] = '\0';

    char* saveptr = NULL;
    char* tok = strtok_r(value_buf, " \t", &saveptr);
    if (!tok) {
        loge("MK1 Config: HUB%d_PORT_%c missing input assignment, line skipped\n", hub_num, port_letter);
        s_skipped_lines++;
        return;
    }

    mkh_input_source_t input = MKH_INPUT_NONE;
    bool input_found = false;
    for (size_t i = 0; i < NUM_INPUT_TOKENS; i++) {
        if (strcasecmp(tok, kInputTokens[i].name) == 0) {
            input = kInputTokens[i].value;
            input_found = true;
            break;
        }
    }
    if (!input_found) {
        loge("MK1 Config: HUB%d_PORT_%c unknown input token \"%s\", line skipped\n", hub_num, port_letter, tok);
        s_skipped_lines++;
        return;
    }

    bool invert = false;
    uint8_t max_percent = 100;

    while ((tok = strtok_r(NULL, " \t", &saveptr)) != NULL) {
        char* attr_eq = strchr(tok, '=');
        if (!attr_eq) {
            loge("MK1 Config: HUB%d_PORT_%c malformed attribute \"%s\", line skipped\n", hub_num, port_letter, tok);
            s_skipped_lines++;
            return;
        }
        *attr_eq = '\0';
        const char* attr_key = tok;
        const char* attr_val = attr_eq + 1;

        if (strcasecmp(attr_key, "invert") == 0) {
            if (strcasecmp(attr_val, "yes") == 0) {
                invert = true;
            } else if (strcasecmp(attr_val, "no") == 0) {
                invert = false;
            } else {
                loge("MK1 Config: HUB%d_PORT_%c invalid invert=\"%s\" (want yes/no), line skipped\n", hub_num,
                     port_letter, attr_val);
                s_skipped_lines++;
                return;
            }
        } else if (strcasecmp(attr_key, "max") == 0) {
            char* endptr = NULL;
            long v = strtol(attr_val, &endptr, 10);
            if (endptr == attr_val || *endptr != '\0' || v < 0 || v > 100) {
                loge("MK1 Config: HUB%d_PORT_%c invalid max=\"%s\" (want 0-100), line skipped\n", hub_num,
                     port_letter, attr_val);
                s_skipped_lines++;
                return;
            }
            max_percent = (uint8_t)v;
        } else if (strcasecmp(attr_key, "curve") == 0) {
            // v1: only "linear" is implemented. Anything else warns but
            // does NOT skip the line - curve is always treated as linear
            // regardless (see mkh_config.h).
            if (strcasecmp(attr_val, "linear") != 0) {
                loge("MK1 Config: HUB%d_PORT_%c unsupported curve=\"%s\", treated as linear\n", hub_num, port_letter,
                     attr_val);
            }
        } else {
            loge("MK1 Config: HUB%d_PORT_%c unknown key \"%s\", line skipped\n", hub_num, port_letter, attr_key);
            s_skipped_lines++;
            return;
        }
    }

    s_table[device_id][port].input = input;
    s_table[device_id][port].invert = invert;
    s_table[device_id][port].max_percent = max_percent;
    s_table[device_id][port].from_file = true;
}

void mkh_config_log_table(void) {
    logi("MK1 Config: resolved mapping table (source: %s):\n", s_source_is_file ? "FILE" : "DEFAULT");
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
            const mkh_port_config_t* c = &s_table[d][p];
            logi("MK1 Config:   HUB%d_PORT_%c = %s invert=%s max=%u curve=linear (source=%s)\n",
                 mkh_device_app_number(d), mkh_port_letter(p), input_token_name(c->input), c->invert ? "yes" : "no",
                 (unsigned)c->max_percent, c->from_file ? "file" : "default");
        }
    }
}
