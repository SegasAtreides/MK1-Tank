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
// parser needs to change. WO10-FINAL rev B: full XWC input universe -
// token strings reported verbatim in the completion report.
typedef struct {
    const char* name;
    mkh_input_source_t value;
} mkh_input_token_t;

static const mkh_input_token_t kInputTokens[] = {
    {"LSNS", MKH_INPUT_LSNS}, {"LSEW", MKH_INPUT_LSEW}, {"RSNS", MKH_INPUT_RSNS}, {"RSEW", MKH_INPUT_RSEW},
    {"LT", MKH_INPUT_LT},     {"RT", MKH_INPUT_RT},     {"A", MKH_INPUT_BTN_A},   {"B", MKH_INPUT_BTN_B},
    {"X", MKH_INPUT_BTN_X},   {"Y", MKH_INPUT_BTN_Y},   {"LB", MKH_INPUT_LB},     {"RB", MKH_INPUT_RB},
    {"DUP", MKH_INPUT_DPAD_UP},   {"DDN", MKH_INPUT_DPAD_DOWN},
    {"DLT", MKH_INPUT_DPAD_LEFT}, {"DRT", MKH_INPUT_DPAD_RIGHT},
    {"L3", MKH_INPUT_L3},     {"R3", MKH_INPUT_R3},
};
#define NUM_INPUT_TOKENS (sizeof(kInputTokens) / sizeof(kInputTokens[0]))

// Table-driven mode tokens, same pattern.
typedef struct {
    const char* name;
    mkh_input_mode_t value;
} mkh_mode_token_t;

static const mkh_mode_token_t kModeTokens[] = {
    {"proportional", MKH_MODE_PROPORTIONAL},
    {"momentary", MKH_MODE_MOMENTARY},
    {"latched", MKH_MODE_LATCHED},
};
#define NUM_MODE_TOKENS (sizeof(kModeTokens) / sizeof(kModeTokens[0]))

static mkh_port_config_t s_table[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS];
static bool s_source_is_file = false;
static int s_skipped_lines = 0;

mkh_input_class_t mkh_config_input_class(mkh_input_source_t input) {
    switch (input) {
        case MKH_INPUT_LSNS:
        case MKH_INPUT_LSEW:
        case MKH_INPUT_RSNS:
        case MKH_INPUT_RSEW:
            return MKH_INPUT_CLASS_AXIS;
        case MKH_INPUT_LT:
        case MKH_INPUT_RT:
            return MKH_INPUT_CLASS_TRIGGER;
        default:
            return MKH_INPUT_CLASS_DIGITAL;
    }
}

// Default mode when a line's mode= attribute is absent - order: "buttons/
// dpad/stick-clicks = momentary (default) | latched. Triggers =
// proportional (default) | momentary | latched." Axis default is
// PROPORTIONAL too, though it's never actually consulted at drive time
// (see processMappedInputs() in sketch.cpp) - stored for consistency
// only, per mkh_input_mode_t's own doc comment.
static mkh_input_mode_t default_mode_for_input(mkh_input_source_t input) {
    if (mkh_config_input_class(input) == MKH_INPUT_CLASS_DIGITAL)
        return MKH_MODE_MOMENTARY;
    return MKH_MODE_PROPORTIONAL;
}

void mkh_config_set_defaults(void) {
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
            s_table[d][p].binding_count = 0;
            s_table[d][p].from_file = false;
        }
    }
    s_skipped_lines = 0;
    // BUGFIX #N: the compiled-in fallback bindings (device0/portA=LSNS,
    // device1/portA=RSNS) used to be pre-seeded right here, before any
    // file line is parsed. That was correct under the pre-WO13 parser's
    // "last line wins" (REPLACE) semantics, but WO13 changed
    // mkh_config_parse_line() to APPEND a binding to whatever a port
    // already has - so a file line addressing either of these two ports
    // (the common case: LSNS/RSNS on port A is the expected default
    // mapping, so most files have it) got appended ON TOP of the
    // pre-seeded compiled-in binding instead of replacing it, silently
    // doubling that port's binding count on every boot. Root cause of
    // the config-duplication bug (see BUGFIX #N completion report) -
    // reproduced 100% deterministically on a blank chip with the stock,
    // unmodified default file, zero editor interaction.
    //
    // Fix: the fallback now only applies AFTER parsing, and only to a
    // port still at binding_count==0 at that point - see
    // mkh_config_apply_compiled_in_fallbacks(), called from every
    // "table resolution is final" point in mkh_storage.cpp. This
    // function now does exactly what its name says - reset to a blank
    // slate - nothing else.
}

// BUGFIX #N: applies the v0.7.0-compatible compiled-in fallback
// (device0/portA=LSNS, device1/portA=RSNS) to those two ports, but ONLY
// if a port is STILL unassigned (binding_count==0) - i.e. only if
// nothing (no file, a file that doesn't mention that port, or a file-
// open/mount failure) ever gave it a real binding. Must run AFTER
// parsing/mount-resolution is complete, never before - see
// mkh_config_set_defaults()'s doc comment for why "before" was the bug.
// Idempotent and safe to call multiple times or on an already-populated
// port (no-ops there, by the same binding_count==0 guard).
void mkh_config_apply_compiled_in_fallbacks(void) {
    if (s_table[0][MKH_PORT_A].binding_count == 0) {
        s_table[0][MKH_PORT_A].bindings[0] =
            (mkh_binding_t){.input = MKH_INPUT_LSNS, .invert = false, .max_percent = 100, .mode = MKH_MODE_PROPORTIONAL};
        s_table[0][MKH_PORT_A].binding_count = 1;
    }
    if (s_table[1][MKH_PORT_A].binding_count == 0) {
        s_table[1][MKH_PORT_A].bindings[0] =
            (mkh_binding_t){.input = MKH_INPUT_RSNS, .invert = false, .max_percent = 100, .mode = MKH_MODE_PROPORTIONAL};
        s_table[1][MKH_PORT_A].binding_count = 1;
    }
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

void mkh_config_set_port_bindings(int device_id, int port, const mkh_binding_t* bindings, int count) {
    if (device_id < 0 || device_id >= MKH_MK6_NUM_DEVICES)
        return;
    if (port < 0 || port >= MKH_MK6_NUM_CHANNELS)
        return;
    if (count < 0)
        return;
    if (count > MKH_MAX_BINDINGS_PER_PORT)
        count = MKH_MAX_BINDINGS_PER_PORT;
    mkh_port_config_t* cfg = &s_table[device_id][port];
    for (int i = 0; i < count; i++) {
        cfg->bindings[i] = bindings[i];
    }
    cfg->binding_count = count;
    cfg->from_file = true;
}

static const char* input_token_name(mkh_input_source_t input) {
    for (size_t i = 0; i < NUM_INPUT_TOKENS; i++) {
        if (kInputTokens[i].value == input)
            return kInputTokens[i].name;
    }
    return "none";
}

const char* mkh_config_input_token_name(mkh_input_source_t input) {
    return input_token_name(input);
}

static const char* mode_token_name(mkh_input_mode_t mode) {
    for (size_t i = 0; i < NUM_MODE_TOKENS; i++) {
        if (kModeTokens[i].value == mode)
            return kModeTokens[i].name;
    }
    return "proportional";
}

int mkh_config_serialize(char* buf, size_t buf_size) {
    size_t used = 0;
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
            const mkh_port_config_t* cfg = &s_table[d][p];
            for (int i = 0; i < cfg->binding_count; i++) {
                const mkh_binding_t* b = &cfg->bindings[i];
                int n;
                if (mkh_config_input_class(b->input) == MKH_INPUT_CLASS_AXIS) {
                    // mode= omitted - meaningless for an axis (mkh_input_mode_t).
                    n = snprintf(buf + used, buf_size - used, "HUB%d_PORT_%c = %s invert=%s max=%u curve=linear\n",
                                 mkh_device_app_number(d), mkh_port_letter(p), input_token_name(b->input),
                                 b->invert ? "yes" : "no", (unsigned)b->max_percent);
                } else {
                    n = snprintf(buf + used, buf_size - used,
                                 "HUB%d_PORT_%c = %s invert=%s max=%u curve=linear mode=%s\n",
                                 mkh_device_app_number(d), mkh_port_letter(p), input_token_name(b->input),
                                 b->invert ? "yes" : "no", (unsigned)b->max_percent, mode_token_name(b->mode));
                }
                if (n < 0 || (size_t)n >= buf_size - used)
                    return -1;
                used += (size_t)n;
            }
        }
    }
    return (int)used;
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

    // WO13: a valid line APPENDS a binding rather than replacing the
    // port - see mkh_config_parse_line()'s doc comment in mkh_config.h.
    // Checked before doing the (otherwise wasted) attribute parsing below.
    if (s_table[device_id][port].binding_count >= MKH_MAX_BINDINGS_PER_PORT) {
        loge("MK1 Config: HUB%d_PORT_%c binding cap (%d) reached, line skipped\n", hub_num, port_letter,
             MKH_MAX_BINDINGS_PER_PORT);
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
    mkh_input_mode_t mode = default_mode_for_input(input);
    bool mode_explicit = false;

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
        } else if (strcasecmp(attr_key, "mode") == 0) {
            // WO10-FINAL rev B. Accepted for any input token regardless
            // of class (kept simple/uniform, per mkh_input_mode_t's doc
            // comment) - an axis port with an explicit mode= parses and
            // stores it without complaint, it's just never consulted at
            // drive time.
            bool mode_found = false;
            for (size_t i = 0; i < NUM_MODE_TOKENS; i++) {
                if (strcasecmp(attr_val, kModeTokens[i].name) == 0) {
                    mode = kModeTokens[i].value;
                    mode_found = true;
                    break;
                }
            }
            if (!mode_found) {
                loge(
                    "MK1 Config: HUB%d_PORT_%c invalid mode=\"%s\" (want "
                    "proportional/momentary/latched), line skipped\n",
                    hub_num, port_letter, attr_val);
                s_skipped_lines++;
                return;
            }
            mode_explicit = true;
        } else {
            loge("MK1 Config: HUB%d_PORT_%c unknown key \"%s\", line skipped\n", hub_num, port_letter, attr_key);
            s_skipped_lines++;
            return;
        }
    }
    (void)mode_explicit;  // mode already defaulted above if this stayed false

    mkh_port_config_t* cfg = &s_table[device_id][port];
    cfg->bindings[cfg->binding_count++] =
        (mkh_binding_t){.input = input, .invert = invert, .max_percent = max_percent, .mode = mode};
    cfg->from_file = true;
}

void mkh_config_log_table(void) {
    logi("MK1 Config: resolved mapping table (source: %s):\n", s_source_is_file ? "FILE" : "DEFAULT");
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
            const mkh_port_config_t* cfg = &s_table[d][p];
            if (cfg->binding_count == 0) {
                logi("MK1 Config:   HUB%d_PORT_%c = none (source=%s)\n", mkh_device_app_number(d),
                     mkh_port_letter(p), cfg->from_file ? "file" : "default");
                continue;
            }
            for (int i = 0; i < cfg->binding_count; i++) {
                const mkh_binding_t* b = &cfg->bindings[i];
                logi("MK1 Config:   HUB%d_PORT_%c[%d] = %s invert=%s max=%u curve=linear mode=%s (source=%s)\n",
                     mkh_device_app_number(d), mkh_port_letter(p), i, input_token_name(b->input),
                     b->invert ? "yes" : "no", (unsigned)b->max_percent, mode_token_name(b->mode),
                     cfg->from_file ? "file" : "default");
            }
        }
    }
}
