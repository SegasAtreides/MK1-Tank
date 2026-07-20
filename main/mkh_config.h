// SPDX-License-Identifier: Apache-2.0
//
// /mk1config.txt parser and resolved mapping table. Pure parsing logic -
// no filesystem dependency at all, so it can be exercised independently
// of the storage subsystem. mkh_storage.cpp (the sole storage access
// point) is the only caller: it reads lines from the config file and
// feeds them here one at a time.
//
// v1 scope (WORK ORDER v0.8.0 Step 1): builds and logs the table only.
// Nothing in main/sketch.cpp's input handling reads this table yet -
// that's Step 2, on explicit go-ahead.

#ifndef MKH_CONFIG_H
#define MKH_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "mkh_protocol.h"  // MKH_MK6_NUM_DEVICES, MKH_MK6_NUM_CHANNELS

#ifdef __cplusplus
extern "C" {
#endif

// v1 input tokens. Table-driven (see mkh_config.c's kInputTokens) so
// adding a new token later means adding one table row and one enum
// value - not touching the parser logic itself. Do not add tokens here
// speculatively; v1 is LSNS/RSNS only, per the work order.
typedef enum {
    MKH_INPUT_NONE = 0,  // unassigned - port stays neutral (0x80)
    MKH_INPUT_LSNS,       // XWC left stick Y
    MKH_INPUT_RSNS,       // XWC right stick Y
} mkh_input_source_t;

typedef struct {
    mkh_input_source_t input;
    bool invert;
    uint8_t max_percent;  // 0..100
    bool from_file;        // true if this port's value came from the config
                            // file; false if it's still the compiled-in default
} mkh_port_config_t;

// Resets the whole table to compiled-in defaults, reproducing v0.7.0
// behavior exactly: HUB1 (device 0) port A = LSNS, HUB2 (device 1) port
// A = RSNS, every other port unassigned. Called once at the start of
// mkh_storage_boot_read(), before any file access is attempted, so the
// table is always in a known-safe state even if storage access never
// succeeds.
void mkh_config_set_defaults(void);

// Parses one line of /mk1config.txt and updates the table in place.
// Blank lines and lines starting with '#' are silently ignored. Any
// parse problem (unrecognized key, out-of-range hub/port, invalid
// invert/max value, unknown attribute) logs a warning and skips the
// WHOLE line - the affected port simply keeps its current (default)
// value. curve is the one exception: an unsupported curve value logs a
// warning but the rest of the line still applies (curve is always
// treated as linear).
void mkh_config_parse_line(const char* raw_line);

// Records whether the config file was successfully opened at all (drives
// the dashboard's CFG:FS / CFG:DEF indicator, and the summary line in
// mkh_config_log_table()). false means every port is using its compiled-
// in default, e.g. because the file didn't exist or was malformed.
void mkh_config_note_source(bool from_file);
bool mkh_config_source_is_file(void);

// v0.9.0 Step 1: count of lines skipped due to malformed content during
// the most recent parse (reset by mkh_config_set_defaults()). Drives
// the dashboard's degraded CFG:FS! state - CFG:FS means the file parsed
// with zero skipped lines, CFG:FS! means it parsed but at least one
// line was rejected (see mkh_config_log_table()'s per-line "skipped"
// warnings for which).
int mkh_config_get_skipped_line_count(void);

// Logs the fully resolved table: every device/port slot, its resolved
// value, and whether that value came from the file or a default.
void mkh_config_log_table(void);

// Read-only accessor for later steps. Returns NULL for an out-of-range
// device_id/port. Not consumed by input handling until Step 2.
const mkh_port_config_t* mkh_config_get(int device_id, int port);

#ifdef __cplusplus
}
#endif

#endif  // MKH_CONFIG_H
