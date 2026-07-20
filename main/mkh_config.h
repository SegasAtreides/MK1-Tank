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
//
// WO10-FINAL rev B: full XWC input universe (was LSNS/RSNS only) plus a
// per-port drive mode (proportional/momentary/latched). Table-driven
// exactly as before - see mkh_config.c's kInputTokens - so this was
// still "add table rows, not touch the parser logic."

#ifndef MKH_CONFIG_H
#define MKH_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mkh_protocol.h"  // MKH_MK6_NUM_DEVICES, MKH_MK6_NUM_CHANNELS

#ifdef __cplusplus
extern "C" {
#endif

// WO10-FINAL rev B: full input universe. Token strings (mkh_config.c's
// kInputTokens) are reported verbatim in the WO10-FINAL rev B
// completion report - short and consistent: LSNS/LSEW/RSNS/RSEW extend
// the pre-existing N/S convention to both stick axes; LT/RT, A/B/X/Y,
// LB/RB, L3/R3 match the order's own naming; DUP/DDN/DLT/DRT for the
// d-pad (kept to the same 2-4 char length as everything else).
typedef enum {
    MKH_INPUT_NONE = 0,  // unassigned - port stays neutral (0x80)
    MKH_INPUT_LSNS,       // left stick Y - axis
    MKH_INPUT_LSEW,       // left stick X - axis
    MKH_INPUT_RSNS,       // right stick Y - axis
    MKH_INPUT_RSEW,       // right stick X - axis
    MKH_INPUT_LT,         // left analog trigger - trigger
    MKH_INPUT_RT,         // right analog trigger - trigger
    MKH_INPUT_BTN_A,      // face button A - digital
    MKH_INPUT_BTN_B,      // face button B - digital
    MKH_INPUT_BTN_X,      // face button X - digital
    MKH_INPUT_BTN_Y,      // face button Y - digital
    MKH_INPUT_LB,         // left bumper - digital
    MKH_INPUT_RB,         // right bumper - digital
    MKH_INPUT_DPAD_UP,    // d-pad up - digital
    MKH_INPUT_DPAD_DOWN,  // d-pad down - digital
    MKH_INPUT_DPAD_LEFT,  // d-pad left - digital
    MKH_INPUT_DPAD_RIGHT, // d-pad right - digital
    MKH_INPUT_L3,         // left stick click - digital
    MKH_INPUT_R3,         // right stick click - digital
} mkh_input_source_t;

// WO10-FINAL rev B: input class - what KIND of signal a token produces,
// independent of the port's chosen mode. Drives default-mode
// resolution (mkh_config.c), the drive sweep's raw-value interpretation
// (sketch.cpp's processMappedInputs()), capture's threshold-vs-press
// branch (checkCapture()), and the settings page's mode-widget
// visibility (2-state for digital, 3-state for trigger, hidden for
// axis - order's own wording).
typedef enum {
    MKH_INPUT_CLASS_AXIS,     // LSNS/LSEW/RSNS/RSEW - always proportional, no mode
    MKH_INPUT_CLASS_TRIGGER,  // LT/RT - proportional (default), momentary, or latched
    MKH_INPUT_CLASS_DIGITAL,  // everything else - momentary (default) or latched
} mkh_input_class_t;

// Returns the input class for a token. MKH_INPUT_NONE resolves to
// MKH_INPUT_CLASS_DIGITAL (an arbitrary but harmless choice - NONE
// ports are always skipped by both the drive sweep and capture before
// class is ever consulted).
mkh_input_class_t mkh_config_input_class(mkh_input_source_t input);

// WO10-FINAL rev B: per-port drive mode. See mkh_config.c's
// default_mode_for_input() for the default-when-absent resolution rule
// and processMappedInputs() (sketch.cpp) for how each mode drives its
// port. Stored per port even for MKH_INPUT_CLASS_AXIS ports (where it's
// semantically meaningless - "stick axes: no mode, always proportional"
// per the order) rather than rejected at parse time; the drive sweep
// simply never consults mode for an axis-class port, which keeps the
// parser's attribute handling uniform across all token types.
typedef enum {
    MKH_MODE_PROPORTIONAL = 0,
    MKH_MODE_MOMENTARY,
    MKH_MODE_LATCHED,
} mkh_input_mode_t;

typedef struct {
    mkh_input_source_t input;
    bool invert;
    uint8_t max_percent;  // 0..100
    mkh_input_mode_t mode;
    bool from_file;  // true if this port's value came from the config
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
// invert/max/mode value, unknown attribute) logs a warning and skips
// the WHOLE line - the affected port simply keeps its current (default)
// value. curve is the one exception: an unsupported curve value logs a
// warning but the rest of the line still applies (curve is always
// treated as linear). mode= is optional - absent resolves to the
// type-appropriate default (see default_mode_for_input() in
// mkh_config.c) so pre-rev-B files stay valid unchanged.
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

// WO10-FINAL: directly writes one port's resolved config into the live
// table - used by the mapping editor's Test-push and Save-apply paths.
// Bypasses parsing entirely (this is already-validated in-memory data,
// not raw file text); out-of-range device_id/port is a silent no-op,
// matching mkh_config_get()'s NULL-on-invalid-range convention.
void mkh_config_set_port(int device_id, int port, mkh_input_source_t input, bool invert, uint8_t max_percent,
                          mkh_input_mode_t mode);

// WO10-FINAL: public token-name lookup (thin wrapper over the private
// table mkh_config.c already keeps for parsing/logging) - lets
// sketch.cpp show the same token strings the parser reads, without
// keeping a second copy of the name table in sync by hand.
const char* mkh_config_input_token_name(mkh_input_source_t input);

// WO10-FINAL: serializes the resolved table into buf, one
// "HUB<n>_PORT_<letter> = <input> invert=<yes|no> max=<0-100>
// curve=linear [mode=<proportional|momentary|latched>]" line per
// assigned port (input != MKH_INPUT_NONE) - exactly the format
// mkh_config_parse_line() reads, so editor-written files round-trip
// through the existing parser unchanged. mode= is omitted for
// MKH_INPUT_CLASS_AXIS ports (meaningless there - see mkh_input_mode_t)
// and written explicitly otherwise, even when it equals the default,
// so a saved file never depends on a future default changing out from
// under it. Unassigned ports are omitted entirely (the parser already
// defaults those to NONE). Returns the number of bytes written
// (excluding the null terminator), or -1 if buf_size is too small to
// hold the whole table (buf's content is unspecified in that case - the
// caller should not use it).
int mkh_config_serialize(char* buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif  // MKH_CONFIG_H
