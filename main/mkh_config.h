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
// WO: PULSE (v1.3.0) adds MKH_MODE_PULSE - a third button-class mode
// alongside momentary/latched. One press-edge emits one fixed-duration
// full-level burst then self-terminates to neutral; taps during an
// active pulse are ignored (no queue/retrigger/extension - "pulses are
// atomic"). See sketch.cpp's MKH_PULSE_DURATION_MS (the bench-tunable
// duration constant) and processMappedInputs() for the state machine.
// Like LATCHED, PULSE is opt-in only (default_mode_for_input() in
// mkh_config.c never resolves to it) and offered for button-class inputs
// only - buttons/bumpers/dpad/stick-clicks (MKH_INPUT_CLASS_DIGITAL) and
// triggers used as buttons (MKH_INPUT_CLASS_TRIGGER, non-proportional).
// Same "meaningless but harmless" treatment as the other non-proportional
// modes on an axis binding - the parser accepts mode=pulse for any token
// uniformly (see mkh_config_parse_line()'s doc comment), it's simply
// never consulted for MKH_INPUT_CLASS_AXIS at drive time; the "sticks:
// not offered" restriction is editor-UI-only, per the order.
typedef enum {
    MKH_MODE_PROPORTIONAL = 0,
    MKH_MODE_MOMENTARY,
    MKH_MODE_LATCHED,
    MKH_MODE_PULSE,
} mkh_input_mode_t;

// WO13: a port carries up to this many bindings. Soft cap - trivially
// raisable, sized generously above any real use (18 input tokens exist
// total, so 8 alternative ways to reach one port already covers nearly
// half the universe). Reported verbatim in the completion report.
#define MKH_MAX_BINDINGS_PER_PORT 8

// WO13: one binding = one input token with its own invert/max/mode.
// Bindings on the same port are ALTERNATIVES ("bound several controls to
// reach it different ways, used one at a time"), not a mixer - see
// mkh_port_config_t's doc comment and processMappedInputs() (sketch.cpp)
// for the sum-and-clamp arbitration that falls out naturally when an
// idle/unpressed binding already contributes zero.
typedef struct {
    mkh_input_source_t input;
    bool invert;
    uint8_t max_percent;  // 0..100
    mkh_input_mode_t mode;
} mkh_binding_t;

// WO13: a port now holds an ordered list of bindings instead of a single
// one. binding_count == 0 means "unassigned" (replaces the old input ==
// MKH_INPUT_NONE check throughout sketch.cpp/mkh_config.c). Arbitration
// across bindings on the same port (when more than one is simultaneously
// active) is entirely a drive-time concern - see processMappedInputs()
// (sketch.cpp): each binding runs through its own invert/max pipeline to
// a signed contribution, contributions sum, the sum clamps to +/-100%,
// and exactly one resulting byte reaches mkh_set_channel() as before.
// Nothing here (or in mkh_broadcast.c) needs to know a port can have more
// than one binding.
typedef struct {
    mkh_binding_t bindings[MKH_MAX_BINDINGS_PER_PORT];
    int binding_count;
    bool from_file;  // true if this port's bindings came from the config
                      // file; false if still the compiled-in default
} mkh_port_config_t;

// Resets the whole table to a blank slate (every port unassigned,
// binding_count=0). Called once at the start of mkh_storage_boot_read()
// and mkh_storage_reload_config(), before any file access is attempted,
// so the table is always in a known-safe state even if storage access
// never succeeds.
//
// BUGFIX #N: no longer pre-seeds the compiled-in fallback bindings
// (HUB1/A=LSNS, HUB2/A=RSNS) itself - see
// mkh_config_apply_compiled_in_fallbacks() below for why, and for where
// that now happens instead.
void mkh_config_set_defaults(void);

// BUGFIX #N: applies the v0.7.0-compatible compiled-in fallback (HUB1
// port A = LSNS, HUB2 port A = RSNS) to those two specific ports, but
// ONLY to a port still at binding_count==0 - i.e. only if nothing (file
// absence, a file that doesn't mention that port, or a storage failure)
// ever gave it a real binding. Call AFTER file parsing/mount-resolution
// is complete, never before (see mkh_config_set_defaults()'s doc comment
// for the bug this ordering fixes). Idempotent.
void mkh_config_apply_compiled_in_fallbacks(void);

// Parses one line of /mk1config.txt and updates the table in place.
// Blank lines and lines starting with '#' are silently ignored. Any
// parse problem (unrecognized key, out-of-range hub/port, invalid
// invert/max/mode value, unknown attribute) logs a warning and skips
// the WHOLE line - the affected port simply keeps whatever bindings it
// already had. curve is the one exception: an unsupported curve value
// logs a warning but the rest of the line still applies (curve is
// always treated as linear). mode= is optional - absent resolves to the
// type-appropriate default (see default_mode_for_input() in
// mkh_config.c) so pre-rev-B files stay valid unchanged.
//
// WO13: a valid line APPENDS a new binding to its port rather than
// replacing it - repeated "HUB<n>_PORT_<letter> = ..." lines for the
// same port accumulate, up to MKH_MAX_BINDINGS_PER_PORT (a line past
// the cap logs a warning and is skipped, same failure posture as any
// other rejected line). This is why a pre-WO13 file with exactly one
// line per port still parses to exactly one binding per port,
// unchanged - the accumulation rule is a strict superset of the old
// "last line wins" single-binding behavior when there's only one line.
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

// WO13 (was mkh_config_set_port(), single-binding): directly writes one
// port's WHOLE binding list into the live table - used by the mapping
// editor's Test-push and Save-apply paths. Replaces whatever bindings
// the port previously had (the editor's pending-edit state is always
// the full, authoritative list for a port it touched, same pattern as
// the pre-WO13 single-binding setter). Bypasses parsing entirely
// (already-validated in-memory data, not raw file text); out-of-range
// device_id/port or count is a silent no-op. count == 0 is valid and
// means "unassign this port" (bindings may be NULL in that case).
void mkh_config_set_port_bindings(int device_id, int port, const mkh_binding_t* bindings, int count);

// WO10-FINAL: public token-name lookup (thin wrapper over the private
// table mkh_config.c already keeps for parsing/logging) - lets
// sketch.cpp show the same token strings the parser reads, without
// keeping a second copy of the name table in sync by hand.
const char* mkh_config_input_token_name(mkh_input_source_t input);

// WO10-FINAL, extended WO13: serializes the resolved table into buf, one
// "HUB<n>_PORT_<letter> = <input> invert=<yes|no> max=<0-100>
// curve=linear [mode=<proportional|momentary|latched>]" line per BINDING
// (not per port - a port with N bindings writes N lines, in binding
// order) - exactly the additive format mkh_config_parse_line() reads
// (repeated "=" lines for the same port accumulate), so editor-written
// files round-trip through the existing parser unchanged, and a
// pre-WO13 single-binding file is just the N==1 case. mode= is omitted
// for MKH_INPUT_CLASS_AXIS bindings (meaningless there - see
// mkh_input_mode_t) and written explicitly otherwise, even when it
// equals the default, so a saved file never depends on a future default
// changing out from under it. Ports with zero bindings are omitted
// entirely. Returns the number of bytes written (excluding the null
// terminator), or -1 if buf_size is too small to hold the whole table
// (buf's content is unspecified in that case - the caller should not
// use it).
int mkh_config_serialize(char* buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif  // MKH_CONFIG_H
