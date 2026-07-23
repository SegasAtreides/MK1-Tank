// SPDX-License-Identifier: Apache-2.0
//
// WO16: Turret Heading-Lock (THL) - global P-control tunables, live
// serial tuning, and status for the stats page / 1Hz serial line. The
// engage/disengage state machine and per-binding control-law wiring
// live in sketch.cpp (same file as LATCHED/PULSE's own per-binding
// state, for consistency) - this file holds only what's genuinely
// global: the four tunables (one active lock at a time is the expected
// real use, per the WO16 report) and the pure control-law math.

#ifndef MKH_THL_H
#define MKH_THL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MKH_THL_STATE_OFF = 0,
    MKH_THL_STATE_ENGAGED,
    MKH_THL_STATE_CALIBRATING_BLOCKED,
} mkh_thl_state_t;

// Compiled defaults, WO21 revert (back to WO16's original ceiling).
// WO17 raised ceiling to 100% for full counter-rotation authority
// during pivots; WO19's field defect (sustained, non-decaying THL
// oscillation) was bench-convicted at ceiling=100% as a floor-deadband
// limit cycle - not a code defect (WO19's inspection and trace stand as
// the exonerating evidence; wrap180() below is correct and kept, it
// simply wasn't the cause). Live serial confirmation (`thl ceiling 60`)
// made the oscillation stop. Reverted here to the known-good ceiling;
// rate-damping (a PD term) is parked as a known lever for a future
// campaign rather than attempted under this WO's P-only constraint.
// deadband=2.0deg, Kp=3%/deg, floor=15%: unchanged. Live-tunable below -
// these are only the boot-time starting point, not read again after boot.
#define MKH_THL_DEFAULT_DEADBAND_DEG 2.0f
#define MKH_THL_DEFAULT_KP_PCT_PER_DEG 3.0f
#define MKH_THL_DEFAULT_FLOOR_PCT 15.0f
#define MKH_THL_DEFAULT_CEILING_PCT 60.0f

// Exclusivity v2 (spec amendment, cruise-control model - supersedes
// WO16 Step 4's plain suppression): manual traverse input on the THL
// port beyond this threshold (percent of full scale from neutral)
// cancels an engaged lock immediately and takes the port that same
// tick - see sketch.cpp's processMappedInputs() for the cancel check.
// Live-tunable like the other four.
#define MKH_THL_DEFAULT_CANCEL_THRESHOLD_PCT 10.0f

float mkh_thl_deadband_deg(void);
float mkh_thl_kp_pct_per_deg(void);
float mkh_thl_floor_pct(void);
float mkh_thl_ceiling_pct(void);
float mkh_thl_cancel_threshold_pct(void);

// Reads and applies at most one pending "thl <param> <value>" line from
// Serial (param one of deadband/kp/floor/ceiling/cancel; value a plain
// number).
// Call once per loop() tick - non-blocking, no-op if no complete line is
// buffered yet. Every accepted or rejected command is logged - never a
// silent failure. Values persist only for this power cycle (matching
// "bench characterization... PM tunes live" - not written to the config
// file; promote a found-good value to the compiled default in a
// follow-up commit once tuning is done).
void mkh_thl_poll_serial_tuning(void);

// The P-only control law: error = referenceDeg - headingDeg (CW-from-
// above positive - see the WO16 report's direction-mapping section).
// Returns the commanded output as a signed percent of full scale
// (magnitude clamped to ceiling, deadband zeroes small error, nonzero
// results below floor are raised to floor). Writes the raw signed error
// (degrees) to *outErrorDeg if non-NULL - callers use this for status
// display, not for anything the control law itself needs back.
float mkh_thl_compute_output_pct(float referenceDeg, float headingDeg, float* outErrorDeg);

// Status snapshot for the stats page / 1Hz serial line - set by
// sketch.cpp's processMappedInputs() from whichever THL binding it most
// recently touched.
void mkh_thl_set_status(mkh_thl_state_t state, float errorDeg, float outputPct);
mkh_thl_state_t mkh_thl_status_state(void);
float mkh_thl_status_error_deg(void);
float mkh_thl_status_output_pct(void);

// Logs the current status once per second (self-gating internally, same
// pattern as mkh_imu.cpp's own 1Hz line) - "lock state and output %"
// alongside the IMU's own 1Hz rate/heading line. Call once per loop()
// iteration, right alongside mkh_imu_poll().
void mkh_thl_log_status_1hz(void);

#ifdef __cplusplus
}
#endif

#endif  // MKH_THL_H
