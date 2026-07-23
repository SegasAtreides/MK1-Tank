// SPDX-License-Identifier: Apache-2.0
//
// WO15: QMI8658 IMU bring-up (pre-loop, heading-lock campaign). Gyro Z
// only - no accel processing beyond what init requires, no fusion, no
// interrupts, no control behavior. Read, integrate, display only. See
// mkh_imu.cpp for the register map / bring-up reasoning.

#ifndef MKH_IMU_H
#define MKH_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configures the QMI8658 gyroscope and runs boot-time bias calibration
// (blocking, ~1.2s). Call once from setup(), AFTER mkh_touch_init() -
// this shares the SAME I2C bus (SDA=48, SCL=47) and does not call
// Wire.begin() itself, relying on touch's bus bring-up. Logs the result;
// never aborts boot on failure (mkh_imu_ready() reports false instead).
void mkh_imu_init(void);

// Polls one Z-axis rate sample and integrates it into the running
// heading using the actually-elapsed time since the last call (not an
// assumed tick length). Also handles the 1Hz serial readout. Call once
// per loop() iteration, unconditionally - no-op if mkh_imu_init() didn't
// succeed.
void mkh_imu_poll(void);

// Current instantaneous Z rate, deg/s, bias-corrected. Display only.
float mkh_imu_rate_dps(void);

// Current integrated heading, degrees, signed, arbitrary zero at boot,
// bias-corrected. Display only.
float mkh_imu_heading_deg(void);

// True once init succeeded and boot bias calibration completed - i.e.
// the two accessors above are meaningful.
bool mkh_imu_ready(void);

#ifdef __cplusplus
}
#endif

#endif  // MKH_IMU_H
