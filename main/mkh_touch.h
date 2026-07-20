// SPDX-License-Identifier: Apache-2.0
//
// v0.9.0 Step 0: CST816D capacitive touch controller bring-up. I2C bus
// shared with the QMI8658 IMU (not yet integrated) - see mkh_touch.cpp
// for the frequency choice and shared-bus reasoning. No dashboard
// integration yet; this step only proves the hardware responds and logs
// touch events to serial.

#ifndef MKH_TOUCH_H
#define MKH_TOUCH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the I2C bus and verifies the CST816D acknowledges its I2C
// address. Call once from setup(). Logs the result; never aborts boot.
void mkh_touch_init(void);

// Polls TP_INT and, if asserted, reads and logs one touch event (every
// PRESS/RELEASE/MOVE, unconditionally - unchanged from Step 0). Call
// once per loop() iteration. No-op (returns false) if mkh_touch_init()
// didn't succeed.
//
// v0.9.0 Step 1: also returns true, with *outDisplayX/*outDisplayY set,
// exactly on the loop() iteration a NEW press begins (a rising edge -
// finger-down only, not held/moved/released) - callers use this to
// hit-test and act on a discrete tap without re-triggering on every
// poll while a finger stays down.
bool mkh_touch_poll(int16_t* outDisplayX, int16_t* outDisplayY);

#ifdef __cplusplus
}
#endif

#endif  // MKH_TOUCH_H
