// SPDX-License-Identifier: Apache-2.0
//
// v0.9.0 Step 0: CST816D touch bring-up proof. See mkh_touch.h.
//
// Bus sharing: TP_SDA=IO48, TP_SCL=IO47 are the SAME physical I2C bus
// the QMI8658 IMU will eventually use (PM-verified from the Waveshare
// schematic, same source as the SD-era shared-SPI-bus findings). The
// IMU is not initialized anywhere in this project yet, but this driver
// must not configure the bus in a way that would make future IMU
// integration harder.
//
// I2C frequency: 100kHz (I2C Standard Mode). Chosen deliberately
// conservative rather than the 400kHz Fast Mode some touch reference
// designs use: this is the bus's first ever use in this project, the
// QMI8658's real maximum bus speed is not confirmed against a datasheet
// here, and 100kHz is compatible with essentially every I2C peripheral
// in existence. Nothing in this driver assumes 100kHz specifically -
// revisit once IMU integration establishes real timing requirements.
//
// TP_INT (IO46): v1 (WO9 Step 0) used this as a cheap polled digitalRead
// gate rather than a true interrupt, reasoning the extra complexity
// wasn't needed. WO9 Step 1's PM bench testing proved that wrong: level-
// polling a pulse-based interrupt line at ~150ms (main loop cadence) or
// even ~40ms (during a switch animation - see sketch.cpp) can simply
// miss a pulse shorter than the polling gap, for a press, a release, or
// both - reproduced as a deterministic "works once after reset, then
// stuck" failure (diagnostic: repeated real finger contact produced NO
// PRESS/RELEASE log lines at all, ruling out a lockout/logic bug in the
// caller). Fixed by moving to a real FALLING-edge hardware interrupt
// (attachInterrupt below): the ISR only sets a flag - no I2C, no
// logging, nothing not ISR-safe - and mkh_touch_poll() below consumes
// that flag and does the actual (non-ISR-context) I2C read. This can no
// longer miss a pulse regardless of how it relates to the loop's own
// timing, since the hardware interrupt fires independently of anything
// this firmware is doing when the edge occurs.
//
// Register map: only FINGER_NUM (0x02) and the X/Y position registers
// (0x03-0x06) are used - these are consistent across the CST816S/T/D
// family. GESTURE_ID and CHIP_ID register addresses are known to differ
// across silicon revisions/vendor docs, so this driver deliberately
// doesn't depend on them. Bring-up is proven instead by a plain I2C
// address ACK (Wire.endTransmission() == 0) at CST816D_I2C_ADDR - a
// direct test of "is the wiring/address/pin-mapping correct" rather
// than trusting an uncertain identity register.
//
// No reset pin: the work order's schematic findings list only
// TP_SDA/TP_SCL/TP_INT, no TP_RST - assuming power-on reset is
// sufficient. Revisit if bring-up proves otherwise.
//
// Touch->display transform (v0.9.0 Step 0b, PM-directed bench
// calibration): derived from an 11-point calibration session (header,
// each dashboard row, row edges, all four screen corners).
//
// display_x = 1.0175*raw_y - 3.0 - a clean, near-1:1 fit of raw_y alone
// against the four screen corners (residual <=2 units out of 319).
//
// display_y = 271.6 - 1.2114*raw_x - fit from the five CENTRAL
// calibration points only (header + the four dashboard rows), which
// are tightly linear in raw_x alone (residual <=3 units out of 239).
// An earlier version of this fit used all four corners too (a 2-variable
// affine in both raw_x and raw_y) on the theory that the extra corner
// data would only sharpen the fit - it didn't: that version put a
// verification tap on the MKH 2 row into the MKH 3 row's band (~47
// units off), because the corners pulled the fit to reflect real
// touch-panel edge nonlinearity (reduced accuracy near the digitizer's
// physical boundary) that governs the corners but not the row-center
// region where every real tap in this UI actually lands. The corners
// remain useful for confirming display_x's range/scale, just not for
// display_y's slope, which is why this constant differs from raw
// corner extrapolation. Re-derive from a fresh calibration pass if the
// panel is ever replaced.

#include "mkh_touch.h"

#include <math.h>

#include <Arduino.h>
#include <Wire.h>

#include "uni_log.h"

#define TP_SDA_PIN 48
#define TP_SCL_PIN 47
#define TP_INT_PIN 46
#define CST816D_I2C_ADDR 0x15
#define I2C_FREQUENCY_HZ 100000

// Read as one contiguous 5-byte block: FINGER_NUM, XPOS_H, XPOS_L,
// YPOS_H, YPOS_L (registers 0x02-0x06).
#define REG_FINGER_NUM 0x02

// Landscape framebuffer bounds - must match sketch.cpp's SCR_W/SCR_H.
#define DISPLAY_W 320
#define DISPLAY_H 240

static bool s_touchReady = false;
static bool s_wasPressed = false;

// Set by touchIsr() (hardware interrupt context - IRAM-resident,
// minimal, no I2C/logging), cleared by mkh_touch_poll() (normal task
// context, where the actual I2C read happens). volatile because it's
// shared between an ISR and the main loop with no other synchronization
// - a plain flag is sufficient here since it's just a single-bit
// "something happened" signal, not a multi-field structure.
static volatile bool s_touchIntFlag = false;

static void IRAM_ATTR touchIsr(void) {
    s_touchIntFlag = true;
}

// The one site implementing the touch->display transform - see the
// derivation comment above this file's header block.
static void mkh_touch_raw_to_display(uint16_t rawX, uint16_t rawY, int16_t* displayX, int16_t* displayY) {
    float dx = 1.0175f * (float)rawY - 3.0f;
    float dy = 271.6f - 1.2114f * (float)rawX;

    if (dx < 0)
        dx = 0;
    if (dx > DISPLAY_W - 1)
        dx = DISPLAY_W - 1;
    if (dy < 0)
        dy = 0;
    if (dy > DISPLAY_H - 1)
        dy = DISPLAY_H - 1;

    *displayX = (int16_t)lroundf(dx);
    *displayY = (int16_t)lroundf(dy);
}

// Reads `len` bytes starting at register `reg` into `buf`. Returns true
// on success (device ACK'd both the register-select write and the
// subsequent read).
static bool cst816ReadRegs(uint8_t reg, uint8_t* buf, size_t len) {
    Wire.beginTransmission(CST816D_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)  // repeated start, keep bus held
        return false;

    size_t got = Wire.requestFrom((int)CST816D_I2C_ADDR, (int)len);
    if (got != len)
        return false;

    for (size_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

void mkh_touch_init(void) {
    pinMode(TP_INT_PIN, INPUT_PULLUP);

    if (!Wire.begin(TP_SDA_PIN, TP_SCL_PIN, I2C_FREQUENCY_HZ)) {
        loge("MK1 Touch: Wire.begin() failed (SDA=%d, SCL=%d)\n", TP_SDA_PIN, TP_SCL_PIN);
        s_touchReady = false;
        return;
    }

    Wire.beginTransmission(CST816D_I2C_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        loge("MK1 Touch: CST816D not responding at I2C 0x%02X (error=%u) - touch disabled\n", CST816D_I2C_ADDR, err);
        s_touchReady = false;
        return;
    }

    logi("MK1 Touch: CST816D acknowledged at I2C 0x%02X (SDA=%d, SCL=%d, %luHz)\n", CST816D_I2C_ADDR, TP_SDA_PIN,
         TP_SCL_PIN, (unsigned long)I2C_FREQUENCY_HZ);

    attachInterrupt(digitalPinToInterrupt(TP_INT_PIN), touchIsr, FALLING);
    s_touchReady = true;
}

bool mkh_touch_poll(int16_t* outDisplayX, int16_t* outDisplayY) {
    if (!s_touchReady)
        return false;

    if (!s_touchIntFlag)
        return false;
    s_touchIntFlag = false;  // clear BEFORE the I2C read - if another edge
                              // arrives mid-read, we want it re-armed for
                              // the NEXT poll, not silently dropped

    uint8_t buf[5];  // FINGER_NUM, XPOS_H, XPOS_L, YPOS_H, YPOS_L
    if (!cst816ReadRegs(REG_FINGER_NUM, buf, sizeof(buf))) {
        loge("MK1 Touch: read failed\n");
        return false;
    }

    uint8_t fingerNum = buf[0];
    uint16_t x = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
    uint16_t y = ((uint16_t)(buf[3] & 0x0F) << 8) | buf[4];
    bool pressed = fingerNum > 0;

    int16_t dispX, dispY;
    mkh_touch_raw_to_display(x, y, &dispX, &dispY);

    // v0.9.0 Step 1: a tap is the rising edge only - computed before
    // s_wasPressed is updated below, so it fires exactly once per
    // press, on the same poll that logs "PRESS".
    bool isNewPress = pressed && !s_wasPressed;

    if (pressed != s_wasPressed) {
        logi("MK1 Touch: %s raw=(x=%u, y=%u) display=(x=%d, y=%d)\n", pressed ? "PRESS" : "RELEASE", (unsigned)x,
             (unsigned)y, dispX, dispY);
        s_wasPressed = pressed;
    } else if (pressed) {
        // Still down - log movement too. loop() runs at ~150ms cadence
        // (see sketch.cpp), so this is not noisy.
        logi("MK1 Touch: MOVE  raw=(x=%u, y=%u) display=(x=%d, y=%d)\n", (unsigned)x, (unsigned)y, dispX, dispY);
    }

    if (isNewPress) {
        if (outDisplayX)
            *outDisplayX = dispX;
        if (outDisplayY)
            *outDisplayY = dispY;
        return true;
    }
    return false;
}

bool mkh_touch_is_pressed(void) {
    return s_wasPressed;
}
