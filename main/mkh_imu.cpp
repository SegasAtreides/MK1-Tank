// SPDX-License-Identifier: Apache-2.0
//
// WO15: IMU bring-up & gyro instrumentation (pre-loop, heading-lock
// campaign). QMI8658 6-axis IMU, gyroscope only - no accel processing
// beyond what init requires, no fusion, no interrupts. Read, integrate,
// display only; no control behavior, motor output, binding, or mode
// (out of scope per the order).
//
// Bus: shares the SAME physical I2C bus as the CST816D touch controller
// (SDA=IO48, SCL=IO47 - see mkh_touch.cpp's own doc comment, which
// flagged this IMU as "not yet integrated" pending this WO). Wire.begin()
// is NOT called here - mkh_touch_init() already brings the bus up, and
// this driver's init runs strictly after it (see setup()'s call order in
// sketch.cpp). Frequency stays whatever mkh_touch_init() chose (100kHz);
// this driver doesn't reconfigure the bus.
//
// I2C address: the QMI8658's SA0 pin selects 0x6A (SA0 high/floating -
// the documented default, and recommended for a stable level) or 0x6B
// (SA0 low). The schematic's flattened netlist didn't cleanly resolve
// which strap this board uses, so - same bring-up philosophy as
// mkh_touch.cpp's own "prove wiring via a plain I2C ACK, don't trust an
// assumed identity" - this driver probes 0x6A first, falling back to
// 0x6B if that doesn't ACK.
//
// Register map (QMI8658A datasheet, QST Corp, Doc# 13-52-25 Rev A,
// cross-checked against a second, board-specific open-source reference
// for this exact ESP32-S3-Touch-LCD-2 board): WHO_AM_I=0x00 (expect
// 0x05), CTRL3=0x04 (gyro full-scale + ODR), CTRL7=0x08 (sensor enable),
// GZ_L/GZ_H=0x3F/0x40 (Z angular rate, 16-bit two's complement).
//
// Gyro config: full-scale = 0b100 (CTRL3 bits[6:4]) = +/-256 dps - the
// order's preferred class for turret dynamics. ODR = 0b0110 (CTRL3
// bits[3:0]) = 112.1 Hz - the lowest available setting still >=100Hz
// (the next step down, 56.05Hz, misses the floor). CTRL3 = 0x46.
// Sensitivity at +/-256dps (datasheet Table 8): 128 LSB/dps - raw GZ /
// 128.0 = deg/s.
//
// CTRL7 = 0x02: gEN=1 (gyro on), aEN=0 (accelerometer never touched at
// all - "no accel processing beyond what init requires" means none),
// gSN=0 (Full Mode, not Snooze). This is the datasheet's own "Gyro Only"
// operating mode (Table 31).
//
// No burst read: GZ_L and GZ_H are read as two independent single-byte
// transactions (register-select write, then a 1-byte read) rather than
// a 2-byte auto-incrementing burst. This sidesteps CTRL1.ADDR_AI/BE
// (auto-increment and byte-order) entirely instead of depending on
// them - simplicity over the marginal efficiency of one fewer I2C
// transaction per ~150-190ms loop tick.
//
// Startup delay: per the datasheet's own Gyro Turn On Time (Table 8),
// enabling the gyro from Power-On Default needs ~150ms (t1, mechanical
// wakeup) + 3/ODR (t5, filter settling, ~27ms @ 112.1Hz) before its
// output is meaningful. mkh_imu_init() blocks for this once, at boot,
// before starting bias calibration.
//
// Boot-time bias calibration: averages IMU_BIAS_SAMPLES samples
// (assumes the tank is stationary at boot - true for every bring-up/
// bench scenario so far) and subtracts the result from every subsequent
// rate reading before integrating. Runs once, blocking, right after the
// startup delay above - adds ~1s to boot (50 samples * 20ms spacing),
// judged cheap relative to the existing >1.6s splash-to-stats boot
// window (see sketch.cpp's own boot-timing log lines).
//
// Integration: heading += rate_dps * dt, where dt is millis()-measured
// elapsed time since the LAST successful sample - not an assumed tick
// length. The main loop's own cadence is known-variable (~150-190ms),
// and assuming a fixed dt would poison exactly the drift measurement
// this WO exists to characterize. A skipped/failed sample simply leaves
// s_lastSampleMs unchanged, so the next successful sample's dt still
// correctly reflects the true elapsed time - no time lost or double
// counted. Uses millis(), the same timing idiom the rest of this
// codebase already uses (MKH_FRAME_INTERVAL_MS, MKH_CONTROL_REPEAT_MS,
// etc.) - deliberately not micros(), which would be needless precision
// against a >>1ms loop tick.

#include "mkh_imu.h"

#include <Arduino.h>
#include <Wire.h>

#include "uni_log.h"

#define IMU_I2C_ADDR_PRIMARY 0x6A    // SA0 high/floating (datasheet default)
#define IMU_I2C_ADDR_SECONDARY 0x6B  // SA0 low

#define REG_WHO_AM_I 0x00
#define REG_CTRL3 0x04
#define REG_CTRL7 0x08
#define REG_GZ_L 0x3F
#define REG_GZ_H 0x40

#define QMI8658_WHO_AM_I_EXPECTED 0x05

// CTRL3: gFS=0b100 (+/-256dps) in bits[6:4], gODR=0b0110 (112.1Hz) in bits[3:0].
#define IMU_CTRL3_VALUE 0x46
// CTRL7: gSN=0, gEN=1, aEN=0 - gyro-only, Full Mode.
#define IMU_CTRL7_VALUE 0x02

// Datasheet Table 8: +/-256dps FSR -> 128 LSB/dps.
#define IMU_GYRO_SENSITIVITY_LSB_PER_DPS 128.0f

// Gyro Turn On Time (datasheet Table 8): t1 (~150ms wakeup) + t5 (3/ODR
// settling, ~27ms @ 112.1Hz) - rounded up for margin.
#define IMU_STARTUP_DELAY_MS 200

#define IMU_BIAS_SAMPLES 50
#define IMU_BIAS_SAMPLE_SPACING_MS 20

#define IMU_SERIAL_LOG_INTERVAL_MS 1000

static bool s_imuReady = false;
static uint8_t s_imuAddr = 0;

static float s_biasDps = 0.0f;
static float s_rateDps = 0.0f;
static float s_headingDeg = 0.0f;
static uint32_t s_lastSampleMs = 0;
static uint32_t s_lastLogMs = 0;

static bool imuWriteReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(s_imuAddr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool imuReadReg(uint8_t reg, uint8_t* value) {
    Wire.beginTransmission(s_imuAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)  // repeated start, keep bus held
        return false;

    if (Wire.requestFrom((int)s_imuAddr, 1) != 1)
        return false;
    *value = Wire.read();
    return true;
}

static bool imuProbeAddress(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// Raw signed 16-bit Z rate, no bias correction applied yet.
static bool imuReadRawGz(int16_t* outRaw) {
    uint8_t lo, hi;
    if (!imuReadReg(REG_GZ_L, &lo))
        return false;
    if (!imuReadReg(REG_GZ_H, &hi))
        return false;
    *outRaw = (int16_t)(((uint16_t)hi << 8) | lo);
    return true;
}

void mkh_imu_init(void) {
    // Bus already brought up by mkh_touch_init() (shared SDA=48/SCL=47) -
    // deliberately not calling Wire.begin() again here.

    if (imuProbeAddress(IMU_I2C_ADDR_PRIMARY)) {
        s_imuAddr = IMU_I2C_ADDR_PRIMARY;
    } else if (imuProbeAddress(IMU_I2C_ADDR_SECONDARY)) {
        s_imuAddr = IMU_I2C_ADDR_SECONDARY;
    } else {
        loge("MK1 IMU: QMI8658 not responding at I2C 0x%02X or 0x%02X - IMU disabled\n", IMU_I2C_ADDR_PRIMARY,
             IMU_I2C_ADDR_SECONDARY);
        s_imuReady = false;
        return;
    }

    uint8_t whoAmI = 0;
    bool idOk = imuReadReg(REG_WHO_AM_I, &whoAmI);
    if (!idOk || whoAmI != QMI8658_WHO_AM_I_EXPECTED) {
        logi("MK1 IMU: WARNING - WHO_AM_I=0x%02X (expected 0x%02X, read %s) at I2C 0x%02X - proceeding anyway, the "
             "bus ACK above is the primary bring-up proof\n",
             whoAmI, QMI8658_WHO_AM_I_EXPECTED, idOk ? "ok" : "FAILED", s_imuAddr);
    }

    if (!imuWriteReg(REG_CTRL3, IMU_CTRL3_VALUE)) {
        loge("MK1 IMU: CTRL3 write failed - IMU disabled\n");
        s_imuReady = false;
        return;
    }
    if (!imuWriteReg(REG_CTRL7, IMU_CTRL7_VALUE)) {
        loge("MK1 IMU: CTRL7 write failed - IMU disabled\n");
        s_imuReady = false;
        return;
    }

    logi("MK1 IMU: QMI8658 acknowledged at I2C 0x%02X, WHO_AM_I=0x%02X, CTRL3=0x%02X (+/-256dps, 112.1Hz), "
         "CTRL7=0x%02X (gyro-only)\n",
         s_imuAddr, whoAmI, IMU_CTRL3_VALUE, IMU_CTRL7_VALUE);

    delay(IMU_STARTUP_DELAY_MS);

    // Boot-time bias calibration - board assumed stationary.
    float sum = 0.0f;
    int gotSamples = 0;
    for (int i = 0; i < IMU_BIAS_SAMPLES; i++) {
        int16_t raw;
        if (imuReadRawGz(&raw)) {
            sum += (float)raw / IMU_GYRO_SENSITIVITY_LSB_PER_DPS;
            gotSamples++;
        }
        delay(IMU_BIAS_SAMPLE_SPACING_MS);
    }

    if (gotSamples == 0) {
        loge("MK1 IMU: bias calibration got zero valid samples - IMU disabled\n");
        s_imuReady = false;
        return;
    }

    s_biasDps = sum / (float)gotSamples;
    logi("MK1 IMU: boot bias calibration done, %d/%d samples, bias=%.3f dps\n", gotSamples, IMU_BIAS_SAMPLES,
         s_biasDps);

    s_lastSampleMs = millis();
    s_lastLogMs = millis();
    s_imuReady = true;
}

void mkh_imu_poll(void) {
    if (!s_imuReady)
        return;

    int16_t raw;
    if (!imuReadRawGz(&raw))
        return;  // transient I2C miss - s_lastSampleMs unchanged, next sample's dt still correct

    uint32_t now = millis();
    float dtSec = (float)(now - s_lastSampleMs) / 1000.0f;
    s_lastSampleMs = now;

    float rawDps = (float)raw / IMU_GYRO_SENSITIVITY_LSB_PER_DPS;
    s_rateDps = rawDps - s_biasDps;
    s_headingDeg += s_rateDps * dtSec;

    if (now - s_lastLogMs >= IMU_SERIAL_LOG_INTERVAL_MS) {
        s_lastLogMs = now;
        logi("MK1 IMU: rate=%.2f dps heading=%.2f deg\n", s_rateDps, s_headingDeg);
    }
}

float mkh_imu_rate_dps(void) {
    return s_rateDps;
}

float mkh_imu_heading_deg(void) {
    return s_headingDeg;
}

bool mkh_imu_ready(void) {
    return s_imuReady;
}
