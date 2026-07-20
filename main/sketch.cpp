// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <math.h>

#include <Arduino.h>
#include <Bluepad32.h>
#include <Arduino_GFX_Library.h>

#include "mkh_broadcast.h"
#include "mkh_config.h"
#include "mkh_ports.h"
#include "mkh_storage.h"
#include "mkh_touch.h"

//
// Display (Waveshare ESP32-S3-Touch-LCD-2, ST7789 over SPI)
// Stage 1: proof-of-life only.
//
#define TFT_RST 0
#define TFT_BL 1
#define TFT_MOSI 38
#define TFT_SCLK 39
#define TFT_DC 42
#define TFT_CS 45

static Arduino_DataBus* displayBus =
    new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, GFX_NOT_DEFINED);
// Native panel is 240x320 (portrait); rotation=1 renders it as 320x240 (landscape).
static Arduino_GFX* display = new Arduino_ST7789(displayBus, TFT_RST, 1, true, 240, 320);

//
// Dashboard (Stage 2)
//
static const char* DASH_TITLE = "MK1 TANK OS v0.3.0";

static const int16_t SCR_W = 320;
static const int16_t SCR_H = 240;
static const int16_t HEADER_H = 28;
static const int16_t ROW_H = 36;
static const int16_t ROW_GAP = 4;
static const int16_t ROW_MARGIN = 6;
static const int16_t ROW_START_Y = HEADER_H + ROW_GAP;

static const uint16_t COLOR_BG = RGB565_BLACK;
static const uint16_t COLOR_HEADER_BG = RGB565_NAVY;
static const uint16_t COLOR_CARD_BG = RGB565(30, 30, 30);
static const uint16_t COLOR_CARD_BORDER = RGB565(70, 70, 70);
static const uint16_t COLOR_LABEL = RGB565_WHITE;
static const uint16_t COLOR_RED = RGB565_RED;
static const uint16_t COLOR_YELLOW = RGB565_YELLOW;
static const uint16_t COLOR_GREEN = RGB565_GREEN;

// XWC = the Xbox Wireless Controller connection, tracked from the real
// Bluepad32 callbacks/data below (see onConnectedController / processControllers).
enum XwcState { XWC_DISCONNECTED, XWC_CONNECTING, XWC_ACTIVE };
static XwcState xwcState = XWC_DISCONNECTED;

// MKH rows = Mould King hub broadcast state, owned by mkh_broadcast.c
// (mkh_hub_broadcasting) - true for device slots being actively broadcast
// to. Row index (ROW_MKH0/1/2) is the internal, zero-based protocol
// device slot; the on-screen label is the hub's own 1-based numbering
// (mkh_device_app_number(), see mkh_ports.h) - populated at runtime by
// initMkhLabels() below since it isn't a compile-time constant.

enum DashRow { ROW_XWC = 0, ROW_MKH0, ROW_MKH1, ROW_MKH2, ROW_UPTIME, ROW_COUNT };
static char mkhLabel0[8];
static char mkhLabel1[8];
static char mkhLabel2[8];
static const char* ROW_LABELS[ROW_COUNT] = {"XWC", mkhLabel0, mkhLabel1, mkhLabel2, "UPTIME"};

static void initMkhLabels() {
    snprintf(mkhLabel0, sizeof(mkhLabel0), "MKH %d", mkh_device_app_number(0));
    snprintf(mkhLabel1, sizeof(mkhLabel1), "MKH %d", mkh_device_app_number(1));
    snprintf(mkhLabel2, sizeof(mkhLabel2), "MKH %d", mkh_device_app_number(2));
}

static uint16_t lastXwcColorDrawn = 0xFFFF;
static uint16_t lastHubColorDrawn[3] = {0xFFFF, 0xFFFF, 0xFFFF};
static uint32_t lastUptimeDrawnSec = 0xFFFFFFFF;

static int16_t rowTop(int idx) {
    return ROW_START_Y + idx * (ROW_H + ROW_GAP);
}

static int16_t textWidth(const char* s, uint8_t size) {
    return (int16_t)strlen(s) * 6 * size;
}

static void drawHeader() {
    display->fillRect(0, 0, SCR_W, HEADER_H, COLOR_HEADER_BG);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    int16_t x = (SCR_W - textWidth(DASH_TITLE, 2)) / 2;
    display->setCursor(x, (HEADER_H - 16) / 2);
    display->print(DASH_TITLE);

    // v0.8.0 Step 1: config-source indicator. Minimal footprint - reuses
    // spare header space at size 1 rather than adding a dashboard row.
    // Config is read once at boot (no hot-reload), so this is drawn once
    // here and never needs to be refreshed like the status dots.
    const char* cfgLabel = mkh_config_source_is_file() ? "CFG:FS" : "CFG:DEF";
    display->setTextSize(1);
    int16_t cfgX = SCR_W - ROW_MARGIN - textWidth(cfgLabel, 1);
    display->setCursor(cfgX, (HEADER_H - 8) / 2);
    display->print(cfgLabel);
}

static void drawCardShell(int idx) {
    int16_t y = rowTop(idx);
    display->fillRoundRect(ROW_MARGIN, y, SCR_W - 2 * ROW_MARGIN, ROW_H, 6, COLOR_CARD_BG);
    display->drawRoundRect(ROW_MARGIN, y, SCR_W - 2 * ROW_MARGIN, ROW_H, 6, COLOR_CARD_BORDER);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor(ROW_MARGIN + 10, y + (ROW_H - 16) / 2);
    display->print(ROW_LABELS[idx]);
}

static void drawStatusDot(int idx, uint16_t color) {
    int16_t y = rowTop(idx);
    int16_t cx = SCR_W - ROW_MARGIN - 22;
    int16_t cy = y + ROW_H / 2;
    display->fillCircle(cx, cy, 10, color);
}

static uint16_t xwcColor() {
    switch (xwcState) {
        case XWC_ACTIVE:
            return COLOR_GREEN;
        case XWC_CONNECTING:
            return COLOR_YELLOW;
        default:
            return COLOR_RED;
    }
}

static void drawUptime(bool force) {
    uint32_t totalSec = millis() / 1000;
    if (!force && totalSec == lastUptimeDrawnSec)
        return;
    lastUptimeDrawnSec = totalSec;

    uint32_t h = totalSec / 3600;
    uint32_t m = (totalSec % 3600) / 60;
    uint32_t s = totalSec % 60;
    char buf[9];
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);

    int16_t y = rowTop(ROW_UPTIME);
    int16_t valW = textWidth("00:00:00", 2);
    int16_t valX = SCR_W - ROW_MARGIN - 10 - valW;
    display->fillRect(valX, y + 4, valW, ROW_H - 8, COLOR_CARD_BG);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor(valX, y + (ROW_H - 16) / 2);
    display->print(buf);
}

static void initDashboard() {
    initMkhLabels();

    display->fillScreen(COLOR_BG);
    drawHeader();

    for (int i = 0; i < ROW_COUNT; i++) {
        drawCardShell(i);
    }

    lastXwcColorDrawn = xwcColor();
    drawStatusDot(ROW_XWC, lastXwcColorDrawn);

    for (int i = 0; i < 3; i++) {
        lastHubColorDrawn[i] = mkh_hub_broadcasting[i] ? COLOR_GREEN : COLOR_RED;
        drawStatusDot(ROW_MKH0 + i, lastHubColorDrawn[i]);
    }

    drawUptime(true);
}

// Redraws only what changed since the last call - keeps status dots steady
// (no per-packet flicker) and avoids repainting the whole screen every loop.
static void updateDashboard() {
    uint16_t c = xwcColor();
    if (c != lastXwcColorDrawn) {
        lastXwcColorDrawn = c;
        drawStatusDot(ROW_XWC, c);
    }

    for (int i = 0; i < 3; i++) {
        uint16_t hc = mkh_hub_broadcasting[i] ? COLOR_GREEN : COLOR_RED;
        if (hc != lastHubColorDrawn[i]) {
            lastHubColorDrawn[i] = hc;
            drawStatusDot(ROW_MKH0 + i, hc);
        }
    }

    drawUptime(false);
}

static void initDisplay() {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    display->begin();
    initDashboard();
}

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

//
// MK6 stick mapping (v0.8.0 Step 2): XWC left stick Y = MKH_INPUT_LSNS,
// right stick Y = MKH_INPUT_RSNS. Which device/port each drives is no
// longer hardcoded here - it's resolved at runtime from the config
// table (mkh_config_get(), populated by mkh_storage_boot_read() at
// boot; see processGamepad() below and mkh_config.h).
//
// Bluepad32's axisY()/axisRY() range is approximately -511 (full up) ..
// 512 (full down), 0 = center - verified against Bluepad32's own
// docs/examples (https://bluepad32.readthedocs.io/,
// ricardoquesada/bluepad32-arduino examples/Controller/Controller.ino),
// not assumed: UP IS NEGATIVE. We want stick-up to mean "forward" (high
// output byte), so the mapping below inverts sign accordingly.
//
// Deadzone is a firmware constant, not a config setting - this is the one
// obvious place it lives.
#define MKH_STICK_AXIS_MAGNITUDE 512
#define MKH_STICK_DEADZONE_PERCENT 8

// Maps a raw axisY()/axisRY() reading to a MK6 telegram byte (0x00..0xFF,
// 0x80 = neutral). Continuous across the deadzone boundary (no output
// jump) and hits 0xFF/0x00 exactly at Bluepad32's documented axis extremes.
static uint8_t mkStickYToChannelByte(int32_t axisY) {
    const int32_t deadzone = (MKH_STICK_AXIS_MAGNITUDE * MKH_STICK_DEADZONE_PERCENT) / 100;

    if (axisY >= -deadzone && axisY <= deadzone) {
        return 0x80;
    }

    bool forward = axisY < 0;  // up (negative) -> forward
    int32_t magnitude = forward ? -axisY : axisY;
    int32_t travel = MKH_STICK_AXIS_MAGNITUDE - deadzone;

    float fraction = (float)(magnitude - deadzone) / (float)travel;
    if (fraction > 1.0f)
        fraction = 1.0f;

    // 0x80..0xFF is a 127-wide span, 0x80..0x00 is a 128-wide span - scale
    // each direction separately so the extremes land exactly on 0xFF/0x00.
    float scale = forward ? 127.0f : 128.0f;
    int value = 128 + (int)lroundf((forward ? 1.0f : -1.0f) * fraction * scale);
    if (value > 0xFF)
        value = 0xFF;
    if (value < 0x00)
        value = 0x00;
    return (uint8_t)value;
}

// Applies a config port's invert/max attributes to a raw MK6 channel
// byte (0x00..0xFF, 0x80=neutral). curve is always linear in v1 (see
// mkh_config.h), so there's no curve step here.
//
// invert mirrors the byte around neutral (0x80). The protocol's byte
// encoding has an inherent 127-wide forward span (0x80..0xFF) vs
// 128-wide reverse span (0x80..0x00) - see mkStickYToChannelByte above -
// so a byte-perfect mirror is impossible at the single extreme value
// 0xFF (mirrors to 0x01, not 0x00); the clamp below is that one-unit
// edge case, not a bug.
static uint8_t mkApplyPortConfig(uint8_t rawByte, const mkh_port_config_t* cfg) {
    int value = rawByte;
    if (cfg->invert) {
        value = 0x100 - value;
        if (value > 0xFF)
            value = 0xFF;
        if (value < 0x00)
            value = 0x00;
    }

    int delta = value - 0x80;
    delta = (delta * (int)cfg->max_percent) / 100;
    value = 0x80 + delta;
    if (value > 0xFF)
        value = 0xFF;
    if (value < 0x00)
        value = 0x00;
    return (uint8_t)value;
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            if (i == 0) {
                // Dashboard: XWC just paired, no input received yet.
                xwcState = XWC_CONNECTING;
            }
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            if (i == 0) {
                // Dashboard: XWC gone.
                xwcState = XWC_DISCONNECTED;

                // Failsafe (v0.8.0 Step 2): force every channel the
                // config table assigns to an XWC input source back to
                // neutral, so no hub keeps coasting on the last value it
                // heard. Table-driven, not hardcoded device indices, so
                // it stays correct regardless of which hub/port the
                // config maps LSNS/RSNS onto.
                for (int dev = 0; dev < MKH_MK6_NUM_DEVICES; dev++) {
                    for (int port = 0; port < MKH_MK6_NUM_CHANNELS; port++) {
                        const mkh_port_config_t* cfg = mkh_config_get(dev, port);
                        if (cfg && cfg->input != MKH_INPUT_NONE) {
                            mkh_set_channel(dev, port, 0x80);
                        }
                    }
                }
            }
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void dumpMouse(ControllerPtr ctl) {
    Console.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    static const char* key_names[] = {
        // clang-format off
        // To avoid having too much noise in this file, only a few keys are mapped to strings.
        // Starts with "A", which is offset 4.
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
        // Special keys
        "Enter", "Escape", "Backspace", "Tab", "Spacebar", "Underscore", "Equal", "OpenBracket", "CloseBracket",
        "Backslash", "Tilde", "SemiColon", "Quote", "GraveAccent", "Comma", "Dot", "Slash", "CapsLock",
        // Function keys
        "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
        // Cursors and others
        "PrintScreen", "ScrollLock", "Pause", "Insert", "Home", "PageUp", "Delete", "End", "PageDown",
        "RightArrow", "LeftArrow", "DownArrow", "UpArrow",
        // clang-format on
    };
    static const char* modifier_names[] = {
        // clang-format off
        // From 0xe0 to 0xe7
        "Left Control", "Left Shift", "Left Alt", "Left Meta",
        "Right Control", "Right Shift", "Right Alt", "Right Meta",
        // clang-format on
    };
    Console.printf("idx=%d, Pressed keys: ", ctl->index());
    for (int key = Keyboard_A; key <= Keyboard_UpArrow; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = key_names[key - 4];
            Console.printf("%s,", keyName);
        }
    }
    for (int key = Keyboard_LeftControl; key <= Keyboard_RightMeta; key++) {
        if (ctl->isKeyPressed(static_cast<KeyboardKey>(key))) {
            const char* keyName = modifier_names[key - 0xe0];
            Console.printf("%s,", keyName);
        }
    }
    Console.printf("\n");
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Console.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

void processGamepad(ControllerPtr ctl) {
    if (ctl->index() == 0) {
        // v0.8.0 Step 2: left stick Y = MKH_INPUT_LSNS, right stick Y =
        // MKH_INPUT_RSNS, applied to whichever device/port slot(s) the
        // config table currently assigns each input source to - see
        // mkh_config.h. Default config reproduces v0.7.0 exactly (HUB1
        // port A = LSNS, HUB2 port A = RSNS, invert=no, max=100).
        uint8_t leftRawByte = mkStickYToChannelByte(ctl->axisY());
        uint8_t rightRawByte = mkStickYToChannelByte(ctl->axisRY());

        for (int dev = 0; dev < MKH_MK6_NUM_DEVICES; dev++) {
            for (int port = 0; port < MKH_MK6_NUM_CHANNELS; port++) {
                const mkh_port_config_t* cfg = mkh_config_get(dev, port);
                if (!cfg)
                    continue;
                if (cfg->input == MKH_INPUT_LSNS) {
                    mkh_set_channel(dev, port, mkApplyPortConfig(leftRawByte, cfg));
                } else if (cfg->input == MKH_INPUT_RSNS) {
                    mkh_set_channel(dev, port, mkApplyPortConfig(rightRawByte, cfg));
                }
            }
        }
    }

    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);

    // See ArduinoController.h for all the available functions.
}

void processMouse(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->scrollWheel() > 0) {
        // Do Something
    } else if (ctl->scrollWheel() < 0) {
        // Do something else
    }

    // See "dumpMouse" for possible things to query.
    dumpMouse(ctl);
}

void processKeyboard(ControllerPtr ctl) {
    if (!ctl->isAnyKeyPressed())
        return;

    // This is just an example.
    if (ctl->isKeyPressed(Keyboard_A)) {
        // Do Something
        Console.println("Key 'A' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        // Do something else
        Console.println("Key 'LEFT SHIFT' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        // Do something else
        Console.println("Key 'Left Arrow' pressed");
    }

    // See "dumpKeyboard" for possible things to query.
    dumpKeyboard(ctl);
}

void processBalanceBoard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->topLeft() > 10000) {
        // Do Something
    }

    // See "dumpBalanceBoard" for possible things to query.
    dumpBalanceBoard(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->index() == 0) {
                // Dashboard: XWC is streaming input.
                xwcState = XWC_ACTIVE;
            }
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
                Console.printf("Unsupported controller\n");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    // v0.8.0 Step 0b: LittleFS storage foundation. Internal flash has no
    // shared-bus constraint with the display (unlike the retired SD
    // path - see mkh_sdcard.cpp), but this stays the first line of
    // setup() to preserve the single-entry-point boot discipline.
    mkh_storage_boot_read();

    // v0.9.0 Step 0: CST816D touch bring-up. Independent I2C bus (SDA=48,
    // SCL=47) - no contention with the display's SPI bus, so ordering
    // relative to initDisplay() below doesn't matter for bus-sharing
    // reasons; placed here to group peripheral bring-up together.
    mkh_touch_init();

    initDisplay();

    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks, and the default behavior for scanning or not.
    // By default, if the "startScanning" parameter is not passed, it will do the "start scanning".
    // Notice that "Start scanning" will try to auto-connect to devices that are compatible with Bluepad32.
    // E.g: if a Gamepad, keyboard or mouse are detected, it will try to auto connect to them.
    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

    // Notice that scanning can be stopped / started at any time by calling:
    // BP32.enableNewBluetoothConnections(enabled);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // v0.9.0 Step 0: touch bring-up proof only - logs events, no
    // dashboard interaction yet (that's Step 1, on explicit PM go).
    mkh_touch_poll();

    updateDashboard();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}
