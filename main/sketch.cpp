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

// WO11: single compile-time firmware version string, used on both the
// boot splash and the stats page (per the order - the dashboard's own
// DASH_TITLE below is a separate, pre-existing string, out of WO11's
// scope to touch).
#define MK1_FW_VERSION "v0.11.0"

// WO11: telegram service rate label for the stats page. Mirrors
// mkh_broadcast.c's MKH_CONTROL_REPEAT_MS (100ms slice re-arm) and
// MKH_TIME_SLICE_NUM_DEVICES (2 active devices) - restated here rather
// than derived, since those are private constants inside a file WO11
// requires zero changes to (broadcast/slicer). This is the *configured*
// rate (100ms * 2 devices = 200ms/hub), not a live measurement - keep
// this string in sync by hand if either constant changes.
static const char* MK1_TELEGRAM_RATE_LABEL = "100ms slice / 200ms per hub";

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
static uint32_t lastUptimeDrawnSec = 0xFFFFFFFF;

// WO10 Step 0: page-switching skeleton. currentPage gates both what
// loop() renders and how it dispatches touch - see goToPage() and
// loop() below. Nothing in the broadcast/slicer/failsafe/XWC path
// reads this: those aren't driven from loop() at all (mkh_broadcast's
// service timer is independent, and BP32.update()/processControllers()
// below run unconditionally every loop() iteration regardless of page),
// so page state is structurally invisible to them, not just
// by-convention invisible.
//
// WO11: added PAGE_SPLASH/PAGE_STATS ahead of PAGE_DASHBOARD in the
// enum, and boot now starts on PAGE_SPLASH instead of landing directly
// on PAGE_DASHBOARD - see setup() below. There is no path back to
// PAGE_SPLASH/PAGE_STATS once left (not in WO11 scope).
//
// WO10 Step 1: PAGE_EDIT_SELECT is repurposed from "generic corridor
// placeholder" to hub select (custom render/dispatch - see
// drawHubSelect()/dispatchHubSelectTouch() below); PAGE_EDIT_PORTS is
// new (port select - drawPortSelect()/dispatchPortSelectTouch()).
// PAGE_EDIT_CAPTURE/PAGE_EDIT_SETTINGS keep the WO10 Step 0 generic
// placeholder mechanism (drawEditorPlaceholder()/dispatchEditorTouch()),
// unchanged except CAPTURE's title now renders the editing context's
// selection instead of a static string.
enum UiPage {
    PAGE_SPLASH = 0,
    PAGE_STATS,
    PAGE_DASHBOARD,
    PAGE_EDIT_SELECT,
    PAGE_EDIT_PORTS,
    PAGE_EDIT_CAPTURE,
    PAGE_EDIT_SETTINGS,
    PAGE_IDLE,  // WO11 Task 3
    PAGE_COUNT
};
static UiPage currentPage = PAGE_SPLASH;

// WO11 Task 3: idle-screen entry timer. Updated whenever loop() sees a
// fresh accepted tap (any page), and on every dashboard (re-)entry -
// see initDashboard() and loop() below. Idle only triggers FROM
// PAGE_DASHBOARD (CC's judgment: never hijack an in-progress editor
// session just because the user paused touching the screen).
static uint32_t lastTouchActivityMs = 0;
static const uint32_t IDLE_TIMEOUT_MS = 30000;

// WO10 Step 1: editing context - selection state only (no pending
// values yet - that's a later step). selected_valid is the "nothing
// selected" sentinel: false until a port tap on PAGE_EDIT_PORTS
// completes a full hub+port selection. Cleared on every fresh editor
// entry from the dashboard (see the settings-button tap handler in
// loop()) and on Back from port select to hub select (see
// dispatchPortSelectTouch() - CC's choice: Back clears rather than
// persists, so hub select is always a clean slate on (re-)entry, never
// partially-stale).
//
// WO10 Step 2: capture_input/capture_valid added. capture_valid tracks
// its own "nothing captured" sentinel, independent of selected_valid -
// selected_valid's meaning (device+port confirmed) is unchanged. Final
// context invariant (resolved per the order - Step 1's port-select Back
// already matched what Step 2 needs, no reconciliation required): Back
// from port select clears the FULL context (device+port+capture); Back
// from the capture page never clears device/port - only DISCARD (from
// the leave-page prompt) clears capture, KEEP clears nothing.
struct EditContext {
    int selected_device;                // protocol device slot 0..2, meaningful only once selected_valid or mid-selection
    int selected_port;                  // channel index 0..5 (MKH_PORT_A..F), meaningful only once selected_valid
    bool selected_valid;                // true only once BOTH device and port are confirmed
    mkh_input_source_t captured_input;  // meaningful only once capture_valid
    bool capture_valid;                 // "nothing captured yet" sentinel, tracked separately from selected_valid
};
static EditContext editCtx = {-1, -1, false, MKH_INPUT_NONE, false};

// WO10-FINAL: pending session - accumulates committed per-port edits
// ACROSS ports (order requirement), independent of editCtx's single-
// port focus. A port becomes dirty=true the moment settings is reached
// for it (capturing the input is an implicit keep, same precedent as
// Step 2's "NEXT never prompts") - see goToPage()'s PAGE_EDIT_SETTINGS
// case. Never touched by mkh_config_get()/the live table directly;
// only pushPendingSessionToLiveTable() (Test-ON, Save) writes it out.
struct PendingPortEdit {
    bool dirty;
    mkh_input_source_t input;
    bool invert;
    uint8_t max_percent;
    mkh_input_mode_t mode;
};
static PendingPortEdit pendingEdits[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS];

// WO10-FINAL: Test toggle, visible on every editor page (hub/port/
// capture/settings - not the dashboard, which has its own unrelated
// broadcast toggles). ON pushes the current pending session to the live
// mapping table once (a one-shot forward push on the OFF->ON edge, not
// a continuous sync - see handleTestToggleTap() below); OFF is inert,
// per the order ("toggling OFF concludes the test, values stay live").
static bool testActive = false;

static bool sessionHasPendingEdits() {
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++)
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++)
            if (pendingEdits[d][p].dirty)
                return true;
    return false;
}

static void clearPendingSession() {
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++)
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++)
            pendingEdits[d][p].dirty = false;
}

// WO11 Task 4 (hardening): forward-declared so pushPendingSessionToLiveTable()
// below can clear a port's runtime latch state at push time - defined with
// the rest of the latch-state machinery further down (see its doc comment
// there for why).
static void clearLatchStateForPort(int dev, int port);

// Writes every dirty pending edit into the live mkh_config table - the
// one place Test-ON and Save actually touch the live table (through
// mkh_config_set_port(), never the protocol/broadcast layer directly).
static void pushPendingSessionToLiveTable() {
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
            if (pendingEdits[d][p].dirty) {
                mkh_config_set_port(d, p, pendingEdits[d][p].input, pendingEdits[d][p].invert,
                                     pendingEdits[d][p].max_percent, pendingEdits[d][p].mode);
                // WO11 Task 4: fresh unlatched start on every push - see
                // clearLatchStateForPort()'s doc comment for the stale-
                // latch-carryover bug this closes.
                clearLatchStateForPort(d, p);
            }
        }
    }
}

// WO10 Step 1/2 behavior, UNCHANGED by WO10-FINAL: clears only the
// current single-port SELECTION (device/port/capture) - still exactly
// what port-select's Back uses (see dispatchPortSelectTouch() below).
// Deliberately does NOT touch the pending session below - "accumulate
// across ports" requires that navigating hub<->port to focus a
// DIFFERENT port must not discard edits already committed for others.
static void clearEditContext() {
    editCtx.selected_device = -1;
    editCtx.selected_port = -1;
    editCtx.selected_valid = false;
    editCtx.captured_input = MKH_INPUT_NONE;
    editCtx.capture_valid = false;
}

// WO10-FINAL: clears the WHOLE editing session - selection AND the
// accumulated pending edits AND Test state. Used only where the order
// actually calls for the session to end: a fresh entry from the
// dashboard, the exit-editor prompt's DISCARD, and Reset ("restores
// saved config to the table and clears the session").
static void clearEditorSession() {
    clearEditContext();
    clearPendingSession();
    testActive = false;
}

// Forward-declared so performSaveAndExit() right below, and the hub/
// port select dispatch functions further down, can call it before
// goToPage() itself is actually defined - goToPage()'s own definition
// and doc comment are unchanged, just declared early.
static void goToPage(UiPage page);

// WO10-FINAL: shared by settings' Save button and the exit-editor
// prompt's SAVE option. The caller is responsible for committing any of
// ITS OWN in-page scratch into pendingEdits[][] first (this only pushes
// what's already committed there) - keeps this one function generic
// across both callers instead of two near-duplicates.
static bool performSaveAndExit() {
    pushPendingSessionToLiveTable();
    bool ok = mkh_storage_save_config();
    Console.printf("MK1 Editor: save %s\n", ok ? "succeeded" : "FAILED");
    clearEditorSession();
    goToPage(PAGE_DASHBOARD);
    return ok;
}

static int16_t rowTop(int idx) {
    return ROW_START_Y + idx * (ROW_H + ROW_GAP);
}

static int16_t textWidth(const char* s, uint8_t size) {
    return (int16_t)strlen(s) * 6 * size;
}

// v0.8.0 Step 1: config-source indicator, three-way (v0.9.0 Step 1
// honesty tweak). CFG:DEF = fell back to compiled-in defaults (no file
// / open failed). CFG:FS = file parsed with zero skipped lines. CFG:FS!
// = file parsed but >=1 line was rejected as malformed (see
// mkh_config_get_skipped_line_count() and the per-line "skipped"
// warnings in the boot log for which).
//
// WO11: factored out of drawHeader() so the stats page's CONFIG line
// can show the exact same value the dashboard's CFG field shows,
// guaranteed - not a second, potentially-drifting copy of this logic.
static const char* mkhCfgLabel() {
    if (!mkh_config_source_is_file()) {
        return "CFG:DEF";
    } else if (mkh_config_get_skipped_line_count() > 0) {
        return "CFG:FS!";
    } else {
        return "CFG:FS";
    }
}

static void drawHeader() {
    display->fillRect(0, 0, SCR_W, HEADER_H, COLOR_HEADER_BG);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    int16_t x = (SCR_W - textWidth(DASH_TITLE, 2)) / 2;
    display->setCursor(x, (HEADER_H - 16) / 2);
    display->print(DASH_TITLE);

    // Config is read once at boot (no hot-reload), so this is drawn once
    // here and never needs to be refreshed like the status dots.
    const char* cfgLabel = mkhCfgLabel();
    Console.printf("MK1 Dashboard: CFG field = %s (skipped_lines=%d)\n", cfgLabel,
                    mkh_config_get_skipped_line_count());
    display->setTextSize(1);
    int16_t cfgX = SCR_W - ROW_MARGIN - textWidth(cfgLabel, 1);
    display->setCursor(cfgX, (HEADER_H - 8) / 2);
    display->print(cfgLabel);
}

// WO10 Step 0: settings button, header corner. Carves a small hit zone
// out of the WO9 Amendment A full-screen MKH split (see
// MKH_ZONE_1_2_BOUNDARY_Y below) rather than shrinking or moving any
// MKH row boundary - loop() checks this rect FIRST, before falling
// through to the unchanged touchYToDeviceId() split. Placed left of the
// (centered) title, clear of both the title text and the right-aligned
// CFG label.
static const int16_t SETTINGS_BTN_X = 2;
static const int16_t SETTINGS_BTN_Y = 2;
static const int16_t SETTINGS_BTN_W = 30;
static const int16_t SETTINGS_BTN_H = HEADER_H - 4;  // 24 - visual size, UNCHANGED

// WO10 CLOSEOUT: PM-reported defect - the 30x24 visual rect was also
// the hit zone, and taps near the corner were landing just outside it,
// falling through to "nearest MKH row" (MKH1) instead. Fix: a SEPARATE,
// larger hit zone (order: "hit zone may exceed the visual") - the drawn
// button stays exactly as it was (SETTINGS_BTN_X/Y/W/H, above,
// untouched), only the invisible tappable area grows. No collision with
// the title or CFG label to solve for, because nothing visual moved -
// investigated "uptime" per the order's suggestion, but uptime lives on
// a dashboard ROW (ROW_UPTIME), not the header at all; the header only
// ever held the title + CFG label + this button, and neither needed to
// change for a hit-zone-only fix. Deliberately anchored at (0,0) rather
// than mirroring SETTINGS_BTN_X/Y - a hit zone is free to start right at
// the panel edge even though the visual button doesn't.
static const int16_t SETTINGS_HIT_X = 0;
static const int16_t SETTINGS_HIT_Y = 0;
static const int16_t SETTINGS_HIT_W = 54;  // >= the ordered 48px minimum
static const int16_t SETTINGS_HIT_H = 40;  // >= the ordered 36px minimum

static void drawSettingsButton() {
    display->fillRoundRect(SETTINGS_BTN_X, SETTINGS_BTN_Y, SETTINGS_BTN_W, SETTINGS_BTN_H, 4, COLOR_CARD_BG);
    display->drawRoundRect(SETTINGS_BTN_X, SETTINGS_BTN_Y, SETTINGS_BTN_W, SETTINGS_BTN_H, 4, COLOR_CARD_BORDER);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    int16_t tx = SETTINGS_BTN_X + (SETTINGS_BTN_W - textWidth("SET", 1)) / 2;
    display->setCursor(tx, SETTINGS_BTN_Y + (SETTINGS_BTN_H - 8) / 2);
    display->print("SET");
}

// WO10-FINAL: Test toggle, visible on every editor page (hub/port/
// capture/settings). Same small-corner-widget class as the settings
// button above (30x24, not this order's 40px/20px-margin rule for
// PRIMARY interactive targets - this is an auxiliary control, same
// precedent as SET) - top-right corner, mirroring SET's top-left.
static const int16_t TEST_TOGGLE_X = SCR_W - 32;  // 288
static const int16_t TEST_TOGGLE_Y = 2;
static const int16_t TEST_TOGGLE_W = 30;
static const int16_t TEST_TOGGLE_H = HEADER_H - 4;  // 24

static void drawTestToggle() {
    uint16_t borderColor = testActive ? COLOR_GREEN : COLOR_CARD_BORDER;
    display->fillRoundRect(TEST_TOGGLE_X, TEST_TOGGLE_Y, TEST_TOGGLE_W, TEST_TOGGLE_H, 4, COLOR_CARD_BG);
    display->drawRoundRect(TEST_TOGGLE_X, TEST_TOGGLE_Y, TEST_TOGGLE_W, TEST_TOGGLE_H, 4, borderColor);
    display->setTextColor(testActive ? COLOR_GREEN : COLOR_LABEL);
    display->setTextSize(1);
    const char* label = "TEST";
    int16_t tx = TEST_TOGGLE_X + (TEST_TOGGLE_W - textWidth(label, 1)) / 2;
    display->setCursor(tx, TEST_TOGGLE_Y + (TEST_TOGGLE_H - 8) / 2);
    display->print(label);
}

static bool testToggleHit(int16_t touchX, int16_t touchY) {
    return touchX >= TEST_TOGGLE_X && touchX < TEST_TOGGLE_X + TEST_TOGGLE_W && touchY >= TEST_TOGGLE_Y &&
           touchY < TEST_TOGGLE_Y + TEST_TOGGLE_H;
}

// ON pushes the pending session once (a one-shot forward push, not a
// continuous sync - see the order's "forward-push only"). OFF is
// inert - "toggling OFF concludes the test, values stay live" means
// exactly that: nothing is reverted.
static void handleTestToggleTap() {
    testActive = !testActive;
    if (testActive) {
        Console.printf("MK1 Editor: TEST ON - pushing pending session (%s) to live table\n",
                        sessionHasPendingEdits() ? "non-empty" : "empty");
        pushPendingSessionToLiveTable();
    } else {
        Console.printf("MK1 Editor: TEST OFF - concluded, live values unchanged\n");
    }
    drawTestToggle();
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

// v0.9.0 Step 1, Amendment B: MKH rows get a toggle SWITCH widget
// (pill track + sliding knob) in place of the status dot, right-aligned
// in the same footprint the dot used to occupy. XWC's dot (above) and
// its red/yellow/green tricolor semantics are UNTOUCHED - switches are
// MKH rows only.
//
// Geometry: track 36x16px, pill-rounded (radius = height/2 = 8); knob
// radius 6px, 2px inset from each track end. Right-aligned to the same
// SCR_W-ROW_MARGIN edge the dot used.
//   track: x=[278, 314], y = rowTop(idx)+10 .. +26
//   knob center y = rowTop(idx)+18 (vertically centered in track)
//   knob center x: OFF (left) = 286, ON (right) = 306 (20px travel)
static const int16_t SWITCH_TRACK_W = 36;
static const int16_t SWITCH_TRACK_H = 16;
static const int16_t SWITCH_KNOB_R = 6;
static const int16_t SWITCH_PAD = 2;
static const int16_t SWITCH_TRACK_RIGHT = SCR_W - ROW_MARGIN;                    // 314
static const int16_t SWITCH_TRACK_LEFT = SWITCH_TRACK_RIGHT - SWITCH_TRACK_W;    // 278
static const int16_t SWITCH_KNOB_CX_OFF = SWITCH_TRACK_LEFT + SWITCH_KNOB_R + SWITCH_PAD;   // 286
static const int16_t SWITCH_KNOB_CX_ON = SWITCH_TRACK_RIGHT - SWITCH_KNOB_R - SWITCH_PAD;   // 306

static void drawToggleSwitch(int idx, int16_t knobCx, uint16_t trackColor) {
    int16_t y = rowTop(idx);
    int16_t trackTop = y + (ROW_H - SWITCH_TRACK_H) / 2;
    int16_t knobCy = trackTop + SWITCH_TRACK_H / 2;

    // Erase the whole widget footprint (track + knob's extra radius)
    // back to the card background before redrawing - same fixed-
    // position-redraw reasoning as drawStatusDot(), just a wider rect
    // since track+knob together exceed a single circle's bounds.
    display->fillRect(SWITCH_TRACK_LEFT - SWITCH_KNOB_R, trackTop - SWITCH_PAD,
                       SWITCH_TRACK_W + 2 * SWITCH_KNOB_R + 2 * SWITCH_PAD, SWITCH_TRACK_H + 2 * SWITCH_PAD,
                       COLOR_CARD_BG);

    display->fillRoundRect(SWITCH_TRACK_LEFT, trackTop, SWITCH_TRACK_W, SWITCH_TRACK_H, SWITCH_TRACK_H / 2,
                            trackColor);
    display->fillCircle(knobCx, knobCy, SWITCH_KNOB_R, COLOR_LABEL);
}

// Slides the knob to its new position over a few frames - presentation
// only. The caller (loop(), below) has ALWAYS already dispatched the
// real toggle sequencing (mkh_broadcast_toggle_off/_on) before calling
// this - drawing never gates or delays it, only follows it.
//
// Commanded vs. observed (WO9 PM decision): this switch shows COMMANDED
// state and animates the instant a tap is accepted, before the neutral-
// then-drop/CONNECT-replay sequence underneath finishes - unlike the
// XWC dot above (xwcColor()/drawStatusDot()), which shows OBSERVED
// state and only ever updates when Bluepad32's own connection state
// actually changes. Both are correct for their widget: XWC's state
// isn't something this firmware commands, so there's nothing to show
// but what's true; the MKH switch's whole point is to tell the user
// their tap landed, immediately, while WO9's safety-ordered sequencing
// still runs to completion regardless of what's drawn.
static void animateToggleSwitch(int idx, bool turningOn) {
    const int FRAMES = 4;
    const int FRAME_DELAY_MS = 40;  // 4 * 40ms = 160ms, inside the 100-200ms target
    int16_t xStart = turningOn ? SWITCH_KNOB_CX_OFF : SWITCH_KNOB_CX_ON;
    int16_t xEnd = turningOn ? SWITCH_KNOB_CX_ON : SWITCH_KNOB_CX_OFF;
    uint16_t trackColor = turningOn ? COLOR_GREEN : COLOR_RED;
    for (int f = 1; f <= FRAMES; f++) {
        int16_t cx = xStart + (int16_t)(((int32_t)(xEnd - xStart) * f) / FRAMES);
        drawToggleSwitch(idx, cx, trackColor);
        delay(FRAME_DELAY_MS);

        // PM bench finding: without this, a real release (or an entire
        // press+release) landing inside this function's ~160ms of
        // blocking delay is invisible to mkh_touch_poll() - it's
        // otherwise only called once per loop() iteration, at the top -
        // which permanently desyncs its press/release edge-detector and
        // silently swallows every tap after the first. Discard the
        // coordinates - this call exists purely to keep that state
        // machine in sync during the blocking window, not to act on a
        // tap mid-animation.
        int16_t ignoredX, ignoredY;
        mkh_touch_poll(&ignoredX, &ignoredY);
    }
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

// WO11: factored out so the stats page's UPTIME line can reuse the
// exact same HH:MM:SS formatting as the dashboard's uptime row.
static void formatUptime(char* buf, size_t bufSize) {
    uint32_t totalSec = millis() / 1000;
    uint32_t h = totalSec / 3600;
    uint32_t m = (totalSec % 3600) / 60;
    uint32_t s = totalSec % 60;
    snprintf(buf, bufSize, "%02u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);
}

static void drawUptime(bool force) {
    uint32_t totalSec = millis() / 1000;
    if (!force && totalSec == lastUptimeDrawnSec)
        return;
    lastUptimeDrawnSec = totalSec;

    char buf[9];
    formatUptime(buf, sizeof(buf));

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
    lastTouchActivityMs = millis();  // WO11 Task 3: every dashboard entry is a fresh idle countdown

    display->fillScreen(COLOR_BG);
    drawHeader();
    drawSettingsButton();

    for (int i = 0; i < ROW_COUNT; i++) {
        drawCardShell(i);
    }

    lastXwcColorDrawn = xwcColor();
    drawStatusDot(ROW_XWC, lastXwcColorDrawn);

    // At boot, commanded and observed state are trivially identical
    // (nothing has been tapped yet) - see the commanded-vs-observed
    // note on animateToggleSwitch() below.
    for (int i = 0; i < 3; i++) {
        uint16_t hc = mkh_hub_broadcasting[i] ? COLOR_GREEN : COLOR_RED;
        int16_t knobCx = mkh_hub_broadcasting[i] ? SWITCH_KNOB_CX_ON : SWITCH_KNOB_CX_OFF;
        drawToggleSwitch(ROW_MKH0 + i, knobCx, hc);
    }

    drawUptime(true);
}

// Redraws only what changed since the last call. XWC's dot is the only
// thing polled/diffed here - it shows OBSERVED state (there is no user
// command for "connected", only Bluepad32's own connection events), so
// polling is the only way it can ever update. MKH switches show
// COMMANDED state instead (WO9 PM decision) and are drawn exclusively
// by loop()'s tap handler via animateToggleSwitch(), immediately on tap
// acceptance - never here, and never polled against mkh_hub_broadcasting[]
// - so a switch's own animation is never second-guessed or overwritten
// by this function once the sequence it's already displaying catches up
// in the background.
static void updateDashboard() {
    uint16_t c = xwcColor();
    if (c != lastXwcColorDrawn) {
        lastXwcColorDrawn = c;
        drawStatusDot(ROW_XWC, c);
    }

    drawUptime(false);
}

// WO10 Step 0: editor page placeholder - PAGE_EDIT_SETTINGS only as of
// WO10 Step 2 (SELECT was replaced by hub select in Step 1, PORTS never
// used this pattern, CAPTURE was replaced by the real capture page in
// Step 2 - see drawCapturePage()/dispatchCaptureTouch() above). Page
// name + Back only now (no Next - see pageHasNext() below). Still
// replaced wholesale when Settings gets built for real.
static const int16_t EDITOR_BTN_Y = 190;
static const int16_t EDITOR_BTN_H = 40;
static const int16_t EDITOR_BACK_X = 10;
static const int16_t EDITOR_BACK_W = 140;
static const int16_t EDITOR_NEXT_X = 170;
static const int16_t EDITOR_NEXT_W = 140;

// WO10 Step 1: generalized out of what used to be drawEditorButton()'s
// whole body (hardcoded to EDITOR_BTN_Y/EDITOR_BTN_H) so the new hub/
// port select pages' Back button - deliberately different geometry,
// see SELECT_BACK_* below - can share the same drawing code instead of
// a near-duplicate. drawEditorButton() is now a thin wrapper preserving
// its exact prior behavior for PAGE_EDIT_SETTINGS's Back button (its
// only remaining caller as of WO10 Step 2).
static void drawButtonAt(int16_t x, int16_t y, int16_t w, int16_t h, const char* label) {
    display->fillRoundRect(x, y, w, h, 6, COLOR_CARD_BG);
    display->drawRoundRect(x, y, w, h, 6, COLOR_CARD_BORDER);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    int16_t tx = x + (w - textWidth(label, 2)) / 2;
    display->setCursor(tx, y + (h - 16) / 2);
    display->print(label);
}

static void drawEditorButton(int16_t x, int16_t w, const char* label) {
    drawButtonAt(x, EDITOR_BTN_Y, w, EDITOR_BTN_H, label);
}

// WO10 Step 1: PAGE_EDIT_SELECT no longer uses this generic
// placeholder/dispatch pair at all (see drawHubSelect()/
// dispatchHubSelectTouch() below).
// WO10 Step 2: PAGE_EDIT_CAPTURE moved off this generic pattern too
// (see dispatchCaptureTouch()/captureHasNext() instead) - no page on
// the remaining generic corridor (PAGE_EDIT_SETTINGS only) has a Next.
static bool pageHasNext(UiPage page) {
    (void)page;
    return false;
}

static void drawEditorPlaceholder(const char* pageName, UiPage page) {
    display->fillScreen(COLOR_BG);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    int16_t x = (SCR_W - textWidth(pageName, 2)) / 2;
    display->setCursor(x, 40);
    display->print(pageName);

    drawEditorButton(EDITOR_BACK_X, EDITOR_BACK_W, "< BACK");
    if (pageHasNext(page)) {
        drawEditorButton(EDITOR_NEXT_X, EDITOR_NEXT_W, "NEXT >");
    }
}

// WO11: boot splash. Static, primitive-only side-profile tank
// silhouette (hull, turret, barrel, tracks+wheels) - no bitmap assets,
// no animation, no delay call anywhere in this function. Boot-time
// neutrality holds by construction: setup() (below) calls this once and
// proceeds immediately to the rest of boot work - this function's own
// runtime is the only cost, and it's a handful of primitive draws, not
// a new phase of boot.
static const uint16_t COLOR_TANK = RGB565(90, 110, 70);
static const uint16_t COLOR_WHEEL = COLOR_CARD_BORDER;

static void drawSplash() {
    display->fillScreen(COLOR_BG);

    const char* title = "MK1";
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(4);
    display->setCursor((SCR_W - textWidth(title, 4)) / 2, 36);
    display->print(title);

    // Side-profile silhouette, centered around x=160. All primitives.
    const int16_t hullX = 110, hullY = 150, hullW = 100, hullH = 30;
    const int16_t turretX = 140, turretY = 122, turretW = 45, turretH = 28;
    const int16_t barrelX = turretX + turretW, barrelY = turretY + turretH / 2 - 3, barrelW = 45, barrelH = 6;
    const int16_t trackX = 95, trackY = hullY + hullH, trackW = 130, trackH = 16;

    display->fillRoundRect(hullX, hullY, hullW, hullH, 4, COLOR_TANK);
    display->fillRoundRect(turretX, turretY, turretW, turretH, 4, COLOR_TANK);
    display->fillRect(barrelX, barrelY, barrelW, barrelH, COLOR_TANK);
    display->fillRoundRect(trackX, trackY, trackW, trackH, 4, COLOR_TANK);
    for (int i = 0; i < 5; i++) {
        int16_t wx = trackX + 14 + i * ((trackW - 28) / 4);
        display->fillCircle(wx, trackY + trackH / 2, 6, COLOR_WHEEL);
    }

    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    int16_t vx = (SCR_W - textWidth(MK1_FW_VERSION, 1)) / 2;
    display->setCursor(vx, trackY + trackH + 16);
    display->print(MK1_FW_VERSION);
}

// WO11 Task 3: idle screen. Display-layer only, by construction -
// entry/exit are driven from loop()'s existing millis()/xwcState reads
// (both already-read state, no new coupling to broadcast/slicer/
// failsafe/XWC processing/config), and the animation is a small
// periodically-redrawn region, not a full-screen redraw loop, so it
// can't meaningfully compete with the BLE stack for cycles. Reuses
// drawSplash()'s exact tank silhouette geometry/colors so idle and
// splash read as "the same tank" - static tank + a small pulsing
// "exhaust" dot near the rear suggesting an idling engine.
static const int16_t IDLE_PUFF_CX = 100;
static const int16_t IDLE_PUFF_CY = 140;
static const int16_t IDLE_PUFF_MAX_R = 10;
static const uint32_t IDLE_ANIM_INTERVAL_MS = 500;
static uint32_t idleAnimLastTickMs = 0;
static int idleAnimFrame = 0;

static void drawIdleTank() {
    const int16_t hullX = 110, hullY = 150, hullW = 100, hullH = 30;
    const int16_t turretX = 140, turretY = 122, turretW = 45, turretH = 28;
    const int16_t barrelX = turretX + turretW, barrelY = turretY + turretH / 2 - 3, barrelW = 45, barrelH = 6;
    const int16_t trackX = 95, trackY = hullY + hullH, trackW = 130, trackH = 16;

    display->fillRoundRect(hullX, hullY, hullW, hullH, 4, COLOR_TANK);
    display->fillRoundRect(turretX, turretY, turretW, turretH, 4, COLOR_TANK);
    display->fillRect(barrelX, barrelY, barrelW, barrelH, COLOR_TANK);
    display->fillRoundRect(trackX, trackY, trackW, trackH, 4, COLOR_TANK);
    for (int i = 0; i < 5; i++) {
        int16_t wx = trackX + 14 + i * ((trackW - 28) / 4);
        display->fillCircle(wx, trackY + trackH / 2, 6, COLOR_WHEEL);
    }
}

// One small localized redraw per animation tick - erase the puff's max
// footprint, draw this frame's dot. Never touches more than a ~22x22px
// region, regardless of how long idle holds.
static void drawIdlePuffFrame(int frame) {
    display->fillCircle(IDLE_PUFF_CX, IDLE_PUFF_CY, IDLE_PUFF_MAX_R + 1, COLOR_BG);
    int16_t r = 3 + (frame % 4) * 2;  // 3,5,7,9 cycling - a "puff" that grows then resets
    uint8_t shade = (uint8_t)(70 + frame * 15);
    display->fillCircle(IDLE_PUFF_CX, IDLE_PUFF_CY, r, RGB565(shade, shade, shade));
}

static void drawIdlePage() {
    display->fillScreen(COLOR_BG);
    const char* title = "IDLE";
    display->setTextColor(COLOR_CARD_BORDER);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(title, 2)) / 2, 20);
    display->print(title);
    drawIdleTank();
    idleAnimFrame = 0;
    idleAnimLastTickMs = millis();
    drawIdlePuffFrame(idleAnimFrame);
}

// Called from loop() only while currentPage == PAGE_IDLE. Cheap - a
// millis() comparison every tick, a small redraw only every
// IDLE_ANIM_INTERVAL_MS (500ms), matching the order's "lightweight,
// don't starve the BLE stack" constraint.
static void updateIdleAnimation() {
    uint32_t now = millis();
    if (now - idleAnimLastTickMs < IDLE_ANIM_INTERVAL_MS)
        return;
    idleAnimLastTickMs = now;
    idleAnimFrame = (idleAnimFrame + 1) % 4;
    drawIdlePuffFrame(idleAnimFrame);
}

// WO11: stats page. One line per item, values sampled at render time
// (see updateStatsUptime() below for the one exception - a cheap,
// once-per-second UPTIME-line-only refresh, piggybacked on loop()'s
// existing cadence, no new timer machinery). Holds until tapped -
// dispatch lives in loop(), not here.
enum StatsLine { STATS_LINE_FW = 0, STATS_LINE_UPTIME, STATS_LINE_CONFIG, STATS_LINE_RATE, STATS_LINE_XWC };
static const int16_t STATS_LINE_X = 20;
static const int16_t STATS_LINE_Y0 = 56;
static const int16_t STATS_LINE_GAP = 18;
static uint32_t lastStatsUptimeSec = 0xFFFFFFFF;

static int16_t statsLineY(int idx) {
    return STATS_LINE_Y0 + idx * STATS_LINE_GAP;
}

static void drawStatsLine(int idx, const char* text) {
    int16_t y = statsLineY(idx);
    display->fillRect(STATS_LINE_X, y, SCR_W - 2 * STATS_LINE_X, 8, COLOR_BG);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    display->setCursor(STATS_LINE_X, y);
    display->print(text);
}

static void drawStatsPage() {
    display->fillScreen(COLOR_BG);

    const char* title = "MK1 STATS";
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(title, 2)) / 2, 14);
    display->print(title);

    char line[48];

    snprintf(line, sizeof(line), "FW: %s", MK1_FW_VERSION);
    drawStatsLine(STATS_LINE_FW, line);

    char uptimeBuf[9];
    formatUptime(uptimeBuf, sizeof(uptimeBuf));
    snprintf(line, sizeof(line), "UPTIME: %s", uptimeBuf);
    drawStatsLine(STATS_LINE_UPTIME, line);
    lastStatsUptimeSec = millis() / 1000;

    snprintf(line, sizeof(line), "CONFIG: %s", mkhCfgLabel());
    drawStatsLine(STATS_LINE_CONFIG, line);

    snprintf(line, sizeof(line), "RATE: %s", MK1_TELEGRAM_RATE_LABEL);
    drawStatsLine(STATS_LINE_RATE, line);

    snprintf(line, sizeof(line), "XWC: %s", xwcState == XWC_ACTIVE ? "CONNECTED" : "NOT CONNECTED");
    drawStatsLine(STATS_LINE_XWC, line);

    const char* hint = "TAP ANYWHERE TO CONTINUE";
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    display->setCursor((SCR_W - textWidth(hint, 1)) / 2, 210);
    display->print(hint);
}

// Called from loop() only while currentPage == PAGE_STATS (see below).
// Cheap once-per-second diff-and-redraw of the UPTIME line only, same
// pattern as the dashboard's drawUptime(force=false) - not required by
// WO11 (static/sampled-at-render is acceptable), included because it's
// effectively free within the existing loop cadence and makes "uptime
// counting from boot" bench-observable while the page is held on screen.
static void updateStatsUptime() {
    uint32_t totalSec = millis() / 1000;
    if (totalSec == lastStatsUptimeSec)
        return;
    lastStatsUptimeSec = totalSec;

    char uptimeBuf[9];
    formatUptime(uptimeBuf, sizeof(uptimeBuf));
    char line[48];
    snprintf(line, sizeof(line), "UPTIME: %s", uptimeBuf);
    drawStatsLine(STATS_LINE_UPTIME, line);
}

// WO10 Step 1: hub select + port select. Both have custom geometry (not
// the generic EDITOR_BACK_*/EDITOR_BTN_Y placeholder pattern) because
// this order's edge-margin requirement (~20px minimum, calibration is
// center-weighted) is stricter than what EDITOR_BTN_Y's existing bottom
// margin gives (230 bottom, only 10px from the 240px screen edge) - so
// a new, separate Back button geometry is used here instead of reusing
// EDITOR_BACK_X/W/EDITOR_BTN_Y/H, even though it's visually similar.
static const int16_t SELECT_BACK_X = 90;
static const int16_t SELECT_BACK_W = 140;
static const int16_t SELECT_BACK_Y = 180;
static const int16_t SELECT_BACK_H = 40;  // bottom=220, 20px from screen edge (240)

// WO10-FINAL: exit-editor prompt (hub select's Back, when the pending
// session has anything dirty) - SAVE/DISCARD, same row SELECT_BACK_*
// occupies, same geometry rules (40px short dim, 20px margins: left
// button left edge 20, right button right edge 300 = 20px from 320).
static const int16_t EXIT_PROMPT_LEFT_X = 20;
static const int16_t EXIT_PROMPT_LEFT_W = 120;
static const int16_t EXIT_PROMPT_RIGHT_X = 180;
static const int16_t EXIT_PROMPT_RIGHT_W = 120;
static bool exitPromptActive = false;

// Mirrors mkh_broadcast.c's MKH_TIME_SLICE_NUM_DEVICES (2) - the third
// hub (device 2 / MKH3) is structurally parked project-wide (see
// mkh_broadcast_toggle_on()'s own device_id >= MKH_TIME_SLICE_NUM_
// DEVICES guard). Restated here, not derived, since that constant is
// private to a file this step must not change - same approach as
// WO11's MK1_TELEGRAM_RATE_LABEL.
static const int MKH_EDIT_MAX_ACTIVE_DEVICE = 2;  // device_id >= this is parked, not editable

static const int16_t HUB_BTN_X = 20;
static const int16_t HUB_BTN_W = 280;                 // right edge 300, 20px from screen edge (320)
static const int16_t HUB_BTN_Y0 = 30;                  // top edge 30, 30px from screen edge (0)
static const int16_t HUB_BTN_H = 40;                   // short dimension - exactly the 40px minimum
static const int16_t HUB_BTN_GAP = 10;

static int16_t hubBtnY(int idx) {
    return HUB_BTN_Y0 + idx * (HUB_BTN_H + HUB_BTN_GAP);
}

static void drawHubButton(int deviceIdx) {
    int16_t y = hubBtnY(deviceIdx);
    bool parked = deviceIdx >= MKH_EDIT_MAX_ACTIVE_DEVICE;
    uint16_t borderColor = parked ? COLOR_RED : COLOR_CARD_BORDER;

    display->fillRoundRect(HUB_BTN_X, y, HUB_BTN_W, HUB_BTN_H, 6, COLOR_CARD_BG);
    display->drawRoundRect(HUB_BTN_X, y, HUB_BTN_W, HUB_BTN_H, 6, borderColor);

    char label[16];
    snprintf(label, sizeof(label), "MKH%d", mkh_device_app_number(deviceIdx));
    display->setTextColor(parked ? COLOR_CARD_BORDER : COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor(HUB_BTN_X + 16, y + (HUB_BTN_H - 16) / 2);
    display->print(label);

    if (parked) {
        const char* tag = "PARKED";
        display->setTextColor(COLOR_CARD_BORDER);
        display->setTextSize(1);
        int16_t tx = HUB_BTN_X + HUB_BTN_W - 16 - textWidth(tag, 1);
        display->setCursor(tx, y + (HUB_BTN_H - 8) / 2);
        display->print(tag);
    }
}

static void drawHubSelect() {
    display->fillScreen(COLOR_BG);

    const char* title = "SELECT HUB";
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(title, 2)) / 2, 6);
    display->print(title);

    for (int i = 0; i < 3; i++) {
        drawHubButton(i);
    }

    exitPromptActive = false;
    drawButtonAt(SELECT_BACK_X, SELECT_BACK_Y, SELECT_BACK_W, SELECT_BACK_H, "< BACK");
    drawTestToggle();
}

static void drawExitPromptOverlay() {
    const char* q = "SAVE CHANGES?";
    display->fillRect(0, 140, SCR_W, 30, COLOR_BG);
    display->setTextColor(COLOR_YELLOW);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(q, 2)) / 2, 146);
    display->print(q);

    display->fillRect(0, SELECT_BACK_Y - 4, SCR_W, SELECT_BACK_H + 8, COLOR_BG);
    drawButtonAt(EXIT_PROMPT_LEFT_X, SELECT_BACK_Y, EXIT_PROMPT_LEFT_W, SELECT_BACK_H, "SAVE");
    drawButtonAt(EXIT_PROMPT_RIGHT_X, SELECT_BACK_Y, EXIT_PROMPT_RIGHT_W, SELECT_BACK_H, "DISCARD");
}

// Hub select's own hit zones - separate from dispatchEditorTouch()
// (never consulted on the same tap) and from the dashboard's
// touchYToDeviceId() split (different page, different geometry
// entirely - this is not a reuse of that logic).
static void dispatchHubSelectTouch(int16_t touchX, int16_t touchY) {
    if (exitPromptActive) {
        bool inSave = touchX >= EXIT_PROMPT_LEFT_X && touchX < EXIT_PROMPT_LEFT_X + EXIT_PROMPT_LEFT_W &&
                      touchY >= SELECT_BACK_Y && touchY < SELECT_BACK_Y + SELECT_BACK_H;
        bool inDiscard = touchX >= EXIT_PROMPT_RIGHT_X && touchX < EXIT_PROMPT_RIGHT_X + EXIT_PROMPT_RIGHT_W &&
                         touchY >= SELECT_BACK_Y && touchY < SELECT_BACK_Y + SELECT_BACK_H;
        if (inSave) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> exit-editor prompt SAVE\n", touchX, touchY);
            exitPromptActive = false;
            performSaveAndExit();
        } else if (inDiscard) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> exit-editor prompt DISCARD\n", touchX, touchY);
            exitPromptActive = false;
            clearEditorSession();
            goToPage(PAGE_DASHBOARD);
        }
        return;
    }

    if (testToggleHit(touchX, touchY)) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> Test toggle (hub select)\n", touchX, touchY);
        handleTestToggleTap();
        return;
    }

    bool inBack = touchX >= SELECT_BACK_X && touchX < SELECT_BACK_X + SELECT_BACK_W && touchY >= SELECT_BACK_Y &&
                  touchY < SELECT_BACK_Y + SELECT_BACK_H;
    if (inBack) {
        if (sessionHasPendingEdits()) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> hub select BACK, prompting (pending session)\n",
                            touchX, touchY);
            exitPromptActive = true;
            drawExitPromptOverlay();
        } else {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> hub select BACK -> dashboard\n", touchX, touchY);
            // No pending edits, but leaving the editor entirely either
            // way - full session end, not just a selection clear
            // (matters if e.g. Test was toggled ON with nothing edited).
            clearEditorSession();
            goToPage(PAGE_DASHBOARD);
        }
        return;
    }

    for (int i = 0; i < 3; i++) {
        int16_t y = hubBtnY(i);
        bool inHub = touchX >= HUB_BTN_X && touchX < HUB_BTN_X + HUB_BTN_W && touchY >= y && touchY < y + HUB_BTN_H;
        if (!inHub)
            continue;

        if (i >= MKH_EDIT_MAX_ACTIVE_DEVICE) {
            // Same "stays parked" response pattern as the dashboard's
            // toggle_on() guard, but this is selection-only - no
            // broadcast function is called, nothing airs differently.
            Console.printf(
                "MK1 Touch: tap display=(x=%d,y=%d) -> hub select MKH%d, ignored - third hub stays parked (not "
                "editable)\n",
                touchX, touchY, mkh_device_app_number(i));
        } else {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> hub select MKH%d -> port select\n", touchX,
                            touchY, mkh_device_app_number(i));
            editCtx.selected_device = i;
            editCtx.selected_port = -1;
            editCtx.selected_valid = false;  // device only so far - not a full selection yet
            goToPage(PAGE_EDIT_PORTS);
        }
        return;
    }
}

static const int16_t PORT_GRID_X0 = 20;                 // left edge 20, 20px from screen edge (0)
static const int16_t PORT_GRID_COL_W = 80;
static const int16_t PORT_GRID_COL_GAP = 20;             // 3 cols: 20+80+20+80+20+80+20=320, right edge 300
static const int16_t PORT_GRID_Y0 = 40;                  // top edge 40, 40px from screen edge (0)
static const int16_t PORT_GRID_ROW_H = 50;                // short dimension of each cell - above the 40px minimum
static const int16_t PORT_GRID_ROW_GAP = 20;

static int16_t portCellX(int col) {
    return PORT_GRID_X0 + col * (PORT_GRID_COL_W + PORT_GRID_COL_GAP);
}
static int16_t portCellY(int row) {
    return PORT_GRID_Y0 + row * (PORT_GRID_ROW_H + PORT_GRID_ROW_GAP);
}

static void drawPortCell(int port) {
    int col = port % 3;
    int row = port / 3;
    int16_t x = portCellX(col);
    int16_t y = portCellY(row);

    display->fillRoundRect(x, y, PORT_GRID_COL_W, PORT_GRID_ROW_H, 6, COLOR_CARD_BG);
    display->drawRoundRect(x, y, PORT_GRID_COL_W, PORT_GRID_ROW_H, 6, COLOR_CARD_BORDER);

    char label[2] = {mkh_port_letter(port), '\0'};
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(3);
    int16_t tx = x + (PORT_GRID_COL_W - textWidth(label, 3)) / 2;
    display->setCursor(tx, y + 8);
    display->print(label);

    // Optional per the order ("skip silently if not trivial" - this
    // was): informational-only LSNS/RSNS tag for ports the mapping
    // table already assigns. Read-only - no behavior difference, never
    // written to.
    const mkh_port_config_t* cfg = mkh_config_get(editCtx.selected_device, port);
    if (cfg && cfg->input != MKH_INPUT_NONE) {
        const char* tag = (cfg->input == MKH_INPUT_LSNS) ? "LSNS" : "RSNS";
        display->setTextColor(COLOR_LABEL);
        display->setTextSize(1);
        int16_t tagX = x + (PORT_GRID_COL_W - textWidth(tag, 1)) / 2;
        display->setCursor(tagX, y + PORT_GRID_ROW_H - 10);
        display->print(tag);
    }
}

static void drawPortSelect() {
    display->fillScreen(COLOR_BG);

    char header[32];
    snprintf(header, sizeof(header), "MKH%d - pick a port", mkh_device_app_number(editCtx.selected_device));
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(header, 2)) / 2, 8);
    display->print(header);

    for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
        drawPortCell(p);
    }

    drawButtonAt(SELECT_BACK_X, SELECT_BACK_Y, SELECT_BACK_W, SELECT_BACK_H, "< BACK");
    drawTestToggle();
}

// Port select's own hit zones - same separation rationale as
// dispatchHubSelectTouch() above.
static void dispatchPortSelectTouch(int16_t touchX, int16_t touchY) {
    if (testToggleHit(touchX, touchY)) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> Test toggle (port select)\n", touchX, touchY);
        handleTestToggleTap();
        return;
    }

    bool inBack = touchX >= SELECT_BACK_X && touchX < SELECT_BACK_X + SELECT_BACK_W && touchY >= SELECT_BACK_Y &&
                  touchY < SELECT_BACK_Y + SELECT_BACK_H;
    if (inBack) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> port select BACK -> hub select\n", touchX, touchY);
        // WO10 Step 1 design choice (reported per the order): clears
        // the current SELECTION (device/port/capture) rather than
        // persisting it - hub select is always a clean slate on
        // re-entry. WO10-FINAL: deliberately still selection-only, NOT
        // a full clearEditorSession() - the pending session (other
        // ports' committed edits) must survive navigating hub<->port to
        // focus a different port, or "accumulate across ports" breaks.
        clearEditContext();
        goToPage(PAGE_EDIT_SELECT);
        return;
    }

    for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
        int col = p % 3;
        int row = p / 3;
        int16_t x = portCellX(col);
        int16_t y = portCellY(row);
        if (touchX >= x && touchX < x + PORT_GRID_COL_W && touchY >= y && touchY < y + PORT_GRID_ROW_H) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> port select MKH%d port %c -> capture\n", touchX,
                            touchY, mkh_device_app_number(editCtx.selected_device), mkh_port_letter(p));
            // WO10 Step 2: a capture is tied to the specific port being
            // edited - re-tapping the SAME port (e.g. after KEEP
            // returned here) walks back into capture with it intact;
            // tapping a DIFFERENT port starts fresh, since the old
            // capture belonged to the port being left behind.
            bool samePort = editCtx.selected_valid && editCtx.selected_port == p;
            if (!samePort) {
                editCtx.captured_input = MKH_INPUT_NONE;
                editCtx.capture_valid = false;
            }
            editCtx.selected_port = p;
            editCtx.selected_valid = true;
            goToPage(PAGE_EDIT_CAPTURE);
            return;
        }
    }
}

// WO10 Step 2: capture page. Replaces the PAGE_EDIT_CAPTURE placeholder
// wholesale - custom render/dispatch, like hub/port select, not the
// generic drawEditorPlaceholder()/dispatchEditorTouch() pattern (see
// the removed PAGE_EDIT_CAPTURE cases in editorBackTarget()/
// editorNextTarget()/pageHasNext() below).
//
// WO10-FINAL rev B: token display now goes through the public
// mkh_config_input_token_name() accessor instead of a local mirror
// table (Step 2's mkhCaptureTokenLabel(), removed) - with 18 tokens
// instead of 2, keeping a second hand-synced copy in sketch.cpp stopped
// being worth it; one source of truth in mkh_config.c now.

// Rebuilt-page geometry (order: "the old 10px-margin placeholder
// buttons die here") - same >=40px short dimension / >=20px edge margin
// rules as WO10 Step 1's hub/port select, not a reuse of EDITOR_BACK_X/
// EDITOR_BTN_Y (still 10px bottom margin, still used by the
// PAGE_EDIT_SETTINGS placeholder, out of this step's scope). Two
// buttons side by side (Back + Next, or Keep + Discard while the leave-
// page prompt is up - same rects, different labels/meaning).
static const int16_t CAPTURE_BTN_Y = 180;
static const int16_t CAPTURE_BTN_H = 40;                 // short dimension - exactly the 40px minimum
static const int16_t CAPTURE_LEFT_X = 20;                 // left edge 20, 20px from screen edge (0)
static const int16_t CAPTURE_LEFT_W = 120;
static const int16_t CAPTURE_RIGHT_X = 180;
static const int16_t CAPTURE_RIGHT_W = 120;               // right edge 300, 20px from screen edge (320)
static const int16_t CAPTURE_STATUS_Y = 70;
static const int16_t CAPTURE_STATUS_H = 34;
static const int16_t CAPTURE_PROMPT_Y = 140;
static const int16_t CAPTURE_PROMPT_H = 30;

// WO10 Step 2: leave-page prompt as IN-PAGE STATE (not a new UiPage
// enum value) - reported per the order's "splice lesson" ask. A new
// PAGE_EDIT_PROMPT page would have needed a goToPage() branch, a
// loop() dispatch branch, and entries in editorBackTarget()/
// editorNextTarget()/pageHasNext() for something no OTHER page can
// navigate into or back out of - it's a sub-state of this one page,
// never a destination in its own right. A plain bool plus an overlay
// redraw keeps every call site that would otherwise need touching
// (goToPage()'s switch, loop()'s currentPage dispatch chain) exactly as
// simple as hub/port select's, at the cost of this page's own dispatch
// function needing an internal branch - contained entirely here.
static bool capturePromptActive = false;

static bool captureHasNext() {
    return editCtx.capture_valid;
}

static void drawCaptureButtons() {
    display->fillRect(0, CAPTURE_BTN_Y - 4, SCR_W, CAPTURE_BTN_H + 8, COLOR_BG);
    drawButtonAt(CAPTURE_LEFT_X, CAPTURE_BTN_Y, CAPTURE_LEFT_W, CAPTURE_BTN_H, "< BACK");
    if (captureHasNext()) {
        drawButtonAt(CAPTURE_RIGHT_X, CAPTURE_BTN_Y, CAPTURE_RIGHT_W, CAPTURE_BTN_H, "NEXT >");
    }
}

static void drawCaptureStatus() {
    display->fillRect(0, CAPTURE_STATUS_Y, SCR_W, CAPTURE_STATUS_H, COLOR_BG);
    display->setTextColor(COLOR_LABEL);
    if (editCtx.capture_valid) {
        char line[24];
        snprintf(line, sizeof(line), "CAPTURED: %s", mkh_config_input_token_name(editCtx.captured_input));
        display->setTextSize(3);
        display->setCursor((SCR_W - textWidth(line, 3)) / 2, CAPTURE_STATUS_Y);
        display->print(line);
    } else {
        const char* waiting = "WAITING FOR INPUT...";
        display->setTextSize(2);
        display->setCursor((SCR_W - textWidth(waiting, 2)) / 2, CAPTURE_STATUS_Y + 6);
        display->print(waiting);
    }
}

static void drawCapturePage(const char* title) {
    display->fillScreen(COLOR_BG);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(title, 2)) / 2, 8);
    display->print(title);

    const char* instruction = "MOVE A STICK OR PRESS A BUTTON";
    display->setTextSize(1);
    display->setCursor((SCR_W - textWidth(instruction, 1)) / 2, 34);
    display->print(instruction);

    drawCaptureStatus();
    drawCaptureButtons();
    drawTestToggle();
}

static void drawCapturePromptOverlay() {
    const char* q = "KEEP CAPTURE?";
    display->fillRect(0, CAPTURE_PROMPT_Y, SCR_W, CAPTURE_PROMPT_H, COLOR_BG);
    display->setTextColor(COLOR_YELLOW);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(q, 2)) / 2, CAPTURE_PROMPT_Y + 6);
    display->print(q);

    display->fillRect(0, CAPTURE_BTN_Y - 4, SCR_W, CAPTURE_BTN_H + 8, COLOR_BG);
    drawButtonAt(CAPTURE_LEFT_X, CAPTURE_BTN_Y, CAPTURE_LEFT_W, CAPTURE_BTN_H, "KEEP");
    drawButtonAt(CAPTURE_RIGHT_X, CAPTURE_BTN_Y, CAPTURE_RIGHT_W, CAPTURE_BTN_H, "DISCARD");
}

// Capture page's own hit zones + the in-page prompt's, same separation
// rationale as dispatchHubSelectTouch()/dispatchPortSelectTouch().
static void dispatchCaptureTouch(int16_t touchX, int16_t touchY) {
    bool inLeftBtn = touchX >= CAPTURE_LEFT_X && touchX < CAPTURE_LEFT_X + CAPTURE_LEFT_W && touchY >= CAPTURE_BTN_Y &&
                     touchY < CAPTURE_BTN_Y + CAPTURE_BTN_H;
    bool inRightBtn = touchX >= CAPTURE_RIGHT_X && touchX < CAPTURE_RIGHT_X + CAPTURE_RIGHT_W &&
                      touchY >= CAPTURE_BTN_Y && touchY < CAPTURE_BTN_Y + CAPTURE_BTN_H;

    if (capturePromptActive) {
        if (inLeftBtn) {
            // KEEP: capture (and device/port) fully retained.
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture prompt KEEP\n", touchX, touchY);
            capturePromptActive = false;
            goToPage(PAGE_EDIT_PORTS);
        } else if (inRightBtn) {
            // DISCARD: clears capture only - device/port untouched.
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture prompt DISCARD\n", touchX, touchY);
            capturePromptActive = false;
            editCtx.captured_input = MKH_INPUT_NONE;
            editCtx.capture_valid = false;
            goToPage(PAGE_EDIT_PORTS);
        }
        return;
    }

    if (testToggleHit(touchX, touchY)) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> Test toggle (capture)\n", touchX, touchY);
        handleTestToggleTap();
        return;
    }

    if (inLeftBtn) {
        if (editCtx.capture_valid) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture BACK, prompting (capture set)\n", touchX,
                            touchY);
            capturePromptActive = true;
            drawCapturePromptOverlay();
        } else {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture BACK, silent (no capture)\n", touchX,
                            touchY);
            goToPage(PAGE_EDIT_PORTS);
        }
    } else if (inRightBtn && captureHasNext()) {
        // NEXT never prompts - forward is an implicit keep (capture is
        // already sitting in editCtx; there is nothing more to commit).
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture NEXT -> settings\n", touchX, touchY);
        goToPage(PAGE_EDIT_SETTINGS);
    }
}

// WO10-FINAL: settings page. Replaces the placeholder wholesale, same
// custom render/dispatch pattern as hub/port/capture. INPUT's own
// commit into the pending session happens on ENTRY (goToPage()'s
// PAGE_EDIT_SETTINGS case, below) - same "reaching this page implies
// keep" precedent as capture's NEXT. invert/max/mode are still
// "uncommitted" scratch on THIS page - settingsBaseline* is what
// they're compared against on Back to decide whether to prompt.
static bool settingsWorkingInvert = false;
static uint8_t settingsWorkingMax = 100;
static mkh_input_mode_t settingsWorkingMode = MKH_MODE_PROPORTIONAL;
static bool settingsBaselineInvert = false;
static uint8_t settingsBaselineMax = 100;
static mkh_input_mode_t settingsBaselineMode = MKH_MODE_PROPORTIONAL;
static bool settingsPromptActive = false;

// WO11 Task 2 (settings-page polish): INVERT and MODE combined onto one
// 40px row (left/right split at SPAGE_TOGGLE_MID_X), and the MAX label
// folded into the slider's own 40px band - both were 28px-tall separate
// rows before, short of the order's 40px minimum (flagged in the
// WO10-FINAL rev B report). Combining rows rather than shrinking gaps
// elsewhere keeps every remaining gap comfortable within the same
// 240px page.
static const int16_t SPAGE_INPUT_Y = 26;
static const int16_t SPAGE_TOGGLE_ROW_Y = 46;
static const int16_t SPAGE_TOGGLE_ROW_H = 40;  // meets the 40px minimum
static const int16_t SPAGE_TOGGLE_MID_X = 160;  // left = invert, right = mode
static const int16_t SPAGE_INVERT_TRACK_X = 14;  // relative to x=0, left half only
static const int16_t SPAGE_MAXSLIDER_Y = 90;
static const int16_t SPAGE_MAXSLIDER_H = 40;  // meets the 40px minimum; label lives inside this same band
static const int16_t SPAGE_SLIDER_X = 20;         // left edge 20, 20px from screen edge (0)
static const int16_t SPAGE_SLIDER_W = 280;         // right edge 300, 20px from screen edge (320)
static const int16_t SPAGE_SLIDER_TRACK_H = 12;
static const int16_t SPAGE_CURVE_Y = 134;

static const int16_t SPAGE_BACK_X = 20;
static const int16_t SPAGE_RESET_X = 120;
static const int16_t SPAGE_SAVE_X = 220;
static const int16_t SPAGE_BTN_W = 80;
static const int16_t SPAGE_BTN_Y = 180;
static const int16_t SPAGE_BTN_H = 40;  // bottom=220, 20px from screen edge (240)

static void drawSettingsInvertRow() {
    display->fillRect(0, SPAGE_TOGGLE_ROW_Y, SPAGE_TOGGLE_MID_X, SPAGE_TOGGLE_ROW_H, COLOR_BG);
    display->drawRect(4, SPAGE_TOGGLE_ROW_Y + 2, SPAGE_TOGGLE_MID_X - 8, SPAGE_TOGGLE_ROW_H - 4, COLOR_CARD_BORDER);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    display->setCursor(SPAGE_INVERT_TRACK_X, SPAGE_TOGGLE_ROW_Y + 6);
    display->print("INVERT");

    // Same pill-track+knob visual style as the dashboard's MKH switches
    // (SWITCH_TRACK_W/H/SWITCH_KNOB_R/SWITCH_PAD, reused as-is) - a
    // small parallel function rather than a refactor of the dashboard's
    // own rowTop()-tied drawToggleSwitch(), to avoid touching dashboard-
    // critical rendering code for an unrelated page.
    int16_t trackTop = SPAGE_TOGGLE_ROW_Y + SPAGE_TOGGLE_ROW_H - SWITCH_TRACK_H - 8;
    int16_t knobCy = trackTop + SWITCH_TRACK_H / 2;
    int16_t knobCx = settingsWorkingInvert ? (SPAGE_INVERT_TRACK_X + SWITCH_TRACK_W - SWITCH_KNOB_R - SWITCH_PAD)
                                            : (SPAGE_INVERT_TRACK_X + SWITCH_KNOB_R + SWITCH_PAD);
    uint16_t trackColor = settingsWorkingInvert ? COLOR_GREEN : COLOR_RED;
    display->fillRoundRect(SPAGE_INVERT_TRACK_X, trackTop, SWITCH_TRACK_W, SWITCH_TRACK_H, SWITCH_TRACK_H / 2,
                            trackColor);
    display->fillCircle(knobCx, knobCy, SWITCH_KNOB_R, COLOR_LABEL);
}

static const char* modeShortLabel(mkh_input_mode_t mode) {
    switch (mode) {
        case MKH_MODE_PROPORTIONAL:
            return "PROPORTIONAL";
        case MKH_MODE_MOMENTARY:
            return "MOMENTARY";
        case MKH_MODE_LATCHED:
            return "LATCHED";
        default:
            return "?";
    }
}

// 2-state cycle for digitals (MOMENTARY<->LATCHED - no proportional
// option, order: "buttons/dpad/stick-clicks = momentary (default) |
// latched"); 3-state cycle for triggers (PROPORTIONAL->MOMENTARY->
// LATCHED->...). Never called for MKH_INPUT_CLASS_AXIS - that row is
// hidden entirely (order: "hidden/disabled for stick axes").
static mkh_input_mode_t nextModeFor(mkh_input_class_t cls, mkh_input_mode_t current) {
    if (cls == MKH_INPUT_CLASS_TRIGGER) {
        switch (current) {
            case MKH_MODE_PROPORTIONAL:
                return MKH_MODE_MOMENTARY;
            case MKH_MODE_MOMENTARY:
                return MKH_MODE_LATCHED;
            default:
                return MKH_MODE_PROPORTIONAL;
        }
    }
    return (current == MKH_MODE_LATCHED) ? MKH_MODE_MOMENTARY : MKH_MODE_LATCHED;
}

// WO10-FINAL rev B mode control: CC's widget judgment (reported) - a
// single "tap row to cycle" control showing the current mode as text,
// used for BOTH the 2-state (digital) and 3-state (trigger) cases
// rather than a true graphical pill/segmented-control, since a plain
// on/off pill has no natural 3-position analogue and building two
// different widget shapes for what's functionally the same interaction
// (tap, see the next state) wasn't worth the extra code. Hidden
// entirely for MKH_INPUT_CLASS_AXIS ports - "no mode" is represented by
// the row simply not existing, not a disabled-but-visible state.
static void drawSettingsModeRow() {
    int16_t x = SPAGE_TOGGLE_MID_X;
    int16_t w = SCR_W - SPAGE_TOGGLE_MID_X;
    display->fillRect(x, SPAGE_TOGGLE_ROW_Y, w, SPAGE_TOGGLE_ROW_H, COLOR_BG);
    if (mkh_config_input_class(editCtx.captured_input) == MKH_INPUT_CLASS_AXIS) {
        return;  // hidden entirely for stick axes - "no mode" is the row not existing
    }
    char line[24];
    snprintf(line, sizeof(line), "MODE: %s", modeShortLabel(settingsWorkingMode));
    display->drawRect(x + 4, SPAGE_TOGGLE_ROW_Y + 2, w - 8, SPAGE_TOGGLE_ROW_H - 4, COLOR_CARD_BORDER);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    display->setCursor(x + 10, SPAGE_TOGGLE_ROW_Y + (SPAGE_TOGGLE_ROW_H - 8) / 2);
    display->print(line);
}

static void drawSettingsMaxRow() {
    display->fillRect(0, SPAGE_MAXSLIDER_Y, SCR_W, SPAGE_MAXSLIDER_H, COLOR_BG);
    char label[16];
    snprintf(label, sizeof(label), "MAX: %u%%", (unsigned)settingsWorkingMax);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    display->setCursor(SPAGE_SLIDER_X, SPAGE_MAXSLIDER_Y);
    display->print(label);

    int16_t trackY = SPAGE_MAXSLIDER_Y + 20;
    display->fillRect(SPAGE_SLIDER_X, trackY, SPAGE_SLIDER_W, SPAGE_SLIDER_TRACK_H, COLOR_CARD_BG);
    display->drawRect(SPAGE_SLIDER_X, trackY, SPAGE_SLIDER_W, SPAGE_SLIDER_TRACK_H, COLOR_CARD_BORDER);
    int16_t fillW = (int16_t)(((int32_t)SPAGE_SLIDER_W * settingsWorkingMax) / 100);
    if (fillW > 0) {
        display->fillRect(SPAGE_SLIDER_X, trackY, fillW, SPAGE_SLIDER_TRACK_H, COLOR_GREEN);
    }
}

static void drawSettingsButtons() {
    display->fillRect(0, SPAGE_BTN_Y - 4, SCR_W, SPAGE_BTN_H + 8, COLOR_BG);
    drawButtonAt(SPAGE_BACK_X, SPAGE_BTN_Y, SPAGE_BTN_W, SPAGE_BTN_H, "BACK");
    drawButtonAt(SPAGE_RESET_X, SPAGE_BTN_Y, SPAGE_BTN_W, SPAGE_BTN_H, "RESET");
    drawButtonAt(SPAGE_SAVE_X, SPAGE_BTN_Y, SPAGE_BTN_W, SPAGE_BTN_H, "SAVE");
}

static void drawSettingsPage() {
    display->fillScreen(COLOR_BG);

    char title[32];
    if (editCtx.selected_valid) {
        snprintf(title, sizeof(title), "MKH%d - Port %c", mkh_device_app_number(editCtx.selected_device),
                 mkh_port_letter(editCtx.selected_port));
    } else {
        snprintf(title, sizeof(title), "EDIT: PORT SETTINGS");
    }
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(title, 2)) / 2, 8);
    display->print(title);

    char inputLine[24];
    snprintf(inputLine, sizeof(inputLine), "INPUT: %s", mkh_config_input_token_name(editCtx.captured_input));
    display->setTextSize(2);
    display->setCursor(20, SPAGE_INPUT_Y);
    display->print(inputLine);

    drawSettingsInvertRow();
    drawSettingsModeRow();
    drawSettingsMaxRow();

    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    display->setCursor(20, SPAGE_CURVE_Y);
    display->print("CURVE: LINEAR");  // v1: not selectable, see mkh_config.h

    settingsPromptActive = false;
    drawSettingsButtons();
    drawTestToggle();
}

static void drawSettingsPromptOverlay() {
    const char* q = "KEEP CHANGES?";
    display->fillRect(0, SPAGE_CURVE_Y - 4, SCR_W, 24, COLOR_BG);
    display->setTextColor(COLOR_YELLOW);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(q, 2)) / 2, SPAGE_CURVE_Y - 2);
    display->print(q);

    display->fillRect(0, SPAGE_BTN_Y - 4, SCR_W, SPAGE_BTN_H + 8, COLOR_BG);
    drawButtonAt(EXIT_PROMPT_LEFT_X, SPAGE_BTN_Y, EXIT_PROMPT_LEFT_W, SPAGE_BTN_H, "KEEP");
    drawButtonAt(EXIT_PROMPT_RIGHT_X, SPAGE_BTN_Y, EXIT_PROMPT_RIGHT_W, SPAGE_BTN_H, "DISCARD");
}

// Settings page's own hit zones + its leave-page prompt's, same
// separation rationale as the other custom pages.
static void dispatchSettingsTouch(int16_t touchX, int16_t touchY) {
    int dev = editCtx.selected_device;
    int port = editCtx.selected_port;
    mkh_input_class_t cls = mkh_config_input_class(editCtx.captured_input);

    if (settingsPromptActive) {
        bool inKeep = touchX >= EXIT_PROMPT_LEFT_X && touchX < EXIT_PROMPT_LEFT_X + EXIT_PROMPT_LEFT_W &&
                      touchY >= SPAGE_BTN_Y && touchY < SPAGE_BTN_Y + SPAGE_BTN_H;
        bool inDiscard = touchX >= EXIT_PROMPT_RIGHT_X && touchX < EXIT_PROMPT_RIGHT_X + EXIT_PROMPT_RIGHT_W &&
                         touchY >= SPAGE_BTN_Y && touchY < SPAGE_BTN_Y + SPAGE_BTN_H;
        if (inKeep) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings prompt KEEP\n", touchX, touchY);
            // WO11 Task 1: dirty/input are no longer guaranteed set by
            // page entry (see goToPage()'s PAGE_EDIT_SETTINGS case) -
            // this KEEP is itself a real, confirmed edit, so it commits
            // the whole port explicitly, not just invert/max/mode.
            pendingEdits[dev][port].dirty = true;
            pendingEdits[dev][port].input = editCtx.captured_input;
            pendingEdits[dev][port].invert = settingsWorkingInvert;
            pendingEdits[dev][port].max_percent = settingsWorkingMax;
            pendingEdits[dev][port].mode = settingsWorkingMode;
            settingsPromptActive = false;
            goToPage(PAGE_EDIT_CAPTURE);
        } else if (inDiscard) {
            // pendingEdits[dev][port] is left exactly as it was on
            // entry (untouched here) - if entry didn't dirty it (WO11
            // Task 1), discarding these working values just returns to
            // that already-correct baseline, nothing to undo.
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings prompt DISCARD\n", touchX, touchY);
            settingsPromptActive = false;
            goToPage(PAGE_EDIT_CAPTURE);
        }
        return;
    }

    if (testToggleHit(touchX, touchY)) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> Test toggle (settings)\n", touchX, touchY);
        handleTestToggleTap();
        return;
    }

    // WO11 Task 2: INVERT (left half) and MODE (right half) now share
    // one 40px row, split at SPAGE_TOGGLE_MID_X.
    bool inToggleRow = touchY >= SPAGE_TOGGLE_ROW_Y && touchY < SPAGE_TOGGLE_ROW_Y + SPAGE_TOGGLE_ROW_H;
    if (inToggleRow && touchX < SPAGE_TOGGLE_MID_X) {
        settingsWorkingInvert = !settingsWorkingInvert;
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings INVERT -> %s\n", touchX, touchY,
                        settingsWorkingInvert ? "yes" : "no");
        drawSettingsInvertRow();
        return;
    }
    if (inToggleRow && touchX >= SPAGE_TOGGLE_MID_X && cls != MKH_INPUT_CLASS_AXIS) {
        settingsWorkingMode = nextModeFor(cls, settingsWorkingMode);
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings MODE -> %s\n", touchX, touchY,
                        modeShortLabel(settingsWorkingMode));
        drawSettingsModeRow();
        return;
    }

    if (touchX >= SPAGE_SLIDER_X && touchX < SPAGE_SLIDER_X + SPAGE_SLIDER_W && touchY >= SPAGE_MAXSLIDER_Y &&
        touchY < SPAGE_MAXSLIDER_Y + SPAGE_MAXSLIDER_H) {
        int32_t rel = touchX - SPAGE_SLIDER_X;
        if (rel < 0)
            rel = 0;
        if (rel > SPAGE_SLIDER_W)
            rel = SPAGE_SLIDER_W;
        settingsWorkingMax = (uint8_t)((rel * 100) / SPAGE_SLIDER_W);
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings MAX slider -> %u%%\n", touchX, touchY,
                        (unsigned)settingsWorkingMax);
        drawSettingsMaxRow();
        return;
    }

    bool inBack = touchX >= SPAGE_BACK_X && touchX < SPAGE_BACK_X + SPAGE_BTN_W && touchY >= SPAGE_BTN_Y &&
                  touchY < SPAGE_BTN_Y + SPAGE_BTN_H;
    bool inReset = touchX >= SPAGE_RESET_X && touchX < SPAGE_RESET_X + SPAGE_BTN_W && touchY >= SPAGE_BTN_Y &&
                   touchY < SPAGE_BTN_Y + SPAGE_BTN_H;
    bool inSave = touchX >= SPAGE_SAVE_X && touchX < SPAGE_SAVE_X + SPAGE_BTN_W && touchY >= SPAGE_BTN_Y &&
                  touchY < SPAGE_BTN_Y + SPAGE_BTN_H;

    if (inBack) {
        bool dirty = (settingsWorkingInvert != settingsBaselineInvert) ||
                     (settingsWorkingMax != settingsBaselineMax) || (settingsWorkingMode != settingsBaselineMode);
        if (dirty) {
            Console.printf(
                "MK1 Touch: tap display=(x=%d,y=%d) -> settings BACK, prompting (uncommitted edits)\n", touchX,
                touchY);
            settingsPromptActive = true;
            drawSettingsPromptOverlay();
        } else {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings BACK, silent (no uncommitted edits)\n",
                            touchX, touchY);
            goToPage(PAGE_EDIT_CAPTURE);
        }
    } else if (inReset) {
        // Order: "Reset (settings page) restores saved config to the
        // table and clears the session." CC's judgment (reported):
        // stays on the settings page rather than navigating away,
        // refreshing this port's fields from the just-reloaded table -
        // a form-reset, not an exit.
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings RESET\n", touchX, touchY);
        mkh_storage_reload_config();
        clearPendingSession();
        testActive = false;
        const mkh_port_config_t* cfg = mkh_config_get(dev, port);
        editCtx.captured_input = cfg ? cfg->input : MKH_INPUT_NONE;
        editCtx.capture_valid = (cfg != nullptr) && (cfg->input != MKH_INPUT_NONE);
        settingsWorkingInvert = cfg ? cfg->invert : false;
        settingsWorkingMax = cfg ? cfg->max_percent : 100;
        settingsWorkingMode = cfg ? cfg->mode : MKH_MODE_PROPORTIONAL;
        settingsBaselineInvert = settingsWorkingInvert;
        settingsBaselineMax = settingsWorkingMax;
        settingsBaselineMode = settingsWorkingMode;
        drawSettingsPage();
    } else if (inSave) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings SAVE\n", touchX, touchY);
        // WO11 Task 1: same as KEEP above - commit the whole port
        // explicitly, entry no longer guarantees dirty/input are set.
        pendingEdits[dev][port].dirty = true;
        pendingEdits[dev][port].input = editCtx.captured_input;
        pendingEdits[dev][port].invert = settingsWorkingInvert;
        pendingEdits[dev][port].max_percent = settingsWorkingMax;
        pendingEdits[dev][port].mode = settingsWorkingMode;
        performSaveAndExit();
    }
}

// Sole navigation entry point (WO10 Step 0 design intent): every future
// page type just needs an enum value, a render branch here, and a
// dispatch branch in loop() - no other call site changes. Renders the
// destination page immediately (full redraw - editor pages have no
// partial-diff update path yet, unlike the dashboard's updateDashboard()).
static void goToPage(UiPage page) {
    currentPage = page;
    switch (page) {
        case PAGE_SPLASH:
            drawSplash();
            break;
        case PAGE_STATS:
            drawStatsPage();
            break;
        case PAGE_DASHBOARD:
            initDashboard();
            break;
        case PAGE_EDIT_SELECT:
            drawHubSelect();
            break;
        case PAGE_EDIT_PORTS:
            drawPortSelect();
            break;
        case PAGE_EDIT_CAPTURE: {
            // WO10 Step 1: title proves the editing context flows
            // forward, instead of a static placeholder string. ASCII
            // hyphen, not the order's example "·" middle-dot, since the
            // display's built-in bitmap font isn't guaranteed to have a
            // glyph for it.
            char title[32];
            if (editCtx.selected_valid) {
                snprintf(title, sizeof(title), "MKH%d - Port %c", mkh_device_app_number(editCtx.selected_device),
                         mkh_port_letter(editCtx.selected_port));
            } else {
                snprintf(title, sizeof(title), "EDIT: CAPTURE INPUT");
            }
            // WO10 Step 2: real capture page now (drawCapturePage(),
            // not the generic placeholder). Prompt state is reset on
            // every entry - it's this page's own sub-state, never
            // meaningful to carry across a navigation.
            capturePromptActive = false;
            drawCapturePage(title);
            break;
        }
        case PAGE_EDIT_SETTINGS: {
            // WO11 Task 1 fix (was: WO10-FINAL unconditionally set
            // pendingEdits[dev][port].dirty = true right here, on every
            // entry - so merely walking capture -> settings -> Back
            // dirtied the whole session and could trip the exit-editor
            // prompt with nothing actually changed). Root cause: entry
            // wrote through the SAME path a real edit does, with no
            // comparison against what's already true for this port.
            //
            // Fix: compute the TRUE current baseline for this port
            // (existing pending edit if one exists, else the resolved
            // live config - including its INPUT, not just invert/max/
            // mode) and only commit into the pending session if the
            // freshly-captured input actually differs from that
            // baseline. A same-input revisit (or a first visit that
            // just confirms what's already configured) no longer
            // dirties anything by itself; invert/max/mode edits still
            // commit via their own KEEP/SAVE paths below, which now
            // also set dirty+input explicitly rather than relying on
            // this entry-time write.
            int dev = editCtx.selected_device;
            int port = editCtx.selected_port;
            bool hadPending = pendingEdits[dev][port].dirty;
            mkh_input_source_t baselineInput;
            if (hadPending) {
                baselineInput = pendingEdits[dev][port].input;
                settingsWorkingInvert = pendingEdits[dev][port].invert;
                settingsWorkingMax = pendingEdits[dev][port].max_percent;
                settingsWorkingMode = pendingEdits[dev][port].mode;
            } else {
                const mkh_port_config_t* cfg = mkh_config_get(dev, port);
                baselineInput = cfg ? cfg->input : MKH_INPUT_NONE;
                settingsWorkingInvert = cfg ? cfg->invert : false;
                settingsWorkingMax = cfg ? cfg->max_percent : 100;
                settingsWorkingMode = cfg ? cfg->mode : MKH_MODE_PROPORTIONAL;
            }
            if (editCtx.captured_input != baselineInput) {
                pendingEdits[dev][port].dirty = true;
                pendingEdits[dev][port].input = editCtx.captured_input;
                pendingEdits[dev][port].invert = settingsWorkingInvert;
                pendingEdits[dev][port].max_percent = settingsWorkingMax;
                pendingEdits[dev][port].mode = settingsWorkingMode;
            }
            settingsBaselineInvert = settingsWorkingInvert;
            settingsBaselineMax = settingsWorkingMax;
            settingsBaselineMode = settingsWorkingMode;
            drawSettingsPage();
            break;
        }
        case PAGE_IDLE:
            drawIdlePage();
            break;
        case PAGE_COUNT:
            break;
    }
}

// WO10 Step 1: PAGE_EDIT_SELECT no longer has a case here - it's never
// passed in (dispatchHubSelectTouch() handles its own Back).
// WO10 Step 2: PAGE_EDIT_CAPTURE's case removed too - it left this
// generic pattern for dispatchCaptureTouch()'s own Back handling
// (prompt-gated, unlike this function's unconditional navigation).
// PAGE_EDIT_SETTINGS is the only page left using this generic path.
static UiPage editorBackTarget(UiPage page) {
    switch (page) {
        case PAGE_EDIT_SETTINGS:
            return PAGE_EDIT_CAPTURE;
        default:
            return PAGE_DASHBOARD;
    }
}

// WO10 Step 2: no case is reachable here anymore - PAGE_EDIT_CAPTURE's
// Next is now dispatchCaptureTouch()'s own (captureHasNext() ->
// PAGE_EDIT_SETTINGS directly), and pageHasNext() below always returns
// false for this function's only remaining caller (PAGE_EDIT_SETTINGS,
// which has no Next). Kept for interface stability rather than deleted.
static UiPage editorNextTarget(UiPage page) {
    return page;
}

// Editor pages' own hit zones (WO10 Step 0's "own hit zones" per page,
// separate from the dashboard's touchYToDeviceId() split above - the
// two are never consulted in the same tap).
static void dispatchEditorTouch(int16_t touchX, int16_t touchY) {
    bool inBack = touchX >= EDITOR_BACK_X && touchX < EDITOR_BACK_X + EDITOR_BACK_W && touchY >= EDITOR_BTN_Y &&
                  touchY < EDITOR_BTN_Y + EDITOR_BTN_H;
    bool inNext = pageHasNext(currentPage) && touchX >= EDITOR_NEXT_X && touchX < EDITOR_NEXT_X + EDITOR_NEXT_W &&
                  touchY >= EDITOR_BTN_Y && touchY < EDITOR_BTN_Y + EDITOR_BTN_H;

    if (inBack) {
        UiPage target = editorBackTarget(currentPage);
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> editor BACK, page %d -> %d\n", touchX, touchY,
                        currentPage, target);
        goToPage(target);
    } else if (inNext) {
        UiPage target = editorNextTarget(currentPage);
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> editor NEXT, page %d -> %d\n", touchX, touchY,
                        currentPage, target);
        goToPage(target);
    }
}

// WO11: no longer draws any page content itself (previously called
// initDashboard() directly) - setup() below now owns which page is
// drawn first via goToPage(), since boot must land on PAGE_SPLASH, not
// PAGE_DASHBOARD. This function is display hardware bring-up only.
static void initDisplay() {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    display->begin();
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

// WO10-FINAL rev B: analog trigger -> MK6 byte, one-sided (0x80 at rest
// ramping up to 0xFF at full pull) unlike the sticks' bidirectional
// mkStickYToChannelByte() - a trigger has no "reverse" reading to
// represent. Bluepad32's brake()/throttle() range is 0..1023 (verified
// against the existing dumpGamepad() comment, which already documented
// this range before this order). Same MKH_STICK_DEADZONE_PERCENT
// reused for consistency, applied to the 1023 magnitude instead of 512.
static uint8_t mkTriggerToChannelByte(int32_t triggerValue) {
    const int32_t magnitude = 1023;
    const int32_t deadzone = (magnitude * MKH_STICK_DEADZONE_PERCENT) / 100;

    if (triggerValue <= deadzone) {
        return 0x80;
    }

    int32_t travel = magnitude - deadzone;
    float fraction = (float)(triggerValue - deadzone) / (float)travel;
    if (fraction > 1.0f)
        fraction = 1.0f;

    int value = 128 + (int)lroundf(fraction * 127.0f);
    if (value > 0xFF)
        value = 0xFF;
    return (uint8_t)value;
}

// WO10-FINAL rev B: latch state - runtime-only ("never persisted, boots
// off" per the order), tracked per port (not per token) since two ports
// could in principle latch the same physical input independently.
// latchPrevPressed[] is the edge detector: a latch toggles on the
// press EDGE, not the level, so holding the button doesn't rapidly
// re-toggle every ~150ms loop tick.
static bool latchState[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS];
static bool latchPrevPressed[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS];

// WO11 Task 4 (hardening): reassigning a port away from latched mode and
// back again via the editor could otherwise resurface a stale latchState
// value from before the reassignment - a port pushed through
// pushPendingSessionToLiveTable() (Test-ON/Save) always gets a fresh,
// unlatched start, same "boots off" guarantee the order gives config
// itself. Forward-declared above (near clearEditorSession()) so
// pushPendingSessionToLiveTable(), defined earlier in the file, can call it.
static void clearLatchStateForPort(int dev, int port) {
    latchState[dev][port] = false;
    latchPrevPressed[dev][port] = false;
}

static void clearLatchStateForDevice(int dev) {
    for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
        clearLatchStateForPort(dev, p);
    }
}

static void clearAllLatchState() {
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        clearLatchStateForDevice(d);
    }
}

// Reads one token's raw controller value: Bluepad32's native -512..511
// range for MKH_INPUT_CLASS_AXIS tokens, 0..1023 for MKH_INPUT_CLASS_
// TRIGGER tokens, or 0/1 for MKH_INPUT_CLASS_DIGITAL tokens. Callers
// interpret the range themselves based on mkh_config_input_class() -
// this function only reads, it doesn't scale or threshold anything.
static int32_t readTokenRaw(ControllerPtr ctl, mkh_input_source_t token) {
    switch (token) {
        case MKH_INPUT_LSNS:
            return ctl->axisY();
        case MKH_INPUT_LSEW:
            return ctl->axisX();
        case MKH_INPUT_RSNS:
            return ctl->axisRY();
        case MKH_INPUT_RSEW:
            return ctl->axisRX();
        case MKH_INPUT_LT:
            return ctl->brake();
        case MKH_INPUT_RT:
            return ctl->throttle();
        case MKH_INPUT_BTN_A:
            return ctl->a() ? 1 : 0;
        case MKH_INPUT_BTN_B:
            return ctl->b() ? 1 : 0;
        case MKH_INPUT_BTN_X:
            return ctl->x() ? 1 : 0;
        case MKH_INPUT_BTN_Y:
            return ctl->y() ? 1 : 0;
        case MKH_INPUT_LB:
            return ctl->l1() ? 1 : 0;
        case MKH_INPUT_RB:
            return ctl->r1() ? 1 : 0;
        case MKH_INPUT_DPAD_UP:
            return (ctl->dpad() & DPAD_UP) ? 1 : 0;
        case MKH_INPUT_DPAD_DOWN:
            return (ctl->dpad() & DPAD_DOWN) ? 1 : 0;
        case MKH_INPUT_DPAD_LEFT:
            return (ctl->dpad() & DPAD_LEFT) ? 1 : 0;
        case MKH_INPUT_DPAD_RIGHT:
            return (ctl->dpad() & DPAD_RIGHT) ? 1 : 0;
        case MKH_INPUT_L3:
            return ctl->thumbL() ? 1 : 0;
        case MKH_INPUT_R3:
            return ctl->thumbR() ? 1 : 0;
        default:
            return 0;
    }
}

// WO10-FINAL rev B: the full-universe drive sweep, replacing v0.8.0
// Step 2's stick-only version. Every assigned port is driven every
// call regardless of mode - proportional inputs scale continuously,
// momentary is a simple pressed/released level, latched toggles on a
// press edge and otherwise holds (see latchState[]/latchPrevPressed[]
// above). All three funnel through the SAME mkApplyPortConfig()
// invert/max pipeline (order: "all through the existing invert/max
// pipeline") - only the pre-pipeline raw byte differs by class/mode.
static void processMappedInputs(ControllerPtr ctl) {
    for (int dev = 0; dev < MKH_MK6_NUM_DEVICES; dev++) {
        for (int port = 0; port < MKH_MK6_NUM_CHANNELS; port++) {
            const mkh_port_config_t* cfg = mkh_config_get(dev, port);
            if (!cfg || cfg->input == MKH_INPUT_NONE)
                continue;

            mkh_input_class_t cls = mkh_config_input_class(cfg->input);
            uint8_t rawByte;

            if (cls == MKH_INPUT_CLASS_AXIS) {
                // Stick axes: no mode, always proportional (order) -
                // cfg->mode is never consulted here, regardless of what
                // it happens to hold.
                rawByte = mkStickYToChannelByte(readTokenRaw(ctl, cfg->input));
            } else if (cls == MKH_INPUT_CLASS_TRIGGER && cfg->mode == MKH_MODE_PROPORTIONAL) {
                rawByte = mkTriggerToChannelByte(readTokenRaw(ctl, cfg->input));
            } else {
                // DIGITAL, or a TRIGGER explicitly set to momentary/
                // latched - both resolve to a simple pressed/not level.
                // Triggers use half-travel (511 of 1023) as the
                // pressed/not-pressed split point for momentary/latched
                // mode - CC's judgment, not specified by the order.
                bool pressed = (cls == MKH_INPUT_CLASS_TRIGGER) ? (readTokenRaw(ctl, cfg->input) > (1023 / 2))
                                                                 : (readTokenRaw(ctl, cfg->input) != 0);

                if (cfg->mode == MKH_MODE_LATCHED) {
                    if (pressed && !latchPrevPressed[dev][port]) {
                        latchState[dev][port] = !latchState[dev][port];
                    }
                    latchPrevPressed[dev][port] = pressed;
                    rawByte = latchState[dev][port] ? 0xFF : 0x80;
                } else {
                    // MOMENTARY - the only other legal mode for a
                    // digital; for a trigger, the explicit momentary case.
                    rawByte = pressed ? 0xFF : 0x80;
                }
            }

            mkh_set_channel(dev, port, mkApplyPortConfig(rawByte, cfg));
        }
    }
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
                //
                // v0.9.0 Step 1 invariant (where the two paths meet):
                // this write is unconditional - it does not check
                // mkh_hub_broadcasting[]/rotation state - and that's
                // fine on both sides. A device still IN rotation picks
                // the neutral value up normally on its next service. A
                // device already toggled OFF is already neutral (that's
                // exactly what mkh_broadcast_toggle_off() commands
                // before it starts the drop countdown - see
                // mkh_broadcast.c), so this write is a redundant no-op
                // for it, not a gap. Either way, no device can resume
                // from a stale non-neutral value after XWC disconnects.
                //
                // WO10-FINAL rev B: "failsafe absolute" - neutralizes
                // ALL ports including latched-on, clears all latch
                // state, no exceptions for any token type. The
                // unconditional mkh_set_channel(...,0x80) below already
                // covers "neutralize latched-on" (a latch's live output
                // IS just 0x80/0xFF through the same pipeline as
                // everything else - see processMappedInputs()); this
                // adds the other half, clearing our own latch tracking
                // so a reconnect doesn't resume already-latched.
                for (int dev = 0; dev < MKH_MK6_NUM_DEVICES; dev++) {
                    for (int port = 0; port < MKH_MK6_NUM_CHANNELS; port++) {
                        const mkh_port_config_t* cfg = mkh_config_get(dev, port);
                        if (cfg && cfg->input != MKH_INPUT_NONE) {
                            mkh_set_channel(dev, port, 0x80);
                        }
                    }
                }
                clearAllLatchState();
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
        // v0.8.0 Step 2 / WO10-FINAL rev B: which device/port slot(s)
        // each token drives is resolved at runtime from the config
        // table (mkh_config_get()) - see processMappedInputs() above
        // for the full 18-token, mode-aware sweep. Default config still
        // reproduces v0.7.0 exactly (HUB1 port A = LSNS, HUB2 port A =
        // RSNS, invert=no, max=100, proportional).
        processMappedInputs(ctl);
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

// WO10 Step 2: capture logic. Read-only observation of Bluepad32's
// existing controller state - myControllers[0] is the exact same
// pointer processControllers()/processGamepad() already read above;
// this is not a second read path and processControllers() itself is
// untouched. Called once per loop() iteration while sitting on the
// capture page (see loop() below) - first input past threshold wins,
// and capture always overwrites on a later distinct crossing (wiggle
// again = new capture, no confirm step, by design).
//
// Threshold: MKH_CAPTURE_THRESHOLD_PERCENT=25, chosen to comfortably
// clear MKH_STICK_DEADZONE_PERCENT (8%) - roughly 3x - so it's well
// above any resting/analog noise the deadzone itself doesn't already
// absorb, while still being a modest, not-full-deflection push that
// any deliberate "wiggle" clears without requiring precision from the
// user. Reused as-is (same percentage) for trigger capture below.
#define MKH_CAPTURE_THRESHOLD_PERCENT 25

static bool stickPastCaptureThreshold(int32_t axisValue) {
    const int32_t threshold = (MKH_STICK_AXIS_MAGNITUDE * MKH_CAPTURE_THRESHOLD_PERCENT) / 100;
    return axisValue <= -threshold || axisValue >= threshold;
}

// Triggers are 0..1023, one-sided (0=released) - see mkTriggerToChannelByte()
// above for the same range used in the drive sweep.
static bool triggerPastCaptureThreshold(int32_t triggerValue) {
    const int32_t threshold = (1023 * MKH_CAPTURE_THRESHOLD_PERCENT) / 100;
    return triggerValue >= threshold;
}

// Applies a new capture, but only if it actually changes anything -
// holding a stick past threshold re-evaluates every ~150ms loop tick,
// and without this guard that would re-log and re-redraw the (already
// current) status line every tick for as long as the stick stays held.
static void applyCapture(mkh_input_source_t token) {
    if (editCtx.capture_valid && editCtx.captured_input == token)
        return;
    bool hadCapture = editCtx.capture_valid;
    editCtx.captured_input = token;
    editCtx.capture_valid = true;
    Console.printf("MK1 Capture: %s captured\n", mkh_config_input_token_name(token));
    drawCaptureStatus();
    if (!hadCapture) {
        drawCaptureButtons();  // NEXT appears for the first time
    }
}

// WO10-FINAL rev B: extended from stick-only to the full 18-token
// universe. Axes/triggers capture past MKH_CAPTURE_THRESHOLD_PERCENT;
// every digital captures on a plain press (order: "digital inputs on
// press") - no threshold concept applies to a boolean. Checked in a
// fixed order (axes, then triggers, then digitals) - "first past
// threshold wins" only matters for same-tick ties, which this order
// resolves the same deterministic way every time; it does not mean any
// one input is preferred once a session is underway (re-capture always
// overwrites, per applyCapture() above).
static void checkCapture() {
    ControllerPtr ctl = myControllers[0];
    if (!ctl || !ctl->isConnected())
        return;

    if (stickPastCaptureThreshold(ctl->axisY())) {
        applyCapture(MKH_INPUT_LSNS);
        return;
    }
    if (stickPastCaptureThreshold(ctl->axisX())) {
        applyCapture(MKH_INPUT_LSEW);
        return;
    }
    if (stickPastCaptureThreshold(ctl->axisRY())) {
        applyCapture(MKH_INPUT_RSNS);
        return;
    }
    if (stickPastCaptureThreshold(ctl->axisRX())) {
        applyCapture(MKH_INPUT_RSEW);
        return;
    }
    if (triggerPastCaptureThreshold(ctl->brake())) {
        applyCapture(MKH_INPUT_LT);
        return;
    }
    if (triggerPastCaptureThreshold(ctl->throttle())) {
        applyCapture(MKH_INPUT_RT);
        return;
    }
    if (ctl->a()) {
        applyCapture(MKH_INPUT_BTN_A);
        return;
    }
    if (ctl->b()) {
        applyCapture(MKH_INPUT_BTN_B);
        return;
    }
    if (ctl->x()) {
        applyCapture(MKH_INPUT_BTN_X);
        return;
    }
    if (ctl->y()) {
        applyCapture(MKH_INPUT_BTN_Y);
        return;
    }
    if (ctl->l1()) {
        applyCapture(MKH_INPUT_LB);
        return;
    }
    if (ctl->r1()) {
        applyCapture(MKH_INPUT_RB);
        return;
    }
    if (ctl->dpad() & DPAD_UP) {
        applyCapture(MKH_INPUT_DPAD_UP);
        return;
    }
    if (ctl->dpad() & DPAD_DOWN) {
        applyCapture(MKH_INPUT_DPAD_DOWN);
        return;
    }
    if (ctl->dpad() & DPAD_LEFT) {
        applyCapture(MKH_INPUT_DPAD_LEFT);
        return;
    }
    if (ctl->dpad() & DPAD_RIGHT) {
        applyCapture(MKH_INPUT_DPAD_RIGHT);
        return;
    }
    if (ctl->thumbL()) {
        applyCapture(MKH_INPUT_L3);
        return;
    }
    if (ctl->thumbR()) {
        applyCapture(MKH_INPUT_R3);
        return;
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

    // WO11: splash goes up the moment the display is ready, then boot
    // work continues exactly as before with no delay/animation inserted
    // - the "zero added boot time" constraint holds because nothing
    // here waits on anything; goToPage(PAGE_SPLASH) just draws and
    // returns. The splash is visible for however long the rest of
    // setup() naturally takes, whatever that is - see the matching
    // goToPage(PAGE_STATS) call at the end of this function, and the
    // two millis()-stamped log lines bracketing that duration for the
    // bench comparison against v0.10.0's boot timing.
    goToPage(PAGE_SPLASH);
    Console.printf("MK1 Boot: splash shown at %lu ms\n", (unsigned long)millis());

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

    // WO11: boot work is complete - switch from splash to stats
    // immediately (no minimum display time; if boot was fast, the
    // splash just wasn't up for long, which is correct per the order).
    Console.printf("MK1 Boot: boot work complete, entering stats at %lu ms\n", (unsigned long)millis());
    goToPage(PAGE_STATS);
}

// v0.9.0 Step 1, Amendment A: MKH row touch hit zones. Calibration
// evidence (WO9 Step 0b) showed a tap can land a few pixels off the
// visual row band, and the 4px ROW_GAP between cards left dead gutters
// where a tap resolved to nothing - so hit zones are NOT the visual row
// bands. Instead, the touchable area is split at the midpoints between
// adjacent MKH row CENTERS, and a tap resolves to the nearest of the
// three MKH rows (full width - Amendment A specifies no horizontal
// restriction). This also means taps on the header/XWC/UPTIME areas -
// which have no other interactive purpose in this UI - resolve to
// whichever MKH row is nearest, rather than being dead zones.
//
// Row centers (see rowTop(), ROW_H=36, ROW_GAP=4, ROW_START_Y=32):
//   MKH1 (ROW_MKH0, idx=1): center y = 90
//   MKH2 (ROW_MKH1, idx=2): center y = 130
//   MKH3 (ROW_MKH2, idx=3): center y = 170
// Zone boundaries = midpoints between adjacent centers:
//   midpoint(90,130) = 110, midpoint(130,170) = 150
// Resulting zones (device_id is the MK6 protocol device slot 0..2):
//   y < 110           -> MKH 1 (device 0)
//   110 <= y < 150    -> MKH 2 (device 1)
//   y >= 150          -> MKH 3 (device 2)
static const int16_t MKH_ZONE_1_2_BOUNDARY_Y = 110;
static const int16_t MKH_ZONE_2_3_BOUNDARY_Y = 150;

static int touchYToDeviceId(int16_t displayY) {
    if (displayY < MKH_ZONE_1_2_BOUNDARY_Y)
        return 0;
    if (displayY < MKH_ZONE_2_3_BOUNDARY_Y)
        return 1;
    return 2;
}

// ~200ms min between accepted taps on the SAME row (WO9 Step 1), tracked
// per device so rapid taps on different rows are never cross-debounced.
static const uint32_t MKH_TOGGLE_DEBOUNCE_MS = 200;
static uint32_t lastToggleMs[MKH_MK6_NUM_DEVICES] = {0, 0, 0};

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // WO10 Step 0: touch dispatch is now page-aware. The dashboard's hit
    // zones (settings button + MKH rows, debounce, transition lockout)
    // apply ONLY in PAGE_DASHBOARD; editor pages use dispatchEditorTouch()
    // instead. The ISR-driven consumption side (mkh_touch_poll() itself)
    // is untouched - only this branch on currentPage is new.
    //
    // WO11: two more currentPage branches - PAGE_STATS (tap anywhere ->
    // dashboard) and PAGE_SPLASH (no interaction, boot-driven only).
    //
    // WO10 Step 1: PAGE_EDIT_SELECT/PAGE_EDIT_PORTS get their own custom
    // dispatch functions now (not dispatchEditorTouch()).
    //
    // WO10 Step 2: PAGE_EDIT_CAPTURE too (dispatchCaptureTouch()) -
    // dispatchEditorTouch() now only serves PAGE_EDIT_SETTINGS.
    int16_t touchX, touchY;
    if (mkh_touch_poll(&touchX, &touchY)) {
        // WO11 Task 3: any accepted tap, on any page, resets the idle
        // countdown - display-layer only, reads the same touch event
        // already being dispatched below, writes nothing broadcast/
        // slicer/failsafe/XWC-processing/config touches.
        lastTouchActivityMs = millis();
        if (currentPage == PAGE_DASHBOARD) {
            // WO10 CLOSEOUT: hit-tested against the enlarged
            // SETTINGS_HIT_* rect, not the small SETTINGS_BTN_* visual -
            // checked FIRST, same order as before, so it still wins over
            // the MKH row split below for every pixel inside it.
            bool inSettingsBtn = touchX >= SETTINGS_HIT_X && touchX < SETTINGS_HIT_X + SETTINGS_HIT_W &&
                                  touchY >= SETTINGS_HIT_Y && touchY < SETTINGS_HIT_Y + SETTINGS_HIT_H;
            if (inSettingsBtn) {
                Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings button -> PAGE_EDIT_SELECT\n", touchX,
                                touchY);
                // WO10 Step 1: every fresh entry from the dashboard
                // starts the editing context clean - see EditContext's
                // doc comment for the other clearing path (Back from
                // port select). WO10-FINAL: now a full session clear
                // (also wipes any pending edits/Test state left over
                // from a prior visit that exited via DISCARD).
                clearEditorSession();
                goToPage(PAGE_EDIT_SELECT);
            } else {
                // v0.9.0 Step 1 (PM feedback pass): hit-test any fresh tap to
                // its nearest MKH row, then dispatch the toggle.
                // mkh_broadcast_toggle_off()/_on() own the full
                // safety-ordered sequencing underneath (see
                // mkh_broadcast.c) - this is just hit-testing, guarding, and
                // requesting.
                //
                // Guard order: transition lockout FIRST (the real
                // double-tap guard - mkh_broadcast_is_transitioning() is
                // true from the moment a toggle is accepted until its
                // sequence actually completes; a tap on a row mid-sequence
                // is ignored and logged, not queued), then the ~200ms
                // debounce on top of that as a secondary guard.
                int deviceId = touchYToDeviceId(touchY);
                uint32_t now = millis();

                if (mkh_broadcast_is_transitioning(deviceId)) {
                    Console.printf(
                        "MK1 Touch: tap display=(x=%d,y=%d) -> MKH row device=%d, ignored - transition in "
                        "progress\n",
                        touchX, touchY, deviceId);
                } else if (now - lastToggleMs[deviceId] >= MKH_TOGGLE_DEBOUNCE_MS) {
                    lastToggleMs[deviceId] = now;
                    int rowIdx = ROW_MKH0 + deviceId;
                    if (mkh_hub_broadcasting[deviceId]) {
                        Console.printf(
                            "MK1 Touch: tap display=(x=%d,y=%d) -> MKH row device=%d, toggle OFF requested\n",
                            touchX, touchY, deviceId);
                        mkh_broadcast_toggle_off(deviceId);
                        // WO10-FINAL rev B: "failsafe absolute... same
                        // on dashboard hub toggle-OFF" - the neutral
                        // command mkh_broadcast_toggle_off() already
                        // issues (mkh_broadcast.c, untouched) covers
                        // this device's latched-on ports' live output;
                        // this clears our own latch tracking so a later
                        // toggle ON doesn't resume already-latched.
                        clearLatchStateForDevice(deviceId);
                        // Commanded-state semantics (WO9 PM decision): animate
                        // immediately on tap acceptance, not on completion. The
                        // neutral-then-drop sequence keeps running underneath,
                        // unaffected by (and not waited on by) this draw call -
                        // see animateToggleSwitch()'s own doc comment for the
                        // contrast with the XWC dot's observed-state semantics.
                        animateToggleSwitch(rowIdx, false);
                    } else {
                        Console.printf(
                            "MK1 Touch: tap display=(x=%d,y=%d) -> MKH row device=%d, toggle ON requested\n",
                            touchX, touchY, deviceId);
                        mkh_broadcast_toggle_on(deviceId);
                        // Same immediate-commanded-state animation as OFF,
                        // EXCEPT for a parked device (the third hub - see
                        // MKH_TIME_SLICE_NUM_DEVICES in mkh_broadcast.c), which
                        // toggle_on() refuses outright (never enters the
                        // transitioning state, so this check runs the very next
                        // instant): check the actual resulting state rather
                        // than assuming success, so a tap on a parked row
                        // correctly stays red/off instead of animating to a
                        // green state that was never true.
                        if (mkh_hub_broadcasting[deviceId]) {
                            animateToggleSwitch(rowIdx, true);
                        }
                    }
                }
            }
        } else if (currentPage == PAGE_STATS) {
            // WO11: any tap anywhere on the stats page proceeds to the
            // dashboard - no sub-regions, no other action.
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> stats page, proceeding to dashboard\n", touchX,
                            touchY);
            goToPage(PAGE_DASHBOARD);
        } else if (currentPage == PAGE_SPLASH) {
            // WO11: splash is boot-driven only (see setup()) - no tap
            // interaction, intentionally ignored.
        } else if (currentPage == PAGE_EDIT_SELECT) {
            // WO10 Step 1: hub select has its own custom hit zones, not
            // the generic editor placeholder's - see dispatchHubSelectTouch().
            dispatchHubSelectTouch(touchX, touchY);
        } else if (currentPage == PAGE_EDIT_PORTS) {
            dispatchPortSelectTouch(touchX, touchY);
        } else if (currentPage == PAGE_EDIT_CAPTURE) {
            // WO10 Step 2: capture page has its own custom hit zones too
            // (Back is prompt-gated, unlike the generic pattern's
            // unconditional navigation) - see dispatchCaptureTouch().
            dispatchCaptureTouch(touchX, touchY);
        } else if (currentPage == PAGE_EDIT_SETTINGS) {
            // WO10-FINAL: settings is real now too - dispatchEditorTouch()/
            // drawEditorPlaceholder()/editorBackTarget()/editorNextTarget()/
            // pageHasNext() have no remaining caller as of this order
            // (left in place, flagged as dead code in the completion
            // report, rather than risk touching already-verified Step 0-2
            // code under this order's single-pass scope).
            dispatchSettingsTouch(touchX, touchY);
        } else if (currentPage == PAGE_IDLE) {
            // WO11 Task 3: any touch wakes instantly, straight to the
            // dashboard - no sub-regions, matching the order's "any touch
            // returns to dashboard instantly."
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> idle screen, waking to dashboard\n", touchX,
                            touchY);
            goToPage(PAGE_DASHBOARD);
        }
    }

    // WO11 Task 3: idle-entry check. Only arms FROM PAGE_DASHBOARD (CC's
    // judgment - never hijack an in-progress editor session just because
    // the user paused touching the screen) and only when XWC is not
    // connected (an active controller session should never be interrupted
    // by an idle screen). Display-layer only: reads currentPage/millis()/
    // xwcState, all already-read elsewhere in loop(); writes nothing
    // broadcast/slicer/failsafe/XWC-processing/config touches.
    if (currentPage == PAGE_DASHBOARD && xwcState != XWC_ACTIVE &&
        (millis() - lastTouchActivityMs) >= IDLE_TIMEOUT_MS) {
        goToPage(PAGE_IDLE);
    }

    if (currentPage == PAGE_DASHBOARD) {
        updateDashboard();
    } else if (currentPage == PAGE_STATS) {
        updateStatsUptime();
    } else if (currentPage == PAGE_IDLE) {
        // WO11 Task 3: lightweight periodic redraw (see
        // updateIdleAnimation()'s doc comment) plus the XWC-connect wake -
        // "if XWC connects while idling, wake to dashboard."
        updateIdleAnimation();
        if (xwcState == XWC_ACTIVE) {
            goToPage(PAGE_DASHBOARD);
        }
    } else if (currentPage == PAGE_EDIT_CAPTURE && !capturePromptActive) {
        // WO10 Step 2: capture-token detection runs every tick while
        // sitting on the capture page - but not while the leave-page
        // prompt is up, so a stick left resting past threshold can't
        // silently change the very capture the prompt is asking about.
        // Broadcast/motor response is untouched by this gate - that's
        // driven by processControllers()/processGamepad() above,
        // unconditionally, on every page (PM ruling: capture never
        // suppresses driving).
        checkCapture();
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}
