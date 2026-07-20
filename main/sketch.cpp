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
    PAGE_COUNT
};
static UiPage currentPage = PAGE_SPLASH;

// WO10 Step 1: editing context - selection state only (no pending
// values yet - that's a later step). selected_valid is the "nothing
// selected" sentinel: false until a port tap on PAGE_EDIT_PORTS
// completes a full hub+port selection. Cleared on every fresh editor
// entry from the dashboard (see the settings-button tap handler in
// loop()) and on Back from port select to hub select (see
// dispatchPortSelectTouch() - CC's choice: Back clears rather than
// persists, so hub select is always a clean slate on (re-)entry, never
// partially-stale).
struct EditContext {
    int selected_device;  // protocol device slot 0..2, meaningful only once selected_valid or mid-selection
    int selected_port;    // channel index 0..5 (MKH_PORT_A..F), meaningful only once selected_valid
    bool selected_valid;  // true only once BOTH device and port are confirmed
};
static EditContext editCtx = {-1, -1, false};

static void clearEditContext() {
    editCtx.selected_device = -1;
    editCtx.selected_port = -1;
    editCtx.selected_valid = false;
}

// Forward-declared so the hub/port select dispatch functions (defined
// before goToPage() itself, further down) can call it - goToPage()'s
// own definition and doc comment are unchanged, just declared early.
static void goToPage(UiPage page);

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
// through to the unchanged touchYToDeviceId() split, so every pixel
// outside this 30x24 header corner resolves exactly as it did in
// v0.9.0. Placed left of the (centered) title, clear of both the title
// text and the right-aligned CFG label.
static const int16_t SETTINGS_BTN_X = 2;
static const int16_t SETTINGS_BTN_Y = 2;
static const int16_t SETTINGS_BTN_W = 30;
static const int16_t SETTINGS_BTN_H = HEADER_H - 4;  // 24

static void drawSettingsButton() {
    display->fillRoundRect(SETTINGS_BTN_X, SETTINGS_BTN_Y, SETTINGS_BTN_W, SETTINGS_BTN_H, 4, COLOR_CARD_BG);
    display->drawRoundRect(SETTINGS_BTN_X, SETTINGS_BTN_Y, SETTINGS_BTN_W, SETTINGS_BTN_H, 4, COLOR_CARD_BORDER);
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(1);
    int16_t tx = SETTINGS_BTN_X + (SETTINGS_BTN_W - textWidth("SET", 1)) / 2;
    display->setCursor(tx, SETTINGS_BTN_Y + (SETTINGS_BTN_H - 8) / 2);
    display->print("SET");
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

// WO10 Step 0: editor page placeholders - CAPTURE and SETTINGS only as
// of WO10 Step 1 (SELECT was replaced by the custom hub-select page
// below; PORTS is new and never used this generic pattern). Page name +
// Back (+ Next, CAPTURE only) - no editor logic, no config reads/writes
// beyond CAPTURE's now-context-aware title. These are replaced wholesale
// by later WO10 steps; they exist only to prove the page-navigation
// corridor end-to-end.
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
// its exact prior behavior for CAPTURE/SETTINGS's Back/Next buttons.
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
// dispatchHubSelectTouch() below) - only PAGE_EDIT_CAPTURE still has a
// Next button (-> PAGE_EDIT_SETTINGS).
static bool pageHasNext(UiPage page) {
    return page == PAGE_EDIT_CAPTURE;
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

    drawButtonAt(SELECT_BACK_X, SELECT_BACK_Y, SELECT_BACK_W, SELECT_BACK_H, "< BACK");
}

// Hub select's own hit zones - separate from dispatchEditorTouch()
// (never consulted on the same tap) and from the dashboard's
// touchYToDeviceId() split (different page, different geometry
// entirely - this is not a reuse of that logic).
static void dispatchHubSelectTouch(int16_t touchX, int16_t touchY) {
    bool inBack = touchX >= SELECT_BACK_X && touchX < SELECT_BACK_X + SELECT_BACK_W && touchY >= SELECT_BACK_Y &&
                  touchY < SELECT_BACK_Y + SELECT_BACK_H;
    if (inBack) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> hub select BACK -> dashboard\n", touchX, touchY);
        clearEditContext();
        goToPage(PAGE_DASHBOARD);
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
}

// Port select's own hit zones - same separation rationale as
// dispatchHubSelectTouch() above.
static void dispatchPortSelectTouch(int16_t touchX, int16_t touchY) {
    bool inBack = touchX >= SELECT_BACK_X && touchX < SELECT_BACK_X + SELECT_BACK_W && touchY >= SELECT_BACK_Y &&
                  touchY < SELECT_BACK_Y + SELECT_BACK_H;
    if (inBack) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> port select BACK -> hub select\n", touchX, touchY);
        // WO10 Step 1 design choice (reported per the order): Back
        // clears the full context rather than persisting the hub
        // selection - hub select is always a clean slate on re-entry,
        // never partially-stale.
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
            editCtx.selected_port = p;
            editCtx.selected_valid = true;
            goToPage(PAGE_EDIT_CAPTURE);
            return;
        }
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
            // WO10 Step 1: title now proves the editing context flows
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
            drawEditorPlaceholder(title, page);
            break;
        }
        case PAGE_EDIT_SETTINGS:
            drawEditorPlaceholder("EDIT: PORT SETTINGS", page);
            break;
        case PAGE_COUNT:
            break;
    }
}

// WO10 Step 1: PAGE_EDIT_SELECT no longer has a case here - it's never
// passed in (dispatchHubSelectTouch() handles its own Back). CAPTURE's
// Back target moved from PAGE_EDIT_SELECT to PAGE_EDIT_PORTS, since
// PORTS is now the immediate predecessor in the corridor.
static UiPage editorBackTarget(UiPage page) {
    switch (page) {
        case PAGE_EDIT_CAPTURE:
            return PAGE_EDIT_PORTS;
        case PAGE_EDIT_SETTINGS:
            return PAGE_EDIT_CAPTURE;
        default:
            return PAGE_DASHBOARD;
    }
}

static UiPage editorNextTarget(UiPage page) {
    switch (page) {
        case PAGE_EDIT_CAPTURE:
            return PAGE_EDIT_SETTINGS;
        default:
            return page;  // no Next from SETTINGS - button isn't drawn there
    }
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
    // dispatch functions now (not dispatchEditorTouch() - that's CAPTURE/
    // SETTINGS only).
    int16_t touchX, touchY;
    if (mkh_touch_poll(&touchX, &touchY)) {
        if (currentPage == PAGE_DASHBOARD) {
            bool inSettingsBtn = touchX >= SETTINGS_BTN_X && touchX < SETTINGS_BTN_X + SETTINGS_BTN_W &&
                                  touchY >= SETTINGS_BTN_Y && touchY < SETTINGS_BTN_Y + SETTINGS_BTN_H;
            if (inSettingsBtn) {
                Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings button -> PAGE_EDIT_SELECT\n", touchX,
                                touchY);
                // WO10 Step 1: every fresh entry from the dashboard
                // starts the editing context clean - see EditContext's
                // doc comment for the other clearing path (Back from
                // port select).
                clearEditContext();
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
        } else {
            // CAPTURE, SETTINGS - still the generic placeholder pattern.
            dispatchEditorTouch(touchX, touchY);
        }
    }

    if (currentPage == PAGE_DASHBOARD) {
        updateDashboard();
    } else if (currentPage == PAGE_STATS) {
        updateStatsUptime();
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
}
