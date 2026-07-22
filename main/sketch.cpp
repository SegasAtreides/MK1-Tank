// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <math.h>

#include <Arduino.h>
#include <Bluepad32.h>
#include <Arduino_GFX_Library.h>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_system.h"

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
//
// Bugfix (version-string audit): was a hand-maintained literal
// ("v0.11.0", last bumped in WO11 and never updated across five
// subsequent releases - the exact bench finding this fix addresses).
// Now derived at build time from the actual git tag/commit via
// extract_version.py (see platformio.ini's extra_scripts) - MK1_GIT_VERSION
// is injected as a CPPDEFINE; the #ifndef fallback only matters if
// someone builds without that script running (e.g. a raw `cmake`
// invocation bypassing PlatformIO), so it's deliberately NOT a plausible-
// looking version string - "unbuilt" can't be mistaken for a real release.
#ifndef MK1_GIT_VERSION
#define MK1_GIT_VERSION "unbuilt"
#endif
#define MK1_FW_VERSION MK1_GIT_VERSION

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

// WO13 Task 3: battery indicator. Display-layer + one ADC read only -
// zero contact with broadcast/slicer/failsafe/input processing, per the
// order; nothing here is read by, or writes to, anything outside this
// block and the two call sites in loop()/updateDashboard()/initDashboard()
// below.
//
// GPIO5 verified from the OFFICIAL ESP32-S3-Touch-LCD-2 schematic
// (Waveshare wiki's own SchDoc.pdf, "BAT" section, resistors R19/R20) -
// deliberately not assumed from the sibling ESP32-S3-Touch-LCD-1.28
// (round) model, which documents GPIO1 for its own battery divider;
// GPIO1 on THIS board is already TFT_BL (see the display pin block
// above), so carrying that sibling's pin over would have been wrong, not
// just unverified. R19=200K (VBAT side), R20=100K (GND side) form the
// divider, so pin voltage = VBAT * R20/(R19+R20) = VBAT/3.
#define MKH_BATTERY_ADC_PIN 5
static const float MKH_BATTERY_DIVIDER_RATIO = 3.0f;  // (R19+R20)/R20 = 300K/100K

// Order: "slow timer (~1s or slower)".
static const uint32_t MKH_BATTERY_READ_INTERVAL_MS = 1000;

// Order: "smooth the reading (simple averaging) so the indicator doesn't
// flicker with load spikes" - a single-state exponential moving average
// (no sample buffer needed). alpha=0.2 settles within ~5 reads (~5s at
// the interval above) - slow enough to reject one spiky sample under a
// motor load transient, fast enough to still track a genuine drain
// within a few seconds.
static const float MKH_BATTERY_EMA_ALPHA = 0.2f;

// Order's own suggested thresholds, reported as chosen, not altered:
// ~3.7V low (a single-cell Li-ion's "getting low" knee, well above hard
// cutoff), ~3.5V critical under load (headroom above the ~3.0-3.2V range
// most single-cell protection circuits cut at, accounting for sag under
// the tank's motor load and the divider/ADC's own measurement noise).
static const float MKH_BATTERY_LOW_V = 3.7f;
static const float MKH_BATTERY_CRITICAL_V = 3.5f;

enum BatteryState { BATTERY_GOOD, BATTERY_LOW, BATTERY_CRITICAL };

static float batterySmoothedV = 0.0f;
static bool batteryHasReading = false;
static uint32_t lastBatteryReadMs = 0;

static BatteryState batteryStateFor(float volts) {
    if (volts <= MKH_BATTERY_CRITICAL_V)
        return BATTERY_CRITICAL;
    if (volts <= MKH_BATTERY_LOW_V)
        return BATTERY_LOW;
    return BATTERY_GOOD;
}

static const char* batteryStateLabel(BatteryState s) {
    switch (s) {
        case BATTERY_CRITICAL:
            return "CRITICAL";
        case BATTERY_LOW:
            return "LOW";
        default:
            return "GOOD";
    }
}

// Called from loop() every tick, gated internally to the ~1s interval
// above - reads the ADC, smooths it, and logs the raw+smoothed value
// alongside the resulting state (self-test requirement: "battery ADC
// value visible in log alongside the displayed state"). Runs regardless
// of currentPage (a background reading, like the idle-screen timer's
// millis() check) - only the DRAWING (drawBatteryBadge(), with the
// dashboard's other draw calls) is dashboard-only.
static void updateBatteryReading() {
    uint32_t now = millis();
    if (batteryHasReading && (now - lastBatteryReadMs < MKH_BATTERY_READ_INTERVAL_MS))
        return;
    lastBatteryReadMs = now;

    uint32_t pinMv = analogReadMilliVolts(MKH_BATTERY_ADC_PIN);
    float packV = (pinMv / 1000.0f) * MKH_BATTERY_DIVIDER_RATIO;

    if (!batteryHasReading) {
        batterySmoothedV = packV;  // seed on the very first sample - nothing to average against yet
        batteryHasReading = true;
    } else {
        batterySmoothedV = batterySmoothedV * (1.0f - MKH_BATTERY_EMA_ALPHA) + packV * MKH_BATTERY_EMA_ALPHA;
    }

    Console.printf("MK1 Battery: pin=%lumV pack=%.2fV smoothed=%.2fV state=%s\n", (unsigned long)pinMv, packV,
                    batterySmoothedV, batteryStateLabel(batteryStateFor(batterySmoothedV)));
}

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

// WO16: separate from lastTouchActivityMs above - marks when THIS idle-
// screen visit began (set in drawIdlePage()), so the auto-sleep countdown
// ("dashboard -> 30s -> idle screen -> N min -> deep sleep", per the
// order's own chain) measures time spent ON the idle screen, not time
// since the last touch (which already includes the 30s it took to get
// here).
static uint32_t idleEnteredMs = 0;

// WO16 Phase 1 (wake-source investigation - schematic: Waveshare
// ESP32-S3-Touch-LCD-2 official SchDoc.pdf, net list). Touch IRQ (TP_INT,
// net NLTP0INT) traces to GPIO46 - NOT RTC-capable: the ESP32-S3's RTC/
// LP-IO domain covers only GPIO0-21, and GPIO46 has no path to the RTC
// controller, so it cannot be used with esp_sleep_enable_ext0/ext1_wakeup.
// This is an SoC-level constraint, not a wiring choice - option (a) from
// the order is infeasible regardless of firmware. The onboard BOOT key
// (Key2, schematic net NLBOOT) traces to GPIO0 - one of the RTC-capable
// pins, idling HIGH via the board's own pull-up (the same strap the SoC
// itself samples at reset to pick "run from flash") and pulled to GND by
// Key2 on press. That's option (b), and it's what this build uses.
//
// GPIO0 is ALSO this project's TFT_RST (see the display pin block above,
// and the schematic - NLIO0/NLLCD0RST/NLBOOT are literally the same net):
// the display driver drives it as a push-pull output, left HIGH, after
// display->begin()'s reset pulse (see initDisplay()). enterDeepSleep()
// below explicitly hands the pin back to plain INPUT before arming ext0
// wake, so the RTC controller - not the display driver's output stage -
// owns the pad while asleep. Harmless: the backlight is off by then, so
// nothing the panel's own GRAM/reset state does is visible regardless.
//
// Consequence worth flagging plainly: the order's target "tap-anywhere-
// on-dark-screen" wake UX was contingent on option (a) working. It
// doesn't, so the real wake gesture is a press of the physical BOOT key,
// not a screen tap.
#define MKH_WAKE_GPIO GPIO_NUM_0

// Order: "suggest 10 min, report chosen value" - kept at the suggestion.
static const uint32_t AUTO_SLEEP_TIMEOUT_MS = 10UL * 60UL * 1000UL;

// WO16: how long the idle screen's SLEEP control must be held (not just
// tapped) before it triggers deep sleep - long enough that the same quick
// tap which would otherwise wake the dashboard can never accidentally
// trigger it too.
static const uint32_t SLEEP_HOLD_MS = 1500;

// WO16: idle-screen SLEEP control hold-tracking. Set when a NEW press
// (mkh_touch_poll()'s edge-triggered return) lands inside the SLEEP hit-
// zone while idling; cleared on release, on a fresh idle-page entry, or
// once consumed by an actual sleep-entry decision - all in loop() below.
static bool idleSleepHoldActive = false;
static uint32_t idleSleepHoldStartMs = 0;

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
    // WO13: which binding of the selected port the settings page is
    // showing/editing. In range [0, binding_count-1] when viewing an
    // existing binding (port-select's direct-to-settings entry, or
    // Prev/Next); == binding_count (one past the end) when a capture is
    // about to be ADDED as a brand new binding rather than replacing one.
    int editing_binding_index;
};
static EditContext editCtx = {-1, -1, false, MKH_INPUT_NONE, false, 0};

// WO13: which page settings' BACK (and its KEEP/DISCARD prompt) returns
// to - true when this settings visit was reached FROM capture (a fresh
// binding just captured, or a REPLACE/ADD choice made), matching the
// pre-WO13 "settings always backs into capture" behavior; false when
// reached directly from port-select to VIEW an already-occupied port's
// existing binding(s), where there is no in-progress capture to walk
// back into - BACK in that case returns to port-select instead. Set
// explicitly at every goToPage(PAGE_EDIT_SETTINGS) call site rather than
// inferred from editCtx state, to keep the two entry paths unambiguous.
// Declared here (not with the rest of the settings-page statics further
// down) since port-select and capture, both earlier in the file, need
// to set it too.
static bool settingsBackToCapture = false;

// WO10-FINAL, extended WO13: pending session - accumulates committed
// per-port edits ACROSS ports (order requirement), independent of
// editCtx's single-port focus. A port becomes dirty=true the moment a
// real change commits for it - see goToPage()'s PAGE_EDIT_SETTINGS case
// and dispatchSettingsTouch()'s KEEP/SAVE handlers. Never touched by
// mkh_config_get()/the live table directly; only
// pushPendingSessionToLiveTable() (Test-ON, Save) writes it out.
//
// WO13: holds the port's WHOLE working binding list (not one binding) -
// the same array-of-bindings shape as mkh_port_config_t, so a dirty
// port's pendingEdits entry is the single, complete, authoritative
// working copy for every binding on it at once (mirrors how the
// pre-WO13 struct held the port's one binding directly).
struct PendingPortEdit {
    bool dirty;
    mkh_binding_t bindings[MKH_MAX_BINDINGS_PER_PORT];
    int binding_count;
};
static PendingPortEdit pendingEdits[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS];

// WO13: resolves a port's CURRENT effective binding list - the pending
// working copy if this session has already touched it, else the live
// resolved table. Every read site that used to dereference
// mkh_config_get() directly for editor purposes now goes through one of
// these two, so "was this port touched this session" is decided in
// exactly one place.
static int effectiveBindingCount(int dev, int port) {
    if (pendingEdits[dev][port].dirty)
        return pendingEdits[dev][port].binding_count;
    const mkh_port_config_t* cfg = mkh_config_get(dev, port);
    return cfg ? cfg->binding_count : 0;
}
static mkh_binding_t effectiveBinding(int dev, int port, int idx) {
    if (pendingEdits[dev][port].dirty)
        return pendingEdits[dev][port].bindings[idx];
    const mkh_port_config_t* cfg = mkh_config_get(dev, port);
    return cfg->bindings[idx];
}

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

// WO11 Task 4 (hardening), WO13: forward-declared so
// pushPendingSessionToLiveTable() below can clear a port's runtime latch
// state at push time - defined with the rest of the latch-state
// machinery further down (see its doc comment there for why). WO13:
// latch state is now per-BINDING (a port can have several bindings each
// independently latchable), so this clears every binding slot on the
// port, not just one value.
static void clearLatchStateForPort(int dev, int port);
// WO13 Task 4: forward-declared so dispatchHubSelectTouch()'s RESET ALL
// confirm (defined earlier in the file than the rest of the latch-state
// machinery) can clear every port's latch state too.
static void clearAllLatchState();

// Writes every dirty pending edit into the live mkh_config table - the
// one place Test-ON and Save actually touch the live table (through
// mkh_config_set_port_bindings(), never the protocol/broadcast layer
// directly).
static void pushPendingSessionToLiveTable() {
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
            if (pendingEdits[d][p].dirty) {
                mkh_config_set_port_bindings(d, p, pendingEdits[d][p].bindings, pendingEdits[d][p].binding_count);
                // WO11 Task 4 / WO13: fresh unlatched start on every push
                // for EVERY binding slot on the port - see
                // clearLatchStateForPort()'s doc comment for the stale-
                // latch-carryover bug this closes, now generalized across
                // bindings (a port reassigned from one latched binding to
                // a different latched binding must not resume "on").
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

// WO13 Task 3: badge sits in the UPTIME row's own middle space - the
// row's label ("UPTIME", left) and its HH:MM:SS value (right) leave a
// wide unused gap between them, so this needed no new row / no
// dashboard re-layout. Purely informational (like the CFG header label
// or the XWC status dot) - not a touch target, so the order's 40px/20px
// interactive-target rule doesn't apply here.
static const int16_t BATTERY_BADGE_X = 106;
static const int16_t BATTERY_BADGE_W = 96;
static BatteryState lastBatteryStateDrawn = BATTERY_GOOD;
static uint32_t lastBatteryBlinkPhaseDrawn = 0xFFFFFFFF;

static void drawBatteryBadge(bool force) {
    if (!batteryHasReading)
        return;  // first ADC read hasn't landed yet - nothing to show
    BatteryState state = batteryStateFor(batterySmoothedV);
    // Order: "a visible warning treatment at critical" - a blinking red
    // background, driven purely off millis() (no new timer/coupling).
    // Redraw on a state change, on every blink-phase flip while
    // critical, or when forced (fresh dashboard entry).
    uint32_t blinkPhase = (millis() / 500) % 2;
    bool blinkChanged = (state == BATTERY_CRITICAL) && (blinkPhase != lastBatteryBlinkPhaseDrawn);
    if (!force && state == lastBatteryStateDrawn && !blinkChanged)
        return;
    lastBatteryStateDrawn = state;
    lastBatteryBlinkPhaseDrawn = blinkPhase;

    int16_t y = rowTop(ROW_UPTIME);
    uint16_t fg = COLOR_GREEN;
    uint16_t bg = COLOR_CARD_BG;
    if (state == BATTERY_LOW) {
        fg = COLOR_YELLOW;
    } else if (state == BATTERY_CRITICAL) {
        fg = COLOR_LABEL;
        bg = (blinkPhase == 0) ? COLOR_RED : COLOR_CARD_BG;
    }

    display->fillRect(BATTERY_BADGE_X, y + 4, BATTERY_BADGE_W, ROW_H - 8, bg);
    display->drawRect(BATTERY_BADGE_X, y + 4, BATTERY_BADGE_W, ROW_H - 8, COLOR_CARD_BORDER);
    char label[16];
    snprintf(label, sizeof(label), "BAT %.1fV", batterySmoothedV);
    display->setTextColor(fg);
    display->setTextSize(1);
    int16_t tx = BATTERY_BADGE_X + (BATTERY_BADGE_W - textWidth(label, 1)) / 2;
    display->setCursor(tx, y + (ROW_H - 8) / 2);
    display->print(label);
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
    drawBatteryBadge(true);
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
    drawBatteryBadge(false);
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

// WO16: SLEEP control - sits clear of the tank/track art (which occupies
// down to y=196, see drawIdleTank()) and away from the right/bottom
// screen edges.
static const int16_t IDLE_SLEEP_BTN_X = 210;
static const int16_t IDLE_SLEEP_BTN_Y = 198;
static const int16_t IDLE_SLEEP_BTN_W = 96;
static const int16_t IDLE_SLEEP_BTN_H = 38;

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

    // WO16: subtle, static one-line hint for the two idle-screen gestures
    // - ASCII hyphen, not a middle-dot, since the display's built-in
    // bitmap font isn't guaranteed to have that glyph (same call as the
    // WO10 capture-page title).
    const char* hint = "hold SLEEP to power down - press BOOT key to wake";
    display->setTextColor(COLOR_CARD_BORDER);
    display->setTextSize(1);
    display->setCursor((SCR_W - textWidth(hint, 1)) / 2, 44);
    display->print(hint);

    drawIdleTank();
    idleAnimFrame = 0;
    idleAnimLastTickMs = millis();
    drawIdlePuffFrame(idleAnimFrame);

    drawButtonAt(IDLE_SLEEP_BTN_X, IDLE_SLEEP_BTN_Y, IDLE_SLEEP_BTN_W, IDLE_SLEEP_BTN_H, "SLEEP");

    // WO16: fresh state on every idle-page entry - the auto-sleep clock
    // restarts, and any in-progress hold from a previous idle visit
    // (shouldn't be possible to carry over, but this makes it structurally
    // impossible rather than relying on that) is cleared.
    idleEnteredMs = millis();
    idleSleepHoldActive = false;
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
// WO13: left-aligned (was centered, X=90/W=140) - hub select's row now
// carries a second button (RESET ALL, EXIT_PROMPT_RIGHT_X/W below,
// unused by port select) and both pages share this same row geometry,
// same two-button-row pattern the capture/exit-prompt overlays already
// use. Purely a repositioning; port select's Back is otherwise unaffected.
static const int16_t SELECT_BACK_X = 20;
static const int16_t SELECT_BACK_W = 120;
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
// WO13 Task 4: reset-all's own confirm prompt - see
// drawResetAllPromptOverlay()/dispatchHubSelectTouch().
static bool resetAllPromptActive = false;

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
    resetAllPromptActive = false;
    drawButtonAt(SELECT_BACK_X, SELECT_BACK_Y, SELECT_BACK_W, SELECT_BACK_H, "< BACK");
    // WO13 Task 4: reset-all lives on hub select - the outermost editor
    // page, same precedent as Test (a global action, not scoped to one
    // hub/port). Shares the row's right slot with port select's (empty)
    // right slot - see SELECT_BACK_X/W's doc comment.
    drawButtonAt(EXIT_PROMPT_RIGHT_X, SELECT_BACK_Y, EXIT_PROMPT_RIGHT_W, SELECT_BACK_H, "RESET ALL");
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

// WO13 Task 4: reset-all confirm prompt - repaints over the hub-button
// area (same precedent drawExitPromptOverlay() above already
// established: hub select isn't interactive while any of its prompts
// are up, so temporarily overwriting button visuals there is fine).
static void drawResetAllPromptOverlay() {
    display->fillRect(0, HUB_BTN_Y0, SCR_W, SELECT_BACK_Y - HUB_BTN_Y0 - 4, COLOR_BG);
    const char* q = "WIPE ALL BINDINGS?";
    display->setTextColor(COLOR_RED);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(q, 2)) / 2, HUB_BTN_Y0 + 30);
    display->print(q);
    const char* warn = "Every port loses every binding. Saved immediately.";
    display->setTextColor(COLOR_YELLOW);
    display->setTextSize(1);
    display->setCursor((SCR_W - textWidth(warn, 1)) / 2, HUB_BTN_Y0 + 60);
    display->print(warn);

    display->fillRect(0, SELECT_BACK_Y - 4, SCR_W, SELECT_BACK_H + 8, COLOR_BG);
    drawButtonAt(EXIT_PROMPT_LEFT_X, SELECT_BACK_Y, EXIT_PROMPT_LEFT_W, SELECT_BACK_H, "CONFIRM");
    drawButtonAt(EXIT_PROMPT_RIGHT_X, SELECT_BACK_Y, EXIT_PROMPT_RIGHT_W, SELECT_BACK_H, "CANCEL");
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

    // WO13 Task 4: confirm/cancel for RESET ALL.
    if (resetAllPromptActive) {
        bool inConfirm = touchX >= EXIT_PROMPT_LEFT_X && touchX < EXIT_PROMPT_LEFT_X + EXIT_PROMPT_LEFT_W &&
                         touchY >= SELECT_BACK_Y && touchY < SELECT_BACK_Y + SELECT_BACK_H;
        bool inCancel = touchX >= EXIT_PROMPT_RIGHT_X && touchX < EXIT_PROMPT_RIGHT_X + EXIT_PROMPT_RIGHT_W &&
                        touchY >= SELECT_BACK_Y && touchY < SELECT_BACK_Y + SELECT_BACK_H;
        if (inConfirm) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> RESET ALL confirmed\n", touchX, touchY);
            // Direct to the live table and straight through the normal
            // save path (order: "writes through the normal save path")
            // - there is no pending-session staging step for a wipe;
            // any pending edits from earlier in this session are
            // superseded and dropped along with everything else.
            for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
                for (int p = 0; p < MKH_MK6_NUM_CHANNELS; p++) {
                    mkh_config_set_port_bindings(d, p, nullptr, 0);
                }
            }
            clearAllLatchState();
            bool ok = mkh_storage_save_config();
            Console.printf("MK1 Editor: RESET ALL save %s\n", ok ? "succeeded" : "FAILED");
            resetAllPromptActive = false;
            clearEditorSession();
            goToPage(PAGE_DASHBOARD);
        } else if (inCancel) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> RESET ALL cancelled\n", touchX, touchY);
            resetAllPromptActive = false;
            drawHubSelect();
        }
        return;
    }

    if (testToggleHit(touchX, touchY)) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> Test toggle (hub select)\n", touchX, touchY);
        handleTestToggleTap();
        return;
    }

    bool inResetAll = touchX >= EXIT_PROMPT_RIGHT_X && touchX < EXIT_PROMPT_RIGHT_X + EXIT_PROMPT_RIGHT_W &&
                      touchY >= SELECT_BACK_Y && touchY < SELECT_BACK_Y + SELECT_BACK_H;
    if (inResetAll) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> hub select RESET ALL, prompting\n", touchX, touchY);
        resetAllPromptActive = true;
        drawResetAllPromptOverlay();
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

    // Informational-only occupied-port tag, read from the EFFECTIVE
    // (pending-aware) binding list so it reflects this session's edits,
    // not just the live table - never written to. WO13: generalized off
    // the old hardcoded LSNS/RSNS-only tag (a pre-full-token-universe
    // leftover) to show binding[0]'s real token name, plus a "+N" suffix
    // when the port carries more than one binding.
    int count = effectiveBindingCount(editCtx.selected_device, port);
    if (count > 0) {
        mkh_binding_t first = effectiveBinding(editCtx.selected_device, port, 0);
        char tag[16];
        if (count > 1) {
            snprintf(tag, sizeof(tag), "%s +%d", mkh_config_input_token_name(first.input), count - 1);
        } else {
            snprintf(tag, sizeof(tag), "%s", mkh_config_input_token_name(first.input));
        }
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

            // WO13: an OCCUPIED port (has >=1 binding, pending-aware) goes
            // straight to settings to VIEW/edit its existing binding(s) -
            // capture is only the entry point for assigning a port's
            // FIRST binding. Add binding (from settings) is the other,
            // explicit way back into capture for an already-occupied port.
            int count = effectiveBindingCount(editCtx.selected_device, p);
            if (count > 0) {
                Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> port select MKH%d port %c -> settings "
                                "(occupied, %d binding%s)\n",
                                touchX, touchY, mkh_device_app_number(editCtx.selected_device), mkh_port_letter(p),
                                count, count == 1 ? "" : "s");
                editCtx.editing_binding_index = 0;
                mkh_binding_t first = effectiveBinding(editCtx.selected_device, p, 0);
                editCtx.captured_input = first.input;
                editCtx.capture_valid = true;  // real existing binding - input is never NONE here
                settingsBackToCapture = false;
                goToPage(PAGE_EDIT_SETTINGS);
            } else {
                Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> port select MKH%d port %c -> capture\n",
                                touchX, touchY, mkh_device_app_number(editCtx.selected_device), mkh_port_letter(p));
                editCtx.editing_binding_index = 0;
                goToPage(PAGE_EDIT_CAPTURE);
            }
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

// WO13: second in-page prompt, same pattern/rationale as
// capturePromptActive above - shown instead of going straight to
// settings when NEXT is tapped on an ALREADY-OCCUPIED port (the port
// had >=1 binding, pending-aware, when capture was entered). Offers
// REPLACE (overwrite the binding at editCtx.editing_binding_index - the
// one that was focused when Add binding was tapped from settings) or ADD
// (append as a brand new binding). Never shown for a fresh/unoccupied
// port - there's nothing to replace, so NEXT goes straight to settings
// exactly as before WO13.
static bool captureReplaceAddPromptActive = false;

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

// WO13: shown instead of drawCapturePromptOverlay() when NEXT is tapped
// on an already-occupied port - see captureReplaceAddPromptActive's doc
// comment.
static void drawCaptureReplaceAddOverlay() {
    const char* q = "REPLACE OR ADD?";
    display->fillRect(0, CAPTURE_PROMPT_Y, SCR_W, CAPTURE_PROMPT_H, COLOR_BG);
    display->setTextColor(COLOR_YELLOW);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(q, 2)) / 2, CAPTURE_PROMPT_Y + 6);
    display->print(q);

    display->fillRect(0, CAPTURE_BTN_Y - 4, SCR_W, CAPTURE_BTN_H + 8, COLOR_BG);
    drawButtonAt(CAPTURE_LEFT_X, CAPTURE_BTN_Y, CAPTURE_LEFT_W, CAPTURE_BTN_H, "REPLACE");
    drawButtonAt(CAPTURE_RIGHT_X, CAPTURE_BTN_Y, CAPTURE_RIGHT_W, CAPTURE_BTN_H, "ADD");
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

    // WO13: REPLACE/ADD prompt (occupied port only) - see
    // captureReplaceAddPromptActive's doc comment.
    if (captureReplaceAddPromptActive) {
        if (inLeftBtn) {
            // REPLACE: editCtx.editing_binding_index already names the
            // binding to overwrite (carried over from settings' Add
            // binding tap, or 0 if capture was somehow re-entered on an
            // occupied port) - goToPage()'s PAGE_EDIT_SETTINGS case does
            // the actual commit using that index.
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture REPLACE binding[%d] -> settings\n", touchX,
                            touchY, editCtx.editing_binding_index);
            captureReplaceAddPromptActive = false;
            settingsBackToCapture = true;
            goToPage(PAGE_EDIT_SETTINGS);
        } else if (inRightBtn) {
            // ADD: target the next free slot (current effective count) -
            // goToPage()'s PAGE_EDIT_SETTINGS case appends there.
            int count = effectiveBindingCount(editCtx.selected_device, editCtx.selected_port);
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture ADD as binding[%d] -> settings\n", touchX,
                            touchY, count);
            captureReplaceAddPromptActive = false;
            editCtx.editing_binding_index = count;
            settingsBackToCapture = true;
            goToPage(PAGE_EDIT_SETTINGS);
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
        // WO13: an occupied port offers REPLACE/ADD instead of going
        // straight to settings - a fresh/unoccupied port behaves exactly
        // as before (NEXT never prompts, implicit keep as binding 0).
        int count = effectiveBindingCount(editCtx.selected_device, editCtx.selected_port);
        if (count > 0) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture NEXT, prompting (occupied port)\n", touchX,
                            touchY);
            captureReplaceAddPromptActive = true;
            drawCaptureReplaceAddOverlay();
        } else {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> capture NEXT -> settings\n", touchX, touchY);
            editCtx.editing_binding_index = 0;
            settingsBackToCapture = true;
            goToPage(PAGE_EDIT_SETTINGS);
        }
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
// settingsBackToCapture is declared earlier, alongside editCtx - see its
// doc comment there for why (port-select and capture, both earlier in
// the file, need to set it too).

// WO11 Task 2 (settings-page polish): INVERT and MODE combined onto one
// 40px row (left/right split at SPAGE_TOGGLE_MID_X), and the MAX label
// folded into the slider's own 40px band - both were 28px-tall separate
// rows before, short of the order's 40px minimum (flagged in the
// WO10-FINAL rev B report). Combining rows rather than shrinking gaps
// elsewhere keeps every remaining gap comfortable within the same
// 240px page.
//
// WO13 re-layout: fitting Add/Remove binding onto an already-full 240px
// page (CC's judgment, reported) - the standalone "CURVE: LINEAR" line
// is dropped (folded into the MAX row's own label instead; curve was
// already fixed/non-selectable in v1, purely decorative) to free a full
// 40px row. Five 40px rows total, each with >=20px screen-edge margins:
// title (0-40, informational text only - see below), invert/mode
// (44-84), max/slider+nudge (88-128), binding actions (132-172),
// back/reset/save (180-220, UNCHANGED from WO11 - still 20px to the
// 240px bottom edge).
//
// PM bench feedback: Prev/Next was originally corner arrows on the title
// row (hit-zone-exceeds-visual, same precedent as the dashboard's
// settings button) - the RIGHT corner turned out to collide with
// TEST_TOGGLE, which sits in that exact corner on every editor page
// (drawTestToggle() is called on capture/settings/hub/port select
// alike, unlike SET which is dashboard-only). Fixed by moving Prev/Next
// off the corners entirely, into the binding-actions row alongside
// Add/Remove (4 buttons, none within 60px of TEST_TOGGLE's y=2-26
// footprint) - the title row is now purely informational text, no
// touch zones of its own.
static const int16_t SPAGE_NAV_Y = 0;
static const int16_t SPAGE_NAV_H = 40;
static const int16_t SPAGE_TOGGLE_ROW_Y = 44;
static const int16_t SPAGE_TOGGLE_ROW_H = 40;  // meets the 40px minimum
static const int16_t SPAGE_TOGGLE_MID_X = 160;  // left = invert, right = mode
static const int16_t SPAGE_INVERT_TRACK_X = 14;  // relative to x=0, left half only
static const int16_t SPAGE_MAXSLIDER_Y = 88;
static const int16_t SPAGE_MAXSLIDER_H = 40;  // meets the 40px minimum; label lives inside this same band
// WO13 Task 2: nudge buttons flank the slider, each a full 40x40 target
// (short dimension >= 40px in BOTH axes, not just height) - the slider
// track/drag zone fills the space between them.
static const int16_t SPAGE_NUDGE_W = 40;
static const int16_t SPAGE_NUDGE_MINUS_X = 20;   // left edge 20, 20px from screen edge (0)
static const int16_t SPAGE_SLIDER_X = SPAGE_NUDGE_MINUS_X + SPAGE_NUDGE_W + 8;   // 68
static const int16_t SPAGE_NUDGE_PLUS_X = 260;    // right edge 300, 20px from screen edge (320)
static const int16_t SPAGE_SLIDER_W = SPAGE_NUDGE_PLUS_X - 8 - SPAGE_SLIDER_X;   // 184
static const int16_t SPAGE_SLIDER_TRACK_H = 12;
static const int16_t SPAGE_BINDING_ACTIONS_Y = 132;
static const int16_t SPAGE_BINDING_ACTIONS_H = 40;
// PREV | ADD | REMOVE | NEXT, left to right - all >=40px tall (this
// row's own height), PREV/NEXT narrower since they're single-glyph.
// 20 + 50 + 10 + 70 + 10 + 70 + 10 + 60 = 300, right edge at 300, 20px
// margin to the 320px screen edge - matches every other row's margin.
static const int16_t SPAGE_PREV_X = 20;
static const int16_t SPAGE_PREV_W = 50;
static const int16_t SPAGE_ADD_X = 80;
static const int16_t SPAGE_ADD_W = 70;
static const int16_t SPAGE_REMOVE_X = 160;
static const int16_t SPAGE_REMOVE_W = 70;
static const int16_t SPAGE_NEXT_X = 240;
static const int16_t SPAGE_NEXT_W = 60;

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

// WO13 Task 2: +/- nudge buttons flank the slider; a drag release within
// 2% of either end snaps exactly to 0/100 (see applyMaxEndSnap() and its
// use in both the drag handler and the nudge handlers below - a nudge
// can walk INTO the snap band same as a drag can land in it).
#define SPAGE_MAX_SNAP_BAND_PERCENT 2

static uint8_t applyMaxEndSnap(int32_t percent) {
    if (percent < 0)
        percent = 0;
    if (percent > 100)
        percent = 100;
    if (percent <= SPAGE_MAX_SNAP_BAND_PERCENT)
        return 0;
    if (percent >= 100 - SPAGE_MAX_SNAP_BAND_PERCENT)
        return 100;
    return (uint8_t)percent;
}

static void drawSettingsMaxRow() {
    display->fillRect(0, SPAGE_MAXSLIDER_Y, SCR_W, SPAGE_MAXSLIDER_H, COLOR_BG);

    // Nudge buttons - full 40x40 targets, same visual language as the
    // page's other buttons (drawButtonAt: bordered box, centered label).
    drawButtonAt(SPAGE_NUDGE_MINUS_X, SPAGE_MAXSLIDER_Y, SPAGE_NUDGE_W, SPAGE_MAXSLIDER_H, "-");
    drawButtonAt(SPAGE_NUDGE_PLUS_X, SPAGE_MAXSLIDER_Y, SPAGE_NUDGE_W, SPAGE_MAXSLIDER_H, "+");

    char label[20];
    snprintf(label, sizeof(label), "MAX:%u%% (linear)", (unsigned)settingsWorkingMax);
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

// WO13 Task 1: Add binding is disabled (still drawn, but a no-op tap)
// once the port hits MKH_MAX_BINDINGS_PER_PORT - the cap is soft/
// raisable but still enforced here, not just in the parser. Remove
// binding has no floor - removing the last binding leaves the port
// unassigned, same "no exclusivity rules" simplicity as the rest of
// Task 1. PREV/NEXT live here too (see SPAGE_NAV_Y's doc comment for
// why they moved off the title row's corners) - greyed out (drawn, but
// a no-op tap) at whichever end of the binding list is already showing,
// and entirely absent when the port has only one binding (nothing to
// page between).
static void drawSettingsBindingActionsRow() {
    display->fillRect(0, SPAGE_BINDING_ACTIONS_Y, SCR_W, SPAGE_BINDING_ACTIONS_H, COLOR_BG);
    int dev = editCtx.selected_device;
    int port = editCtx.selected_port;
    int idx = editCtx.editing_binding_index;
    int count = pendingEdits[dev][port].dirty ? pendingEdits[dev][port].binding_count
                                               : effectiveBindingCount(dev, port);
    bool canAdd = count < MKH_MAX_BINDINGS_PER_PORT;

    if (count > 1) {
        drawButtonAt(SPAGE_PREV_X, SPAGE_BINDING_ACTIONS_Y, SPAGE_PREV_W, SPAGE_BINDING_ACTIONS_H,
                     idx > 0 ? "<" : "-");
        drawButtonAt(SPAGE_NEXT_X, SPAGE_BINDING_ACTIONS_Y, SPAGE_NEXT_W, SPAGE_BINDING_ACTIONS_H,
                     idx < count - 1 ? ">" : "-");
    }
    drawButtonAt(SPAGE_ADD_X, SPAGE_BINDING_ACTIONS_Y, SPAGE_ADD_W, SPAGE_BINDING_ACTIONS_H,
                 canAdd ? "ADD" : "ADD (MAX)");
    drawButtonAt(SPAGE_REMOVE_X, SPAGE_BINDING_ACTIONS_Y, SPAGE_REMOVE_W, SPAGE_BINDING_ACTIONS_H, "REMOVE");
}

static void drawSettingsButtons() {
    display->fillRect(0, SPAGE_BTN_Y - 4, SCR_W, SPAGE_BTN_H + 8, COLOR_BG);
    drawButtonAt(SPAGE_BACK_X, SPAGE_BTN_Y, SPAGE_BTN_W, SPAGE_BTN_H, "BACK");
    drawButtonAt(SPAGE_RESET_X, SPAGE_BTN_Y, SPAGE_BTN_W, SPAGE_BTN_H, "RESET");
    drawButtonAt(SPAGE_SAVE_X, SPAGE_BTN_Y, SPAGE_BTN_W, SPAGE_BTN_H, "SAVE");
}

// WO13: title + binding Prev/Next navigation, combined into one 40px
// band (arrow hit-zones span the full band height, "hit zone may exceed
// the visual" - see the geometry comment above SPAGE_NAV_Y). Arrows only
// drawn/hit-tested when the port carries more than one binding right
// now - a single-binding port shows the plain title, same as pre-WO13.
static void drawSettingsNavRow() {
    display->fillRect(0, SPAGE_NAV_Y, SCR_W, SPAGE_NAV_H, COLOR_BG);

    int dev = editCtx.selected_device;
    int port = editCtx.selected_port;
    int count = pendingEdits[dev][port].dirty ? pendingEdits[dev][port].binding_count
                                               : effectiveBindingCount(dev, port);

    char title[40];
    if (editCtx.selected_valid) {
        if (count > 1) {
            snprintf(title, sizeof(title), "MKH%d-%c %s (%d/%d)", mkh_device_app_number(dev), mkh_port_letter(port),
                      mkh_config_input_token_name(editCtx.captured_input), editCtx.editing_binding_index + 1, count);
        } else {
            snprintf(title, sizeof(title), "MKH%d - Port %c", mkh_device_app_number(dev), mkh_port_letter(port));
        }
    } else {
        snprintf(title, sizeof(title), "EDIT: PORT SETTINGS");
    }
    display->setTextColor(COLOR_LABEL);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(title, 2)) / 2, 8);
    display->print(title);

    if (count <= 1) {
        // No neighbors to page through - INPUT line only, same spot the
        // pre-WO13 single-line layout used.
        char inputLine[24];
        snprintf(inputLine, sizeof(inputLine), "INPUT: %s", mkh_config_input_token_name(editCtx.captured_input));
        display->setTextSize(1);
        display->setCursor((SCR_W - textWidth(inputLine, 1)) / 2, 28);
        display->print(inputLine);
    }
    // count > 1: the "(i/N)" already folded into the title above is
    // enough context here - Prev/Next themselves are buttons on the
    // binding-actions row (see drawSettingsBindingActionsRow()), not
    // touch zones on this row.
}

static void drawSettingsPage() {
    display->fillScreen(COLOR_BG);

    drawSettingsNavRow();
    drawSettingsInvertRow();
    drawSettingsModeRow();
    drawSettingsMaxRow();
    drawSettingsBindingActionsRow();

    settingsPromptActive = false;
    drawSettingsButtons();
    drawTestToggle();
}

static void drawSettingsPromptOverlay() {
    const char* q = "KEEP CHANGES?";
    display->fillRect(0, SPAGE_BINDING_ACTIONS_Y - 4, SCR_W, 24, COLOR_BG);
    display->setTextColor(COLOR_YELLOW);
    display->setTextSize(2);
    display->setCursor((SCR_W - textWidth(q, 2)) / 2, SPAGE_BINDING_ACTIONS_Y - 2);
    display->print(q);

    display->fillRect(0, SPAGE_BTN_Y - 4, SCR_W, SPAGE_BTN_H + 8, COLOR_BG);
    drawButtonAt(EXIT_PROMPT_LEFT_X, SPAGE_BTN_Y, EXIT_PROMPT_LEFT_W, SPAGE_BTN_H, "KEEP");
    drawButtonAt(EXIT_PROMPT_RIGHT_X, SPAGE_BTN_Y, EXIT_PROMPT_RIGHT_W, SPAGE_BTN_H, "DISCARD");
}

// WO13: lazily seeds pendingEdits[dev][port] from the live table if this
// session hasn't touched the port yet - shared by every settings-page
// action that needs a mutable working copy (KEEP/SAVE, Prev/Next,
// Remove). Idempotent once dirty.
static void settingsSeedPendingIfNeeded(int dev, int port) {
    if (pendingEdits[dev][port].dirty)
        return;
    const mkh_port_config_t* cfg = mkh_config_get(dev, port);
    pendingEdits[dev][port].binding_count = cfg ? cfg->binding_count : 0;
    for (int i = 0; i < pendingEdits[dev][port].binding_count; i++) {
        pendingEdits[dev][port].bindings[i] = cfg->bindings[i];
    }
}

// WO13: commits the CURRENT binding's working invert/max/mode into the
// pending array if they've actually changed since baseline - an
// implicit keep, same precedent as capture's NEXT, used before Prev/Next
// navigation so paging away from a binding never silently drops an
// in-progress edit to it.
static void settingsCommitWorkingIfDirty(int dev, int port, int idx) {
    bool uncommitted = (settingsWorkingInvert != settingsBaselineInvert) ||
                        (settingsWorkingMax != settingsBaselineMax) || (settingsWorkingMode != settingsBaselineMode);
    if (!uncommitted)
        return;
    settingsSeedPendingIfNeeded(dev, port);
    if (idx < pendingEdits[dev][port].binding_count) {
        pendingEdits[dev][port].bindings[idx].invert = settingsWorkingInvert;
        pendingEdits[dev][port].bindings[idx].max_percent = settingsWorkingMax;
        pendingEdits[dev][port].bindings[idx].mode = settingsWorkingMode;
        pendingEdits[dev][port].dirty = true;
    }
}

// WO13: loads binding `idx` (pending-aware) into the working/baseline
// state and redraws - used by Prev/Next. Deliberately NOT goToPage(): it
// must not re-run the capture-commit logic in goToPage()'s
// PAGE_EDIT_SETTINGS case, which is only for a fresh capture landing on
// this page, not for paging between bindings already here.
static void settingsLoadBindingAt(int dev, int port, int idx) {
    editCtx.editing_binding_index = idx;
    mkh_binding_t b = effectiveBinding(dev, port, idx);
    settingsWorkingInvert = b.invert;
    settingsWorkingMax = b.max_percent;
    settingsWorkingMode = b.mode;
    settingsBaselineInvert = settingsWorkingInvert;
    settingsBaselineMax = settingsWorkingMax;
    settingsBaselineMode = settingsWorkingMode;
    editCtx.captured_input = b.input;
    editCtx.capture_valid = true;
    drawSettingsPage();
}

// Settings page's own hit zones + its leave-page prompt's, same
// separation rationale as the other custom pages.
static void dispatchSettingsTouch(int16_t touchX, int16_t touchY) {
    int dev = editCtx.selected_device;
    int port = editCtx.selected_port;
    int idx = editCtx.editing_binding_index;
    mkh_input_class_t cls = mkh_config_input_class(editCtx.captured_input);
    // WO13: BACK's destination (and the KEEP/DISCARD prompt's) depends
    // on how this settings visit was reached - see settingsBackToCapture's
    // doc comment.
    UiPage backTarget = settingsBackToCapture ? PAGE_EDIT_CAPTURE : PAGE_EDIT_PORTS;

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
            // this binding explicitly.
            settingsSeedPendingIfNeeded(dev, port);
            if (idx < pendingEdits[dev][port].binding_count) {
                pendingEdits[dev][port].bindings[idx] = {editCtx.captured_input, settingsWorkingInvert,
                                                           settingsWorkingMax, settingsWorkingMode};
                pendingEdits[dev][port].dirty = true;
            }
            settingsPromptActive = false;
            goToPage(backTarget);
        } else if (inDiscard) {
            // pendingEdits[dev][port] is left exactly as it was on
            // entry (untouched here) - if entry didn't dirty it (WO11
            // Task 1), discarding these working values just returns to
            // that already-correct baseline, nothing to undo.
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings prompt DISCARD\n", touchX, touchY);
            settingsPromptActive = false;
            goToPage(backTarget);
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

    // WO13 Task 2: nudge buttons, 1% per tap, end-snap applied same as a
    // drag release (see applyMaxEndSnap()).
    bool inMaxRow = touchY >= SPAGE_MAXSLIDER_Y && touchY < SPAGE_MAXSLIDER_Y + SPAGE_MAXSLIDER_H;
    if (inMaxRow && touchX >= SPAGE_NUDGE_MINUS_X && touchX < SPAGE_NUDGE_MINUS_X + SPAGE_NUDGE_W) {
        settingsWorkingMax = applyMaxEndSnap((int32_t)settingsWorkingMax - 1);
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings MAX nudge- -> %u%%\n", touchX, touchY,
                        (unsigned)settingsWorkingMax);
        drawSettingsMaxRow();
        return;
    }
    if (inMaxRow && touchX >= SPAGE_NUDGE_PLUS_X && touchX < SPAGE_NUDGE_PLUS_X + SPAGE_NUDGE_W) {
        settingsWorkingMax = applyMaxEndSnap((int32_t)settingsWorkingMax + 1);
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings MAX nudge+ -> %u%%\n", touchX, touchY,
                        (unsigned)settingsWorkingMax);
        drawSettingsMaxRow();
        return;
    }
    if (inMaxRow && touchX >= SPAGE_SLIDER_X && touchX < SPAGE_SLIDER_X + SPAGE_SLIDER_W) {
        int32_t rel = touchX - SPAGE_SLIDER_X;
        if (rel < 0)
            rel = 0;
        if (rel > SPAGE_SLIDER_W)
            rel = SPAGE_SLIDER_W;
        // WO13 Task 2: end-snap - a release within 2% of either end lands
        // exactly on 0/100 (drag-as-is otherwise, per the order).
        settingsWorkingMax = applyMaxEndSnap((rel * 100) / SPAGE_SLIDER_W);
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings MAX slider -> %u%%\n", touchX, touchY,
                        (unsigned)settingsWorkingMax);
        drawSettingsMaxRow();
        return;
    }

    // WO13 Task 1: PREV / ADD / REMOVE / NEXT row (see SPAGE_NAV_Y's doc
    // comment for why Prev/Next live here instead of the title row's
    // corners).
    bool inBindingActionsRow = touchY >= SPAGE_BINDING_ACTIONS_Y && touchY < SPAGE_BINDING_ACTIONS_Y + SPAGE_BINDING_ACTIONS_H;
    bool inPrevBtn = inBindingActionsRow && touchX >= SPAGE_PREV_X && touchX < SPAGE_PREV_X + SPAGE_PREV_W;
    bool inAddBtn = inBindingActionsRow && touchX >= SPAGE_ADD_X && touchX < SPAGE_ADD_X + SPAGE_ADD_W;
    bool inRemoveBtn = inBindingActionsRow && touchX >= SPAGE_REMOVE_X && touchX < SPAGE_REMOVE_X + SPAGE_REMOVE_W;
    bool inNextBtn = inBindingActionsRow && touchX >= SPAGE_NEXT_X && touchX < SPAGE_NEXT_X + SPAGE_NEXT_W;
    if (inPrevBtn || inNextBtn) {
        int count = pendingEdits[dev][port].dirty ? pendingEdits[dev][port].binding_count
                                                   : effectiveBindingCount(dev, port);
        if (inPrevBtn && count > 1 && idx > 0) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings PREV binding (%d -> %d)\n", touchX,
                            touchY, idx, idx - 1);
            settingsCommitWorkingIfDirty(dev, port, idx);
            settingsLoadBindingAt(dev, port, idx - 1);
        } else if (inNextBtn && count > 1 && idx < count - 1) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings NEXT binding (%d -> %d)\n", touchX,
                            touchY, idx, idx + 1);
            settingsCommitWorkingIfDirty(dev, port, idx);
            settingsLoadBindingAt(dev, port, idx + 1);
        }
        return;
    }
    if (inAddBtn) {
        int count = pendingEdits[dev][port].dirty ? pendingEdits[dev][port].binding_count
                                                   : effectiveBindingCount(dev, port);
        if (count >= MKH_MAX_BINDINGS_PER_PORT) {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings ADD binding, ignored (cap %d reached)\n",
                            touchX, touchY, MKH_MAX_BINDINGS_PER_PORT);
        } else {
            Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings ADD binding -> capture\n", touchX,
                            touchY);
            // editCtx.editing_binding_index is left UNCHANGED - it's the
            // REPLACE target if capture's REPLACE/ADD prompt chooses
            // REPLACE; ADD there computes its own fresh append index.
            editCtx.captured_input = MKH_INPUT_NONE;
            editCtx.capture_valid = false;
            settingsBackToCapture = true;
            goToPage(PAGE_EDIT_CAPTURE);
        }
        return;
    }
    if (inRemoveBtn) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings REMOVE binding[%d]\n", touchX, touchY, idx);
        settingsSeedPendingIfNeeded(dev, port);
        int count = pendingEdits[dev][port].binding_count;
        if (idx < count) {
            for (int i = idx; i < count - 1; i++) {
                pendingEdits[dev][port].bindings[i] = pendingEdits[dev][port].bindings[i + 1];
            }
            pendingEdits[dev][port].binding_count = --count;
            pendingEdits[dev][port].dirty = true;
        }
        if (count == 0) {
            // Port now unassigned - nothing left to show on this page.
            editCtx.captured_input = MKH_INPUT_NONE;
            editCtx.capture_valid = false;
            goToPage(backTarget);
        } else {
            settingsLoadBindingAt(dev, port, idx >= count ? count - 1 : idx);
        }
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
            goToPage(backTarget);
        }
    } else if (inReset) {
        // Order: "Reset (settings page) restores saved config to the
        // table and clears the session." CC's judgment (reported):
        // stays on the settings page rather than navigating away,
        // refreshing this port's fields from the just-reloaded table -
        // a form-reset, not an exit. WO13: reloads the WHOLE binding
        // list, then re-shows binding 0 (the just-reloaded table may
        // have fewer bindings than were being viewed).
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings RESET\n", touchX, touchY);
        mkh_storage_reload_config();
        clearPendingSession();
        testActive = false;
        const mkh_port_config_t* cfg = mkh_config_get(dev, port);
        if (cfg && cfg->binding_count > 0) {
            editCtx.editing_binding_index = 0;
            mkh_binding_t b = cfg->bindings[0];
            editCtx.captured_input = b.input;
            editCtx.capture_valid = true;
            settingsWorkingInvert = b.invert;
            settingsWorkingMax = b.max_percent;
            settingsWorkingMode = b.mode;
        } else {
            editCtx.editing_binding_index = 0;
            editCtx.captured_input = MKH_INPUT_NONE;
            editCtx.capture_valid = false;
            settingsWorkingInvert = false;
            settingsWorkingMax = 100;
            settingsWorkingMode = MKH_MODE_PROPORTIONAL;
        }
        settingsBaselineInvert = settingsWorkingInvert;
        settingsBaselineMax = settingsWorkingMax;
        settingsBaselineMode = settingsWorkingMode;
        drawSettingsPage();
    } else if (inSave) {
        Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> settings SAVE\n", touchX, touchY);
        // WO11 Task 1: same as KEEP above - commit this binding
        // explicitly, entry no longer guarantees dirty/input are set.
        settingsSeedPendingIfNeeded(dev, port);
        if (idx < pendingEdits[dev][port].binding_count) {
            pendingEdits[dev][port].bindings[idx] = {editCtx.captured_input, settingsWorkingInvert, settingsWorkingMax,
                                                       settingsWorkingMode};
            pendingEdits[dev][port].dirty = true;
        }
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
            // meaningful to carry across a navigation. WO13: same for
            // the REPLACE/ADD prompt.
            capturePromptActive = false;
            captureReplaceAddPromptActive = false;
            drawCapturePage(title);
            break;
        }
        case PAGE_EDIT_SETTINGS: {
            // WO11 Task 1 fix, generalized per-binding by WO13 (was:
            // WO10-FINAL unconditionally set pendingEdits[dev][port].
            // dirty = true right here, on every entry - so merely
            // walking capture -> settings -> Back dirtied the whole
            // session and could trip the exit-editor prompt with
            // nothing actually changed). Root cause: entry wrote
            // through the SAME path a real edit does, with no
            // comparison against what's already true for this binding.
            //
            // Fix: compute the TRUE current baseline for the binding at
            // editCtx.editing_binding_index (existing pending binding if
            // one exists, else the resolved live binding, else - for a
            // brand new slot one past the end - the compiled-in blank
            // default) and only commit into the pending session if the
            // currently-shown input actually differs from that
            // baseline. A same-input revisit (port-select's direct entry
            // to view an existing, unchanged binding) no longer dirties
            // anything by itself; invert/max/mode edits still commit via
            // their own KEEP/SAVE paths below.
            int dev = editCtx.selected_device;
            int port = editCtx.selected_port;
            int idx = editCtx.editing_binding_index;

            // Seed the pending working copy from the live table on the
            // FIRST touch this session (not yet dirty) - the whole
            // binding list, so later edits/add/remove have a mutable
            // in-memory copy distinct from the live table to work from.
            if (!pendingEdits[dev][port].dirty) {
                const mkh_port_config_t* cfg = mkh_config_get(dev, port);
                pendingEdits[dev][port].binding_count = cfg ? cfg->binding_count : 0;
                for (int i = 0; i < pendingEdits[dev][port].binding_count; i++) {
                    pendingEdits[dev][port].bindings[i] = cfg->bindings[i];
                }
            }

            bool idxIsNewSlot = (idx >= pendingEdits[dev][port].binding_count);
            mkh_binding_t baseline = idxIsNewSlot ? mkh_binding_t{MKH_INPUT_NONE, false, 100, MKH_MODE_PROPORTIONAL}
                                                   : pendingEdits[dev][port].bindings[idx];

            if (editCtx.capture_valid && editCtx.captured_input != baseline.input) {
                baseline.input = editCtx.captured_input;
                if (idxIsNewSlot) {
                    pendingEdits[dev][port].bindings[pendingEdits[dev][port].binding_count++] = baseline;
                } else {
                    pendingEdits[dev][port].bindings[idx] = baseline;
                }
                pendingEdits[dev][port].dirty = true;
            }

            settingsWorkingInvert = baseline.invert;
            settingsWorkingMax = baseline.max_percent;
            settingsWorkingMode = baseline.mode;
            settingsBaselineInvert = settingsWorkingInvert;
            settingsBaselineMax = settingsWorkingMax;
            settingsBaselineMode = settingsWorkingMode;
            // Keep the displayed token in sync with whichever binding is
            // now actually showing (matters when idxIsNewSlot was true
            // but capture_valid was somehow false - defensive, shouldn't
            // happen via the normal capture/port-select entry paths).
            editCtx.captured_input = baseline.input;
            editCtx.capture_valid = (baseline.input != MKH_INPUT_NONE);
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

// Applies ONE binding's invert/max attributes to a raw MK6 channel byte
// (0x00..0xFF, 0x80=neutral). curve is always linear in v1 (see
// mkh_config.h), so there's no curve step here. WO13: was "a config
// port's" - now a single binding's, since a port can carry several; the
// function itself is unchanged, only its parameter type (mkh_port_
// config_t -> mkh_binding_t), since invert/max/mode live on the binding
// now, not the port.
//
// invert mirrors the byte around neutral (0x80). The protocol's byte
// encoding has an inherent 127-wide forward span (0x80..0xFF) vs
// 128-wide reverse span (0x80..0x00) - see mkStickYToChannelByte above -
// so a byte-perfect mirror is impossible at the single extreme value
// 0xFF (mirrors to 0x01, not 0x00); the clamp below is that one-unit
// edge case, not a bug.
static uint8_t mkApplyBindingConfig(uint8_t rawByte, const mkh_binding_t* b) {
    int value = rawByte;
    if (b->invert) {
        value = 0x100 - value;
        if (value > 0xFF)
            value = 0xFF;
        if (value < 0x00)
            value = 0x00;
    }

    int delta = value - 0x80;
    delta = (delta * (int)b->max_percent) / 100;
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
//
// WO13: now per-BINDING (a third array dimension) - a port can carry
// several bindings, each independently in momentary/latched/proportional
// mode, so each needs its own latch/edge state. Sized to
// MKH_MAX_BINDINGS_PER_PORT regardless of how many bindings a port
// actually has right now, same "leave room" pattern as the rest of the
// per-device/per-channel arrays.
static bool latchState[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS][MKH_MAX_BINDINGS_PER_PORT];
static bool latchPrevPressed[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS][MKH_MAX_BINDINGS_PER_PORT];

// WO11 Task 4 (hardening), extended WO13: reassigning a port away from
// latched mode and back again via the editor could otherwise resurface a
// stale latchState value from before the reassignment - a port pushed
// through pushPendingSessionToLiveTable() (Test-ON/Save) always gets a
// fresh, unlatched start for EVERY binding slot, same "boots off"
// guarantee the order gives config itself. Forward-declared above (near
// clearEditorSession()) so pushPendingSessionToLiveTable(), defined
// earlier in the file, can call it.
static void clearLatchStateForPort(int dev, int port) {
    for (int b = 0; b < MKH_MAX_BINDINGS_PER_PORT; b++) {
        latchState[dev][port][b] = false;
        latchPrevPressed[dev][port][b] = false;
    }
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
// Step 2's stick-only version. Every assigned binding is driven every
// call regardless of mode - proportional inputs scale continuously,
// momentary is a simple pressed/released level, latched toggles on a
// press edge and otherwise holds (see latchState[]/latchPrevPressed[]
// above). All three funnel through the SAME mkApplyBindingConfig()
// invert/max pipeline (order: "evaluate each through its own invert/max
// pipeline") - only the pre-pipeline raw byte differs by class/mode.
//
// WO13: a port can now carry several bindings ("alternatives... used one
// at a time" per the order's intent line, not a mixer). Arbitration is
// exactly the order's spec - each binding's contribution is computed
// independently through its own pipeline above, the contributions SUM,
// and the sum clamps to +/-100% before one final value reaches
// mkh_set_channel(). No "is this binding active" branching is needed to
// get "used one at a time" in the common case: an idle/unpressed binding
// already resolves to neutral (0x80, i.e. zero contribution) by
// construction - proportional axes read 0 within the deadzone,
// unpressed digitals/triggers read 0x80 - so summing every binding's
// contribution unconditionally naturally reduces to "whichever one is
// actually being touched" when only one is, and correctly adds when more
// than one is touched at once, exactly as ordered.
static void processMappedInputs(ControllerPtr ctl) {
    for (int dev = 0; dev < MKH_MK6_NUM_DEVICES; dev++) {
        for (int port = 0; port < MKH_MK6_NUM_CHANNELS; port++) {
            const mkh_port_config_t* cfg = mkh_config_get(dev, port);
            if (!cfg || cfg->binding_count == 0)
                continue;

            // Signed delta-from-neutral domain (0x80=neutral=0) for
            // summing - the byte encoding's own asymmetric span
            // (0x80..0xFF is 127-wide, 0x80..0x00 is 128-wide, see
            // mkStickYToChannelByte()'s doc comment) is exactly the
            // +127/-128 clamp band used below, so "clamp to +/-100%"
            // needs no separate percent conversion.
            int sumDelta = 0;
            for (int bIdx = 0; bIdx < cfg->binding_count; bIdx++) {
                const mkh_binding_t* binding = &cfg->bindings[bIdx];
                mkh_input_class_t cls = mkh_config_input_class(binding->input);
                uint8_t rawByte;

                if (cls == MKH_INPUT_CLASS_AXIS) {
                    // Stick axes: no mode, always proportional (order) -
                    // binding->mode is never consulted here, regardless
                    // of what it happens to hold.
                    rawByte = mkStickYToChannelByte(readTokenRaw(ctl, binding->input));
                } else if (cls == MKH_INPUT_CLASS_TRIGGER && binding->mode == MKH_MODE_PROPORTIONAL) {
                    rawByte = mkTriggerToChannelByte(readTokenRaw(ctl, binding->input));
                } else {
                    // DIGITAL, or a TRIGGER explicitly set to momentary/
                    // latched - both resolve to a simple pressed/not
                    // level. Triggers use half-travel (511 of 1023) as
                    // the pressed/not-pressed split point for momentary/
                    // latched mode - CC's judgment, not specified by the
                    // order.
                    bool pressed = (cls == MKH_INPUT_CLASS_TRIGGER) ? (readTokenRaw(ctl, binding->input) > (1023 / 2))
                                                                     : (readTokenRaw(ctl, binding->input) != 0);

                    if (binding->mode == MKH_MODE_LATCHED) {
                        if (pressed && !latchPrevPressed[dev][port][bIdx]) {
                            latchState[dev][port][bIdx] = !latchState[dev][port][bIdx];
                        }
                        latchPrevPressed[dev][port][bIdx] = pressed;
                        rawByte = latchState[dev][port][bIdx] ? 0xFF : 0x80;
                    } else {
                        // MOMENTARY - the only other legal mode for a
                        // digital; for a trigger, the explicit momentary case.
                        rawByte = pressed ? 0xFF : 0x80;
                    }
                }

                uint8_t finalByte = mkApplyBindingConfig(rawByte, binding);
                sumDelta += (int)finalByte - 0x80;
            }

            if (sumDelta > 127)
                sumDelta = 127;
            if (sumDelta < -128)
                sumDelta = -128;
            mkh_set_channel(dev, port, (uint8_t)(0x80 + sumDelta));
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
                        if (cfg && cfg->binding_count > 0) {
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
    // WO16: log the wakeup cause before anything else - proves (from the
    // boot log alone) whether this boot is a fresh power-on/reset or a
    // return from deep sleep via the BOOT-key wake, per the order's own
    // wake self-test/acceptance criteria ("tap -> boots to dashboard").
    esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
    esp_reset_reason_t resetReason = esp_reset_reason();
    Console.printf("MK1 Boot: wakeup cause=%d (%s), reset reason=%d\n", (int)wakeCause,
                    wakeCause == ESP_SLEEP_WAKEUP_EXT0 ? "EXT0 BOOT-key wake from deep sleep" : "power-on/other reset",
                    (int)resetReason);

    // v0.8.0 Step 0b: LittleFS storage foundation. Internal flash has no
    // shared-bus constraint with the display (unlike the retired SD
    // path - see mkh_sdcard.cpp); this stays the first STORAGE/hardware
    // action of setup() (only the WO16 wake-cause log above it, which
    // touches nothing) to preserve the single-entry-point boot discipline.
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

// WO16: sleep entry - reached either from the idle screen's SLEEP hold
// gesture or the auto-sleep timer (both in loop() below). Never returns
// on success (esp_deep_sleep_start() halts the CPU); the only way back is
// a full reboot via the BOOT-key wake (see MKH_WAKE_GPIO's doc comment
// above for why touch can't be the wake source).
//
// Mandatory sequence per the order: neutral to every hub BEFORE anything
// else, wait for that to actually finish airing, THEN blank the display,
// THEN deep sleep. Reuses mkh_broadcast_toggle_off() - the same safety-
// ordered "command neutral now, keep airing it for a few more services,
// then drop" primitive the dashboard's own per-hub OFF button already
// uses (mkh_broadcast.c/mkh_protocol.c/.h untouched) - rather than
// inventing a second path to the same guarantee.
static void enterDeepSleep() {
    Console.printf("MK1 Sleep: entry requested, commanding neutral to all hubs before shutdown\n");

    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        mkh_broadcast_toggle_off(d);
    }

    // Poll for the neutral-then-drop sequence to actually finish (WO12's
    // event-driven send means the neutral COMMAND is usually already on
    // air within ~25ms of the toggle_off calls above, but this also waits
    // for the full OFF-grace drop, so the log line below reports what
    // actually happened, not just what was requested). Bounded: if a
    // device somehow never clears, sleep still proceeds after the
    // timeout rather than hanging forever - the neutral command itself
    // already went out synchronously inside toggle_off() regardless of
    // whether the drop bookkeeping ever completes.
    const uint32_t NEUTRAL_WAIT_TIMEOUT_MS = 2000;
    uint32_t waitStart = millis();
    bool allClear = false;
    while (millis() - waitStart < NEUTRAL_WAIT_TIMEOUT_MS) {
        allClear = true;
        for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
            if (mkh_hub_broadcasting[d] || mkh_broadcast_is_transitioning(d)) {
                allClear = false;
                break;
            }
        }
        if (allClear)
            break;
        delay(20);
    }
    Console.printf("MK1 Sleep: broadcast stop %s (%lums)\n",
                    allClear ? "confirmed clean - all hubs neutral and dropped" : "timed out, proceeding anyway",
                    (unsigned long)(millis() - waitStart));

    digitalWrite(TFT_BL, LOW);
    display->fillScreen(COLOR_BG);

    // WO16 Phase 1 finding: GPIO0 is shared between TFT_RST (currently
    // driven HIGH by the display driver) and the BOOT-key wake source.
    // Hand it back to plain INPUT before the RTC controller takes
    // ownership for ext0 wake - harmless, since the backlight is already
    // off by this point.
    pinMode(TFT_RST, INPUT);

    // WO16 bench fix: the RTC pad's own pull configuration does NOT
    // inherit whatever the digital-domain GPIO driver had set - without
    // this, GPIO0 can float once the RTC controller takes over the pad,
    // and a level-triggered ext0 wake (armed just below) on a floating
    // pin can fire immediately/spuriously instead of waiting for an
    // actual BOOT-key press (observed at the bench: screen goes dark,
    // then reboots almost immediately rather than staying asleep).
    // Explicitly enabling the RTC pad's own internal pull-up removes the
    // dependency on the board's external pull-up being strong enough
    // once GPIO0 leaves the digital domain.
    rtc_gpio_pullup_en(MKH_WAKE_GPIO);
    rtc_gpio_pulldown_dis(MKH_WAKE_GPIO);

    esp_sleep_enable_ext0_wakeup(MKH_WAKE_GPIO, 0);  // wake on LOW (BOOT key press)
    Console.printf("MK1 Sleep: display off, entering deep sleep now (wake source = BOOT key / GPIO0 LOW)\n");
    esp_deep_sleep_start();
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // WO13 Task 3: battery ADC read - runs every tick regardless of
    // page, gated internally to its own ~1s interval (see
    // updateBatteryReading()'s doc comment). Display-layer + one ADC
    // read only; drawing (dashboard-only) happens separately via
    // initDashboard()/updateDashboard().
    updateBatteryReading();

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
            // WO16: a NEW press starting inside the SLEEP button no
            // longer wakes immediately - it arms the hold-to-sleep timer
            // instead (checked every tick further down in loop(), via
            // mkh_touch_is_pressed() rather than this edge-triggered
            // return, since a hold needs continuous state). Any other
            // tap keeps WO11's original "any touch wakes instantly"
            // behavior unchanged.
            bool inSleepBtn = touchX >= IDLE_SLEEP_BTN_X && touchX < IDLE_SLEEP_BTN_X + IDLE_SLEEP_BTN_W &&
                               touchY >= IDLE_SLEEP_BTN_Y && touchY < IDLE_SLEEP_BTN_Y + IDLE_SLEEP_BTN_H;
            if (inSleepBtn) {
                Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> SLEEP button, hold-to-sleep armed\n", touchX,
                                touchY);
                idleSleepHoldActive = true;
                idleSleepHoldStartMs = millis();
            } else {
                Console.printf("MK1 Touch: tap display=(x=%d,y=%d) -> idle screen, waking to dashboard\n", touchX,
                                touchY);
                goToPage(PAGE_DASHBOARD);
            }
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
        // WO16: log the transition itself (WO11 never did) - needed to
        // log-verify the auto-sleep chain's first link from the boot log
        // alone.
        Console.printf("MK1 Idle: %lums since last touch, no XWC - entering idle screen\n",
                        (unsigned long)(millis() - lastTouchActivityMs));
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
            // An active controller connection always wins, even over an
            // in-progress SLEEP hold - matches WO11's original priority
            // (idling is never allowed to block a live control session).
            goToPage(PAGE_DASHBOARD);
        } else if (idleSleepHoldActive) {
            // WO16: hold-to-sleep in progress - checked every tick via
            // mkh_touch_is_pressed() (continuous state), not
            // mkh_touch_poll()'s return (edge-triggered, already consumed
            // above).
            if (!mkh_touch_is_pressed()) {
                // Released before the hold threshold - the order's own
                // accidental-activation guard: treat it exactly like any
                // other tap on the idle screen and just wake, the same as
                // if the tap had landed anywhere else.
                Console.printf("MK1 Sleep: SLEEP hold released early (%lums) - waking to dashboard instead\n",
                                (unsigned long)(millis() - idleSleepHoldStartMs));
                idleSleepHoldActive = false;
                goToPage(PAGE_DASHBOARD);
            } else if (millis() - idleSleepHoldStartMs >= SLEEP_HOLD_MS) {
                Console.printf("MK1 Sleep: SLEEP held for %lums - entering deep sleep\n",
                                (unsigned long)SLEEP_HOLD_MS);
                idleSleepHoldActive = false;
                enterDeepSleep();  // never returns
            }
        } else if ((millis() - idleEnteredMs) >= AUTO_SLEEP_TIMEOUT_MS) {
            // WO16 auto-sleep: "dashboard -> 30s -> idle screen -> N min
            // -> deep sleep." Armed only here (currentPage == PAGE_IDLE)
            // and only when XWC is not active - guaranteed by this
            // being the trailing else-if under the XWC_ACTIVE check above.
            Console.printf("MK1 Sleep: auto-sleep timeout (%lu min idle, no touch, no XWC) - entering deep sleep\n",
                            (unsigned long)(AUTO_SLEEP_TIMEOUT_MS / 60000UL));
            enterDeepSleep();  // never returns
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
