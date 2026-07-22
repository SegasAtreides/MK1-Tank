// SPDX-License-Identifier: Apache-2.0
//
// MK6 Broadcast. Stage 1 proved a non-connectable BLE advertisement can run
// on the same BTstack instance Bluepad32 already drives, concurrently with
// the XWC (Xbox Wireless Controller) connection. Stage 2 replaces the test
// payload with the real Mould King 6.0 protocol (see mkh_protocol.h/.c):
// broadcast the CONNECT telegram to switch a hub into Bluetooth mode, then
// switch to repeating a CONTROL telegram. v0.6.0 wires mkh_set_channel()
// (see below) to the XWC left stick (see sketch.cpp) - device 0, port A,
// live proportional control. v0.7.0 Step 1 adds a second hub via time-
// slicing: only one telegram can be on air at a time, so
// mkh_broadcast_control() alternates which device's CONTROL telegram it
// stages each ~100ms cycle (see MKH_TIME_SLICE_NUM_DEVICES below). The
// CONNECT telegram stays a single shared broadcast - it is not device-
// addressed (verified against BrickController2: MK6.cs's Telegram_Connect
// is one constant shared across all three device slots), so
// mkh_broadcast_connect() below is unchanged from v0.6.0.
//
// Coexistence note (verified against the vendored BTstack source,
// components/btstack/src/hci.c): BTstack only disables advertising when
// the number of *peripheral/slave-role* LE connections reaches
// hci_stack->le_max_number_peripheral_connections (see
// hci_update_advertisements_enabled_for_current_roles()). Bluepad32
// connects to controllers via gap_connect() (components/bluepad32/bt/
// uni_bt_le.c), i.e. this device is the LE *central*/master for that
// link, which is never counted as a peripheral/slave connection. A
// non-connectable advertisement (ADV_NONCONN_IND) is therefore not
// consumed or auto-stopped by that link. We still defensively re-assert
// gap_advertisements_enable(1) on connection/disconnection HCI events,
// driven entirely from the packet handler below (no polling timer).
//
// The legacy gap_advertisements_set_params/set_data/enable calls
// (components/btstack/src/hci.c) only stage flags in hci_stack and call
// hci_run(); BTstack's own state machine sequences the underlying
// "disable -> reconfigure -> re-enable" HCI commands as needed, so these
// calls are safe to invoke without manually gating on
// hci_can_send_command_packet_now() (that gating is only required for raw
// hci_send_cmd() calls, which this module does not use).
//
// Two things repeat at ~100ms, for different reasons. BLE's own
// hardware-level advertising interval (MKH_ADV_INTERVAL_UNITS below)
// re-transmits whatever payload is currently set - that's "free" and
// needs no software timer. Separately, mkh_broadcast_control() itself
// re-arms on a ~100ms BTstack timer (MKH_CONTROL_REPEAT_MS) so it keeps
// rebuilding the payload from the live, controller-driven channel values
// (mkh_channel_value[], set via mkh_set_channel()) - without that, the
// broadcast would freeze at whatever value happened to be current the one
// time the timer fired. gap_advertisements_set_data() only stages new
// advertising *data*, not parameters, so calling it every cycle does not
// stop/restart advertising (see mkh_broadcast_control()).

#include "mkh_broadcast.h"

#include <btstack.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mkh_ports.h"
#include "mkh_protocol.h"
#include "uni_log.h"

// WO v1.4.0: reduced from 160 (100ms) to 40 units - 40 * 0.625ms = 25ms
// advertising interval. Sanctioned amendment to the broadcast-timing
// freeze, specifically to fix the PULSE misfire bug (see the WO's own
// completion report): a PULSE binding's full-level payload is only
// staged for its short, fixed duration (~100-165ms at the current
// default), and the hardware radio's own independent retransmission
// clock - this constant - was the sole determinant of how many actual
// on-air chances a hub got to receive it before it reverted to neutral.
// At the old 100ms interval, that was only 1-2 transmissions; at 25ms,
// it's roughly 4x that within the same window. Verified within BTstack's
// own documented valid range for this HCI parameter (components/btstack/
// src/hci_cmd.c: hci_le_set_advertising_parameters, [0x0020,0x4000] =
// [32,16384] units - 40 comfortably clears the 32-unit floor) before
// this was chosen; no HCI/GAP error was observed after flashing (see
// mkh_check_command_complete() below, unchanged, still watching for
// exactly this).
#define MKH_ADV_INTERVAL_UNITS 40
// Non-connectable, non-scannable undirected advertising (BT Core spec
// legacy advertising type enum, see components/btstack/src/hci.c around
// the ADV_NONCONN_IND decode and hci_cmd.c's hci_le_set_advertising_parameters).
#define MKH_ADV_TYPE_NONCONN_IND 3

// MK company ID (BrickController2 MKProtocol.ManufacturerID / Python
// MouldKingHub.ManufacturerID = bytes([0xFF, 0xF0])), little-endian on air.
#define MKH_COMPANY_ID_LO 0xF0
#define MKH_COMPANY_ID_HI 0xFF

// How long to broadcast CONNECT before switching to CONTROL (a handful of
// repeats at the ~100ms advertising interval).
#define MKH_CONNECT_PHASE_MS 600

// How often mkh_broadcast_control() re-arms itself to rebuild/stage a
// CONTROL telegram - see the file header comment.
#define MKH_CONTROL_REPEAT_MS 100

// Number of devices ELIGIBLE for the CONTROL time-slice rotation. Step 1
// (v0.9.0) covers devices 0 and 1 (two physical hubs, now individually
// toggleable on/off - see mkh_device_active[] below); device slot 2
// already exists in the per-device arrays ("leave room for 3") but is
// not eligible yet - WO9 does not authorize activating the third hub's
// live broadcast, so mkh_broadcast_toggle_on() refuses it (logs why,
// does nothing else). Bump this to MKH_MK6_NUM_DEVICES on a future,
// explicit go-ahead to let it join.
#define MKH_TIME_SLICE_NUM_DEVICES 2

// How many times a device must actually be SERVICED (a real telegram
// airing its now-neutral values) after a toggle-off request before it
// is safe to drop from rotation - see mkh_broadcast_toggle_off() and
// the countdown in mkh_broadcast_control(). WO9: ">=3 full cycles so
// neutral actually airs". With MKH_TIME_SLICE_NUM_DEVICES devices
// sharing the rotation, "this device serviced N times" and "N full
// rotation cycles" are the same thing (each full cycle services every
// active device exactly once).
#define MKH_OFF_GRACE_SERVICES 3

static btstack_packet_callback_registration_t mkh_hci_event_callback_registration;
static bool mkh_advertising_started = false;
static btstack_timer_source_t mkh_broadcast_timer;

// Definition for the extern declared in mkh_broadcast.h.
bool mkh_hub_broadcasting[MKH_MK6_NUM_DEVICES] = {false};

// Live per-device, per-channel raw telegram byte values (0x80 = neutral),
// set via mkh_set_channel() and read by mkh_broadcast_control() when
// building each device's CONTROL telegram. Protected by mkh_channel_mutex
// since the setter and the broadcast timer run on different FreeRTOS
// tasks. Explicitly initialized to 0x80 per channel - a plain zero-init
// would default to 0x00 (full reverse), not neutral. Sized for all
// MKH_MK6_NUM_DEVICES device slots even though only devices 0 and 1 are
// in the time-slice rotation right now (see MKH_TIME_SLICE_NUM_DEVICES).
#define MKH_NEUTRAL_CHANNELS {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}
static uint8_t mkh_channel_value[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS] = {
    MKH_NEUTRAL_CHANNELS,
    MKH_NEUTRAL_CHANNELS,
    MKH_NEUTRAL_CHANNELS,
};
static SemaphoreHandle_t mkh_channel_mutex;

// mkh_set_channel() itself is defined further down (after mkh_set_adv_payload()
// and the WO12 Phase 2 event-send machinery it now triggers) - see there.

// v0.9.0 Step 1: which devices are eligible for the CONTROL rotation
// right now. Distinct from mkh_hub_broadcasting[] (the dashboard-facing
// "true broadcast state" WO9 asks for) - during a toggle-off's neutral-
// airing grace period a device is still active (still rotating, still
// dashboard-green) even though its removal is already pending; see
// mkh_pending_off_services[] below. Initialized in mkh_broadcast_init().
static bool mkh_device_active[MKH_MK6_NUM_DEVICES];

// >0 while device_id is in its OFF grace period (see
// MKH_OFF_GRACE_SERVICES); decremented once per actual service in
// mkh_broadcast_control(), and the device is dropped from rotation when
// it reaches 0. 0 means no pending removal.
static int mkh_pending_off_services[MKH_MK6_NUM_DEVICES];

// See mkh_broadcast_is_transitioning() in the header for the contract.
static bool mkh_device_transitioning[MKH_MK6_NUM_DEVICES];

bool mkh_broadcast_is_transitioning(int device_id) {
    if (device_id < 0 || device_id >= MKH_MK6_NUM_DEVICES)
        return false;
    return mkh_device_transitioning[device_id];
}

static bd_addr_t mkh_null_addr = {0, 0, 0, 0, 0, 0};

// Advertising Data buffer: Flags AD + Manufacturer Specific Data AD. Must
// be static - gap_advertisements_set_data stores the pointer, not a copy.
#define MKH_ADV_BUF_LEN (3 + 4 + MKH_PAYLOAD_CONTROL_LEN)
static uint8_t mkh_adv_buf[MKH_ADV_BUF_LEN];

static void mkh_log_hex(const char* label, const uint8_t* data, size_t len) {
    char hex[3 * MKH_PAYLOAD_CONTROL_LEN + 1];
    size_t pos = 0;
    for (size_t i = 0; i < len && pos + 3 < sizeof(hex); i++) {
        int n = snprintf(hex + pos, sizeof(hex) - pos, "%02X ", data[i]);
        if (n <= 0)
            break;
        pos += (size_t)n;
    }
    logi("MK6 Broadcast: %s (%u bytes): %s\n", label, (unsigned)len, hex);
}

static void mkh_set_adv_payload(const uint8_t* payload, size_t payload_len) {
    size_t i = 0;
    mkh_adv_buf[i++] = 0x02;
    mkh_adv_buf[i++] = BLUETOOTH_DATA_TYPE_FLAGS;
    mkh_adv_buf[i++] = 0x06;

    mkh_adv_buf[i++] = (uint8_t)(1 + 2 + payload_len);
    mkh_adv_buf[i++] = BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA;
    mkh_adv_buf[i++] = MKH_COMPANY_ID_LO;
    mkh_adv_buf[i++] = MKH_COMPANY_ID_HI;
    memcpy(&mkh_adv_buf[i], payload, payload_len);
    i += payload_len;

    gap_advertisements_set_data((uint8_t)i, mkh_adv_buf);
}

// WO12 Phase 2: shared by the periodic keep-alive path (mkh_broadcast_control())
// and the new event-driven out-of-turn path (mkh_event_send_callback() below) -
// builds a CONTROL telegram from the given channel values, encrypts it, and hands
// it to the stack. Telegram content/encoding is untouched (same
// mkh_protocol_control_telegram_neutral()/mkh_protocol_set_channel()/
// mkh_protocol_encrypt() calls either path already made) - only which of the two
// call sites triggers a send, and when, is new. Does NOT touch the periodic
// timer's own bookkeeping (mkh_time_slice_last_device, the OFF-grace countdown,
// changed-logging) - those stay exclusively owned by mkh_broadcast_control(),
// since an event send doesn't advance the round-robin or count as one of the OFF
// sequence's required neutral-airings (the periodic cycle's own next service
// still airs it and does that counting, per WO9's ">=3 full cycles" contract).
static void mkh_stage_and_send_values(int device_id, const uint8_t values[MKH_MK6_NUM_CHANNELS]) {
    uint8_t telegram[MKH_TELEGRAM_CONTROL_LEN];
    mkh_protocol_control_telegram_neutral(device_id, telegram);
    for (int ch = 0; ch < MKH_MK6_NUM_CHANNELS; ch++) {
        float power = ((float)values[ch] - 128.0f) / 128.0f;
        mkh_protocol_set_channel(telegram, ch, power);
    }

    uint8_t payload[MKH_PAYLOAD_CONTROL_LEN];
    size_t payload_len = mkh_protocol_encrypt(telegram, sizeof(telegram), payload, sizeof(payload));
    mkh_set_adv_payload(payload, payload_len);

    // Dashboard: this device is now (and continues to be) serviced - true
    // broadcast state, per WO9. Set here too (not just in
    // mkh_broadcast_control()) so an event send alone is enough to flip a
    // freshly-toggled-on device's dashboard row green without waiting for its
    // first periodic cycle.
    mkh_hub_broadcasting[device_id] = true;
}

// WO12 Phase 2: event-driven immediate sends. Root cause (see the completion
// report): the periodic ~100ms software timer was the ONLY thing that ever
// rebuilt/restaged a telegram, so an input change could sit unsent for up to
// one full timer period - doubled to ~200ms whenever a second hub is also in
// the round-robin, which is the normal both-hubs-on boot state, not an edge
// case. mkh_set_channel() below now triggers an immediate out-of-turn send the
// moment a port's value actually changes, for any active device; the periodic
// timer is unchanged and keeps running as a keep-alive so a value that stops
// changing still gets re-affirmed on air.
//
// Rate-limited per device (not globally - each device's own telegram is
// independent) so a wiggling/noisy stick can't flood the BLE stack; the ~20-30ms
// floor is well above the deadzone-filtered sweep's natural output rate for a
// single deliberate move but far below anything a human could perceive as lag.
#define MKH_EVENT_SEND_MIN_INTERVAL_US 25000
static int64_t mkh_last_event_send_us[MKH_MK6_NUM_DEVICES];

// mkh_set_channel() runs on whichever FreeRTOS task calls processMappedInputs()
// (the Arduino/main task, via sketch.cpp's loop()) - a different task than the
// one running the BTstack run loop. Calling BTstack APIs (gap_advertisements_
// set_data() et al, inside mkh_stage_and_send_values()) directly from that task
// would not be safe - BTstack's core is single-threaded by design.
// btstack_run_loop_execute_on_main_thread() is BTstack's documented, non-
// deprecated mechanism for exactly this: marshal a callback onto the correct
// thread instead of calling BTstack APIs directly from another task (see
// btstack_run_loop.h's doc comment on the function). One registration struct
// PER DEVICE (not one shared one) so a pending event for device 0 can never be
// clobbered by one for device 1 racing in before the first is processed.
static btstack_context_callback_registration_t mkh_event_callback_reg[MKH_MK6_NUM_DEVICES];

static void mkh_event_send_callback(void* context) {
    int device_id = (int)(intptr_t)context;
    uint8_t values[MKH_MK6_NUM_CHANNELS];
    xSemaphoreTake(mkh_channel_mutex, portMAX_DELAY);
    memcpy(values, mkh_channel_value[device_id], sizeof(values));
    xSemaphoreGive(mkh_channel_mutex);
    mkh_stage_and_send_values(device_id, values);
    logi("MK6 Broadcast: CONTROL telegram sent event-driven (device=%d)\n", device_id);
}

void mkh_set_channel(int device_id, int port, uint8_t value) {
    if (device_id < 0 || device_id >= MKH_MK6_NUM_DEVICES)
        return;
    if (port < 0 || port >= MKH_MK6_NUM_CHANNELS)
        return;

    xSemaphoreTake(mkh_channel_mutex, portMAX_DELAY);
    bool changed = mkh_channel_value[device_id][port] != value;
    mkh_channel_value[device_id][port] = value;
    xSemaphoreGive(mkh_channel_mutex);

    // WO12 Phase 2: event-driven out-of-turn send - see the machinery above.
    // Gated on mkh_device_active[device_id], the same flag mkh_next_active_
    // device() uses to decide who's in the periodic rotation: processMappedInputs()
    // (sketch.cpp) calls mkh_set_channel() for every CONFIGURED port every tick
    // regardless of a hub's dashboard on/off state (unchanged, pre-existing
    // behavior - config assignment and broadcast on/off are independent), so
    // without this gate a toggled-OFF or never-activated hub would get stray
    // event sends. A device mid-transition (mkh_device_transitioning[]) is still
    // mkh_device_active[] and correctly still gets event sends - transitioning is
    // a dashboard-tap debounce concept, not a broadcast-eligibility one.
    if (changed && mkh_device_active[device_id]) {
        int64_t now = esp_timer_get_time();
        if (now - mkh_last_event_send_us[device_id] >= MKH_EVENT_SEND_MIN_INTERVAL_US) {
            mkh_last_event_send_us[device_id] = now;
            mkh_event_callback_reg[device_id].callback = mkh_event_send_callback;
            mkh_event_callback_reg[device_id].context = (void*)(intptr_t)device_id;
            btstack_run_loop_execute_on_main_thread(&mkh_event_callback_reg[device_id]);
        }
    }
}

// Last-logged snapshot per device, so the (verbose) per-telegram log only
// prints when that device's channel values actually change, not on every
// ~100ms cycle it's serviced.
static uint8_t mkh_last_logged_values[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS];
static bool mkh_control_logged_once[MKH_MK6_NUM_DEVICES] = {false};

// Which device's CONTROL telegram gets staged next. Only one telegram can
// be on air at a time (unaddressed hubs simply coast until a telegram
// with their address arrives), so this rotates through whichever devices
// are currently mkh_device_active[] each time this function runs - i.e.
// each specific device's own telegram effectively refreshes every
// (active_count * MKH_CONTROL_REPEAT_MS)ms, not every MKH_CONTROL_REPEAT_MS.
// v0.9.0 Step 1: the active set is now runtime-toggleable (see
// mkh_broadcast_toggle_off()/_on()), not the fixed 0..MKH_TIME_SLICE_
// NUM_DEVICES-1 range Step 1 (v0.7.0) hardcoded - mkh_next_active_device()
// below walks around whichever devices are active starting just after
// the last-serviced one.
static int mkh_time_slice_last_device = -1;

// Returns the next active device after `after` (round-robin), or -1 if
// none are active right now (e.g. both hubs toggled off - see WO9 Step
// 2 bench plan: "Then both off, both on").
static int mkh_next_active_device(int after) {
    for (int i = 1; i <= MKH_MK6_NUM_DEVICES; i++) {
        int candidate = (after + i + MKH_MK6_NUM_DEVICES) % MKH_MK6_NUM_DEVICES;
        if (mkh_device_active[candidate])
            return candidate;
    }
    return -1;
}

static void mkh_broadcast_control(btstack_timer_source_t* ts) {
    int device_id = mkh_next_active_device(mkh_time_slice_last_device);
    if (device_id < 0) {
        // Nothing active right now - nothing to stage. Still re-arm so
        // we notice as soon as a hub is toggled back on.
        btstack_run_loop_set_timer(ts, MKH_CONTROL_REPEAT_MS);
        btstack_run_loop_add_timer(ts);
        return;
    }
    mkh_time_slice_last_device = device_id;

    uint8_t values[MKH_MK6_NUM_CHANNELS];
    xSemaphoreTake(mkh_channel_mutex, portMAX_DELAY);
    memcpy(values, mkh_channel_value[device_id], sizeof(values));
    xSemaphoreGive(mkh_channel_mutex);

    uint8_t telegram[MKH_TELEGRAM_CONTROL_LEN];
    mkh_protocol_control_telegram_neutral(device_id, telegram);

    bool changed =
        !mkh_control_logged_once[device_id] || memcmp(values, mkh_last_logged_values[device_id], sizeof(values)) != 0;

    char ports_log[MKH_MK6_NUM_CHANNELS * 8 + 1];
    size_t pos = 0;
    for (int ch = 0; ch < MKH_MK6_NUM_CHANNELS; ch++) {
        // Raw byte (0x00..0xFF, 0x80=neutral) -> power (-1.0..1.0), the
        // unit mkh_protocol_set_channel expects. Exact at the boundaries
        // (0x00, 0x80, 0xFF); mkh_protocol.c itself is untouched.
        float power = ((float)values[ch] - 128.0f) / 128.0f;
        mkh_protocol_set_channel(telegram, ch, power);

        if (changed) {
            int n = snprintf(ports_log + pos, sizeof(ports_log) - pos, "%c=%02X ", mkh_port_letter(ch), values[ch]);
            if (n > 0)
                pos += (size_t)n;
        }
    }

    uint8_t payload[MKH_PAYLOAD_CONTROL_LEN];
    size_t payload_len = mkh_protocol_encrypt(telegram, sizeof(telegram), payload, sizeof(payload));

    mkh_set_adv_payload(payload, payload_len);

    // Dashboard: this device is now (and continues to be) serviced by
    // the time-slicer - true broadcast state, per WO9.
    mkh_hub_broadcasting[device_id] = true;

    // v0.9.0 Step 1: safety-ordered OFF sequence, second half. The first
    // half (commanding neutral, starting the countdown) already ran
    // synchronously in mkh_broadcast_toggle_off() when the user tapped.
    // This device just aired a real telegram (neutral, since toggle_off
    // already committed 0x80 to every channel) - only NOW does it count
    // as one of the >=3 required neutral-airings. Reaching zero here is
    // the sole point that actually drops the device from rotation and
    // flips the dashboard to red; never done synchronously in the
    // toggle request itself, so a hub already coasting on a non-neutral
    // order can never be dropped before neutral has genuinely gone out.
    if (mkh_pending_off_services[device_id] > 0) {
        mkh_pending_off_services[device_id]--;
        if (mkh_pending_off_services[device_id] == 0) {
            mkh_device_active[device_id] = false;
            mkh_hub_broadcasting[device_id] = false;
            mkh_device_transitioning[device_id] = false;  // OFF sequence genuinely complete now
            logi("MK6 Broadcast: device=%d dropped from rotation (neutral aired %d times)\n", device_id,
                 MKH_OFF_GRACE_SERVICES);
        }
    }

    if (changed) {
        mkh_log_hex("CONTROL raw telegram", telegram, sizeof(telegram));
        mkh_log_hex("CONTROL encrypted payload", payload, payload_len);
        logi("MK6 Broadcast: CONTROL telegram updated (device=%d), ports: %s\n", device_id, ports_log);
        memcpy(mkh_last_logged_values[device_id], values, sizeof(values));
        mkh_control_logged_once[device_id] = true;
    }

    // Re-arm: rebuild/stage the next device's telegram every ~100ms.
    // gap_advertisements_set_data() only stages new advertising *data*
    // (not parameters), which BTstack sends without stopping/restarting
    // advertising - confirmed against components/btstack/src/hci.c: the
    // stop-before-reconfigure path is gated on
    // LE_ADVERTISEMENT_TASKS_SET_PARAMS, not SET_ADV_DATA. Safe to call
    // every cycle even though which device it targets now changes cycle
    // to cycle.
    btstack_run_loop_set_timer(ts, MKH_CONTROL_REPEAT_MS);
    btstack_run_loop_add_timer(ts);
}

static void mkh_broadcast_connect(void) {
    uint8_t telegram[MKH_TELEGRAM_CONNECT_LEN];
    mkh_protocol_connect_telegram(telegram);
    mkh_log_hex("CONNECT raw telegram", telegram, sizeof(telegram));

    uint8_t payload[MKH_PAYLOAD_CONNECT_LEN];
    size_t payload_len = mkh_protocol_encrypt(telegram, sizeof(telegram), payload, sizeof(payload));
    mkh_log_hex("CONNECT encrypted payload", payload, payload_len);

    mkh_set_adv_payload(payload, payload_len);
    gap_advertisements_enable(1);

    // mkh_hub_broadcasting[] is set per-device once the time-slicer
    // actually starts staging that device's CONTROL telegram - see
    // mkh_broadcast_control().
    logi("MK6 Broadcast: advertising STARTED, broadcasting CONNECT telegram for %dms before switching to CONTROL\n",
         MKH_CONNECT_PHASE_MS);

    btstack_run_loop_set_timer_handler(&mkh_broadcast_timer, &mkh_broadcast_control);
    btstack_run_loop_set_timer(&mkh_broadcast_timer, MKH_CONNECT_PHASE_MS);
    btstack_run_loop_add_timer(&mkh_broadcast_timer);
}

// v0.9.0 Step 1: per-hub broadcast toggles - see mkh_broadcast.h for the
// caller-facing contract (idempotent, own their full sequencing).

void mkh_broadcast_toggle_off(int device_id) {
    if (device_id < 0 || device_id >= MKH_MK6_NUM_DEVICES)
        return;
    if (!mkh_device_active[device_id] || mkh_pending_off_services[device_id] > 0)
        return;  // already off, or already dropping - idempotent

    mkh_device_transitioning[device_id] = true;  // cleared when the drop actually completes, below

    for (int ch = 0; ch < MKH_MK6_NUM_CHANNELS; ch++) {
        mkh_set_channel(device_id, ch, 0x80);
    }
    mkh_pending_off_services[device_id] = MKH_OFF_GRACE_SERVICES;
    logi("MK6 Broadcast: device=%d toggle OFF requested - neutral commanded, staying in rotation for %d more "
         "services before drop\n",
         device_id, MKH_OFF_GRACE_SERVICES);
}

void mkh_broadcast_toggle_on(int device_id) {
    if (device_id < 0 || device_id >= MKH_MK6_NUM_DEVICES)
        return;
    if (device_id >= MKH_TIME_SLICE_NUM_DEVICES) {
        logi("MK6 Broadcast: device=%d toggle ON ignored - third hub stays parked per WO9 (not authorized to "
             "activate live broadcast)\n",
             device_id);
        return;
    }
    if (mkh_device_active[device_id] && mkh_pending_off_services[device_id] == 0)
        return;  // already fully on - idempotent

    mkh_device_transitioning[device_id] = true;  // cleared right before returning below - ON is synchronous/atomic

    mkh_pending_off_services[device_id] = 0;  // cancel any pending drop - re-enabling wins

    for (int ch = 0; ch < MKH_MK6_NUM_CHANNELS; ch++) {
        mkh_set_channel(device_id, ch, 0x80);
    }

    // Re-run the CONNECT telegram sequence. It is broadcast, not
    // addressed (see the file header comment), so this is harmless to
    // other hubs - it re-acquires a hub that may have timed out off-
    // rotation, same principle as BrickController2's idle-reconnect.
    uint8_t telegram[MKH_TELEGRAM_CONNECT_LEN];
    mkh_protocol_connect_telegram(telegram);
    mkh_log_hex("CONNECT raw telegram (toggle ON replay)", telegram, sizeof(telegram));
    uint8_t payload[MKH_PAYLOAD_CONNECT_LEN];
    size_t payload_len = mkh_protocol_encrypt(telegram, sizeof(telegram), payload, sizeof(payload));
    mkh_log_hex("CONNECT encrypted payload (toggle ON replay)", payload, payload_len);
    mkh_set_adv_payload(payload, payload_len);

    mkh_device_active[device_id] = true;
    mkh_hub_broadcasting[device_id] = true;
    mkh_device_transitioning[device_id] = false;  // re-add complete
    logi("MK6 Broadcast: device=%d toggle ON - CONNECT replayed, re-added to rotation at neutral\n", device_id);
}

static void mkh_start(void) {
    gap_advertisements_set_params(MKH_ADV_INTERVAL_UNITS, MKH_ADV_INTERVAL_UNITS, MKH_ADV_TYPE_NONCONN_IND, 0,
                                   mkh_null_addr, 0x07 /* all 3 adv channels */, 0 /* no filter */);
    mkh_broadcast_connect();
}

static void mkh_check_command_complete(const uint8_t* packet) {
    uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
    if (opcode != hci_le_set_advertising_parameters.opcode && opcode != hci_le_set_advertising_data.opcode &&
        opcode != hci_le_set_advertise_enable.opcode) {
        return;
    }

    uint8_t status = hci_event_command_complete_get_return_parameters(packet)[0];
    if (status != ERROR_CODE_SUCCESS) {
        loge("MK6 Broadcast: GAP error, opcode=0x%04x, status=0x%02x\n", opcode, status);
    }
}

static void mkh_broadcast_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    (void)channel;
    (void)size;

    if (packet_type != HCI_EVENT_PACKET)
        return;

    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING && !mkh_advertising_started) {
                mkh_advertising_started = true;
                mkh_start();
            }
            break;

        case HCI_EVENT_COMMAND_COMPLETE:
            mkh_check_command_complete(packet);
            break;

        // Defensive re-assert: not expected to be needed for non-connectable
        // advertising against a central-role link (see coexistence note
        // above), but cheap insurance against any state we didn't predict.
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            if (mkh_advertising_started) {
                logi("MK6 Broadcast: disconnection event seen, re-asserting advertising enable\n");
                gap_advertisements_enable(1);
            }
            break;

        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE_V1:
                case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE_V2:
                    if (mkh_advertising_started) {
                        logi("MK6 Broadcast: LE connection-complete event seen, re-asserting advertising enable\n");
                        gap_advertisements_enable(1);
                    }
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
}

void mkh_broadcast_init(void) {
    mkh_channel_mutex = xSemaphoreCreateMutex();

    // v0.9.0 Step 1: devices below MKH_TIME_SLICE_NUM_DEVICES start
    // active, reproducing v0.7.0/v0.8.0 behavior exactly (both hubs
    // live from boot); anything at or beyond it (the third hub) starts
    // parked. No device starts with a pending OFF grace period.
    for (int d = 0; d < MKH_MK6_NUM_DEVICES; d++) {
        mkh_device_active[d] = (d < MKH_TIME_SLICE_NUM_DEVICES);
        mkh_pending_off_services[d] = 0;
        mkh_device_transitioning[d] = false;
    }

    mkh_hci_event_callback_registration.callback = &mkh_broadcast_packet_handler;
    hci_add_event_handler(&mkh_hci_event_callback_registration);
}
