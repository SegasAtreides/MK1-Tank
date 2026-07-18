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

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mkh_ports.h"
#include "mkh_protocol.h"
#include "uni_log.h"

// 160 * 0.625ms = 100ms advertising interval.
#define MKH_ADV_INTERVAL_UNITS 160
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

// Number of devices actively serviced by the time-slice rotation. Step 1
// covers devices 0 and 1 (a second physical hub); device slot 2 already
// exists in the per-device arrays below ("leave room for 2") but isn't in
// rotation yet - bump this to MKH_MK6_NUM_DEVICES when it joins.
#define MKH_TIME_SLICE_NUM_DEVICES 2

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

void mkh_set_channel(int device_id, int port, uint8_t value) {
    if (device_id < 0 || device_id >= MKH_MK6_NUM_DEVICES)
        return;
    if (port < 0 || port >= MKH_MK6_NUM_CHANNELS)
        return;

    xSemaphoreTake(mkh_channel_mutex, portMAX_DELAY);
    mkh_channel_value[device_id][port] = value;
    xSemaphoreGive(mkh_channel_mutex);
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

// Last-logged snapshot per device, so the (verbose) per-telegram log only
// prints when that device's channel values actually change, not on every
// ~100ms cycle it's serviced.
static uint8_t mkh_last_logged_values[MKH_MK6_NUM_DEVICES][MKH_MK6_NUM_CHANNELS];
static bool mkh_control_logged_once[MKH_MK6_NUM_DEVICES] = {false};

// Which device's CONTROL telegram gets staged next. Only one telegram can
// be on air at a time (unaddressed hubs simply coast until a telegram
// with their address arrives), so this rotates through the devices in
// MKH_TIME_SLICE_NUM_DEVICES each time this function runs - i.e. each
// specific device's own telegram effectively refreshes every
// (MKH_TIME_SLICE_NUM_DEVICES * MKH_CONTROL_REPEAT_MS)ms, not every
// MKH_CONTROL_REPEAT_MS.
static int mkh_time_slice_next_device = 0;

static void mkh_broadcast_control(btstack_timer_source_t* ts) {
    int device_id = mkh_time_slice_next_device;
    mkh_time_slice_next_device = (mkh_time_slice_next_device + 1) % MKH_TIME_SLICE_NUM_DEVICES;

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

    // Dashboard: this device is now (and henceforth) serviced by the
    // time-slicer. Nothing currently drops a device back out of rotation
    // (no stop/idle trigger exists yet), so there is no corresponding
    // "set false" path yet.
    mkh_hub_broadcasting[device_id] = true;

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

    mkh_hci_event_callback_registration.callback = &mkh_broadcast_packet_handler;
    hci_add_event_handler(&mkh_hci_event_callback_registration);
}
