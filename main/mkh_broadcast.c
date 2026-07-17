// SPDX-License-Identifier: Apache-2.0
//
// MK6 Broadcast. Stage 1 proved a non-connectable BLE advertisement can run
// on the same BTstack instance Bluepad32 already drives, concurrently with
// the XWC (Xbox Wireless Controller) connection. Stage 2 replaces the test
// payload with the real Mould King 6.0 protocol (see mkh_protocol.h/.c):
// broadcast the CONNECT telegram to switch a hub into Bluetooth mode, then
// switch to repeating a CONTROL telegram. Hardcoded to device 0, port A,
// full speed forward - not yet controller-driven.
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
// The "repeat the CONTROL telegram every ~100-200ms" requirement is met by
// BLE's own hardware-level advertising interval (MKH_ADV_INTERVAL_UNITS
// below) once the payload is set - no software repeat timer is needed for
// steady-state. A single one-shot BTstack timer sequences the CONNECT ->
// CONTROL switch on boot.

#include "mkh_broadcast.h"

#include <btstack.h>
#include <stdio.h>
#include <string.h>

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

// Hardcoded Stage 2 test target: device slot 0, port A, full forward.
#define MKH_TEST_DEVICE_ID 0
#define MKH_TEST_PORT MKH_PORT_A
#define MKH_TEST_POWER 1.0f

// How long to broadcast CONNECT before switching to CONTROL (a handful of
// repeats at the ~100ms advertising interval).
#define MKH_CONNECT_PHASE_MS 600

static btstack_packet_callback_registration_t mkh_hci_event_callback_registration;
static bool mkh_advertising_started = false;
static btstack_timer_source_t mkh_connect_to_control_timer;

// Definition for the extern declared in mkh_broadcast.h.
bool mkh_hub_broadcasting[MKH_MK6_NUM_DEVICES] = {false};

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

static void mkh_broadcast_control(btstack_timer_source_t* ts) {
    (void)ts;

    uint8_t telegram[MKH_TELEGRAM_CONTROL_LEN];
    mkh_protocol_control_telegram_neutral(MKH_TEST_DEVICE_ID, telegram);
    mkh_protocol_set_channel(telegram, MKH_TEST_PORT, MKH_TEST_POWER);
    mkh_log_hex("CONTROL raw telegram", telegram, sizeof(telegram));

    uint8_t payload[MKH_PAYLOAD_CONTROL_LEN];
    size_t payload_len = mkh_protocol_encrypt(telegram, sizeof(telegram), payload, sizeof(payload));
    mkh_log_hex("CONTROL encrypted payload", payload, payload_len);

    mkh_set_adv_payload(payload, payload_len);

    logi("MK6 Broadcast: now repeating CONTROL telegram (device=%d, port=%c, power=full-forward) at ~100ms\n",
         MKH_TEST_DEVICE_ID, mkh_port_letter(MKH_TEST_PORT));
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

    // Dashboard: device MKH_TEST_DEVICE_ID is now actively being broadcast
    // to. Nothing currently stops broadcasting once started (no stop/idle
    // trigger exists yet - that arrives with real controller-driven
    // control), so there is no corresponding "set false" path yet.
    mkh_hub_broadcasting[MKH_TEST_DEVICE_ID] = true;

    logi("MK6 Broadcast: advertising STARTED, broadcasting CONNECT telegram for %dms before switching to CONTROL\n",
         MKH_CONNECT_PHASE_MS);

    btstack_run_loop_set_timer_handler(&mkh_connect_to_control_timer, &mkh_broadcast_control);
    btstack_run_loop_set_timer(&mkh_connect_to_control_timer, MKH_CONNECT_PHASE_MS);
    btstack_run_loop_add_timer(&mkh_connect_to_control_timer);
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
    mkh_hci_event_callback_registration.callback = &mkh_broadcast_packet_handler;
    hci_add_event_handler(&mkh_hci_event_callback_registration);
}
