// SPDX-License-Identifier: Apache-2.0
//
// MK6 Broadcast Stage 1: proves the firmware can run a non-connectable BLE
// advertisement on the same BTstack instance Bluepad32 already drives,
// concurrently with the XWC (Xbox Wireless Controller) connection.
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

#include "mkh_broadcast.h"

#include <btstack.h>

#include "uni_log.h"

// 160 * 0.625ms = 100ms advertising interval.
#define MKH_ADV_INTERVAL_UNITS 160
// Non-connectable, non-scannable undirected advertising (BT Core spec
// legacy advertising type enum, see components/btstack/src/hci.c around
// the ADV_NONCONN_IND decode and hci_cmd.c's hci_le_set_advertising_parameters).
#define MKH_ADV_TYPE_NONCONN_IND 3

static btstack_packet_callback_registration_t mkh_hci_event_callback_registration;
static bool mkh_advertising_started = false;

static bd_addr_t mkh_null_addr = {0, 0, 0, 0, 0, 0};

// AD structures: Flags + Manufacturer Specific Data carrying the
// recognizable test payload 0xAA 0xBB 0xCC (company id 0xFFFF is the
// "no assigned company" placeholder - this is a test broadcast only,
// not a real Mould King telegram).
static const uint8_t mkh_adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    0x06, BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA, 0xFF, 0xFF, 0xAA, 0xBB, 0xCC,
};

static void mkh_start_advertising(void) {
    gap_advertisements_set_params(MKH_ADV_INTERVAL_UNITS, MKH_ADV_INTERVAL_UNITS, MKH_ADV_TYPE_NONCONN_IND, 0,
                                   mkh_null_addr, 0x07 /* all 3 adv channels */, 0 /* no filter */);
    gap_advertisements_set_data(sizeof(mkh_adv_data), (uint8_t*)mkh_adv_data);
    gap_advertisements_enable(1);

    logi("MK6 Broadcast: advertising STARTED (non-connectable, ~100ms interval, payload AA BB CC)\n");
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
                mkh_start_advertising();
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
