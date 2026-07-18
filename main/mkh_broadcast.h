// SPDX-License-Identifier: Apache-2.0
//
// MK6 Broadcast Stage 1: proves the firmware can run a non-connectable BLE
// advertisement on the same BTstack instance Bluepad32 already drives,
// concurrently with the XWC (Xbox Wireless Controller) connection.

#ifndef MKH_BROADCAST_H
#define MKH_BROADCAST_H

#include <stdbool.h>

#include "mkh_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// Registers the HCI event handler that drives the advertising state
// machine. Must be called after btstack_init() (so the HCI stack exists)
// and before hci_power_control(HCI_POWER_ON) is issued (so no
// BTSTACK_EVENT_STATE / HCI_STATE_WORKING transition is missed).
void mkh_broadcast_init(void);

// True for device slots this firmware is actively broadcasting
// CONNECT/CONTROL telegrams to (index = MK6 protocol device slot 0..2,
// zero-based). The dashboard's MKH rows read this directly by the same
// index (see mkh_ports.h for the app-facing 1-based MKH 1/2/3 numbering).
extern bool mkh_hub_broadcasting[MKH_MK6_NUM_DEVICES];

// Thread-safe setter for a device's live channel values, read by the
// broadcast timer when building that device's CONTROL telegram.
// device_id is the MK6 device slot 0..2; port is a channel index 0..5
// (see mkh_ports.h for the A..F labels); value is the raw telegram byte
// (0x00..0xFF, 0x80 = neutral), matching the wire format directly. Safe
// to call from any task - internally serialized against the broadcast
// timer's read via a mutex, since callers (e.g. a Bluepad32 controller
// callback) run on a different FreeRTOS task than the BTstack run loop
// that builds telegrams.
void mkh_set_channel(int device_id, int port, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif  // MKH_BROADCAST_H
