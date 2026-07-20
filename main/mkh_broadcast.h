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

// v0.9.0 Step 1: per-hub broadcast toggles. Both functions own their
// full safety-ordered sequencing internally (see mkh_broadcast.c) -
// callers (the touch handler in sketch.cpp) just dispatch the request;
// they must not try to replicate the ordering themselves. Both are
// idempotent (calling toggle_off while already off/dropping, or
// toggle_on while already fully on, is a no-op) and safe to call from
// any task (same mutex-protected path as mkh_set_channel()).
//
// toggle_off: commands every channel of device_id to neutral (0x80)
// immediately, but keeps the device in the CONTROL rotation for several
// more services so neutral actually airs before the device is dropped -
// never removes a hub that might still be coasting on a non-neutral
// order.
void mkh_broadcast_toggle_off(int device_id);

// toggle_on: re-broadcasts the (unaddressed, shared) CONNECT telegram
// once - harmless to other hubs, and re-acquires a hub that may have
// timed out off-rotation, same principle as BC2's idle-reconnect - then
// re-adds device_id to the CONTROL rotation at neutral. Devices at or
// beyond MKH_TIME_SLICE_NUM_DEVICES (see mkh_broadcast.c) stay parked -
// WO9 does not authorize activating the third hub's live broadcast;
// calling toggle_on for a parked device logs why and does nothing else
// (does not enter the transitioning state below).
void mkh_broadcast_toggle_on(int device_id);

// v0.9.0 Step 1 (PM feedback pass): true from the moment a real (non-
// idempotent) toggle request is accepted until its sequence actually
// completes - the drop for OFF, the re-add for ON. This is the real
// guard against a tap landing mid-sequence; callers (sketch.cpp) must
// check this BEFORE dispatching a new toggle request for the same
// device and ignore the tap if true, logging why. The ~200ms debounce
// stays as an additional, secondary guard on top of this, not a
// replacement for it.
bool mkh_broadcast_is_transitioning(int device_id);

#ifdef __cplusplus
}
#endif

#endif  // MKH_BROADCAST_H
