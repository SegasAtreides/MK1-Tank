// SPDX-License-Identifier: Apache-2.0
//
// MK6 Broadcast Stage 1: proves the firmware can run a non-connectable BLE
// advertisement on the same BTstack instance Bluepad32 already drives,
// concurrently with the XWC (Xbox Wireless Controller) connection.

#ifndef MKH_BROADCAST_H
#define MKH_BROADCAST_H

#ifdef __cplusplus
extern "C" {
#endif

// Registers the HCI event handler that drives the advertising state
// machine. Must be called after btstack_init() (so the HCI stack exists)
// and before hci_power_control(HCI_POWER_ON) is issued (so no
// BTSTACK_EVENT_STATE / HCI_STATE_WORKING transition is missed).
void mkh_broadcast_init(void);

#ifdef __cplusplus
}
#endif

#endif  // MKH_BROADCAST_H
