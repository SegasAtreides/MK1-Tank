// SPDX-License-Identifier: Apache-2.0
//
// Human-facing labels for MK6 hub addressing - presentation layer only.
//
// The protocol layer (mkh_protocol.h/.c) always addresses hub outputs by
// numeric channel index 0..5 and hub device slots by numeric index 0..2,
// matching the wire format directly - those mappings are proven-correct
// and untouched by this header. This file is the single source of truth
// for how those same numeric indices are labeled everywhere human-facing
// (dashboard, serial logs, config): the physical port letters A..F
// silkscreened on the hub, and the hub's own 1-based numbering (matching
// its blue-flash LED blink count when you set its address).

#ifndef MKH_PORTS_H
#define MKH_PORTS_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
    MKH_PORT_A = 0,
    MKH_PORT_B = 1,
    MKH_PORT_C = 2,
    MKH_PORT_D = 3,
    MKH_PORT_E = 4,
    MKH_PORT_F = 5,
};

// Returns the physical port letter ('A'..'F') for a channel index (0..5).
static inline char mkh_port_letter(int channel) {
    return (char)('A' + channel);
}

// Returns the hub's own 1-based device number (matching its blue-flash
// LED blink count) for a protocol device slot index (0..2). Protocol
// device 0 -> hub/app number 1, device 1 -> 2, device 2 -> 3.
static inline int mkh_device_app_number(int device_id) {
    return device_id + 1;
}

#ifdef __cplusplus
}
#endif

#endif  // MKH_PORTS_H
