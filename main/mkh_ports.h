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

#include "mkh_protocol.h"  // MKH_MK6_NUM_DEVICES, MKH_MK6_NUM_CHANNELS

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

// Returns the channel index (0..5) for a physical port letter ('A'..'F',
// case-insensitive), or -1 if not a valid port letter. Inverse of
// mkh_port_letter() - used by the config parser (mkh_config.c) so the
// letter<->index mapping lives in exactly one place.
static inline int mkh_port_from_letter(char letter) {
    char upper = (letter >= 'a' && letter <= 'z') ? (char)(letter - 'a' + 'A') : letter;
    if (upper < 'A' || upper >= 'A' + MKH_MK6_NUM_CHANNELS)
        return -1;
    return upper - 'A';
}

// Returns the hub's own 1-based device number (matching its blue-flash
// LED blink count) for a protocol device slot index (0..2). Protocol
// device 0 -> hub/app number 1, device 1 -> 2, device 2 -> 3.
static inline int mkh_device_app_number(int device_id) {
    return device_id + 1;
}

// Returns the protocol device slot index (0..2) for a hub's own 1-based
// device number, or -1 if out of range (1..MKH_MK6_NUM_DEVICES). Inverse
// of mkh_device_app_number() - used by the config parser.
static inline int mkh_device_protocol_index(int app_number) {
    if (app_number < 1 || app_number > MKH_MK6_NUM_DEVICES)
        return -1;
    return app_number - 1;
}

#ifdef __cplusplus
}
#endif

#endif  // MKH_PORTS_H
