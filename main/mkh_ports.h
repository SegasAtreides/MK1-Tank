// SPDX-License-Identifier: Apache-2.0
//
// Human-facing port labels for MK6 hub outputs - presentation layer only.
//
// The protocol layer (mkh_protocol.h/.c) always addresses hub outputs by
// numeric channel index 0..5, matching the byte offsets in the real
// telegram wire format - that mapping is proven-correct and untouched by
// this header. This file is the single source of truth for how those
// same 0..5 indices are labeled everywhere human-facing (dashboard,
// serial logs, config): the physical port letters A..F silkscreened on
// the hub.

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

#ifdef __cplusplus
}
#endif

#endif  // MKH_PORTS_H
