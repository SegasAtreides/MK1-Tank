// SPDX-License-Identifier: Apache-2.0
//
// Mould King 6.0 hub RC telegram encoder.
//
// Ported from, and cross-checked byte-for-byte against, three independent
// reverse-engineering references:
//  - BrickController2 (C#): github.com/vicocz/brickcontroller2
//    DeviceManagement/MouldKing/MK6.cs, Protocols/CryptTools.cs, Protocols/MKProtocol.cs
//  - Espruino mouldking.js: github.com/espruino/EspruinoDocs
//    devices/mouldking.js (get_nrf_payload / whitening_encode / check_crc16)
//  - J0EK3R/mkconnect-python: github.com/J0EK3R/mkconnect-python
//    MouldKing/MouldKingCrypt.py, MouldKing/MouldKing_Hub_6.py
//
// All three agree on the telegram bytes, the seed/header constants, and the
// two-pass LFSR whitening + CRC16 transform, including the "whiten the
// whole staging buffer from offset 0" quirk (see mkh_protocol.c).

#ifndef MKH_PROTOCOL_H
#define MKH_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Raw (pre-encryption) telegram sizes.
#define MKH_TELEGRAM_CONNECT_LEN 8
#define MKH_TELEGRAM_CONTROL_LEN 10

// Encrypted over-the-air payload sizes (raw length + 10, per the protocol's
// header(3) + seed(5) + checksum(2) framing).
#define MKH_PAYLOAD_CONNECT_LEN (MKH_TELEGRAM_CONNECT_LEN + 10)
#define MKH_PAYLOAD_CONTROL_LEN (MKH_TELEGRAM_CONTROL_LEN + 10)

// Number of MK6 device slots (hub address 0..2, set on the hub via its
// short-press blue-flash count).
#define MKH_MK6_NUM_DEVICES 3
#define MKH_MK6_NUM_CHANNELS 6

// Fills out[MKH_TELEGRAM_CONNECT_LEN] with the raw MK6 CONNECT telegram
// (switches the hub into Bluetooth mode). Same for every device slot.
void mkh_protocol_connect_telegram(uint8_t* out);

// Fills out[MKH_TELEGRAM_CONTROL_LEN] with the raw MK6 CONTROL (base)
// telegram for the given device slot (0..2), with every channel neutral
// (0x80 = stopped).
void mkh_protocol_control_telegram_neutral(int device_id, uint8_t* out);

// Sets one channel (0..5) of an already-built control telegram (as
// produced by mkh_protocol_control_telegram_neutral) to the given power.
// power is clamped to [-1.0, 1.0]; negative = reverse, 0 = stop.
void mkh_protocol_set_channel(uint8_t* telegram, int channel, float power);

// Encrypts a raw telegram (the GetRfPayload / Crypt transform) into its
// over-the-air payload. data_len must be MKH_TELEGRAM_CONNECT_LEN or
// MKH_TELEGRAM_CONTROL_LEN. out_capacity must be at least data_len + 10.
// Returns the payload length written (data_len + 10), or 0 if out_capacity
// is too small.
size_t mkh_protocol_encrypt(const uint8_t* data, size_t data_len, uint8_t* out, size_t out_capacity);

#ifdef __cplusplus
}
#endif

#endif  // MKH_PROTOCOL_H
