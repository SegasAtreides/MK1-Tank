// SPDX-License-Identifier: Apache-2.0
//
// Mould King 6.0 hub RC telegram encoder. See mkh_protocol.h for the
// reference sources this was cross-checked against.

#include "mkh_protocol.h"

#include <string.h>

// SeedArray / HeaderArray / CTX values: identical across all three
// references (BrickController2 MKProtocol.cs, Espruino DEVICE_ADDRESS +
// get_nrf_payload constants, mkconnect-python Array_C1C2C3C4C5).
static const uint8_t kSeed[5] = {0xC1, 0xC2, 0xC3, 0xC4, 0xC5};
static const uint8_t kHeader[3] = {0x71, 0x0f, 0x55};
static const uint8_t kCtxValue1 = 0x3f;  // whitens seed+data+checksum
static const uint8_t kCtxValue2 = 0x25;  // whitens the whole staging buffer

// MK6 raw telegrams (device slot 0/1/2 = hub address set via short-press
// blue-flash count). Identical byte-for-byte across all three references.
static const uint8_t kTelegramConnect[MKH_TELEGRAM_CONNECT_LEN] = {0x6D, 0x7B, 0xA7, 0x80, 0x80, 0x80, 0x80, 0x92};
static const uint8_t kTelegramBase[MKH_MK6_NUM_DEVICES][MKH_TELEGRAM_CONTROL_LEN] = {
    {0x61, 0x7B, 0xA7, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x9E},  // device 0
    {0x62, 0x7B, 0xA7, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x9D},  // device 1
    {0x63, 0x7B, 0xA7, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x9C},  // device 2
};
// Channel 0 lives at byte offset 3 in the control telegram (channels 0..5
// occupy offsets 3..8; offset 9 is the fixed per-device trailer byte).
static const int kControlChannelStartOffset = 3;

void mkh_protocol_connect_telegram(uint8_t* out) {
    memcpy(out, kTelegramConnect, MKH_TELEGRAM_CONNECT_LEN);
}

void mkh_protocol_control_telegram_neutral(int device_id, uint8_t* out) {
    memcpy(out, kTelegramBase[device_id], MKH_TELEGRAM_CONTROL_LEN);
}

void mkh_protocol_set_channel(uint8_t* telegram, int channel, float power) {
    if (power > 1.0f)
        power = 1.0f;
    if (power < -1.0f)
        power = -1.0f;

    uint8_t byte_value;
    if (power > 0) {
        float scaled = power * 0x80;
        if (scaled > 0x80)
            scaled = 0x80;
        int v = 0x80 + (int)scaled;
        byte_value = (v > 0xFF) ? 0xFF : (uint8_t)v;
    } else if (power < 0) {
        float scaled = -power * 0x80;
        if (scaled > 0x80)
            scaled = 0x80;
        int v = 0x80 - (int)scaled;
        byte_value = (v < 0x00) ? 0x00 : (uint8_t)v;
    } else {
        byte_value = 0x80;
    }

    telegram[kControlChannelStartOffset + channel] = byte_value;
}

static uint8_t reverse_bits8(uint8_t value) {
    uint8_t result = 0;
    for (int bit = 0; bit < 8; bit++) {
        if (value & (1 << bit))
            result |= (uint8_t)(1 << (7 - bit));
    }
    return result;
}

static uint16_t reverse_bits16(uint16_t value) {
    uint16_t result = 0;
    for (int bit = 0; bit < 16; bit++) {
        if (value & (1 << bit))
            result |= (uint16_t)(1 << (15 - bit));
    }
    return result;
}

// CRC16-CCITT (poly 0x1021, init 0xFFFF): seed processed in reverse byte
// order, data processed forward with each byte bit-reversed first, final
// result bit-reversed and XORed with 0xFFFF. Identical in all three
// references (CryptTools.CheckCRC16 / check_crc16 / __calc_checksum_from_arrays).
static uint16_t crc16(const uint8_t* seed, size_t seed_len, const uint8_t* data, size_t data_len) {
    uint32_t result = 0xFFFF;

    for (size_t i = seed_len; i-- > 0;) {
        result ^= (uint32_t)seed[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            result = (result & 0x8000) ? ((result << 1) ^ 0x1021) : (result << 1);
            result &= 0xFFFF;
        }
    }

    for (size_t i = 0; i < data_len; i++) {
        result ^= (uint32_t)reverse_bits8(data[i]) << 8;
        for (int bit = 0; bit < 8; bit++) {
            result = (result & 0x8000) ? ((result << 1) ^ 0x1021) : (result << 1);
            result &= 0xFFFF;
        }
    }

    return (uint16_t)(reverse_bits16((uint16_t)result) ^ 0xFFFF);
}

// LFSR whitening (7-stage), keyed by a 6-bit value: ctx[0]=1, ctx[1..6] =
// bits 5..0 of the key. Each output bit is the pre-shift ctx[6]; the shift
// register then advances ctx[0..6] <- {ctx[6], ctx[0], ctx[1], ctx[2],
// ctx[3]^ctx[6], ctx[4], ctx[5]}. Identical in all three references
// (WhiteningInit/WhiteningOutput, whitening_init/whitening step,
// __create_magic_array/__shift_magic_array).
static void whitening_init(uint8_t key, uint8_t ctx[7]) {
    ctx[0] = 1;
    for (int i = 0; i < 6; i++)
        ctx[i + 1] = (key >> (5 - i)) & 1;
}

static uint8_t whitening_step(uint8_t ctx[7]) {
    uint8_t c3 = ctx[3];
    uint8_t c6 = ctx[6];
    ctx[3] = ctx[2];
    ctx[2] = ctx[1];
    ctx[1] = ctx[0];
    ctx[0] = ctx[6];
    ctx[6] = ctx[5];
    ctx[5] = ctx[4];
    ctx[4] = (uint8_t)(c3 ^ c6);
    return ctx[0];
}

static void whitening_encode(uint8_t* buf, size_t start, size_t len, uint8_t ctx[7]) {
    for (size_t i = 0; i < len; i++) {
        uint8_t b = buf[start + i];
        uint8_t out = 0;
        for (int bit = 0; bit < 8; bit++) {
            uint8_t keystream_bit = whitening_step(ctx);
            out |= (uint8_t)((keystream_bit ^ ((b >> bit) & 1)) << bit);
        }
        buf[start + i] = out;
    }
}

// GetRfPayload / get_nrf_payload / Crypt: stages header+reversed-seed+data+
// checksum into a buffer starting at a fixed offset of 15 (a quirk shared
// by all three references - the first 15 bytes are never written with
// meaningful data, but whitening pass 2 runs over the buffer from index 0,
// so those 15 bytes' worth of LFSR steps still have to happen to land on
// the correct keystream phase for the real payload). Only the final
// [15, 15+result_len) slice is meaningful and returned.
size_t mkh_protocol_encrypt(const uint8_t* data, size_t data_len, uint8_t* out, size_t out_capacity) {
    const size_t seed_len = sizeof(kSeed);
    const size_t header_len = sizeof(kHeader);
    const size_t header_offset = 15;
    const size_t seed_offset = header_offset + header_len;
    const size_t data_offset = seed_offset + seed_len;
    const size_t checksum_offset = data_offset + data_len;
    const size_t result_len = header_len + seed_len + data_len + 2;
    const size_t buffer_len = checksum_offset + 2;

    if (result_len > out_capacity)
        return 0;

    uint8_t buf[64] = {0};
    if (buffer_len > sizeof(buf))
        return 0;

    memcpy(buf + header_offset, kHeader, header_len);
    for (size_t i = 0; i < seed_len; i++)
        buf[seed_offset + i] = kSeed[seed_len - 1 - i];
    for (size_t i = 0; i < header_len + seed_len; i++)
        buf[header_offset + i] = reverse_bits8(buf[header_offset + i]);

    memcpy(buf + data_offset, data, data_len);

    uint16_t crc = crc16(kSeed, seed_len, data, data_len);
    buf[checksum_offset] = (uint8_t)(crc & 0xFF);
    buf[checksum_offset + 1] = (uint8_t)((crc >> 8) & 0xFF);

    uint8_t ctx1[7];
    whitening_init(kCtxValue1, ctx1);
    whitening_encode(buf, seed_offset, seed_len + data_len + 2, ctx1);

    uint8_t ctx2[7];
    whitening_init(kCtxValue2, ctx2);
    whitening_encode(buf, 0, buffer_len, ctx2);

    memcpy(out, buf + header_offset, result_len);
    return result_len;
}
