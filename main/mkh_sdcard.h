// SPDX-License-Identifier: Apache-2.0
//
// RETIRED (v0.8.0): the microSD card is no longer used as a config
// medium - this is a PM product decision (fragility across three
// independent failure classes: a bad card's broken SPI mode, board-level
// shared-SPI-bus boot-order requirements, and an 8.3-filename trap from
// disabled LFN), not a technical dead end, and it is not being revisited.
// Config storage moved to internal flash (LittleFS) - see mkh_storage.h.
//
// This module is EXCLUDED from the build (removed from main/CMakeLists.txt)
// and is not called anywhere in the boot path. It is kept only as
// hardware documentation - see the NOTES block in mkh_sdcard.cpp - for
// anyone who ever revisits this microSD slot for an unrelated purpose
// (e.g. future data logging, which would itself be a new PM decision).
// Do not re-wire it without an explicit new decision to do so.

#ifndef MKH_SDCARD_H
#define MKH_SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif

// Retired - not called. Left declared only so this header still matches
// the .cpp for anyone reading the pair. Do not add a call site.
void mkh_sdcard_boot_read(void);

#ifdef __cplusplus
}
#endif

#endif  // MKH_SDCARD_H
