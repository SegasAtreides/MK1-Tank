// SPDX-License-Identifier: Apache-2.0
//
// v0.8.0 Step 0b: LittleFS foundation. Single boot-time entry point for
// internal-flash storage, following the same one-entry-point discipline
// established by the (now retired) mkh_sdcard module - see mkh_sdcard.h.
// This step only proves the storage foundation (mount + a fixed-content
// marker file round trip); the real config file (/mk1config.txt) is
// wired in during Step 1.

#ifndef MKH_STORAGE_H
#define MKH_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

void mkh_storage_boot_read(void);

#ifdef __cplusplus
}
#endif

#endif  // MKH_STORAGE_H
