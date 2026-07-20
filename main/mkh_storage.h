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

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void mkh_storage_boot_read(void);

// WO10-FINAL: mapping editor's Reset button. Re-derives the live table
// from the saved file exactly as boot does (defaults, then re-parse) -
// LittleFS is already mounted by this point, so this never re-mounts or
// re-seeds. If the file is absent/malformed, behaves exactly like boot
// under the same conditions (falls back to compiled-in defaults) - same
// failure posture, never fails outright.
void mkh_storage_reload_config(void);

// WO10-FINAL: mapping editor's Save. Serializes the CURRENT live table
// (mkh_config_serialize()) to a temp file, then renames over the real
// config file - atomic from a reader's perspective, so a reboot or a
// failed write mid-way never observes a half-written file. Returns
// false (config file left exactly as it was) if serialization or any
// filesystem step fails; true on success.
bool mkh_storage_save_config(void);

#ifdef __cplusplus
}
#endif

#endif  // MKH_STORAGE_H
