// SPDX-License-Identifier: Apache-2.0
//
// v0.8.0 Step 1: LittleFS config file. Mounts the internal-flash
// "storage" partition (see partitions.csv), seeds /mk1config.txt with a
// compiled-in default on first boot (or after a full erase), then opens
// and parses it via mkh_config.c. This is the sole entry point allowed
// to touch the storage medium at boot - see mkh_storage.h.
//
// Component choice: joltwallet/littlefs (esp_littlefs) - unchanged from
// Step 0b; see the v0.8.0-step0b commit for the full justification.
//
// Deployment path (Step 1 decision): seed-from-compiled-in-default on
// first boot, NOT a separate LittleFS image flash target (pio run
// --target uploadfs). Reasons:
//   1. This project already requires periodic full chip erases whenever
//      the partition table changes (see v0.8.0-step0b), which would
//      silently wipe any config previously deployed via uploadfs. A
//      device that comes up with no working config after a routine
//      erase is a worse failure mode than a self-seeding one.
//   2. The eventual on-device config editor (WO10, not yet authorized)
//      will create/edit this file at runtime anyway - seeding runs the
//      same read/write code path a real session would exercise, so
//      there's no separate deployment tooling to keep working.
//   3. Testable end-to-end with no physical media and no separate
//      flashing step, per this step's own test requirement.
// Tradeoff accepted: DEFAULT_CONFIG_TEXT below must be kept in sync by
// hand with mk1config.sample.txt (that file is the annotated reference;
// this copy is deliberately minimal).

#include "mkh_storage.h"

#include <LittleFS.h>

#include "mkh_config.h"
#include "uni_log.h"

static const char* CONFIG_FILE_PATH = "/mk1config.txt";
static const char* STORAGE_PARTITION_LABEL = "storage";

// Reproduces v0.7.0 behavior exactly - see mk1config.sample.txt for the
// annotated format reference.
static const char* DEFAULT_CONFIG_TEXT =
    "HUB1_PORT_A = LSNS invert=no max=100 curve=linear\n"
    "HUB2_PORT_A = RSNS invert=no max=100 curve=linear\n";

void mkh_storage_boot_read(void) {
    mkh_config_set_defaults();

    if (!LittleFS.begin(true, "/littlefs", 10, STORAGE_PARTITION_LABEL)) {
        loge("MK1 storage: LittleFS mount failed (even after format attempt), using compiled-in config defaults\n");
        mkh_config_note_source(false);
        mkh_config_log_table();
        return;
    }

    logi("MK1 storage: LittleFS mounted OK (partition=%s, total=%luB, used=%luB)\n", STORAGE_PARTITION_LABEL,
         (unsigned long)LittleFS.totalBytes(), (unsigned long)LittleFS.usedBytes());

    if (!LittleFS.exists(CONFIG_FILE_PATH)) {
        logi("MK1 storage: %s absent, seeding compiled-in default (first boot on this flash image)\n",
             CONFIG_FILE_PATH);
        File wf = LittleFS.open(CONFIG_FILE_PATH, FILE_WRITE, true);
        if (!wf) {
            loge("MK1 storage: could not create %s, using compiled-in config defaults\n", CONFIG_FILE_PATH);
            mkh_config_note_source(false);
            mkh_config_log_table();
            return;
        }
        wf.print(DEFAULT_CONFIG_TEXT);
        wf.close();
    }

    File f = LittleFS.open(CONFIG_FILE_PATH, FILE_READ);
    if (!f) {
        loge("MK1 storage: could not open %s, using compiled-in config defaults\n", CONFIG_FILE_PATH);
        mkh_config_note_source(false);
        mkh_config_log_table();
        return;
    }

    logi("MK1 storage: parsing %s\n", CONFIG_FILE_PATH);
    while (f.available()) {
        String line = f.readStringUntil('\n');
        mkh_config_parse_line(line.c_str());
    }
    f.close();
    mkh_config_note_source(true);

    mkh_config_log_table();
}
