// SPDX-License-Identifier: Apache-2.0
//
// v0.8.0 Step 0b: LittleFS foundation proof. Mounts the internal-flash
// "storage" partition (see partitions.csv) and proves a clean mount +
// write + read round trip via a fixed-content marker file. This is the
// sole entry point allowed to touch the storage medium at boot - see
// mkh_storage.h.
//
// Component choice: joltwallet/littlefs (esp_littlefs), not a new
// dependency - it is already pulled in transitively by
// components/arduino/idf_component.yml's own dependency on
// joltwallet/littlefs, and vendored at
// managed_components/joltwallet__littlefs. Using the Arduino LittleFS.h
// wrapper (components/arduino/libraries/LittleFS) over that same
// component mirrors the SD.h idiom used by the retired mkh_sdcard
// module. Note the default partitionLabel for LittleFS.begin() is
// "spiffs" - it must be passed explicitly here as "storage" to bind to
// the partition defined in partitions.csv.

#include "mkh_storage.h"

#include <LittleFS.h>

#include "mkh_config.h"
#include "uni_log.h"

static const char* MARKER_FILE_PATH = "/mk1test.txt";
static const char* MARKER_FILE_CONTENT = "MK1 LittleFS boot marker - v0.8.0 Step 0b\n";
static const char* STORAGE_PARTITION_LABEL = "storage";

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

    if (!LittleFS.exists(MARKER_FILE_PATH)) {
        logi("MK1 storage: %s absent, writing marker (first boot on this flash image)\n", MARKER_FILE_PATH);
        File wf = LittleFS.open(MARKER_FILE_PATH, FILE_WRITE, true);
        if (!wf) {
            loge("MK1 storage: could not create %s, using compiled-in config defaults\n", MARKER_FILE_PATH);
            mkh_config_note_source(false);
            mkh_config_log_table();
            return;
        }
        wf.print(MARKER_FILE_CONTENT);
        wf.close();
    }

    File rf = LittleFS.open(MARKER_FILE_PATH, FILE_READ);
    if (!rf) {
        loge("MK1 storage: could not open %s for readback, using compiled-in config defaults\n", MARKER_FILE_PATH);
        mkh_config_note_source(false);
        mkh_config_log_table();
        return;
    }
    String readBack = rf.readString();
    rf.close();
    logi("MK1 storage: marker readback (%d bytes): %s", readBack.length(), readBack.c_str());

    // Step 0b proves the LittleFS foundation only; the real config file
    // (/mk1config.txt) is parsed starting in Step 1, so config source
    // stays "not from file" here - the CFG:FS/CFG:DEF dashboard field
    // will reflect reality once Step 1 wires that up.
    mkh_config_note_source(false);
    mkh_config_log_table();
}
