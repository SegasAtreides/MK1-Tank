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
//
// WO10-FINAL rev B hand-sync note: the format grew (full XWC token
// universe, optional mode= attribute - see mkh_config.h/.c) but
// DEFAULT_CONFIG_TEXT itself needs no change - both its lines assign
// stick axes (LSNS/RSNS), which never carry mode= (see
// mkh_config_serialize()'s axis-class omission), so the two lines below
// remain byte-for-byte what a fresh save of the default mapping would
// produce today.

#include "mkh_storage.h"

#include <LittleFS.h>

#include "mkh_config.h"
#include "uni_log.h"

static const char* CONFIG_FILE_PATH = "/mk1config.txt";
static const char* CONFIG_TMP_PATH = "/mk1config.txt.tmp";
static const char* STORAGE_PARTITION_LABEL = "storage";

// BUGFIX #N: explicit numeric counterpart to CONFIG_SCHEMA_HEADER below -
// used by the boot-time migration gate (see readAndParseConfigFile()).
// Before this fix, the header was purely a human-readable comment with
// "no parser-observable distinction" between versions (see the version-
// history comment on CONFIG_SCHEMA_HEADER) - there was no code path that
// ever READ a file's version back, only ever wrote the current one.
#define CONFIG_SCHEMA_VERSION_CURRENT 3

// WO10-FINAL: written as the first line of every editor Save. A '#'
// comment line, so mkh_config_parse_line() - unchanged, per this
// order's constraints - already ignores it silently like any other
// comment; round-tripping needed no parser changes at all.
//
// WO10-FINAL rev B: bumped 1 -> 2. Decision (reported per the order):
// this is NOT a backward-compat break - a v1 file (LSNS/RSNS only, no
// mode=) still parses correctly under the rev B parser unchanged, and a
// rev-B-written file with mode= parses correctly too (mode= is just
// another optional attribute, same code path as invert=/max=). The bump
// exists purely to mark "written under the expanded rev B token/mode
// vocabulary" for humans/future tooling reading the file - there is no
// parser-observable distinction between the two version numbers today.
//
// WO13: bumped 2 -> 3 for multi-binding (repeated "HUB<n>_PORT_<letter>
// = ..." lines for the same port now accumulate into a binding list
// instead of "last line wins" - see mkh_config_parse_line()'s doc
// comment in mkh_config.h). Same non-break reasoning as the rev B bump:
// a pre-v3 file has at most one line per port, which parses under the
// v3 parser to a 1-binding port - identical resolved behavior. The bump
// is again purely informational, marking "written under the multi-
// binding vocabulary" for humans/future tooling; there is no parser-
// observable distinction between v2 and v3 files today.
//
// BUGFIX #N: must match CONFIG_SCHEMA_VERSION_CURRENT above (kept as two
// separate constants - a string for the literal written to disk, an int
// for the comparison below - rather than one, since C has no clean way
// to build a compile-time string from a #define'd int; the two are
// small and co-located enough to keep in sync by hand, same tradeoff
// this file already accepts for DEFAULT_CONFIG_TEXT below).
static const char* CONFIG_SCHEMA_HEADER = "# MK1CONFIG_SCHEMA_VERSION=3\n";

// Reproduces v0.7.0 behavior exactly - see mk1config.sample.txt for the
// annotated format reference. BUGFIX #N: now carries the current schema
// header itself (previously had none), so a freshly-seeded first-ever
// file is already at CONFIG_SCHEMA_VERSION_CURRENT and the migration
// check below does not fire a needless write-back on first boot.
static const char* DEFAULT_CONFIG_TEXT =
    "# MK1CONFIG_SCHEMA_VERSION=3\n"
    "HUB1_PORT_A = LSNS invert=no max=100 curve=linear\n"
    "HUB2_PORT_A = RSNS invert=no max=100 curve=linear\n";

// BUGFIX #N: parses the "# MK1CONFIG_SCHEMA_VERSION=N" header. Only
// meaningful when called on a file's FIRST line (see its one call site
// below) - matches nothing on any other line, which is fine, since a
// non-matching first line just means "pre-versioning / no header",
// correctly resolved to 0 (oldest, always < CONFIG_SCHEMA_VERSION_CURRENT).
static int parseSchemaVersion(const char* first_line) {
    int version = 0;
    if (sscanf(first_line, "# MK1CONFIG_SCHEMA_VERSION=%d", &version) == 1) {
        return version;
    }
    return 0;
}

// Factored out of mkh_storage_boot_read() (WO10-FINAL) so
// mkh_storage_reload_config() can re-derive the live table from the
// same saved file without duplicating the open/parse/log sequence.
// Caller is responsible for mkh_config_set_defaults() first - this
// function only opens, parses, and notes/logs the result.
//
// BUGFIX #N: this is also the one and only place old-schema migration
// can fire (see the version check + one-shot write-back at the end).
// Confirmed via a grepped, empty-handed search of main/*.c* before this
// fix: no prior version of this file had ANY migration/normalization
// step, at boot or otherwise - the schema header was purely
// informational (see CONFIG_SCHEMA_HEADER's doc comment above). Confirmed
// via five consecutive instrumented power cycles on the reporter's own
// inflated file that plain re-parsing (this function, unchanged) never
// grows the table or writes anything on its own - see the completion
// report for the full smoking-gun writeup.
static void readAndParseConfigFile(void) {
    File f = LittleFS.open(CONFIG_FILE_PATH, FILE_READ);
    if (!f) {
        loge("MK1 storage: could not open %s, using compiled-in config defaults\n", CONFIG_FILE_PATH);
        mkh_config_note_source(false);
        mkh_config_apply_compiled_in_fallbacks();
        mkh_config_log_table();
        return;
    }

    logi("MK1 storage: parsing %s\n", CONFIG_FILE_PATH);
    int lineNum = 0;
    int fileSchemaVersion = -1;  // -1 = not yet determined (empty file)
    while (f.available()) {
        String line = f.readStringUntil('\n');
        if (lineNum == 0) {
            fileSchemaVersion = parseSchemaVersion(line.c_str());
        }
        mkh_config_parse_line(line.c_str());
        lineNum++;
    }
    f.close();
    mkh_config_note_source(true);

    // BUGFIX #N: fallback applied AFTER parsing, only for ports parsing
    // left untouched (binding_count still 0) - see
    // mkh_config_apply_compiled_in_fallbacks()'s doc comment for why this
    // ordering (not "before parsing", the pre-fix behavior) is the fix.
    mkh_config_apply_compiled_in_fallbacks();
    mkh_config_log_table();

    // BUGFIX #N: explicit, gated migration - runs if and only if the
    // file's OWN declared version is older than current. A file already
    // at (or somehow above) current version takes the plain load path
    // above with zero migration log lines, by construction - there is no
    // separate "did we migrate" flag to fall out of sync with reality.
    // The write-back reuses mkh_storage_save_config() as-is (the just-
    // parsed table IS the migrated table - no separate transform step
    // exists because no old schema version actually differs in parsed
    // meaning from the current one, see CONFIG_SCHEMA_HEADER's history
    // above) and happens exactly once per stale file, never per boot.
    if (fileSchemaVersion >= 0 && fileSchemaVersion < CONFIG_SCHEMA_VERSION_CURRENT) {
        logi("MK1 storage: MIGRATION - %s schema version=%d is older than current=%d, writing back once\n",
             CONFIG_FILE_PATH, fileSchemaVersion, CONFIG_SCHEMA_VERSION_CURRENT);
        bool ok = mkh_storage_save_config();
        logi("MK1 storage: MIGRATION write-back %s\n", ok ? "succeeded" : "FAILED");
    }
}

void mkh_storage_boot_read(void) {
    // BUGFIX #N: this is the ONLY place the live table is populated at
    // boot, and it always starts from a full reset - see
    // mkh_config_set_defaults()'s own doc comment ("Called once at the
    // start of mkh_storage_boot_read()..."). There is no merge path
    // anywhere in the boot sequence; every port's binding_count is zeroed
    // before a single file line is parsed, so a re-parse of an unchanged
    // file always reproduces the same table, never an accumulating one.
    mkh_config_set_defaults();

    if (!LittleFS.begin(true, "/littlefs", 10, STORAGE_PARTITION_LABEL)) {
        loge("MK1 storage: LittleFS mount failed (even after format attempt), using compiled-in config defaults\n");
        mkh_config_note_source(false);
        mkh_config_apply_compiled_in_fallbacks();
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
            mkh_config_apply_compiled_in_fallbacks();
            mkh_config_log_table();
            return;
        }
        wf.print(DEFAULT_CONFIG_TEXT);
        wf.close();
    }

    readAndParseConfigFile();
}

void mkh_storage_reload_config(void) {
    // BUGFIX #N: same full-reset guarantee as mkh_storage_boot_read()
    // above - see its doc comment.
    mkh_config_set_defaults();
    readAndParseConfigFile();
}

bool mkh_storage_save_config(void) {
    char buf[2048];
    int n = mkh_config_serialize(buf, sizeof(buf));
    if (n < 0) {
        loge("MK1 storage: config too large to serialize (buffer=%u bytes), save aborted\n", (unsigned)sizeof(buf));
        return false;
    }

    File wf = LittleFS.open(CONFIG_TMP_PATH, FILE_WRITE, true);
    if (!wf) {
        loge("MK1 storage: could not open %s for writing, save aborted\n", CONFIG_TMP_PATH);
        return false;
    }
    wf.print(CONFIG_SCHEMA_HEADER);
    wf.write((const uint8_t*)buf, (size_t)n);
    wf.close();

    if (LittleFS.exists(CONFIG_FILE_PATH) && !LittleFS.remove(CONFIG_FILE_PATH)) {
        loge("MK1 storage: could not remove old %s before rename, save aborted (temp file left at %s)\n",
             CONFIG_FILE_PATH, CONFIG_TMP_PATH);
        return false;
    }
    if (!LittleFS.rename(CONFIG_TMP_PATH, CONFIG_FILE_PATH)) {
        loge("MK1 storage: rename %s -> %s failed, save aborted\n", CONFIG_TMP_PATH, CONFIG_FILE_PATH);
        return false;
    }

    logi("MK1 storage: saved %s (%d bytes)\n", CONFIG_FILE_PATH, n);
    mkh_config_note_source(true);
    return true;
}
