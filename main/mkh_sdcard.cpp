// SPDX-License-Identifier: Apache-2.0
//
// RETIRED (v0.8.0) - see mkh_sdcard.h. This file is excluded from the
// build entirely (not listed in main/CMakeLists.txt). It is kept as
// hardware documentation only.
//
// =====================================================================
// NOTES - permanent hardware documentation for the Waveshare
// ESP32-S3-Touch-LCD-2's onboard TF (microSD) card slot, for anyone who
// ever revisits it for a purpose OTHER than config storage (e.g. future
// data logging - itself a new PM decision, not authorized by this file).
// =====================================================================
//
// PIN ROUTING (PM-verified against the official schematic,
// https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-2/ESP32-S3-Touch-LCD-2-SchDoc.pdf):
// the SD card and the LCD share one physical SPI bus.
//   IO38 = LCD_MOSI = SD_MOSI (shared)
//   IO39 = LCD_SCLK = SD_SCLK (shared)
//   IO40 = SD_MISO             IO41 = SD_CS
//   IO45 = LCD_CS               IO42 = LCD_DC
// (Net labels on the schematic: NLIO38/NLSD0MOSI, NLIO39/NLSD0SCLK,
// NLIO40/NLSD0MISO, NLIO41/NLSD0CS, NLIO45/NLLCD0CS - cross-confirmed by
// the schematic's own GPIO pinout table, columns GPIO | Camera | LCD |
// SD_Card | Other, which lists IO38/IO39 under BOTH the LCD and SD_Card
// columns.)
//
// Consequence: any future SD use on this board MUST either run before
// the display is initialized (as this retired module did - see git
// history), or otherwise guarantee the display driver is never
// mid-transaction on the shared MOSI/SCLK lines while SD is active, and
// MUST explicitly deassert both chip-selects (LCD_CS=GPIO45, SD_CS=GPIO41)
// before starting SD bus activity.
//
// FILESYSTEM TRAP: this project's sdkconfig has CONFIG_FATFS_LFN_NONE
// (the ESP-IDF default) unless explicitly overridden, which restricts
// FatFs to strict 8.3 filenames (max 8-char base name). A file like
// "mk1config.txt" (9-char base) is NOT openable by that name under LFN
// disabled - only via its auto-generated short alias (e.g.
// MK1CON~1.TXT). A 7-char-base file ("mk1test.txt") worked fine and
// masked this trap during initial hardware bring-up. If SD is ever
// revisited: either keep all filenames <=8 characters, or set
// CONFIG_FATFS_LFN_HEAP=y (or _STACK) in sdkconfig.defaults.
//
// SEPARATE FINDING, also real but NOT what caused the above: a physical
// card that mounted and read fine on a PC failed 5/5 cold boots on this
// board at the lowest SPI level (sdSelectCard) with a different, smaller
// (~2GB) card, even after implementing full boot-order/CS discipline. A
// second, larger card resolved it immediately with no code change. Root
// cause was never conclusively identified beyond "that specific card was
// bad in some way not observable from a PC" - budget in card-brand
// variability if SD is ever revisited.
// =====================================================================

#include "mkh_sdcard.h"

#include <SD.h>
#include <SPI.h>

#include "mkh_config.h"
#include "uni_log.h"

#define SD_CS_PIN 41
#define SD_MOSI_PIN 38
#define SD_SCLK_PIN 39
#define SD_MISO_PIN 40
#define LCD_CS_PIN 45

static SPIClass sdSPI(HSPI);
static const char* CONFIG_FILE_PATH = "/mk1config.txt";

void mkh_sdcard_boot_read(void) {
    mkh_config_set_defaults();

    pinMode(LCD_CS_PIN, OUTPUT);
    digitalWrite(LCD_CS_PIN, HIGH);
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);

    if (!sdSPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN)) {
        loge("MK1 SD: SPI.begin() failed, using compiled-in config defaults\n");
        mkh_config_note_source(false);
        mkh_config_log_table();
        return;
    }

    if (!SD.begin(SD_CS_PIN, sdSPI)) {
        loge("MK1 SD: mount failed (no card present, or not FAT-formatted), using compiled-in config defaults\n");
        mkh_config_note_source(false);
        mkh_config_log_table();
        return;
    }

    sdcard_type_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        loge("MK1 SD: mounted but no card detected, using compiled-in config defaults\n");
        SD.end();
        mkh_config_note_source(false);
        mkh_config_log_table();
        return;
    }

    logi("MK1 SD: mounted OK (type=%d, size=%lluMB)\n", (int)cardType,
         (unsigned long long)(SD.cardSize() / (1024 * 1024)));

    File f = SD.open(CONFIG_FILE_PATH);
    if (!f) {
        loge("MK1 SD: could not open %s, using compiled-in config defaults\n", CONFIG_FILE_PATH);
        SD.end();
        mkh_config_note_source(false);
        mkh_config_log_table();
        return;
    }

    logi("MK1 SD: parsing %s\n", CONFIG_FILE_PATH);
    while (f.available()) {
        String line = f.readStringUntil('\n');
        mkh_config_parse_line(line.c_str());
    }
    f.close();
    mkh_config_note_source(true);

    SD.end();
    logi("MK1 SD: unmounted cleanly\n");

    mkh_config_log_table();
}
