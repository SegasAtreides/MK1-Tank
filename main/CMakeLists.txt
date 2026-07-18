set(srcs
        "main.c"
        "mkh_broadcast.c"
        "mkh_config.c"
        "mkh_protocol.c"
        "mkh_storage.cpp"
        "sketch.cpp")
# mkh_sdcard.cpp is intentionally NOT built - the microSD card is retired
# as a config medium (v0.8.0 PM decision). Kept on disk as hardware
# documentation only; see mkh_sdcard.h for why.

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "Arduino_GFX" "joltwallet__littlefs")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
