set(srcs
        "main.c"
        "mkh_broadcast.c"
        "mkh_protocol.c"
        "sketch.cpp")

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "Arduino_GFX")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
