set(COMPONENT_SRCDIRS ".")
set(COMPONENT_ADD_INCLUDEDIRS ".")

idf_component_register(SRCS "app_main.c"
                            "components/esp32-smbus/smbus.c"
                            "components/esp32-i2c-lcd1602/i2c-lcd1602.c"
                       INCLUDE_DIRS ".")