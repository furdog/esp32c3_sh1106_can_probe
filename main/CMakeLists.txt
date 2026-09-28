idf_component_register(
    SRCS "esp32c3_sh1106_can_probe.c"
    INCLUDE_DIRS "."
    REQUIRES esp_timer esp_driver_twai u8g2 u8g2-hal-esp-idf
)
