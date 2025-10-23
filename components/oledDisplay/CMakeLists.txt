idf_component_register(SRCS "oledDisplay.c" "font6x8.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ssd1306
                       REQUIRES esp_driver_gpio
                       REQUIRES esp_driver_dac)

