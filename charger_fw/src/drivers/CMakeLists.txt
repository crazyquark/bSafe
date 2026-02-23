idf_component_register(
    SRCS "bq25895.c" "ina226.c" "tca9535.c" "display.c" "ui_demo.c"
    INCLUDE_DIRS "."
    REQUIRES "esp_driver_i2c" "freertos" "log" "esp_lcd"
)
