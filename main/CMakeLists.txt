idf_component_register(SRCS "i2cPlayground.c" "Dashboard.h" "Dashboard.c"
        REQUIRES esp_driver_i2c log driver
        INCLUDE_DIRS ".")
