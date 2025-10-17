idf_component_register(SRCS "i2cPlayground.c" "Dashboard.h" "Dashboard.c" "MCP4725.c" "ADS1115.c" "DeltaTime.c" "Servo.c"
        REQUIRES esp_driver_i2c log driver esp_timer
        INCLUDE_DIRS ".")
