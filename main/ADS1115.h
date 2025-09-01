#pragma once

#include <stdio.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct {
    uint64_t timestamp_us;  // timestamp of sweep
    float data[4];  // AIN0..3 in volts
} ads1115_snapshot_t;
enum {
    ADS1115_Mux_AIN0 = 0b100,
    ADS1115_Mux_AIN1 = 0b101,
    ADS1115_Mux_AIN2 = 0b110,
    ADS1115_Mux_AIN3 = 0b111,
};

esp_err_t ADS1115_Read_Voltage(i2c_master_dev_handle_t dev, float *res, uint8_t mux, uint8_t timeout);
esp_err_t ADS1115_StartSampler(i2c_master_dev_handle_t dev);
bool ADS1115_GetLatest(ads1115_snapshot_t *out, uint32_t wait_ticks);