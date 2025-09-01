#pragma once


#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t MCP4725_Set_Output_Voltage(i2c_master_dev_handle_t dev, float voltage);
