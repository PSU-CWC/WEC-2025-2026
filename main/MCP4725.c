#include <math.h>
#include "MCP4725.h"
esp_err_t MCP4725_Set_Output_Voltage(i2c_master_dev_handle_t dev, float voltage) {

    uint16_t code = (uint16_t) lrintf(voltage * 4095.0f / 3.3f);

    // Fast mode payload:
    // Byte0: [7:6]=00, [5:4]=PD(00 normal), [3:0]=D11..D8
    // Byte1: D7..D0
    uint8_t payload[2] = {
            (uint8_t) (((0 & 0x3) << 4) | ((code >> 8) & 0x0F)),
            (uint8_t) (code & 0xFF)
    };
    return i2c_master_transmit(dev, payload, sizeof(payload), 100);
}