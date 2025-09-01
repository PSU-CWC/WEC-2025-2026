#include <stdio.h>
#include <esp_check.h>
#include <math.h>
#include <esp_rom_uart.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"

#include "Dashboard.h"
#include "MCP4725.h"
#include "ADS1115.h"

static const char *TAG = "Main";

#define I2C_PORT            0                   // I2C_NUM_0
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_FREQ_HZ         100000
#define UART_PORT UART_NUM_0
#define BUF_SIZE  1024
enum {
    ADS1115_I2C_Addr = 0x48, // ADDR -> GND
    DAC1_I2C_ADDR = 0x60,
    DAC2_I2C_ADDR = 0x61,
    Conversion_Register_Addr = 00,
    Config_Register_Addr = 01,
    Sample_Time_MS = 4,
};

bool send_data(const char *data, uint32_t size) {
    uart_write_bytes(UART_PORT, data, size);
    return true;
}

uint32_t read_uart(char *buf, uint32_t size) {
    return uart_read_bytes(UART_PORT, buf, size, pdMS_TO_TICKS(100));
}

bool get_uart_data_size(uint32_t *size) {
    uart_get_buffered_data_len(UART_PORT, (size_t *) size);
    return true;
}

void app_main(void) {
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_PORT,
            .sda_io_num = I2C_SDA_PIN,
            .scl_io_num = I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .intr_priority = 0,
            .glitch_ignore_cnt = 7,
            .flags = {
                    .enable_internal_pullup = false, // use external 4.7k pull-ups
            },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_master_dev_handle_t ads = NULL;
    i2c_master_dev_handle_t dev_dac1 = NULL;
    i2c_master_dev_handle_t dev_dac2 = NULL;
    i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = ADS1115_I2C_Addr,
            .scl_speed_hz    = I2C_FREQ_HZ,
            .scl_wait_us     = 0,
    };
    i2c_device_config_t dev_dac1_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = DAC1_I2C_ADDR,
            .scl_speed_hz    = I2C_FREQ_HZ,
            .scl_wait_us     = 0,
    };
    i2c_device_config_t dev_dac2_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = DAC2_I2C_ADDR,
            .scl_speed_hz    = I2C_FREQ_HZ,
            .scl_wait_us     = 0,
    };
    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
    };


    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &ads));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_dac1_cfg, &dev_dac1));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_dac2_cfg, &dev_dac2));

    ESP_LOGI(TAG, "I2C ready on SDA=%d SCL=%d @ %d Hz, ADS1115 @ 0x%02X", I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ,
             ADS1115_I2C_Addr);

    ESP_ERROR_CHECK(MCP4725_Set_Output_Voltage(dev_dac1, 1.45f));


    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));


    Dashboard_Init(send_data, read_uart, get_uart_data_size);
    int32_t num = 1;
    int32_t num2 = 3;
    float f = 1.9f;
    Dashboard_Register_LiveInt("AA", "VoltNum", &num);
    Dashboard_Register_LiveInt("AB", "VoltNum2", &num2);
    Dashboard_Register_LiveFloat("AC", "Float1", &f);
    ESP_ERROR_CHECK(ADS1115_StartSampler(ads));
    ads1115_snapshot_t snap;
    uint64_t lastTimeStamp = 0;
    while (1) {
        if (num == 11) {
            Dashboard_Alert("idek anymore");
        }
        ESP_ERROR_CHECK(MCP4725_Set_Output_Voltage(dev_dac2, f));
        if (ADS1115_GetLatest(&snap, 10)) {
            Dashboard_Telemetry_Float("AN0", snap.data[0]);
            Dashboard_Telemetry_Float("AN1", snap.data[1]);
            Dashboard_Telemetry_Float("AN2", snap.data[2]);
            Dashboard_Telemetry_Float("AN3", snap.data[3]);
            Dashboard_Telemetry_Int("Time Diff", (snap.timestamp_us / 1000));
            lastTimeStamp = snap.timestamp_us;
        } else {
            ESP_LOGI(TAG, "AN3 failed");
        }
        Dashboard_Telemetry_Int("Number", num);
        Dashboard_Telemetry_Int("Number2", num2);
        Dashboard_Telemetry_Float("Float1", f);
        Dashboard_Send();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(ads));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}
