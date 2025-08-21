#include <stdio.h>
#include <esp_check.h>
#include <math.h>
#include <esp_rom_uart.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
//
#include "Dashboard.h"

static const char *TAG = "ADS1115";

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
    Mux_AIN0 = 0b100,
    Mux_AIN1 = 0b101,
    Mux_AIN2 = 0b110,
    Mux_AIN3 = 0b111,
    Sample_Time_MS = 4,
};
static uint8_t muxArray[] = {Mux_AIN0, Mux_AIN1, Mux_AIN2, Mux_AIN3};

static inline uint16_t ads1115_make_config(uint8_t mux) {
    // OS=1 (start single conversion)
    // MUX=100 (AIN0 vs GND)
    // PGA=001 (±4.096V)
    // MODE=1 (single-shot)
    // DR=101 (250 SPS)
    // COMP_MODE=0, POL=0, LAT=0, COMP_QUE=11 (disable comparator)
    return (1u << 15) | (mux << 12) | (0b001u << 9) | (1u << 8) |
           (0b101u << 5) | (0u << 4) | (0u << 3) | (0u << 2) | (0b11u);
}


// Write [pointer, MSB, LSB] to config register
static esp_err_t ads1115_write_config(i2c_master_dev_handle_t dev, uint16_t cfg) {
    uint8_t buf[3] = {
            Config_Register_Addr,
            (uint8_t) (cfg >> 8),
            (uint8_t) (cfg & 0xFF)
    };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

// Read a 16-bit register: write pointer, then read 2 bytes
static esp_err_t ads1115_read_u16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *out) {

    // 1115 is big endian, however ESP is little endian
    // can't make rx into uint16_t because rx = 0x1234 which is wrong in little endian
    uint8_t rx[2];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, rx, 2, 100);
    if (err != ESP_OK) return err;
    // *out = 0x1234 => *out gets stored as 0x3412 by the computer BUT != memcpy(rx, 0x1234)
    *out = ((uint16_t) rx[0] << 8) | rx[1];
    return ESP_OK;
}

static bool is_data_ready(uint16_t data) {
    uint16_t mask = 1u << 15;
    return (data & mask) != 1;
}

static esp_err_t read_data(i2c_master_dev_handle_t dev, float *res, uint8_t timeout) {
    uint8_t timeout_counter = 0;
    while (timeout_counter < timeout) {
        vTaskDelay(pdMS_TO_TICKS(Sample_Time_MS));
        uint16_t config;
        ESP_RETURN_ON_ERROR(ads1115_read_u16(dev, Config_Register_Addr, &config), TAG,
                            "Failed to read config register");
        if (is_data_ready(config)) {
            uint16_t raw_data;
            ESP_RETURN_ON_ERROR(ads1115_read_u16(dev, Conversion_Register_Addr, &raw_data), TAG,
                                "Failed to read conversion register");
            int16_t raw = (int16_t) raw_data;
            float lsb = 4.096f / 32768.0f;
            float data = (float) raw * lsb;
            if (data < 0) {
                data = 0.0f;
            }
            *res = data;
            ESP_LOGI(TAG, "TOOK %u to sample", timeout_counter);
            return ESP_OK;
        }
        timeout_counter += Sample_Time_MS;
    }
    return ESP_FAIL;
}

static esp_err_t write_dac_voltage(i2c_master_dev_handle_t dev, float voltage) {

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

bool send_data(const char *data, uint32_t size) {
    uart_write_bytes(UART_PORT, data, size);
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

    ESP_ERROR_CHECK(write_dac_voltage(dev_dac1, 1.45f));
    ESP_ERROR_CHECK(write_dac_voltage(dev_dac2, 2.27f));
    uint32_t t0 = esp_log_timestamp();
    for (int i = 0; i < 4; i++) {
        uint16_t config = ads1115_make_config(muxArray[i]);
        ESP_ERROR_CHECK(ads1115_write_config(ads, config));
        float res;
        if (read_data(ads, &res, 100) == ESP_OK) {
            ESP_LOGI(TAG, "AN%d has %.2f", i, res);
        } else {
            ESP_LOGI(TAG, "AN%d failed");
        }
    }
    uint32_t dt = esp_log_timestamp() - t0;  // ms
    ESP_LOGI(TAG, "elapsed = %u ms", (unsigned) dt);

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    //fflush(stdout);  // flush stdio buffers (newlib)
    //esp_rom_output_tx_wait_idle(UART_NUM_0);   // wait until UART0 finished transmitting
    Dashboard_Init(send_data);
    int32_t num = 0;
    while (1) {
        Dashboard_Telemetry_Int("test1", num);
        Dashboard_Send();
        num += 1;
        //send_data("ping\r\n",6);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // (Unreached in this loop; shown for completeness)
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(ads));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}
