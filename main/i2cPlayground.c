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
#include "DeltaTime.h"
#include "Util.h"
#include "Servo.h"

static const char *TAG = "Main";

#define I2C_PORT            0                   // I2C_NUM_0
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_FREQ_HZ         100000
#define UART_PORT UART_NUM_0
#define BUF_SIZE  1024
#define V_SNS1_Ratio 15.87f
#define Step_Ratio 0.000244f
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
    //i2c_master_dev_handle_t dev_dac2 = NULL;
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
    //ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_dac2_cfg, &dev_dac2));

    ESP_LOGI(TAG, "I2C ready on SDA=%d SCL=%d @ %d Hz, ADS1115 @ 0x%02X", I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ,
             ADS1115_I2C_Addr);

    ESP_ERROR_CHECK(MCP4725_Set_Output_Voltage(dev_dac1, 1.45f));


    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));


    Dashboard_Init(send_data, read_uart, get_uart_data_size);
    float RESISTANCE = 30;
    int32_t targetPower = 2;
    float OutputVoltage = 0.00f;
    float kp = 0.002f;
    int32_t pos = 50;
    // Dashboard_Register_LiveFloat("AA", "Resistance", &RESISTANCE);
    Dashboard_Register_LiveInt("AB", "targetPower", &targetPower);
    // Dashboard_Register_LiveFloat("AC", "DACVolt", &OutputVoltage);
    Dashboard_Register_LiveInt("AD", "ServoPos", &pos);
    ESP_ERROR_CHECK(ADS1115_StartSampler(ads));
    ads1115_snapshot_t snap;
    DeltaTime_t dt;
    DeltaTime_Init(&dt);
    DeltaTime_Update(&dt);


//    gpio_config_t io_conf = {
//            .pin_bit_mask = 1ULL << GPIO_NUM_13,
//            .mode = GPIO_MODE_OUTPUT,
//            .pull_up_en = GPIO_PULLUP_DISABLE,
//            .pull_down_en = GPIO_PULLDOWN_DISABLE,
//            .intr_type = GPIO_INTR_DISABLE
//    };
//
//    gpio_config(&io_conf);
//    gpio_set_level(GPIO_NUM_13, 1);
    bool ledStatus = false;


    Servo_Init();
    while (1) {
        ESP_ERROR_CHECK(MCP4725_Set_Output_Voltage(dev_dac1, OutputVoltage));
        Servo_Set_Pulse_US((uint8_t) pos);
        if (ADS1115_GetLatest(&snap, 10)) {

            float V_F1 = snap.data[0];

            float Real_Current = V_F1 * 4;
            float V_Load = snap.data[3] * V_SNS1_Ratio;
            float actual_resistance = V_Load / Real_Current;

            float curPower = V_Load * Real_Current;
            //Const Power
            Dashboard_Telemetry_Float("CurPower", curPower);
//            if (DeltaTime_Get_MS(&dt) > 5) {
//                Dashboard_Telemetry_Float("Diff Power", ((float) targetPower - curPower) * kp);
//                OutputVoltage += ((float) targetPower - curPower) * kp;
//                OutputVoltage = clamp(OutputVoltage, 0, 3.3f);
//                DeltaTime_Update(&dt);
//            }

//            if (DeltaTime_Get_MS(&dt) > 5) {
//                if (RESISTANCE - actual_resistance > 0) {
//                    //ESP_LOGI(TAG, "Decreasing float");
//                    OutputVoltage -= Step_Ratio;
//                } else {
//                    //ESP_LOGI(TAG, "Increasing float");
//                    OutputVoltage += Step_Ratio;
//                }
//                DeltaTime_Update(&dt);
//            }

            Dashboard_Telemetry_Float("Load_Voltage", V_Load);
            Dashboard_Telemetry_Float("kp", kp);
            Dashboard_Telemetry_Int("Time Diff", DeltaTime_Get_MS(&dt));
            Dashboard_Telemetry_Float("Resistance", actual_resistance);
            Dashboard_Telemetry_Float("DAC_CMD_Output", OutputVoltage);
        } else {
            ESP_LOGI(TAG, "AN3 failed");
        }
//        if(ledStatus){
//            gpio_set_level(GPIO_NUM_13, 1);
//
//        }else{
//            gpio_set_level(GPIO_NUM_13, 0);
//        }
        ledStatus = !ledStatus;

        Dashboard_Send();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(ads));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}
