#include "ADS1115.h"

#include <esp_check.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/queue.h"

#include "esp_timer.h"

static QueueHandle_t s_ads_latest_q = NULL;
enum task_priority {
    PRIORITY_IDLE = tskIDLE_PRIORITY,
    PRIORITY_LOW = tskIDLE_PRIORITY + 3,
    PRIORITY_MEDIUM = tskIDLE_PRIORITY + 5,
    PRIORITY_HIGH = tskIDLE_PRIORITY + 7
};

enum {
    ADS1115_I2C_Addr = 0x48, // ADDR -> GND
    DAC1_I2C_ADDR = 0x60,
    DAC2_I2C_ADDR = 0x61,
    Conversion_Register_Addr = 00,
    Config_Register_Addr = 01,
    Sample_Time_MS = 2,
};
static const char *TAG = "ADS1115";
static uint8_t muxArray[] = {ADS1115_Mux_AIN0, ADS1115_Mux_AIN1, ADS1115_Mux_AIN2, ADS1115_Mux_AIN3};

static uint16_t ads1115_make_config(uint8_t mux) {
    // OS=1 (start single conversion)
    // MUX=100 (AIN0 vs GND)
    // PGA=001 (±4.096V)
    // MODE=1 (single-shot)
    // DR=101 (250 SPS)
    // COMP_MODE=0, POL=0, LAT=0, COMP_QUE=11 (disable comparator)
    return (1u << 15) | (mux << 12) | (0b001u << 9) | (1u << 8) |
           (0b110u << 5) | (0u << 4) | (0u << 3) | (0u << 2) | (0b11u);
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

// bit 15 is 1 when conversion is not happening
static bool is_data_ready(uint16_t data) {
    uint16_t mask = 1u << 15;
    return (data & mask) != 0;
}

static void ads1115_sampler_task(void *arg) {
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t) arg;

    // Prime the queue with an initial empty snapshot so readers can peek immediately
    static ads1115_snapshot_t snap = {.timestamp_us = 0, .data = {0, 0, 0, 0}};
    xQueueOverwrite(s_ads_latest_q, &snap);

    for (;;) {
        snap.timestamp_us = esp_timer_get_time();

        for (int ch = 0; ch < 4; ++ch) {
            float v = 0.0f;
            esp_err_t err = ADS1115_Read_Voltage(dev, &v, muxArray[ch], 100);
            if (err != ESP_OK) {
                // Keep going; leave previous value if a single read fails
                ESP_EARLY_LOGW(TAG, "CH%d read err=%d", ch, (int) err);
            } else {
                snap.data[ch] = v;
            }
        }

        // Publish "latest only" (length-1 queue)
        xQueueOverwrite(s_ads_latest_q, &snap);

        // Optional: tiny yield to let other tasks run. At 4ch*~1.2ms ~= 4.8ms,
        // this loop already yields on I2C and esp_rom_delay_us, so no vTaskDelay is required.
        //taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t ADS1115_Read_Voltage(i2c_master_dev_handle_t dev, float *res, uint8_t mux, uint8_t timeout) {
    uint16_t config = ads1115_make_config(mux);
    ESP_RETURN_ON_ERROR(ads1115_write_config(dev, config), TAG, "Failed to write ads1115 config");
    uint8_t timeout_counter = 0;
    while (timeout_counter < timeout) {
        vTaskDelay(pdMS_TO_TICKS(Sample_Time_MS));

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
            //ESP_LOGI(TAG, "TOOK %u to sample", timeout_counter);
            return ESP_OK;
        }
        timeout_counter += Sample_Time_MS;
    }
    ESP_LOGE(TAG, "Read timeout");
    return ESP_FAIL;
}

esp_err_t ADS1115_StartSampler(i2c_master_dev_handle_t dev) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!s_ads_latest_q) {
        s_ads_latest_q = xQueueCreate(1, sizeof(ads1115_snapshot_t));
        if (!s_ads_latest_q) return ESP_ERR_NO_MEM;
    }
    return xTaskCreate(ads1115_sampler_task, "ads1115_sampler", 4096, (void *) dev, PRIORITY_IDLE, NULL) == pdPASS
           ? ESP_OK : ESP_FAIL;

}

// Non-destructive read of the latest values. wait_ticks=0 for immediate.
bool ADS1115_GetLatest(ads1115_snapshot_t *out, TickType_t wait_ticks) {
    if (!s_ads_latest_q || !out) return false;
    return xQueuePeek(s_ads_latest_q, out, wait_ticks) == pdTRUE;
}