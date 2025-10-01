#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

esp_err_t WebSocketServer_Start(void);

esp_err_t WebSocketServer_BroadcastText(uint8_t *buf, uint32_t size);