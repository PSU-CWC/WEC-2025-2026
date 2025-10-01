#pragma once
#define RETURN_ON_ERROR(x)                           \
    do {                                             \
        esp_err_t __err = (x);                       \
        if (__err != ESP_OK) return __err;           \
    } while (0)