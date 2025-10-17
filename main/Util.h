#pragma once
#define RETURN_ON_ERROR(x)                           \
    do {                                             \
        esp_err_t __err = (x);                       \
        if (__err != ESP_OK) return __err;           \
    } while (0)


float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}