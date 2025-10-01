#include "DeltaTime.h"

#include <esp_timer.h>

void DeltaTime_Init(DeltaTime_t *dt) {
    dt->lastTime = esp_timer_get_time();
    dt->currentTime = dt->lastTime;
}

void DeltaTime_Update(DeltaTime_t *dt) {
    dt->lastTime = dt->currentTime;
    dt->currentTime = esp_timer_get_time();
}

void DeltaTime_Reset(DeltaTime_t *dt) {
    dt->lastTime = esp_timer_get_time();
    dt->currentTime = dt->lastTime;
}

uint64_t DeltaTime_Get_MS(DeltaTime_t *dt) {
    return (esp_timer_get_time() - dt->lastTime)/1000;
}