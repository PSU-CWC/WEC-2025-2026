#pragma once

#include <stdint.h>

typedef struct {
    uint64_t lastTime;
    uint64_t currentTime;
} DeltaTime_t;

void DeltaTime_Init(DeltaTime_t *dt);

void DeltaTime_Update(DeltaTime_t *dt);

void DeltaTime_Reset(DeltaTime_t *dt);

uint64_t DeltaTime_Get_MS(DeltaTime_t *dt);