#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef bool (*fpSendData)(const char *, uint32_t);

void Dashboard_Init(fpSendData fp);

void Dashboard_Telemetry_Str(const char *key, const char *value);

void Dashboard_Telemetry_Int(const char *key, int32_t num);

void Dashboard_Telemetry_Float(const char *key, float num);

void Dashboard_Send();