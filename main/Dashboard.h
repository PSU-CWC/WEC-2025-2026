#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef bool (*fpSendData)(const char *, uint32_t);

typedef uint32_t (*fpReadData)(char *buf, uint32_t size);

typedef bool (*fpHasData)(uint32_t *size);

void Dashboard_Init(fpSendData fp, fpReadData fpRead, fpHasData fpHas);

void Dashboard_Telemetry_Str(const char *key, const char *value);

void Dashboard_Telemetry_Int(const char *key, int32_t num);

void Dashboard_Telemetry_Float(const char *key, float num);

void Dashboard_Register_LiveInt(const char *key, const char *v, uint32_t *ptr);

void Dashboard_Send();