
#include "Dashboard.h"

#include <string.h>
#include <stdio.h>

enum {
    kTelemetryBufSize = 400,
    kNumBufSize = 12,
    kFloatBufSize = 12,
};

typedef struct Dashboard {
    uint32_t telemetrySize;
    fpSendData sendData;
} Dashboard_t;
static char buffer[kTelemetryBufSize] = {0};
static char numBuffer[kNumBufSize] = {0};
static char floatBuffer[kFloatBufSize] = {0};

static Dashboard_t dashboard;


int integer_to_string(int32_t x);

int float_to_string(float x);

void Dashboard_Init(fpSendData fp) {
    strcat(buffer, "CWC!");
    dashboard.sendData = fp;
    dashboard.telemetrySize = 4;
}

void Dashboard_Telemetry_Str(const char *key, const char *value) {
    uint32_t keySize = strlen(key);
    uint32_t valueSize = strlen(value);
    if (dashboard.telemetrySize + keySize + valueSize < kTelemetryBufSize) {
        strcat(buffer, key);
        strcat(buffer, ":");
        strcat(buffer, value);
        strcat(buffer, ";");
        dashboard.telemetrySize += keySize + valueSize + 2;
    }
}

void Dashboard_Telemetry_Int(const char *key, int32_t num) {
    integer_to_string(num);
    Dashboard_Telemetry_Str(key, numBuffer);
}

void Dashboard_Telemetry_Float(const char *key, float num) {
    float_to_string(num);
    Dashboard_Telemetry_Str(key, floatBuffer);
}

void Dashboard_Send() {
    buffer[dashboard.telemetrySize] = '\r';
    buffer[dashboard.telemetrySize + 1] = '\n';
    dashboard.sendData(buffer, dashboard.telemetrySize + 2);
    memset(buffer, 0, kTelemetryBufSize);
    strcat(buffer, "CWC!");
    dashboard.telemetrySize = 4;
}

int integer_to_string(int32_t x) {
    int size = sprintf(numBuffer, "%ld", x);
    numBuffer[size] = 0;
    return size;
}

int float_to_string(float x) {
    int size = sprintf(floatBuffer, "%f", x);
    floatBuffer[size] = 0;
    return size;
}