
#include "Dashboard.h"

#include <string.h>
#include <stdio.h>
#include <esp_log.h>

enum {
    kTelemetryBufSize = 400,
    kAlertBufSize = 200,
    kMapBufSize = 50,
    kNumBufSize = 12,
    kFloatBufSize = 12,
    mapEntries = 5,
};

typedef struct __attribute__((packed)) Packet {
    uint8_t packet_id;
    uint16_t packet_length;
    char data_buf[3];
    union {
        float float_data;
        int32_t int_data;
    };
    uint32_t checksum;
} Packet_t;

typedef struct Dashboard {
    uint32_t telemetrySize;
    uint32_t liveDataSize;
    uint32_t alertSize;
    fpSendData sendData;
    fpReadData readData;
    fpHasData hasData;
    Packet_t packet;
    uint8_t intCount;
    uint8_t floatCount;
} Dashboard_t;


char floatMapKeys[mapEntries][3] = {{0}};
char integerMapKeys[mapEntries][3] = {{0}};
int32_t *intMapValues[mapEntries] = {0};
float *floatMapValues[mapEntries] = {0};

static char numBuffer[kNumBufSize] = {0};
static char floatBuffer[kFloatBufSize] = {0};

static char buffer[kTelemetryBufSize] = {0};
static char mapBuffer[kMapBufSize] = {0};
static char alertBuffer[kAlertBufSize] = {0};

static Dashboard_t dashboard;

int integer_to_string(int32_t x);

int float_to_string(float x);

uint32_t crc32(char *s, uint32_t n);

void processData();

void Dashboard_Init(fpSendData fp, fpReadData fpRead, fpHasData fpHas) {
    memcpy(buffer, "CWC!", 4);
    memcpy(mapBuffer, "CWCM", 4);
    memcpy(alertBuffer, "CWCA", 4);

    dashboard.sendData = fp;
    dashboard.hasData = fpHas;
    dashboard.readData = fpRead;
    dashboard.telemetrySize = 4;
    dashboard.liveDataSize = 4;
    dashboard.alertSize = 4;
}

void Dashboard_Alert(const char *str) {
    uint32_t size = strlen(str);
    memcpy(&alertBuffer[dashboard.alertSize], str, size);
    dashboard.alertSize += size;
    memcpy(&alertBuffer[dashboard.alertSize], ";", 1);
    dashboard.alertSize += 1;
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
    processData();

    buffer[dashboard.telemetrySize] = '\r';
    buffer[dashboard.telemetrySize + 1] = '\n';
    dashboard.sendData(buffer, dashboard.telemetrySize + 2);

    memset(buffer, 0, kTelemetryBufSize);
    memcpy(buffer, "CWC!", 4);
    dashboard.telemetrySize = 4;

    mapBuffer[dashboard.liveDataSize] = '\r';
    mapBuffer[dashboard.liveDataSize + 1] = '\n';
    dashboard.sendData(mapBuffer, dashboard.liveDataSize + 2);


    alertBuffer[dashboard.alertSize] = '\r';
    alertBuffer[dashboard.alertSize + 1] = '\n';
    dashboard.sendData(alertBuffer, dashboard.alertSize + 2);

    memset(alertBuffer, 0, kAlertBufSize);
    memcpy(alertBuffer, "CWCA", 4);
    dashboard.alertSize = 4;
}

void Dashboard_Register_LiveInt(const char *key, const char *v, int32_t *ptr) {

    uint32_t valueSize = strlen(v);
    memcpy(integerMapKeys[dashboard.intCount], "I", 1);
    memcpy(integerMapKeys[dashboard.intCount] + 1, key, 2);
    intMapValues[dashboard.intCount] = ptr;

    memcpy(&mapBuffer[dashboard.liveDataSize], integerMapKeys[dashboard.intCount], sizeof(integerMapKeys[0]));
    dashboard.liveDataSize += sizeof(integerMapKeys[0]);
    memcpy(&mapBuffer[dashboard.liveDataSize], ":", 1);
    dashboard.liveDataSize += 1;
    memcpy(&mapBuffer[dashboard.liveDataSize], v, valueSize);
    dashboard.liveDataSize += valueSize;
    memcpy(&mapBuffer[dashboard.liveDataSize], ";", 1);
    dashboard.liveDataSize += 1;
    dashboard.intCount += 1;
}

void Dashboard_Register_LiveFloat(const char *key, const char *v, float *ptr) {

    uint32_t valueSize = strlen(v);
    memcpy(floatMapKeys[dashboard.floatCount], "F", 1);
    memcpy(floatMapKeys[dashboard.floatCount] + 1, key, 2);
    floatMapValues[dashboard.floatCount] = ptr;

    memcpy(&mapBuffer[dashboard.liveDataSize], floatMapKeys[dashboard.floatCount], sizeof(floatMapKeys[0]));
    dashboard.liveDataSize += sizeof(floatMapKeys[0]);
    memcpy(&mapBuffer[dashboard.liveDataSize], ":", 1);
    dashboard.liveDataSize += 1;
    memcpy(&mapBuffer[dashboard.liveDataSize], v, valueSize);
    dashboard.liveDataSize += valueSize;
    memcpy(&mapBuffer[dashboard.liveDataSize], ";", 1);
    dashboard.liveDataSize += 1;
    dashboard.floatCount += 1;
}

void processData() {
    uint32_t size;
    bool status = dashboard.hasData(&size);

    if (status == true && size >= sizeof(Packet_t)) {
        uint32_t readSize = dashboard.readData((char *) &dashboard.packet, sizeof(dashboard.packet.packet_id));

        if (readSize != sizeof(dashboard.packet.packet_id) || dashboard.packet.packet_id != 0x13) {
            return;
        }
        ESP_LOGI("Dashboard", "Received packet id");
        uint32_t remainingPacketSize = sizeof(Packet_t) - sizeof(dashboard.packet.packet_id);
        readSize = dashboard.readData((char *) &dashboard.packet.packet_length, remainingPacketSize);
        uint32_t packetSizeWithCRC = sizeof(Packet_t) - sizeof(dashboard.packet.checksum);
        //if (readSize != remainingPacketSize ||crc32((char *) &dashboard.packet, packetSizeWithCRC) != dashboard.packet.checksum) {
        if (readSize != remainingPacketSize) {
            return;
        }
        ESP_LOGI("Dashboard", "Received entire packet");
        char dataType = dashboard.packet.data_buf[0];
        if (dataType == 'I') {
            ESP_LOGI("Dashboard", "Integer type found");

            int8_t idx = -1;
            ESP_LOGI("Dashboard", "b (hex) = %02X %02X %02X", (unsigned) dashboard.packet.data_buf[0],
                     (unsigned) dashboard.packet.data_buf[1], (unsigned) dashboard.packet.data_buf[2]);

            for (int8_t i = 0; i < dashboard.intCount; i++) {
                if (memcmp(dashboard.packet.data_buf, integerMapKeys[i], sizeof(integerMapKeys[0])) == 0) {
                    idx = i;
                    break;
                }
            }
            if (idx != -1) {
                ESP_LOGI("Dashboard", "received int %d", dashboard.packet.int_data);

                *intMapValues[idx] = dashboard.packet.int_data;
            }
        } else if (dataType == 'F') {
            ESP_LOGI("Dashboard", "Float type found");

            int8_t idx = -1;
            ESP_LOGI("Dashboard", "b (hex) = %02X %02X %02X", (unsigned) dashboard.packet.data_buf[0],
                     (unsigned) dashboard.packet.data_buf[1], (unsigned) dashboard.packet.data_buf[2]);

            for (int8_t i = 0; i < dashboard.floatCount; i++) {
                if (memcmp(dashboard.packet.data_buf, floatMapKeys[i], sizeof(floatMapKeys[0])) == 0) {
                    idx = i;
                    break;
                }
            }
            if (idx != -1) {
                ESP_LOGI("Dashboard", "received float %d", dashboard.packet.float_data);

                *floatMapValues[idx] = dashboard.packet.float_data;
            }
        }
        memset(&dashboard.packet, 0, sizeof(Packet_t));
    }
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

uint32_t crc32(char *s, uint32_t n) {
    uint32_t crc = 0xFFFFFFFF;

    for (uint32_t i = 0; i < n; i++) {
        char ch = s[i];
        for (uint32_t j = 0; j < 8; j++) {
            uint32_t b = (ch ^ crc) & 1;
            crc >>= 1;
            if (b) crc = crc ^ 0xEDB88320;
            ch >>= 1;
        }
    }

    return ~crc;
}