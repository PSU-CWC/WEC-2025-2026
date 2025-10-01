#include "WifiHandler.h"

#include "esp_mac.h"        // defines MACSTR / MAC2STR
#include <string.h>
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "Util.h"

static const char *TAG = "WIFI";
#define WIFI_SSID      "TestWifi"
#define WIFI_PASS      "password"
#define WIFI_CHANNEL   1
#define MAX_CONN       5


static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    /* Handler for device connected to access point */
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    }
        /* Handler for device disconnected from access point */
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

esp_err_t WifiHandler_Init() {
    RETURN_ON_ERROR(esp_netif_init());
    RETURN_ON_ERROR(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    /* Configure wifi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    RETURN_ON_ERROR(esp_wifi_init(&cfg));

    /* Register event handler */
    RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    /* Configure Access Point */
    wifi_config_t wifi_config = {
            .ap = {
                    .ssid = WIFI_SSID,
                    .ssid_len = strlen(WIFI_SSID),
                    .channel = WIFI_CHANNEL,
                    .password = WIFI_PASS,
                    .max_connection = MAX_CONN,
                    .authmode = WIFI_AUTH_WPA2_PSK,
                    .pmf_cfg = {
                            .required = true,
                    },

            },
    };

    /* Start access point */
    RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP));
    RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    RETURN_ON_ERROR(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

    return ESP_OK;
}