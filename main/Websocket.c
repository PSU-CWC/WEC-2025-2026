

#include "Websocket.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_server.h"

static const char *WS_TAG = "WS";
static httpd_handle_t s_server = NULL;

static const char kIndexHtml[] =
        "<!doctype html><html><body>"
        "<h1>ESP32 WebSocket</h1>"
        "<button onclick=\"ws.send('ping')\">Send ping</button>"
        "<pre id='log'></pre>"
        "<script>"
        "const log = m => document.getElementById('log').textContent += m + '\\n';"
        "const ws = new WebSocket('ws://' + location.host + '/ws');"
        "ws.onopen    = () => log('open');"
        "ws.onclose   = () => log('close');"
        "ws.onerror   = e  => log('error');"
        "ws.onmessage = e  => log('rx: ' + e.data);"
        "</script>"
        "</body></html>";

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // WebSocket handshake done by server; nothing else to do here.
        ESP_LOGI(WS_TAG, "Client connected to /ws");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    // 1) Probe for length
    esp_err_t err = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (err != ESP_OK) return err;

    uint8_t *buf = NULL;
    if (ws_pkt.len) {
        buf = (uint8_t *) malloc(ws_pkt.len + 1);
        if (!buf) return ESP_ERR_NO_MEM;
        ws_pkt.payload = buf;
        err = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (err != ESP_OK) {
            free(buf);
            return err;
        }
        buf[ws_pkt.len] = 0; // NUL-terminate for logs
    }

    ESP_LOGI(WS_TAG, "WS frame: type=%d, len=%u, payload=\"%s\"",
             ws_pkt.type, (unsigned) ws_pkt.len, buf ? (char *) buf : "");

    // Echo back to the same client
    httpd_ws_frame_t resp = {
            .final   = true,
            .fragmented = false,
            .type    = ws_pkt.type,    // echo same type (TEXT/BINARY)
            .payload = ws_pkt.payload,
            .len     = ws_pkt.len,
    };
    err = httpd_ws_send_frame(req, &resp);

    free(buf);
    return err;
}

esp_err_t WebSocketServer_BroadcastText(uint8_t *msg, uint32_t size) {
    if (!s_server) return ESP_ERR_INVALID_STATE;
    size_t max = CONFIG_LWIP_MAX_LISTENING_TCP; // best-effort; not exact client count
    int fds[CONFIG_LWIP_MAX_LISTENING_TCP > 16 ? 32 : 16];
    size_t cnt = sizeof(fds) / sizeof(fds[0]);

    if (httpd_get_client_list(s_server, &cnt, fds) != ESP_OK) return ESP_FAIL;

    httpd_ws_frame_t frame = {
            .final = true,
            .type  = HTTPD_WS_TYPE_TEXT,
            .payload = msg,
            .len = size,
    };

    for (size_t i = 0; i < cnt; ++i) {
        int sock = fds[i];
        // Send asynchronously (safe outside a request context)
        httpd_ws_send_frame_async(s_server, sock, &frame);
    }
    return ESP_OK;
}

esp_err_t WebSocketServer_Start(void) {
    if (s_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;     // free handlers if out of slots
    cfg.server_port = 80;            // default; AP clients hit http://192.168.4.1/
    cfg.recv_wait_timeout = 10;      // seconds (tweak as needed)
    cfg.send_wait_timeout = 10;

    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) return err;

    // Root page for quick testing
    httpd_uri_t root = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_get_handler,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &root);

    // WebSocket endpoint
    httpd_uri_t ws = {
            .uri      = "/ws",
            .method   = HTTP_GET,
            .handler  = ws_handler,
            .is_websocket = true,
            .user_ctx = NULL
    };
    httpd_register_uri_handler(s_server, &ws);

    ESP_LOGI(WS_TAG, "HTTP server started, WS at ws://<ap-ip>/ws");
    return ESP_OK;
}

void WebSocketServer_Stop(void) {
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}

