#include "network_ota.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define AP_SSID "rover"
#define AP_CHANNEL 6
#define AP_MAX_CLIENTS 4
#define MAX_SCAN_RESULTS 20
#define WIFI_FORM_LIMIT 384
#define OTA_RX_CHUNK 4096

static const char *TAG = "network_ota";
static SemaphoreHandle_t network_mutex;
static esp_timer_handle_t reconnect_timer;
static network_ota_safe_stop_fn safe_stop_callback;
static bool station_connected;
static bool controller_active;
static char station_ip[16] = "0.0.0.0";
static char station_error[96];
static uint32_t ap_join_count;
static uint32_t ap_leave_count;
static uint16_t last_ap_aid;
static uint8_t last_ap_mac[6];
static uint8_t last_router_disconnect_reason;
static char last_network_event[96] = "network initialization";

static const char page_css[] =
"*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#17202a;font-family:Arial,sans-serif}"
"header{background:#17212b;color:#fff;border-bottom:3px solid #e2a400;padding:13px}header div,main{max-width:720px;margin:auto}"
"h1{font-size:20px;margin:0}nav{margin-top:8px}nav a{color:#fff;margin-right:16px}main{padding:14px}"
"section{background:#fff;border:1px solid #cbd3da;border-radius:8px;padding:13px;margin-bottom:14px}h2{font-size:17px;margin:0 0 10px}"
"label{display:grid;gap:5px;margin:10px 0;font-size:13px;font-weight:700}input,select,button{width:100%;min-height:46px;font-size:16px;border:1px solid #adb8c2;border-radius:6px;padding:8px;background:#fff}"
"button{font-weight:750;background:#1769aa;color:#fff;border-color:#1769aa}.secondary{background:#e7ecf0;color:#17202a;border-color:#adb8c2}"
"pre,.preview{white-space:pre-wrap;overflow-wrap:anywhere;background:#111a22;color:#d8f5e4;padding:10px;border-radius:6px;font:13px monospace;min-height:44px}"
".warn{border-left:4px solid #d39900;padding-left:9px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.bar{height:9px;background:#dbe1e6;border-radius:4px;overflow:hidden}.bar i{display:block;height:100%;background:#16816a;width:0}"
"@media(max-width:520px){.grid{grid-template-columns:1fr}}";

static const char wifi_page[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'><title>Robot Wi-Fi</title><style>"
"%s</style></head><body><header><div><h1>Robot Wi-Fi</h1><nav><a href='/'>Motor controls</a><a href='/steppers'>Steppers</a><a href='/odrive'>ODrive</a><a href='/battery'>Battery</a><a href='/update'>Firmware update</a></nav></div></header><main>"
"<section><h2>Connection</h2><pre id=status>Loading...</pre></section>"
"<section><h2>Choose network</h2><button class=secondary onclick=scan()>Scan nearby networks</button>"
"<label>Available networks<select id=net onchange=choose()><option value=''>Scan first</option></select></label>"
"<label>Network name<input id=ssid maxlength=32 autocomplete=off spellcheck=false></label>"
"<label>Password shown exactly as typed<input id=password type=text maxlength=63 autocomplete=new-password spellcheck=false oninput=preview()></label>"
"<div class=preview id=raw>Password: []\nCharacters: 0\nWhitespace view: []</div>"
"<p class=warn>Leading and trailing spaces are preserved. The whitespace view displays each space as a middle dot. The password is not returned by the device after submission.</p>"
"<button onclick=connectWifi()>Save and connect</button></section></main><script>"
"const e=id=>document.getElementById(id);"
"function preview(){const p=e('password').value;e('raw').textContent='Password: ['+p+']\\nCharacters: '+p.length+'\\nWhitespace view: ['+p.replaceAll(' ','·').replaceAll('\\t','→')+']'}"
"function choose(){if(e('net').value)e('ssid').value=e('net').value}"
"function status(){fetch('/api/wifi/status',{cache:'no-store'}).then(r=>r.json()).then(s=>{e('status').textContent='AP: '+s.ap_ssid+' | 192.168.4.1 | channel '+s.channel+'\\nAP clients: '+s.ap_clients+' | joins '+s.ap_joins+' | leaves '+s.ap_leaves+'\\nLast AP client: '+s.last_ap_client+' | AID '+s.last_ap_aid+'\\nRouter network: '+(s.saved_ssid||'not configured')+'\\nRouter connected: '+s.connected+' | IP: '+s.ip+' | last reason '+s.router_reason+'\\nEvent: '+s.last_event+'\\n'+(s.error||'')})}"
"function scan(){e('net').innerHTML='<option>Scanning...</option>';fetch('/api/wifi/scan',{cache:'no-store'}).then(r=>r.json()).then(x=>{e('net').innerHTML='<option value=\"\">Select a network</option>';x.networks.forEach(n=>{let o=document.createElement('option');o.value=n.ssid;o.textContent=n.ssid+' | '+n.rssi+' dBm | '+n.auth;e('net').appendChild(o)})}).catch(x=>e('status').textContent='Scan failed: '+x)}"
"function connectWifi(){let body=new URLSearchParams({ssid:e('ssid').value,password:e('password').value});fetch('/api/wifi/connect',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body}).then(r=>r.json()).then(x=>{e('status').textContent=x.message;e('password').value='';preview();setTimeout(status,2500)}).catch(x=>e('status').textContent='Connect failed: '+x)}"
"preview();status();</script></body></html>";

static const char update_page[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'><title>Robot Firmware</title><style>"
"%s</style></head><body><header><div><h1>Firmware Update</h1><nav><a href='/'>Motor controls</a><a href='/steppers'>Steppers</a><a href='/odrive'>ODrive</a><a href='/battery'>Battery</a><a href='/wifi'>Wi-Fi settings</a></nav></div></header><main>"
"<section><h2>Native ESP-IDF OTA</h2><pre id=status>Loading...</pre><label>Compiled application image<input id=file type=file accept='.bin,application/octet-stream'></label>"
"<div class=bar><i id=progress></i></div><p class=warn>Motors are stopped before writing. Keep power connected until the board verifies the image and reboots.</p>"
"<button id=install onclick=upload()>Install firmware</button></section></main><script>"
"const e=id=>document.getElementById(id);function load(){fetch('/api/ota/status',{cache:'no-store'}).then(r=>r.json()).then(s=>e('status').textContent='Running: '+s.version+' | '+s.partition+'\\nRunning slot: '+s.running_slot_bytes+' bytes\\nUpdate slot: '+s.slot_bytes+' bytes\\nPSRAM free: '+s.psram_free+' / '+s.psram_total+' bytes')}"
"function upload(){let f=e('file').files[0];if(!f){e('status').textContent='Choose an application .bin first';return}if(!confirm('Stop motors and install '+f.name+'?'))return;e('install').disabled=true;let x=new XMLHttpRequest();x.open('POST','/api/ota');x.setRequestHeader('Content-Type','application/octet-stream');x.upload.onprogress=v=>{if(v.lengthComputable)e('progress').style.width=Math.round(v.loaded*100/v.total)+'%%'};x.onload=()=>{e('status').textContent=x.responseText+'\\nThe board will reboot.'};x.onerror=()=>{e('status').textContent='Upload failed';e('install').disabled=false};x.send(f)}load();</script></body></html>";

static void json_escape(char *output, size_t output_size, const char *input)
{
    size_t used = 0;
    for (size_t i = 0; input[i] && used + 2 < output_size; ++i) {
        unsigned char c = (unsigned char)input[i];
        if (c == '"' || c == '\\') {
            output[used++] = '\\';
        }
        if (c >= 0x20) {
            output[used++] = (char)c;
        }
    }
    output[used] = '\0';
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool form_value(const char *body, const char *key, char *output,
                       size_t output_size)
{
    size_t key_length = strlen(key);
    const char *item = body;
    while (*item) {
        const char *equals = strchr(item, '=');
        if (!equals) return false;
        const char *end = strchr(equals + 1, '&');
        if (!end) end = body + strlen(body);
        if ((size_t)(equals - item) == key_length &&
            strncmp(item, key, key_length) == 0) {
            size_t used = 0;
            for (const char *p = equals + 1; p < end && used + 1 < output_size; ++p) {
                if (*p == '+' ) {
                    output[used++] = ' ';
                } else if (*p == '%' && p + 2 < end) {
                    int high = hex_value(p[1]);
                    int low = hex_value(p[2]);
                    if (high < 0 || low < 0) return false;
                    output[used++] = (char)((high << 4) | low);
                    p += 2;
                } else {
                    output[used++] = *p;
                }
            }
            output[used] = '\0';
            return true;
        }
        item = *end ? end + 1 : end;
    }
    return false;
}

static esp_err_t receive_body(httpd_req_t *request, char *body, size_t capacity)
{
    if (request->content_len <= 0 || (size_t)request->content_len >= capacity) {
        return ESP_ERR_INVALID_SIZE;
    }
    int received = 0;
    while (received < request->content_len) {
        int result = httpd_req_recv(request, body + received,
                                    request->content_len - received);
        if (result == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (result <= 0) return ESP_FAIL;
        received += result;
    }
    body[received] = '\0';
    return ESP_OK;
}

static const char *auth_name(wifi_auth_mode_t auth)
{
    switch (auth) {
        case WIFI_AUTH_OPEN: return "open";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA";
        case WIFI_AUTH_WPA2_PSK: return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
        case WIFI_AUTH_WPA3_PSK: return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
        default: return "secured";
    }
}

static void reconnect_timer_callback(void *argument)
{
    (void)argument;
    wifi_config_t config = {0};
    xSemaphoreTake(network_mutex, portMAX_DELAY);
    bool allowed = !controller_active;
    xSemaphoreGive(network_mutex);
    if (allowed && esp_wifi_get_config(WIFI_IF_STA, &config) == ESP_OK && config.sta.ssid[0]) {
        esp_wifi_connect();
    }
}

static void wifi_event_handler(void *argument, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)argument;
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        reconnect_timer_callback(NULL);
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = event_data;
        xSemaphoreTake(network_mutex, portMAX_DELAY);
        station_connected = false;
        last_router_disconnect_reason = event->reason;
        snprintf(station_ip, sizeof(station_ip), "0.0.0.0");
        bool retry = !controller_active;
        snprintf(station_error, sizeof(station_error), retry
                 ? "Router disconnected (reason %u); retrying"
                 : "Router paused while handheld controller is active",
                 event->reason);
        snprintf(last_network_event, sizeof(last_network_event),
                 "router disconnected reason %u", event->reason);
        xSemaphoreGive(network_mutex);
        esp_timer_stop(reconnect_timer);
        if (retry) esp_timer_start_once(reconnect_timer, 3000000);
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = event_data;
        xSemaphoreTake(network_mutex, portMAX_DELAY);
        station_connected = true;
        snprintf(station_ip, sizeof(station_ip), IPSTR, IP2STR(&event->ip_info.ip));
        station_error[0] = '\0';
        xSemaphoreGive(network_mutex);
        ESP_LOGI(TAG, "Router connected, web server IP %s", station_ip);
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = event_data;
        xSemaphoreTake(network_mutex, portMAX_DELAY);
        ap_join_count++;
        last_ap_aid = event->aid;
        memcpy(last_ap_mac, event->mac, sizeof(last_ap_mac));
        snprintf(last_network_event, sizeof(last_network_event),
                 "AP client joined AID %u", event->aid);
        xSemaphoreGive(network_mutex);
        ESP_LOGI(TAG, "AP client " MACSTR " joined AID %u",
                 MAC2STR(event->mac), event->aid);
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = event_data;
        xSemaphoreTake(network_mutex, portMAX_DELAY);
        ap_leave_count++;
        last_ap_aid = event->aid;
        memcpy(last_ap_mac, event->mac, sizeof(last_ap_mac));
        snprintf(last_network_event, sizeof(last_network_event),
                 "AP client left AID %u", event->aid);
        xSemaphoreGive(network_mutex);
        ESP_LOGW(TAG, "AP client " MACSTR " left AID %u",
                 MAC2STR(event->mac), event->aid);
    }
}

void network_ota_set_controller_active(bool active)
{
    if (!network_mutex) return;
    xSemaphoreTake(network_mutex, portMAX_DELAY);
    if (controller_active == active) {
        xSemaphoreGive(network_mutex);
        return;
    }
    controller_active = active;
    if (active) {
        snprintf(station_error, sizeof(station_error),
                 "Router paused while handheld controller is active");
    } else {
        snprintf(station_error, sizeof(station_error),
                 "Handheld controller disconnected; reconnecting router");
    }
    xSemaphoreGive(network_mutex);

    esp_timer_stop(reconnect_timer);
    if (active) {
        ESP_LOGI(TAG, "Authenticated handheld active; pausing router STA");
        esp_wifi_disconnect();
    } else {
        ESP_LOGI(TAG, "Handheld inactive; restoring saved router STA");
        reconnect_timer_callback(NULL);
    }
}

bool network_ota_controller_active(void)
{
    if (!network_mutex) return false;
    xSemaphoreTake(network_mutex, portMAX_DELAY);
    bool active = controller_active;
    xSemaphoreGive(network_mutex);
    return active;
}

bool network_ota_get_saved_credentials(char *ssid, size_t ssid_size,
                                       char *password, size_t password_size)
{
    if (!ssid || ssid_size < 2 || !password || password_size < 1) return false;
    wifi_config_t config = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &config) != ESP_OK ||
        config.sta.ssid[0] == '\0') {
        return false;
    }
    strlcpy(ssid, (const char *)config.sta.ssid, ssid_size);
    strlcpy(password, (const char *)config.sta.password, password_size);
    return true;
}

esp_err_t network_ota_start(void)
{
    network_mutex = xSemaphoreCreateMutex();
    if (!network_mutex) return ESP_ERR_NO_MEM;

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    const esp_timer_create_args_t timer_args = {
        .callback = reconnect_timer_callback,
        .name = "wifi_reconnect",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &reconnect_timer), TAG,
                        "reconnect timer");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   wifi_event_handler, NULL), TAG,
                        "wifi event");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   wifi_event_handler, NULL), TAG,
                        "ip event");

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_FLASH), TAG, "wifi storage");

    wifi_config_t ap = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = sizeof(AP_SSID) - 1,
            .channel = AP_CHANNEL,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = AP_MAX_CLIENTS,
            .pmf_cfg = {.required = false},
        },
    };
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "APSTA mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap), TAG, "AP config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    ESP_LOGI(TAG, "AP '%s' ready at http://192.168.4.1/", AP_SSID);
    return ESP_OK;
}

static esp_err_t wifi_page_handler(httpd_req_t *request)
{
    size_t size = strlen(wifi_page) + strlen(page_css) + 32;
    char *page = malloc(size);
    if (!page) return httpd_resp_send_500(request);
    snprintf(page, size, wifi_page, page_css);
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_send(request, page, HTTPD_RESP_USE_STRLEN);
    free(page);
    return result;
}

static esp_err_t wifi_status_handler(httpd_req_t *request)
{
    wifi_config_t config = {0};
    esp_wifi_get_config(WIFI_IF_STA, &config);
    char ssid[65];
    char error[193];
    char event_text[193];
    uint8_t primary_channel = 0;
    wifi_second_chan_t secondary_channel = WIFI_SECOND_CHAN_NONE;
    wifi_sta_list_t station_list = {0};
    (void)esp_wifi_get_channel(&primary_channel, &secondary_channel);
    (void)esp_wifi_ap_get_sta_list(&station_list);
    json_escape(ssid, sizeof(ssid), (const char *)config.sta.ssid);
    xSemaphoreTake(network_mutex, portMAX_DELAY);
    bool connected = station_connected;
    uint32_t joins = ap_join_count;
    uint32_t leaves = ap_leave_count;
    uint16_t aid = last_ap_aid;
    uint8_t mac[6];
    memcpy(mac, last_ap_mac, sizeof(mac));
    uint8_t router_reason = last_router_disconnect_reason;
    char ip[sizeof(station_ip)];
    snprintf(ip, sizeof(ip), "%s", station_ip);
    json_escape(error, sizeof(error), station_error);
    json_escape(event_text, sizeof(event_text), last_network_event);
    xSemaphoreGive(network_mutex);
    char body[760];
    snprintf(body, sizeof(body),
             "{\"ap_ssid\":\"%s\",\"saved_ssid\":\"%s\","
             "\"channel\":%u,\"ap_clients\":%u,\"ap_joins\":%lu,\"ap_leaves\":%lu,"
             "\"last_ap_client\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"last_ap_aid\":%u,"
             "\"router_reason\":%u,\"last_event\":\"%s\","
             "\"connected\":%s,\"controller_active\":%s,\"ip\":\"%s\",\"error\":\"%s\"}",
             AP_SSID, ssid, primary_channel, station_list.num,
             (unsigned long)joins, (unsigned long)leaves,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], aid,
             router_reason, event_text, connected ? "true" : "false",
             network_ota_controller_active() ? "true" : "false", ip, error);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, body);
}

static esp_err_t wifi_scan_handler(httpd_req_t *request)
{
    wifi_scan_config_t scan = {0};
    esp_err_t result = esp_wifi_scan_start(&scan, true);
    if (result != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(result));
        return result;
    }
    uint16_t count = MAX_SCAN_RESULTS;
    wifi_ap_record_t records[MAX_SCAN_RESULTS];
    memset(records, 0, sizeof(records));
    ESP_RETURN_ON_ERROR(esp_wifi_scan_get_ap_records(&count, records), TAG,
                        "scan records");

    char *body = malloc(4096);
    if (!body) return httpd_resp_send_500(request);
    size_t used = snprintf(body, 4096, "{\"networks\":[");
    for (uint16_t i = 0; i < count; ++i) {
        char ssid[65];
        json_escape(ssid, sizeof(ssid), (const char *)records[i].ssid);
        used += snprintf(body + used, 4096 - used,
                         "%s{\"ssid\":\"%s\",\"rssi\":%d,\"auth\":\"%s\"}",
                         i ? "," : "", ssid, records[i].rssi,
                         auth_name(records[i].authmode));
        if (used > 3900) break;
    }
    snprintf(body + used, 4096 - used, "]}");
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    result = httpd_resp_sendstr(request, body);
    free(body);
    return result;
}

static esp_err_t wifi_connect_handler(httpd_req_t *request)
{
    if (network_ota_controller_active()) {
        httpd_resp_set_status(request, "409 Conflict");
        httpd_resp_set_type(request, "text/plain");
        return httpd_resp_sendstr(
            request, "disconnect handheld controller before changing router");
    }
    char body[WIFI_FORM_LIMIT];
    char ssid[33] = {0};
    char password[64] = {0};
    if (receive_body(request, body, sizeof(body)) != ESP_OK ||
        !form_value(body, "ssid", ssid, sizeof(ssid)) ||
        !form_value(body, "password", password, sizeof(password)) || !ssid[0]) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid Wi-Fi form");
        return ESP_FAIL;
    }

    wifi_config_t config = {0};
    strlcpy((char *)config.sta.ssid, ssid, sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, password, sizeof(config.sta.password));
    config.sta.threshold.authmode = password[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;

    esp_wifi_disconnect();
    esp_err_t result = esp_wifi_set_config(WIFI_IF_STA, &config);
    memset(password, 0, sizeof(password));
    memset(body, 0, sizeof(body));
    if (result == ESP_OK) result = esp_wifi_connect();
    if (result != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(result));
        return result;
    }
    ESP_LOGI(TAG, "Connecting to saved router SSID '%s'", ssid);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request,
        "{\"message\":\"Credentials saved. Connecting while the setup AP remains available.\"}");
}

static esp_err_t update_page_handler(httpd_req_t *request)
{
    size_t size = strlen(update_page) + strlen(page_css) + 32;
    char *page = malloc(size);
    if (!page) return httpd_resp_send_500(request);
    snprintf(page, size, update_page, page_css);
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_send(request, page, HTTPD_RESP_USE_STRLEN);
    free(page);
    return result;
}

static esp_err_t ota_status_handler(httpd_req_t *request)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    const esp_app_desc_t *app = esp_app_get_description();
    char body[320];
    snprintf(body, sizeof(body),
             "{\"version\":\"%s\",\"partition\":\"%s\","
             "\"running_slot_bytes\":%u,\"slot_bytes\":%u,"
             "\"psram_total\":%u,\"psram_free\":%u}",
             app->version, running ? running->label : "unknown",
             running ? (unsigned)running->size : 0,
             next ? (unsigned)next->size : 0,
             (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, body);
}

static void restart_task(void *argument)
{
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
}

static esp_err_t ota_upload_handler(httpd_req_t *request)
{
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update || request->content_len <= 0 ||
        (size_t)request->content_len > update->size) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                            "image is empty or exceeds OTA slot");
        return ESP_ERR_INVALID_SIZE;
    }
    if (safe_stop_callback) safe_stop_callback();

    esp_ota_handle_t handle = 0;
    esp_err_t result = esp_ota_begin(update, request->content_len, &handle);
    if (result != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(result));
        return result;
    }

    uint8_t *buffer = malloc(OTA_RX_CHUNK);
    if (!buffer) {
        esp_ota_abort(handle);
        return httpd_resp_send_500(request);
    }
    int remaining = request->content_len;
    while (remaining > 0) {
        int received = httpd_req_recv(request, (char *)buffer,
                                      remaining > OTA_RX_CHUNK ? OTA_RX_CHUNK : remaining);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (received <= 0) {
            result = ESP_FAIL;
            break;
        }
        result = esp_ota_write(handle, buffer, received);
        if (result != ESP_OK) break;
        remaining -= received;
    }
    free(buffer);
    if (result != ESP_OK) {
        esp_ota_abort(handle);
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "firmware receive/write failed");
        return result;
    }
    result = esp_ota_end(handle);
    if (result == ESP_OK) result = esp_ota_set_boot_partition(update);
    if (result != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                            "image validation failed");
        return result;
    }

    ESP_LOGI(TAG, "OTA image verified in %s; reboot scheduled", update->label);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_sendstr(request,
        "{\"ok\":true,\"message\":\"Firmware verified; rebooting\"}");
    xTaskCreate(restart_task, "ota_restart", 2048, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t network_ota_register_routes(httpd_handle_t server,
                                      network_ota_safe_stop_fn safe_stop)
{
    safe_stop_callback = safe_stop;
    const httpd_uri_t routes[] = {
        {.uri = "/wifi", .method = HTTP_GET, .handler = wifi_page_handler},
        {.uri = "/api/wifi/status", .method = HTTP_GET, .handler = wifi_status_handler},
        {.uri = "/api/wifi/scan", .method = HTTP_GET, .handler = wifi_scan_handler},
        {.uri = "/api/wifi/connect", .method = HTTP_POST, .handler = wifi_connect_handler},
        {.uri = "/update", .method = HTTP_GET, .handler = update_page_handler},
        {.uri = "/api/ota/status", .method = HTTP_GET, .handler = ota_status_handler},
        {.uri = "/api/ota", .method = HTTP_POST, .handler = ota_upload_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &routes[i]), TAG,
                            "register route");
    }
    return ESP_OK;
}

void network_ota_mark_running_app_valid(void)
{
    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running && esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
        ESP_LOGI(TAG, "OTA image marked valid after network and web startup");
    }
}
