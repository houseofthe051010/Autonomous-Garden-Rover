#include "network_service.h"

#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "lwip/ip4_addr.h"
#include "nvs.h"

#define WIFI_SCAN_LIMIT 20

static const char *TAG = "network";
static esp_netif_t *s_ap_netif;
static esp_netif_t *s_sta_netif;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_sta_configured;
static bool s_sta_connected;
static char s_sta_ssid[33];

static void network_event(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        portENTER_CRITICAL(&s_status_lock);
        s_sta_connected = false;
        bool configured = s_sta_configured;
        portEXIT_CRITICAL(&s_status_lock);
        if (configured) esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        portENTER_CRITICAL(&s_status_lock);
        s_sta_connected = true;
        portEXIT_CRITICAL(&s_status_lock);
        ESP_LOGI(TAG, "router connection established");
    }
}

static bool load_credentials(char *ssid, size_t ssid_size, char *password, size_t password_size)
{
    nvs_handle_t nvs;
    if (nvs_open("camera_wifi", NVS_READONLY, &nvs) != ESP_OK) return false;
    size_t ssid_len = ssid_size;
    size_t password_len = password_size;
    esp_err_t ssid_err = nvs_get_str(nvs, "ssid", ssid, &ssid_len);
    esp_err_t pass_err = nvs_get_str(nvs, "password", password, &password_len);
    nvs_close(nvs);
    return ssid_err == ESP_OK && pass_err == ESP_OK && ssid[0] != '\0';
}

static esp_err_t save_credentials(const char *ssid, const char *password)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open("camera_wifi", NVS_READWRITE, &nvs), TAG,
                        "could not open Wi-Fi settings");
    esp_err_t err = nvs_set_str(nvs, "ssid", ssid);
    if (err == ESP_OK) err = nvs_set_str(nvs, "password", password);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

esp_err_t network_service_init(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "network interface initialization failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop initialization failed");
    s_ap_netif = esp_netif_create_default_wifi_ap();
    s_sta_netif = esp_netif_create_default_wifi_sta();
    ESP_RETURN_ON_FALSE(s_ap_netif && s_sta_netif, ESP_ERR_NO_MEM, TAG,
                        "default network interfaces unavailable");

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "Wi-Fi initialization failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Wi-Fi RAM storage failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   network_event, NULL), TAG,
                        "Wi-Fi event registration failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   network_event, NULL), TAG,
                        "IP event registration failed");

    wifi_config_t ap_config = {0};
    strlcpy((char *)ap_config.ap.ssid, CAMERA_AP_SSID, sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = strlen(CAMERA_AP_SSID);
    ap_config.ap.channel = CAMERA_AP_CHANNEL;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
    ap_config.ap.max_connection = CAMERA_AP_CLIENTS;
    ap_config.ap.beacon_interval = 100;

    char password[65] = {0};
    wifi_config_t sta_config = {0};
    bool configured = load_credentials(s_sta_ssid, sizeof(s_sta_ssid),
                                       password, sizeof(password));
    if (configured) {
        strlcpy((char *)sta_config.sta.ssid, s_sta_ssid, sizeof(sta_config.sta.ssid));
        strlcpy((char *)sta_config.sta.password, password, sizeof(sta_config.sta.password));
        sta_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        sta_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        sta_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        sta_config.sta.pmf_cfg.capable = true;
        sta_config.sta.pmf_cfg.required = false;
    }

    portENTER_CRITICAL(&s_status_lock);
    s_sta_configured = configured;
    portEXIT_CRITICAL(&s_status_lock);
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "AP+STA mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config), TAG, "AP config failed");
    if (configured) {
        ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_config), TAG,
                            "station config failed");
    }
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_max_tx_power(80));
    if (configured) esp_wifi_connect();

    ESP_LOGI(TAG, "AP '%s' is active and open at 192.168.4.1", CAMERA_AP_SSID);
    return ESP_OK;
}

void network_service_get_status(network_status_t *status)
{
    if (!status) return;
    memset(status, 0, sizeof(*status));
    strlcpy(status->ap_ip, "192.168.4.1", sizeof(status->ap_ip));
    status->sta_rssi = -127;
    status->ap_client_rssi = -127;

    portENTER_CRITICAL(&s_status_lock);
    status->sta_configured = s_sta_configured;
    status->sta_connected = s_sta_connected;
    strlcpy(status->sta_ssid, s_sta_ssid, sizeof(status->sta_ssid));
    portEXIT_CRITICAL(&s_status_lock);

    esp_netif_ip_info_t ip_info;
    if (s_ap_netif && esp_netif_get_ip_info(s_ap_netif, &ip_info) == ESP_OK) {
        esp_ip4addr_ntoa(&ip_info.ip, status->ap_ip, sizeof(status->ap_ip));
    }
    if (status->sta_connected && s_sta_netif &&
        esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
        esp_ip4addr_ntoa(&ip_info.ip, status->sta_ip, sizeof(status->sta_ip));
        wifi_ap_record_t record;
        if (esp_wifi_sta_get_ap_info(&record) == ESP_OK) {
            status->sta_rssi = record.rssi;
            status->channel = record.primary;
        }
    }

    wifi_sta_list_t stations;
    if (esp_wifi_ap_get_sta_list(&stations) == ESP_OK) {
        status->ap_clients = stations.num;
        for (int i = 0; i < stations.num; ++i) {
            if (stations.sta[i].rssi > status->ap_client_rssi) {
                status->ap_client_rssi = stations.sta[i].rssi;
            }
        }
    }
    if (!status->channel) {
        uint8_t secondary;
        wifi_second_chan_t second;
        if (esp_wifi_get_channel(&secondary, &second) == ESP_OK) status->channel = secondary;
    }
}

esp_err_t network_service_connect(const char *ssid, const char *password)
{
    ESP_RETURN_ON_FALSE(ssid && password && ssid[0] && strlen(ssid) <= 32 &&
                        strlen(password) <= 63, ESP_ERR_INVALID_ARG, TAG,
                        "invalid router credentials");
    ESP_RETURN_ON_ERROR(save_credentials(ssid, password), TAG, "could not save credentials");

    wifi_config_t config = {0};
    strlcpy((char *)config.sta.ssid, ssid, sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, password, sizeof(config.sta.password));
    config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;

    esp_wifi_disconnect();
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config), TAG,
                        "station config update failed");
    portENTER_CRITICAL(&s_status_lock);
    strlcpy(s_sta_ssid, ssid, sizeof(s_sta_ssid));
    s_sta_configured = true;
    s_sta_connected = false;
    portEXIT_CRITICAL(&s_status_lock);
    return esp_wifi_connect();
}

esp_err_t network_service_forget(void)
{
    esp_wifi_disconnect();
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("camera_wifi", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_erase_all(nvs);
        if (err == ESP_OK) err = nvs_commit(nvs);
        nvs_close(nvs);
    }
    portENTER_CRITICAL(&s_status_lock);
    s_sta_configured = false;
    s_sta_connected = false;
    s_sta_ssid[0] = '\0';
    portEXIT_CRITICAL(&s_status_lock);
    return err;
}

static bool append_json_string(char *out, size_t capacity, size_t *used, const char *text)
{
    if (*used + 2 >= capacity) return false;
    out[(*used)++] = '"';
    for (const unsigned char *p = (const unsigned char *)text; *p; ++p) {
        if (*used + 7 >= capacity) return false;
        if (*p == '"' || *p == '\\') {
            out[(*used)++] = '\\';
            out[(*used)++] = *p;
        } else if (*p >= 0x20) {
            out[(*used)++] = *p;
        }
    }
    out[(*used)++] = '"';
    out[*used] = '\0';
    return true;
}

esp_err_t network_service_scan_json(char *json, size_t capacity, size_t *written)
{
    ESP_RETURN_ON_FALSE(json && capacity >= 32 && written, ESP_ERR_INVALID_ARG, TAG,
                        "invalid scan output buffer");
    wifi_scan_config_t scan = {.show_hidden = false, .scan_type = WIFI_SCAN_TYPE_ACTIVE};
    ESP_RETURN_ON_ERROR(esp_wifi_scan_start(&scan, true), TAG, "network scan failed");

    uint16_t count = WIFI_SCAN_LIMIT;
    wifi_ap_record_t records[WIFI_SCAN_LIMIT];
    memset(records, 0, sizeof(records));
    ESP_RETURN_ON_ERROR(esp_wifi_scan_get_ap_records(&count, records), TAG,
                        "could not retrieve scan results");

    size_t used = 0;
    json[used++] = '[';
    for (uint16_t i = 0; i < count; ++i) {
        if (records[i].ssid[0] == '\0') continue;
        if (used > 1) json[used++] = ',';
        int n = snprintf(json + used, capacity - used, "{\"ssid\":");
        if (n < 0 || (size_t)n >= capacity - used) return ESP_ERR_NO_MEM;
        used += n;
        if (!append_json_string(json, capacity, &used, (const char *)records[i].ssid)) {
            return ESP_ERR_NO_MEM;
        }
        n = snprintf(json + used, capacity - used,
                     ",\"rssi\":%d,\"channel\":%u,\"open\":%s}",
                     records[i].rssi, records[i].primary,
                     records[i].authmode == WIFI_AUTH_OPEN ? "true" : "false");
        if (n < 0 || (size_t)n >= capacity - used) return ESP_ERR_NO_MEM;
        used += n;
    }
    if (used + 2 > capacity) return ESP_ERR_NO_MEM;
    json[used++] = ']';
    json[used] = '\0';
    *written = used;
    return ESP_OK;
}
