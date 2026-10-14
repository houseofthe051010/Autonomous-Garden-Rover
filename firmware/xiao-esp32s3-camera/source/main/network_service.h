#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool sta_configured;
    bool sta_connected;
    char sta_ssid[33];
    char sta_ip[16];
    char ap_ip[16];
    int sta_rssi;
    int ap_client_rssi;
    uint16_t ap_clients;
    uint8_t channel;
} network_status_t;

esp_err_t network_service_init(void);
void network_service_get_status(network_status_t *status);
esp_err_t network_service_connect(const char *ssid, const char *password);
esp_err_t network_service_forget(void);
esp_err_t network_service_scan_json(char *json, size_t capacity, size_t *written);
