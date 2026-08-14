#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "esp_http_server.h"

typedef void (*network_ota_safe_stop_fn)(void);

esp_err_t network_ota_start(void);
esp_err_t network_ota_register_routes(httpd_handle_t server,
                                      network_ota_safe_stop_fn safe_stop);
void network_ota_mark_running_app_valid(void);
void network_ota_set_controller_active(bool active);
bool network_ota_controller_active(void);
bool network_ota_get_saved_credentials(char *ssid, size_t ssid_size,
                                       char *password, size_t password_size);
