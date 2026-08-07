#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t odesc_link_start(void);
esp_err_t odesc_link_register_routes(httpd_handle_t server);
void odesc_link_stop_all(void);
