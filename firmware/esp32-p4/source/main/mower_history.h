#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t mower_history_start(void);
esp_err_t mower_history_register_routes(httpd_handle_t server);
