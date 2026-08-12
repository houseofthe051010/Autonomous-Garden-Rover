#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t odesc_persistent_log_start(void);
void odesc_persistent_log_event(int64_t uptime_ms, const char *message);
esp_err_t odesc_persistent_log_handler(httpd_req_t *request);
