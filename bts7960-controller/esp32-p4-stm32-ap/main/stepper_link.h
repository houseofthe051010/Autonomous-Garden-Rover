#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t stepper_link_start(void);
esp_err_t stepper_link_register_routes(httpd_handle_t server);
void stepper_link_quick_stop(void);
bool stepper_link_set_direct_rpm(float x_rpm, float y_rpm, float z_rpm,
                                 int lease_ms);
