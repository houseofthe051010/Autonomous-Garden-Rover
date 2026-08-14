#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t stepper_link_start(void);
esp_err_t stepper_link_register_routes(httpd_handle_t server);
void stepper_link_quick_stop(void);
bool stepper_link_set_direct_rpm(float x_rpm, float y_rpm, float z_rpm,
                                 int lease_ms);
bool stepper_link_move_counted(int32_t x_steps, int32_t y_steps,
                               int32_t z_steps, int x_rpm, int y_rpm,
                               int z_rpm);
bool stepper_link_counted_active(void);
