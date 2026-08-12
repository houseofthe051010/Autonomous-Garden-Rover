#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"

typedef bool (*bno080_motion_begin_fn)(void);
typedef bool (*bno080_motion_drive_fn)(int left, int right,
                                      unsigned ramp_percent_per_second,
                                      bool immediate_stop);
typedef void (*bno080_motion_end_fn)(void);

esp_err_t bno080_link_start(bno080_motion_begin_fn begin_motion,
                            bno080_motion_drive_fn drive,
                            bno080_motion_end_fn end_motion);
esp_err_t bno080_link_register_routes(httpd_handle_t server);
void bno080_link_abort_calibration(void);
