#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_camera.h"

esp_err_t camera_service_init(void);
sensor_t *camera_service_sensor(void);
int camera_service_get_fps(void);
void camera_service_set_fps(int fps);
void camera_service_set_led(int duty);
int camera_service_get_led(void);
void camera_service_stream_started(void);
void camera_service_stream_stopped(void);
const char *camera_service_framesize_name(framesize_t size);
bool camera_service_parse_framesize(const char *name, framesize_t *size);
