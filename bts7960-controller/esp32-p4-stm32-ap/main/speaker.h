#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

typedef struct {
    bool ready;
    bool storage_mounted;
    bool storage_is_sd;
    bool playing;
    bool paused;
    bool recording;
    bool recording_pending;
    uint8_t source_button;
    uint8_t kind;
    uint16_t tone_hz;
    uint8_t mic_gain_db;
    uint8_t speaker_volume;
    uint8_t speaker_boost_db;
    uint32_t recording_ms;
    uint64_t recording_bytes;
    uint64_t total_bytes;
    uint64_t free_bytes;
    char current[64];
    char recording_name[64];
    char last_recording[64];
    char assignment[2][64];
    char error[128];
} speaker_status_t;

esp_err_t speaker_start(void);
esp_err_t speaker_register_routes(httpd_handle_t server);
void speaker_controller_buttons(uint16_t button_mask);
void speaker_get_status(speaker_status_t *status);
