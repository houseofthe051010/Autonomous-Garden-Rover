#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "board_config.h"

typedef struct {
    int16_t samples[MIC_BLOCK_SAMPLES];
    size_t count;
    uint32_t sequence;
} audio_block_t;

typedef struct {
    bool ready;
    bool listener_active;
    float gain_db;
    float rms_dbfs;
    float peak_dbfs;
    uint32_t clipped_samples;
    uint32_t dropped_blocks;
    uint32_t sequence;
} audio_status_t;

esp_err_t audio_service_init(void);
bool audio_service_take(audio_block_t *block, uint32_t timeout_ms);
void audio_service_set_listener(bool active);
esp_err_t audio_service_set_gain(float gain_db);
void audio_service_get_status(audio_status_t *status);
