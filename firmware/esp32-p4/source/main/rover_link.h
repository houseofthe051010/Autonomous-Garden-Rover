#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef void (*rover_link_drive_fn)(int left, int right,
                                    unsigned ramp_percent_per_second,
                                    bool immediate_stop,
                                    int16_t x_rpm, int16_t y_rpm, int16_t z_rpm);

typedef struct {
    bool active;
    int64_t age_ms;
    uint32_t sequence;
    uint16_t right_x;
    uint16_t right_y;
    uint16_t button_mask;
    int16_t stepper_rpm[3];
    uint32_t valid_packets;
    uint32_t rejected_packets;
    uint32_t datagrams;
    uint32_t wrong_size_packets;
    uint32_t auth_rejects;
    uint32_t range_rejects;
    uint32_t stale_packets;
    uint32_t acks_sent;
    uint32_t ack_failures;
} rover_link_status_t;

esp_err_t rover_link_start(rover_link_drive_fn drive_callback);
void rover_link_get_status(rover_link_status_t *status);
