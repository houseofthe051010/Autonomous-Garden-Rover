#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

typedef struct {
    bool connected;
    bool voltage_valid;
    bool voltage_clipped;
    bool current_valid;
    float voltage_v;
    float current_a;
    float power_w;
    int64_t voltage_age_ms;
} odesc_power_snapshot_t;

typedef struct {
    bool connected;
    bool voltage_clipped;
    bool active;
    bool controller_owned;
    bool current_valid;
    bool faulted;
    float target_turns_s;
    float minimum_turns_s;
    float maximum_turns_s;
    float estimated_rpm;
    float voltage_v;
    float current_a;
    float power_w;
    int axis_state;
} odesc_controller_snapshot_t;

typedef struct {
    bool connected;
    bool voltage_valid;
    bool current_valid;
    bool motion_valid;
    bool active;
    bool faulted;
    int64_t telemetry_age_ms;
    float voltage_v;
    float bus_current_a;
    float bus_power_w;
    float command_turns_s;
    float estimated_turns_s;
    float estimated_rpm;
    float iq_measured_a;
    float iq_setpoint_a;
    float id_measured_a;
    float id_setpoint_a;
    float phase_current_magnitude_a;
    float motor_voltage_v;
    float motor_power_w;
    float fet_temperature_c;
    int axis_state;
    uint32_t system_error;
    uint32_t axis_error;
    uint32_t motor_error;
    uint32_t controller_error;
    uint32_t estimator_error;
} odesc_mower_snapshot_t;

esp_err_t odesc_link_start(void);
esp_err_t odesc_link_register_routes(httpd_handle_t server);
void odesc_link_stop_all(void);
bool odesc_link_get_power(odesc_power_snapshot_t *snapshot);
bool odesc_link_get_mower_telemetry(odesc_mower_snapshot_t *snapshot);
void odesc_link_controller_request(bool enabled, float velocity_turns_s);
bool odesc_link_get_controller(odesc_controller_snapshot_t *snapshot);
