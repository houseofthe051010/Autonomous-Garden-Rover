#include "odesc_link.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "odesc_persistent_log.h"

#define ODESC_UART UART_NUM_3
#define ODESC_BAUD 115200
#define ODESC_TX_GPIO 27
#define ODESC_RX_GPIO 47
#define ODESC_QUERY_TIMEOUT_MS 650
#define ODESC_POLL_MS 2000
#define ODESC_LINK_TIMEOUT_MS 4000
#define ODESC_MOTION_LEASE_MS 2000
#define ODESC_LINE_CAPACITY 128
#define ODESC_MOTION_POLL_MS 650
#define ODESC_IDLE_MOTION_POLL_MS 5000
#define ODESC_CONFIG_POLL_MS 60000
#define ODESC_AXIS_WATCHDOG_SECONDS 3.0f
#define ODESC_MOTOR_KV_RPM_PER_V 170.0f
#define ODESC_RECOMMENDED_VOLTAGE_MARGIN 0.80f
#define ODESC_MAX_COMMAND_RPM 7000.0f
#define ODESC_VBUS_DIVIDER_RATIO 11.0f
#define ODESC_VBUS_ADC_REFERENCE_V 3.3f
#define ODESC_VBUS_ADC_CLIP_MARGIN_V 0.00f
#define ODESC_VBUS_ADC_CLIP_V \
    (ODESC_VBUS_DIVIDER_RATIO * ODESC_VBUS_ADC_REFERENCE_V - \
     ODESC_VBUS_ADC_CLIP_MARGIN_V)
#define ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S \
    (ODESC_MAX_COMMAND_RPM / 60.0f)
#define ODESC_MIN_CURRENT_LIMIT_A 1.0f
#define ODESC_MIN_CURRENT_MARGIN_A 2.0f
#define ODESC_MIN_REGEN_CURRENT_A 0.1f
#define ODESC_MAX_REGEN_CURRENT_A 10.0f
#define ODESC_MIN_VELOCITY_RAMP_TURNS_S2 0.5f
#define ODESC_MAX_VELOCITY_RAMP_TURNS_S2 50.0f
#define ODESC_BLACKBOX_RECORDS 256
#define ODESC_BLACKBOX_MESSAGE_SIZE 104
#define ODESC_HISTORY_RECORDS 96
#define ODESC_HISTORY_WINDOW_MS 60000

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_SENSORLESS_CLOSED_LOOP 8
#define INPUT_MODE_VEL_RAMP 2

typedef struct {
    float current_limit_a;
    float current_limit_margin_a;
    float max_allowed_current_a;
    float velocity_limit_turns_s;
    float velocity_ramp_turns_s2;
    float sensorless_min_turns_s;
    int pole_pairs;
    int input_mode;
    float overvoltage_trip_v;
} odesc_limits_t;

typedef struct {
    float selected_v;
    float internal_v;
    float external_adc_v;
    float external_scale;
    bool external_supported;
    bool external_valid;
    bool external_fault;
    unsigned external_status;
} odesc_vbus_t;

typedef struct {
    bool connected;
    bool detecting;
    int tx_gpio;
    int rx_gpio;
    int active_axis;
    int64_t last_reply_ms;
    int64_t last_voltage_ms;
    int64_t motion_deadline_ms;
    float vbus_voltage;
    bool voltage_clipped;
    float vbus_voltage_internal;
    bool external_vbus_supported;
    bool external_vbus_valid;
    bool external_vbus_fault;
    unsigned external_vbus_status;
    float ibus_a;
    bool current_valid;
    bool motion_telemetry_valid;
    int64_t last_motion_ms;
    int axis_state;
    int system_error;
    int axis_error;
    int motor_error;
    int controller_error;
    int estimator_error;
    int pole_pairs;
    float sensorless_velocity_turns_s;
    float sensorless_rpm;
    float iq_measured_a;
    float iq_setpoint_a;
    float id_measured_a;
    float id_setpoint_a;
    float fet_temperature_c;
    float motor_power_w;
    float motor_voltage_v;
    float command_velocity_turns_s;
    bool controller_requested;
    bool controller_owned;
    bool controller_rearm_required;
    float controller_velocity_turns_s;
    int64_t controller_deadline_ms;
    float sensorless_min_turns_s;
    bool config_valid;
    float current_limit_a;
    float current_limit_margin_a;
    float max_allowed_current_a;
    float velocity_limit_turns_s;
    float velocity_ramp_turns_s2;
    float overvoltage_trip_v;
    float external_adc_v;
    float external_scale;
    int input_mode;
    unsigned reconnects;
    unsigned queries;
    unsigned failures;
    unsigned uart_fifo_overflows;
    unsigned uart_buffer_full;
    unsigned uart_frame_errors;
    unsigned uart_parity_errors;
    unsigned uart_breaks;
    bool odesc_log_supported;
    unsigned odesc_log_records;
    unsigned odesc_uart_rx_bytes;
    unsigned odesc_dma_restarts;
    unsigned odesc_uart_errors;
    unsigned odesc_silence_events;
    char last_command[96];
    char last_reply[96];
    char last_action[128];
    char last_error[128];
} odesc_state_t;

typedef struct {
    int64_t timestamp_ms;
    char message[ODESC_BLACKBOX_MESSAGE_SIZE];
} odesc_blackbox_record_t;

typedef struct {
    int64_t timestamp_ms;
    float speed_rpm;
    float bus_current_a;
    float bus_power_w;
    float iq_current_a;
    float fet_temperature_c;
    float motor_power_w;
    float motor_voltage_v;
    float bus_voltage_v;
} odesc_history_record_t;

static const char *TAG = "odesc_uart";
static odesc_state_t state = {
    .tx_gpio = -1,
    .rx_gpio = -1,
    .active_axis = -1,
    .fet_temperature_c = NAN,
    .motor_power_w = NAN,
    .motor_voltage_v = NAN,
};
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t io_mutex;
static SemaphoreHandle_t blackbox_mutex;
static QueueHandle_t uart_event_queue;
static bool uart_installed;
static char line_buffer[ODESC_LINE_CAPACITY];
static size_t line_length;
static odesc_blackbox_record_t blackbox[ODESC_BLACKBOX_RECORDS];
static size_t blackbox_head;
static size_t blackbox_count;
static odesc_history_record_t history[ODESC_HISTORY_RECORDS];
static size_t history_head;
static size_t history_count;
static unsigned odesc_log_seen;
static int external_vbus_capability = -1;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool voltage_is_clipped(const odesc_vbus_t *vbus)
{
    if (!vbus || !isfinite(vbus->selected_v)) return true;
    if (vbus->external_supported) {
        return !vbus->external_valid || vbus->external_fault ||
               vbus->external_status != 0;
    }
    return vbus->selected_v >= ODESC_VBUS_ADC_CLIP_V;
}

static void blackbox_log(const char *format, ...)
{
    if (!blackbox_mutex || !format) return;
    char message[ODESC_BLACKBOX_MESSAGE_SIZE];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(message, sizeof(message), format, arguments);
    va_end(arguments);

    xSemaphoreTake(blackbox_mutex, portMAX_DELAY);
    int64_t timestamp_ms = now_ms();
    odesc_blackbox_record_t *record = &blackbox[blackbox_head];
    record->timestamp_ms = timestamp_ms;
    snprintf(record->message, sizeof(record->message), "%s", message);
    blackbox_head = (blackbox_head + 1) % ODESC_BLACKBOX_RECORDS;
    if (blackbox_count < ODESC_BLACKBOX_RECORDS) blackbox_count++;
    xSemaphoreGive(blackbox_mutex);
    odesc_persistent_log_event(timestamp_ms, message);
}

static void history_log_locked(void)
{
    odesc_history_record_t *record = &history[history_head];
    record->timestamp_ms = now_ms();
    record->speed_rpm = state.sensorless_rpm;
    record->bus_current_a = state.current_valid ? state.ibus_a : NAN;
    record->bus_power_w = state.current_valid ?
                          state.vbus_voltage * state.ibus_a : NAN;
    record->iq_current_a = state.iq_measured_a;
    record->fet_temperature_c = state.fet_temperature_c;
    record->motor_power_w = state.motor_power_w;
    record->motor_voltage_v = state.motor_voltage_v;
    record->bus_voltage_v = state.vbus_voltage;
    history_head = (history_head + 1) % ODESC_HISTORY_RECORDS;
    if (history_count < ODESC_HISTORY_RECORDS) history_count++;
}

static void uart_close_locked(void)
{
    if (uart_installed) {
        uart_driver_delete(ODESC_UART);
        uart_installed = false;
    }
    uart_event_queue = NULL;
    gpio_reset_pin(ODESC_TX_GPIO);
    gpio_reset_pin(ODESC_RX_GPIO);
    gpio_set_direction(ODESC_TX_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(ODESC_RX_GPIO, GPIO_MODE_INPUT);
    line_length = 0;
}

static bool uart_open_locked(int tx_gpio, int rx_gpio)
{
    uart_close_locked();
    const uart_config_t config = {
        .baud_rate = ODESC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t result = uart_driver_install(ODESC_UART, 1024, 0, 16,
                                           &uart_event_queue, 0);
    if (result == ESP_OK) result = uart_param_config(ODESC_UART, &config);
    if (result == ESP_OK) {
        result = uart_set_pin(ODESC_UART, tx_gpio, rx_gpio,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    uart_installed = result == ESP_OK;
    if (!uart_installed) {
        blackbox_log("UART open failed TX%d RX%d: %s", tx_gpio, rx_gpio,
                     esp_err_to_name(result));
        uart_close_locked();
        return false;
    }
    gpio_set_pull_mode(rx_gpio, GPIO_PULLUP_ONLY);
    uart_flush_input(ODESC_UART);
    uart_write_bytes(ODESC_UART, "\n\n", 2);
    uart_wait_tx_done(ODESC_UART, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_flush_input(ODESC_UART);
    blackbox_log("UART opened TX%d RX%d at %d baud", tx_gpio, rx_gpio, ODESC_BAUD);
    return true;
}

static void drain_uart_events_locked(void)
{
    if (!uart_event_queue) return;
    uart_event_t event;
    while (xQueueReceive(uart_event_queue, &event, 0) == pdTRUE) {
        const char *name = NULL;
        unsigned *counter = NULL;
        switch (event.type) {
        case UART_FIFO_OVF:
            name = "FIFO_OVF";
            counter = &state.uart_fifo_overflows;
            uart_flush_input(ODESC_UART);
            line_length = 0;
            break;
        case UART_BUFFER_FULL:
            name = "BUFFER_FULL";
            counter = &state.uart_buffer_full;
            uart_flush_input(ODESC_UART);
            line_length = 0;
            break;
        case UART_FRAME_ERR:
            name = "FRAME_ERR";
            counter = &state.uart_frame_errors;
            break;
        case UART_PARITY_ERR:
            name = "PARITY_ERR";
            counter = &state.uart_parity_errors;
            break;
        case UART_BREAK:
            name = "BREAK";
            counter = &state.uart_breaks;
            break;
        default:
            break;
        }
        if (counter) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            unsigned count = ++*counter;
            xSemaphoreGive(state_mutex);
            blackbox_log("FAULT UART %s count=%u buffered=%u", name, count,
                         (unsigned)event.size);
        }
    }
}

static bool read_line_locked(char *output, size_t output_size, uint32_t timeout_ms)
{
    int64_t deadline = now_ms() + timeout_ms;
    uint8_t byte;
    while (uart_installed && now_ms() < deadline) {
        drain_uart_events_locked();
        int remaining = (int)(deadline - now_ms());
        TickType_t wait = pdMS_TO_TICKS(remaining > 20 ? 20 : remaining);
        if (uart_read_bytes(ODESC_UART, &byte, 1, wait) != 1) continue;
        if (byte == '\n') {
            if (!line_length) continue;
            line_buffer[line_length] = '\0';
            snprintf(output, output_size, "%s", line_buffer);
            line_length = 0;
            return true;
        }
        if (byte == '\r') continue;
        if (line_length < sizeof(line_buffer) - 1) {
            line_buffer[line_length++] = (char)byte;
        } else {
            line_length = 0;
        }
    }
    return false;
}

static bool send_locked(const char *command)
{
    if (!uart_installed || !command || !command[0] || strchr(command, '\n') ||
        strchr(command, '\r')) return false;
    int length = (int)strlen(command);
    bool sent = uart_write_bytes(ODESC_UART, command, length) == length &&
                uart_write_bytes(ODESC_UART, "\n", 1) == 1;
    if (sent) uart_wait_tx_done(ODESC_UART, pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "TX: %s", command);
    blackbox_log("TX%s %s", sent ? "" : " FAILED", command);
    return sent;
}

static bool query_locked(const char *command, char *reply, size_t reply_size,
                         uint32_t timeout_ms)
{
    if (!uart_installed) return false;
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        uart_flush_input(ODESC_UART);
        line_length = 0;
        if (!send_locked(command)) return false;
        if (read_line_locked(reply, reply_size, timeout_ms)) {
            ESP_LOGI(TAG, "RX: %s", reply);
            blackbox_log("RX %s", reply);
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.last_reply_ms = now_ms();
            state.queries++;
            snprintf(state.last_reply, sizeof(state.last_reply), "%.95s", reply);
            xSemaphoreGive(state_mutex);
            return true;
        }
        blackbox_log("RX timeout attempt %u: %s", attempt + 1, command);
        if (attempt == 0) vTaskDelay(pdMS_TO_TICKS(25));
    }
    return false;
}

static bool parse_float_reply(const char *reply, float *value)
{
    errno = 0;
    char *end = NULL;
    float parsed = strtof(reply, &end);
    while (end && (*end == ' ' || *end == '\t')) ++end;
    if (errno || end == reply || !end || *end || !isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

static bool parse_int_reply(const char *reply, int *value)
{
    char buffer[ODESC_LINE_CAPACITY];
    snprintf(buffer, sizeof(buffer), "%s", reply);
    size_t length = strlen(buffer);
    if (length && buffer[length - 1] == 'd') buffer[--length] = 0;
    errno = 0;
    char *end = NULL;
    long parsed = strtol(buffer, &end, 0);
    while (end && (*end == ' ' || *end == '\t')) ++end;
    if (errno || end == buffer || !end || *end || parsed < INT32_MIN ||
        parsed > INT32_MAX) return false;
    *value = (int)parsed;
    return true;
}

static bool query_float_locked(const char *property, float *value)
{
    char command[96];
    char reply[ODESC_LINE_CAPACITY];
    snprintf(command, sizeof(command), "r %s", property);
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        if (!query_locked(command, reply, sizeof(reply), ODESC_QUERY_TIMEOUT_MS)) {
            continue;
        }
        if (parse_float_reply(reply, value)) return true;
        blackbox_log("FAULT malformed float reply %s: %.72s", property, reply);
    }
    return false;
}

static bool query_int_locked(const char *property, int *value)
{
    char command[96];
    char reply[ODESC_LINE_CAPACITY];
    snprintf(command, sizeof(command), "r %s", property);
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        if (!query_locked(command, reply, sizeof(reply), ODESC_QUERY_TIMEOUT_MS)) {
            continue;
        }
        if (parse_int_reply(reply, value)) return true;
        blackbox_log("FAULT malformed int reply %s: %.72s", property, reply);
    }
    return false;
}

static bool query_vbus_locked(odesc_vbus_t *vbus)
{
    if (!vbus) return false;
    memset(vbus, 0, sizeof(*vbus));
    if (!query_float_locked("vbus_voltage", &vbus->selected_v) ||
        vbus->selected_v < 0.0f || vbus->selected_v > 100.0f) {
        return false;
    }

    if (external_vbus_capability == 0) {
        vbus->internal_v = vbus->selected_v;
        return true;
    }

    int valid = 0;
    int fault = 0;
    int status = 0;
    bool complete =
        query_float_locked("vbus_voltage_internal", &vbus->internal_v) &&
        query_float_locked("vbus_voltage_external_adc", &vbus->external_adc_v) &&
        query_float_locked("vbus_voltage_external_scale", &vbus->external_scale) &&
        query_int_locked("vbus_voltage_external_valid", &valid) &&
        query_int_locked("vbus_voltage_external_fault", &fault) &&
        query_int_locked("vbus_voltage_external_status", &status);
    if (!complete) {
        if (external_vbus_capability > 0) return false;
        external_vbus_capability = 0;
        vbus->internal_v = vbus->selected_v;
        blackbox_log("VBUS legacy firmware; onboard ADC clip interlock active");
        return true;
    }

    external_vbus_capability = 1;
    vbus->external_supported = true;
    vbus->external_valid = valid != 0;
    vbus->external_fault = fault != 0;
    vbus->external_status = status < 0 ? UINT32_MAX : (unsigned)status;
    return true;
}

static bool query_int_quick_locked(const char *property, int *value)
{
    char command[96];
    char reply[ODESC_LINE_CAPACITY];
    snprintf(command, sizeof(command), "r %s", property);
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        if (!query_locked(command, reply, sizeof(reply), 100)) continue;
        if (parse_int_reply(reply, value)) return true;
        blackbox_log("FAULT malformed quick int reply %s: %.64s", property, reply);
    }
    return false;
}

static bool query_float_quick_locked(const char *property, float *value)
{
    char command[96];
    char reply[ODESC_LINE_CAPACITY];
    snprintf(command, sizeof(command), "r %s", property);
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        if (!query_locked(command, reply, sizeof(reply), 100)) continue;
        if (parse_float_reply(reply, value)) return true;
        blackbox_log("FAULT malformed quick float reply %s: %.64s", property,
                     reply);
    }
    return false;
}

static bool fetch_odesc_blackbox_locked(void)
{
    char reply[ODESC_LINE_CAPACITY];
    unsigned count = 0, rx_bytes = 0, restarts = 0, errors = 0, silence = 0;
    if (!query_locked("ds", reply, sizeof(reply), ODESC_QUERY_TIMEOUT_MS) ||
        sscanf(reply, "DS %u %u %u %u %u", &count, &rx_bytes, &restarts,
               &errors, &silence) != 5) {
        return false;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.odesc_log_supported = true;
    state.odesc_log_records = count;
    state.odesc_uart_rx_bytes = rx_bytes;
    state.odesc_dma_restarts = restarts;
    state.odesc_uart_errors = errors;
    state.odesc_silence_events = silence;
    xSemaphoreGive(state_mutex);

    if (odesc_log_seen > count) odesc_log_seen = 0;
    if (odesc_log_seen == 0 && count > 64) odesc_log_seen = count - 64;
    unsigned stop = count;
    if (stop - odesc_log_seen > 8) stop = odesc_log_seen + 8;
    for (unsigned index = odesc_log_seen; index < stop; ++index) {
        const char chunks[] = {'a', 'b', 'c', 'd'};
        for (size_t part = 0; part < sizeof(chunks); ++part) {
            char command[20];
            snprintf(command, sizeof(command), "d%c %u", chunks[part], index);
            if (!query_locked(command, reply, sizeof(reply), ODESC_QUERY_TIMEOUT_MS)) {
                return false;
            }
            blackbox_log("ODESCLOG %s", reply);
        }
        odesc_log_seen = index + 1;
    }
    return true;
}

static bool write_property_locked(int axis, const char *property, int value)
{
    char command[96];
    snprintf(command, sizeof(command), "w axis%d.%s %d", axis, property, value);
    return send_locked(command);
}

static bool write_property_float_locked(int axis, const char *property, float value)
{
    char command[96];
    snprintf(command, sizeof(command), "w axis%d.%s %.9g", axis, property, value);
    return send_locked(command);
}

static bool write_global_property_float_locked(const char *property, float value)
{
    char command[96];
    snprintf(command, sizeof(command), "w %s %.9g", property, value);
    return send_locked(command);
}

static bool set_idle_locked(int axis)
{
    char command[48];
    snprintf(command, sizeof(command), "v %d 0 0", axis);
    bool sent = send_locked(command);
    bool idle = write_property_locked(axis, "requested_state", AXIS_STATE_IDLE);
    bool watchdog_off = write_property_locked(axis, "config.enable_watchdog", 0);
    return sent && idle && watchdog_off;
}

static bool feed_watchdog_locked(int axis)
{
    char command[16];
    snprintf(command, sizeof(command), "u %d", axis);
    return send_locked(command);
}

static bool read_limits_locked(odesc_limits_t *limits)
{
    float ramp_velocity = 0.0f;
    if (!limits ||
        !query_float_locked("axis0.motor.config.current_lim",
                            &limits->current_limit_a) ||
        !query_float_locked("axis0.motor.config.current_lim_margin",
                            &limits->current_limit_margin_a) ||
        !query_float_locked("axis0.motor.max_allowed_current",
                            &limits->max_allowed_current_a) ||
        !query_float_locked("axis0.controller.config.vel_limit",
                            &limits->velocity_limit_turns_s) ||
        !query_float_locked("axis0.controller.config.vel_ramp_rate",
                            &limits->velocity_ramp_turns_s2) ||
        !query_float_locked("config.dc_bus_overvoltage_trip_level",
                            &limits->overvoltage_trip_v) ||
        !query_int_locked("axis0.controller.config.input_mode",
                          &limits->input_mode) ||
        !query_int_locked("axis0.motor.config.pole_pairs", &limits->pole_pairs) ||
        !query_float_locked("axis0.config.sensorless_ramp.vel", &ramp_velocity) ||
        limits->pole_pairs <= 0) {
        return false;
    }
    limits->sensorless_min_turns_s = fabsf(ramp_velocity) /
        (2.0f * (float)M_PI * limits->pole_pairs);
    return isfinite(limits->current_limit_a) &&
           isfinite(limits->current_limit_margin_a) &&
           isfinite(limits->max_allowed_current_a) &&
           isfinite(limits->velocity_limit_turns_s) &&
           isfinite(limits->velocity_ramp_turns_s2) &&
           isfinite(limits->overvoltage_trip_v) &&
           isfinite(limits->sensorless_min_turns_s);
}

static void store_limits(const odesc_limits_t *limits)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (limits) {
        state.config_valid = true;
        state.current_limit_a = limits->current_limit_a;
        state.current_limit_margin_a = limits->current_limit_margin_a;
        state.max_allowed_current_a = limits->max_allowed_current_a;
        state.velocity_limit_turns_s = limits->velocity_limit_turns_s;
        state.velocity_ramp_turns_s2 = limits->velocity_ramp_turns_s2;
        state.overvoltage_trip_v = limits->overvoltage_trip_v;
        state.sensorless_min_turns_s = limits->sensorless_min_turns_s;
        state.pole_pairs = limits->pole_pairs;
        state.input_mode = limits->input_mode;
    } else {
        state.config_valid = false;
    }
    xSemaphoreGive(state_mutex);
}

static bool refresh_limits(void)
{
    odesc_limits_t limits = {0};
    bool okay = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(2500)) == pdTRUE) {
        okay = read_limits_locked(&limits);
        xSemaphoreGive(io_mutex);
    }
    store_limits(okay ? &limits : NULL);
    return okay;
}

static bool probe_fixed_link(odesc_vbus_t *vbus)
{
    bool ok = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1500)) != pdTRUE) return false;
    if (uart_open_locked(ODESC_TX_GPIO, ODESC_RX_GPIO)) {
        vTaskDelay(pdMS_TO_TICKS(30));
        external_vbus_capability = -1;
        ok = query_vbus_locked(vbus);
    }
    xSemaphoreGive(io_mutex);
    return ok;
}

static bool find_link(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.detecting = true;
    state.connected = false;
    state.tx_gpio = -1;
    state.rx_gpio = -1;
    state.motion_telemetry_valid = false;
    state.axis_state = -1;
    state.sensorless_velocity_turns_s = 0.0f;
    state.sensorless_rpm = 0.0f;
    state.iq_measured_a = 0.0f;
    xSemaphoreGive(state_mutex);

    odesc_vbus_t vbus = {0};
    ESP_LOGI(TAG, "Probing ODESC TX GPIO%d RX GPIO%d",
             ODESC_TX_GPIO, ODESC_RX_GPIO);
    if (probe_fixed_link(&vbus)) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.connected = true;
        state.detecting = false;
        state.tx_gpio = ODESC_TX_GPIO;
        state.rx_gpio = ODESC_RX_GPIO;
        state.vbus_voltage = vbus.selected_v;
        state.voltage_clipped = voltage_is_clipped(&vbus);
        state.vbus_voltage_internal = vbus.internal_v;
        state.external_adc_v = vbus.external_adc_v;
        state.external_scale = vbus.external_scale;
        state.external_vbus_supported = vbus.external_supported;
        state.external_vbus_valid = vbus.external_valid;
        state.external_vbus_fault = vbus.external_fault;
        state.external_vbus_status = vbus.external_status;
        state.last_voltage_ms = now_ms();
        state.reconnects++;
        state.failures = 0;
        state.last_error[0] = '\0';
        snprintf(state.last_action, sizeof(state.last_action),
                 state.voltage_clipped ?
                 "ODESC UART verified; VBUS measurement unsafe at %.2f V" :
                 "ODESC UART verified at %.2f V", vbus.selected_v);
        xSemaphoreGive(state_mutex);
        ESP_LOGI(TAG, "ODESC verified: TX GPIO%d RX GPIO%d VBUS %.3f V",
                 ODESC_TX_GPIO, ODESC_RX_GPIO, vbus.selected_v);
        blackbox_log("LINK connected VBUS %.3f V internal %.3f V ext=%u/%u/%u",
                     vbus.selected_v, vbus.internal_v,
                     vbus.external_supported, vbus.external_valid,
                     vbus.external_status);
        return true;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.detecting = false;
    state.connected = false;
    snprintf(state.last_error, sizeof(state.last_error),
             "No ODESC vbus reply: TX GPIO27 RX GPIO47");
    xSemaphoreGive(state_mutex);
    blackbox_log("LINK probe failed TX27 RX47");
    return false;
}

static bool refresh_voltage(bool include_current)
{
    odesc_vbus_t vbus = {0};
    float current = 0.0f;
    bool voltage_ok = false;
    bool current_ok = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1500)) != pdTRUE) return false;
    voltage_ok = query_vbus_locked(&vbus);
    if (voltage_ok && include_current) current_ok = query_float_locked("ibus", &current);
    xSemaphoreGive(io_mutex);

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (voltage_ok) {
        state.vbus_voltage = vbus.selected_v;
        state.voltage_clipped = voltage_is_clipped(&vbus);
        state.vbus_voltage_internal = vbus.internal_v;
        state.external_adc_v = vbus.external_adc_v;
        state.external_scale = vbus.external_scale;
        state.external_vbus_supported = vbus.external_supported;
        state.external_vbus_valid = vbus.external_valid;
        state.external_vbus_fault = vbus.external_fault;
        state.external_vbus_status = vbus.external_status;
        state.last_voltage_ms = now_ms();
        state.failures = 0;
        if (state.voltage_clipped) {
            snprintf(state.last_error, sizeof(state.last_error),
                     "VBUS measurement invalid: motion locked");
        } else {
            state.last_error[0] = '\0';
        }
        if (include_current && current_ok) {
            state.ibus_a = current;
            state.current_valid = true;
        }
    } else {
        state.failures++;
        if (external_vbus_capability > 0) {
            state.voltage_clipped = true;
            state.external_vbus_valid = false;
        }
        snprintf(state.last_error, sizeof(state.last_error),
                 "No valid ODESC vbus_voltage reply");
    }
    xSemaphoreGive(state_mutex);
    return voltage_ok;
}

static bool refresh_motion_telemetry(void)
{
    static unsigned slow_detail_slot;
    int axis_state = 0;
    int system_error = 0;
    int axis_error = 0;
    int motor_error = 0;
    int controller_error = 0;
    int estimator_error = 0;
    int pole_pairs = 0;
    float sensorless_velocity = 0.0f;
    float iq_measured = 0.0f;
    float iq_setpoint = 0.0f;
    float id_measured = 0.0f;
    float id_setpoint = 0.0f;
    float bus_current = 0.0f;
    float fet_temperature = 0.0f;
    float motor_power = 0.0f;
    float voltage_alpha = 0.0f;
    float voltage_beta = 0.0f;
    bool bus_current_ok = false;
    bool fet_temperature_ok = false;
    bool motor_power_ok = false;
    bool motor_voltage_ok = false;
    bool okay = false;
    bool must_stop = false;
    bool was_active = false;
    int previous_system_error = 0;
    int previous_axis_error = 0;
    int previous_motor_error = 0;
    int previous_controller_error = 0;
    int previous_estimator_error = 0;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    pole_pairs = state.pole_pairs;
    was_active = state.active_axis >= 0;
    previous_system_error = state.system_error;
    previous_axis_error = state.axis_error;
    previous_motor_error = state.motor_error;
    previous_controller_error = state.controller_error;
    previous_estimator_error = state.estimator_error;
    xSemaphoreGive(state_mutex);

    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1500)) != pdTRUE) return false;
    okay = query_int_quick_locked("error", &system_error) &&
           query_int_quick_locked("axis0.current_state", &axis_state) &&
           query_int_quick_locked("axis0.error", &axis_error) &&
           query_int_quick_locked("axis0.motor.error", &motor_error) &&
           query_int_quick_locked("axis0.controller.error", &controller_error) &&
           query_int_quick_locked("axis0.sensorless_estimator.error",
                                  &estimator_error) &&
           query_float_quick_locked("axis0.sensorless_estimator.vel_estimate",
                                    &sensorless_velocity) &&
           query_float_quick_locked("axis0.motor.current_control.Iq_measured",
                                    &iq_measured) &&
           query_float_quick_locked("axis0.motor.current_control.Iq_setpoint",
                                    &iq_setpoint) &&
           query_float_quick_locked("axis0.motor.current_control.Id_measured",
                                    &id_measured) &&
           query_float_quick_locked("axis0.motor.current_control.Id_setpoint",
                                    &id_setpoint) &&
           query_float_quick_locked("axis0.motor.I_bus", &bus_current) &&
           query_float_quick_locked("axis0.motor.current_control.power",
                                    &motor_power);
    bus_current_ok = okay;
    motor_power_ok = okay;
    if (okay && pole_pairs <= 0) {
        okay = query_int_quick_locked("axis0.motor.config.pole_pairs", &pole_pairs);
    }
    if (okay) {
        switch (slow_detail_slot++ % 2) {
        case 0:
            fet_temperature_ok = query_float_quick_locked(
                "axis0.motor.fet_thermistor.temperature", &fet_temperature);
            break;
        default:
            motor_voltage_ok = query_float_quick_locked(
                "axis0.motor.current_control.final_v_alpha", &voltage_alpha) &&
                query_float_quick_locked(
                "axis0.motor.current_control.final_v_beta", &voltage_beta);
            break;
        }
    }
    if (okay && (system_error || axis_error || motor_error || controller_error ||
                 estimator_error)) {
        must_stop = true;
        if (system_error != previous_system_error ||
            axis_error != previous_axis_error ||
            motor_error != previous_motor_error ||
            controller_error != previous_controller_error ||
            estimator_error != previous_estimator_error) {
            blackbox_log("FAULT system=%d axis=%d motor=%d controller=%d estimator=%d",
                         system_error, axis_error, motor_error, controller_error,
                         estimator_error);
        }
        if (was_active || axis_state != AXIS_STATE_IDLE) set_idle_locked(0);
    }
    xSemaphoreGive(io_mutex);

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (okay && pole_pairs > 0) {
        state.axis_state = axis_state;
        state.system_error = system_error;
        state.axis_error = axis_error;
        state.motor_error = motor_error;
        state.controller_error = controller_error;
        state.estimator_error = estimator_error;
        state.pole_pairs = pole_pairs;
        state.sensorless_velocity_turns_s = sensorless_velocity;
        state.sensorless_rpm = state.sensorless_velocity_turns_s * 60.0f;
        state.iq_measured_a = iq_measured;
        state.iq_setpoint_a = iq_setpoint;
        state.id_measured_a = id_measured;
        state.id_setpoint_a = id_setpoint;
        if (bus_current_ok) {
            state.ibus_a = bus_current;
            state.current_valid = true;
        }
        if (fet_temperature_ok) state.fet_temperature_c = fet_temperature;
        if (motor_power_ok) state.motor_power_w = motor_power;
        if (motor_voltage_ok) {
            state.motor_voltage_v = hypotf(voltage_alpha, voltage_beta);
        }
        state.motion_telemetry_valid = true;
        state.last_motion_ms = now_ms();
        if (must_stop) {
            state.active_axis = -1;
            state.motion_deadline_ms = 0;
            snprintf(state.last_error, sizeof(state.last_error),
                     "ODESC fault: system=%d axis=%d motor=%d controller=%d estimator=%d",
                     system_error, axis_error, motor_error, controller_error,
                     estimator_error);
        } else if (state.active_axis == 0 &&
                   axis_state != AXIS_STATE_SENSORLESS_CLOSED_LOOP) {
            state.active_axis = -1;
            state.motion_deadline_ms = 0;
            snprintf(state.last_error, sizeof(state.last_error),
                     "M0 left sensorless closed loop state 8 (state=%d)", axis_state);
        }
        history_log_locked();
    } else {
        state.motion_telemetry_valid = false;
    }
    xSemaphoreGive(state_mutex);
    return okay;
}

static bool start_sensorless_locked(float velocity, char *error,
                                    size_t error_size, float *minimum_speed,
                                    int *pole_pairs_out);

static void service_task(void *argument)
{
    (void)argument;
    int64_t next_poll = 0;
    int64_t next_motion_poll = 0;
    int64_t next_config_poll = 0;
    int64_t next_diag_poll = 0;
    unsigned poll_count = 0;
    int64_t next_controller_command = 0;
    while (true) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool connected = state.connected;
        int axis = state.active_axis;
        bool lease_expired = axis >= 0 && state.motion_deadline_ms &&
                             now_ms() >= state.motion_deadline_ms;
        if (lease_expired) state.motion_deadline_ms = 0;
        unsigned failures = state.failures;
        bool controller_requested = state.controller_requested;
        bool controller_owned = state.controller_owned;
        bool controller_rearm_required = state.controller_rearm_required;
        float controller_velocity = state.controller_velocity_turns_s;
        bool controller_expired = controller_requested &&
            now_ms() >= state.controller_deadline_ms;
        if (controller_expired) {
            state.controller_requested = false;
            controller_requested = false;
        }
        xSemaphoreGive(state_mutex);

        if (!connected) {
            if (!find_link()) vTaskDelay(pdMS_TO_TICKS(1200));
            next_poll = now_ms();
            next_config_poll = now_ms();
            continue;
        }

        if (lease_expired) {
            blackbox_log("DEADMAN expired axis=%d", axis);
            if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                set_idle_locked(axis);
                xSemaphoreGive(io_mutex);
            }
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.active_axis = -1;
            state.command_velocity_turns_s = 0.0f;
            snprintf(state.last_action, sizeof(state.last_action),
                     "Motion deadman stopped axis %d", axis);
            xSemaphoreGive(state_mutex);
        }

        if ((!controller_requested && controller_owned) || controller_expired) {
            blackbox_log("HANDHELD mower stop%s%s",
                         controller_expired ? " (controller timeout)" : "",
                         controller_rearm_required ?
                         " (speed reduction; release to rearm)" : "");
            bool stopped = false;
            if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1200)) == pdTRUE) {
                stopped = set_idle_locked(0);
                xSemaphoreGive(io_mutex);
            }
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.controller_owned = false;
            state.active_axis = -1;
            state.motion_deadline_ms = 0;
            state.command_velocity_turns_s = 0.0f;
            snprintf(state.last_action, sizeof(state.last_action),
                     "Handheld mower stopped%s%s",
                     controller_expired ? " after link timeout" : "",
                     controller_rearm_required ?
                     "; release mower button before restart" : "");
            if (!stopped) snprintf(state.last_error, sizeof(state.last_error),
                                  "Handheld mower stop write failed");
            xSemaphoreGive(state_mutex);
            controller_owned = false;
            axis = -1;
        }

        if (controller_requested && now_ms() >= next_controller_command) {
            next_controller_command = now_ms() + 220;
            bool command_ok = false;
            char error[128] = {0};
            float minimum = 0.0f;
            int pole_pairs = 0;
            if (!controller_owned && axis < 0) {
                if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1800)) == pdTRUE) {
                    command_ok = start_sensorless_locked(controller_velocity,
                        error, sizeof(error), &minimum, &pole_pairs);
                    xSemaphoreGive(io_mutex);
                } else {
                    snprintf(error, sizeof(error), "ODESC UART busy");
                }
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                if (command_ok) {
                    state.controller_owned = true;
                    state.active_axis = 0;
                    state.axis_state = AXIS_STATE_SENSORLESS_CLOSED_LOOP;
                    state.pole_pairs = pole_pairs;
                    state.sensorless_min_turns_s = minimum;
                    state.command_velocity_turns_s = controller_velocity;
                    state.motion_deadline_ms = now_ms() + ODESC_MOTION_LEASE_MS;
                    state.last_error[0] = 0;
                    snprintf(state.last_action, sizeof(state.last_action),
                             "Handheld started MOWER at %.1f turns/s",
                             controller_velocity);
                    blackbox_log("HANDHELD START MOWER %.1f turns/s",
                                 controller_velocity);
                } else {
                    snprintf(state.last_error, sizeof(state.last_error), "%s",
                             error[0] ? error : "Handheld mower start failed");
                }
                xSemaphoreGive(state_mutex);
                controller_owned = command_ok;
                if (command_ok) axis = 0;
            } else if (controller_owned && axis == 0) {
                char command[48];
                snprintf(command, sizeof(command), "v 0 %.6f 0",
                         controller_velocity);
                if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    command_ok = send_locked(command) && feed_watchdog_locked(0);
                    xSemaphoreGive(io_mutex);
                }
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                if (command_ok) {
                    state.command_velocity_turns_s = controller_velocity;
                    state.motion_deadline_ms = now_ms() + ODESC_MOTION_LEASE_MS;
                    state.last_error[0] = 0;
                } else {
                    snprintf(state.last_error, sizeof(state.last_error),
                             "Handheld mower velocity refresh failed");
                }
                xSemaphoreGive(state_mutex);
            }
        }

        if (now_ms() >= next_poll) {
            next_poll = now_ms() + ODESC_POLL_MS;
            refresh_voltage((++poll_count % 2) == 0);
        }

        if (axis < 0 && now_ms() >= next_config_poll) {
            next_config_poll = now_ms() + ODESC_CONFIG_POLL_MS;
            refresh_limits();
        }

        if (axis < 0 && now_ms() >= next_diag_poll) {
            next_diag_poll = now_ms() + 60000;
            if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(2500)) == pdTRUE) {
                fetch_odesc_blackbox_locked();
                xSemaphoreGive(io_mutex);
            }
        }

        int64_t motion_period = axis >= 0 ? ODESC_MOTION_POLL_MS :
                                                   ODESC_IDLE_MOTION_POLL_MS;
        if (now_ms() >= next_motion_poll) {
            next_motion_poll = now_ms() + motion_period;
            refresh_motion_telemetry();
        }

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        failures = state.failures;
        int64_t last_reply = state.last_reply_ms;
        xSemaphoreGive(state_mutex);
        if (failures >= 3 || (last_reply && now_ms() - last_reply > ODESC_LINK_TIMEOUT_MS)) {
            blackbox_log("LINK timeout failures=%u reply_age=%lld ms", failures,
                         last_reply ? now_ms() - last_reply : -1);
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.connected = false;
            state.tx_gpio = -1;
            state.rx_gpio = -1;
            state.active_axis = -1;
            state.command_velocity_turns_s = 0.0f;
            state.controller_requested = false;
            state.controller_owned = false;
            state.current_valid = false;
            state.motion_telemetry_valid = false;
            state.axis_state = -1;
            state.sensorless_velocity_turns_s = 0.0f;
            state.sensorless_rpm = 0.0f;
            state.iq_measured_a = 0.0f;
            state.config_valid = false;
            snprintf(state.last_error, sizeof(state.last_error), "ODESC link timed out");
            xSemaphoreGive(state_mutex);
            if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                uart_close_locked();
                xSemaphoreGive(io_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static bool query_value(httpd_req_t *request, const char *key,
                        char *output, size_t output_size)
{
    char query[256];
    return httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK &&
           httpd_query_key_value(query, key, output, output_size) == ESP_OK;
}

static bool request_axis(httpd_req_t *request, int *axis)
{
    char text[8];
    if (!query_value(request, "axis", text, sizeof(text))) {
        *axis = 0;
        return true;
    }
    char *end = NULL;
    long value = strtol(text, &end, 10);
    if (!end || *end || value != 0) return false;
    *axis = 0;
    return true;
}

static bool start_sensorless_locked(float velocity, char *error, size_t error_size,
                                    float *minimum_speed, int *pole_pairs_out)
{
    int calibrated = 0;
    int sensorless_enabled = 0;
    int pole_pairs = 0;
    int axis_state = 0;
    int system_error = 0;
    int axis_error = 0;
    int motor_error = 0;
    int estimator_error = 0;
    float ramp_velocity = 0.0f;
    float ramp_acceleration = 0.0f;
    float velocity_limit = 0.0f;
    odesc_vbus_t vbus = {0};
    bool okay = set_idle_locked(0);
    vTaskDelay(pdMS_TO_TICKS(60));
    okay = okay && query_int_locked("axis0.motor.config.pre_calibrated", &calibrated) &&
           query_int_locked("axis0.config.enable_sensorless_mode", &sensorless_enabled) &&
           query_int_locked("axis0.motor.config.pole_pairs", &pole_pairs) &&
           query_float_locked("axis0.config.sensorless_ramp.vel", &ramp_velocity) &&
           query_float_locked("axis0.config.sensorless_ramp.accel", &ramp_acceleration) &&
           query_float_locked("axis0.controller.config.vel_limit", &velocity_limit) &&
           query_vbus_locked(&vbus) &&
           query_int_locked("error", &system_error) &&
           query_int_locked("axis0.error", &axis_error) &&
           query_int_locked("axis0.motor.error", &motor_error) &&
           query_int_locked("axis0.sensorless_estimator.error", &estimator_error);
    if (!okay) {
        snprintf(error, error_size, "Could not read M0 sensorless configuration");
        return false;
    }
    if (voltage_is_clipped(&vbus)) {
        snprintf(error, error_size,
                 "VBUS measurement unsafe at %.2f V; motion locked",
                 vbus.selected_v);
        return false;
    }
    if (!calibrated) {
        snprintf(error, error_size, "M0 motor is not calibrated");
        return false;
    }
    if (!sensorless_enabled) {
        snprintf(error, error_size, "M0 sensorless mode is disabled");
        return false;
    }
    if (pole_pairs <= 0 || pole_pairs > 100 || ramp_acceleration == 0.0f) {
        snprintf(error, error_size, "Invalid M0 pole-pair/ramp configuration");
        return false;
    }
    if (system_error || axis_error || motor_error || estimator_error) {
        snprintf(error, error_size,
                 "Existing fault system=%d axis=%d motor=%d estimator=%d",
                 system_error, axis_error, motor_error, estimator_error);
        return false;
    }
    float scale = 2.0f * (float)M_PI * pole_pairs;
    float minimum = fabsf(ramp_velocity) / scale;
    float maximum = fminf(velocity_limit,
                          ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S);
    if (fabsf(velocity) + 0.001f < minimum || fabsf(velocity) > maximum) {
        snprintf(error, error_size, "Speed must be %.2f through %.2f turns/s",
                 minimum, maximum);
        return false;
    }
    float signed_ramp = copysignf(fabsf(ramp_velocity), velocity);
    okay = write_property_locked(0, "controller.config.input_mode",
                                 INPUT_MODE_VEL_RAMP) &&
           write_property_float_locked(0, "config.sensorless_ramp.vel", signed_ramp) &&
           write_property_locked(0, "requested_state", AXIS_STATE_SENSORLESS_CLOSED_LOOP);
    vTaskDelay(pdMS_TO_TICKS(250));
    okay = okay && query_int_locked("error", &system_error) &&
           query_int_locked("axis0.current_state", &axis_state) &&
           query_int_locked("axis0.error", &axis_error) &&
           query_int_locked("axis0.motor.error", &motor_error) &&
           query_int_locked("axis0.sensorless_estimator.error", &estimator_error);
    if (!okay || system_error ||
        axis_state != AXIS_STATE_SENSORLESS_CLOSED_LOOP || axis_error ||
        motor_error || estimator_error) {
        set_idle_locked(0);
        snprintf(error, error_size,
                 "Start failed state=%d system=%d axis=%d motor=%d estimator=%d",
                 axis_state, system_error, axis_error, motor_error,
                 estimator_error);
        return false;
    }
    okay = write_property_float_locked(0, "config.watchdog_timeout",
                                       ODESC_AXIS_WATCHDOG_SECONDS) &&
           feed_watchdog_locked(0) &&
           write_property_locked(0, "config.enable_watchdog", 1) &&
           feed_watchdog_locked(0);
    if (!okay) {
        set_idle_locked(0);
        snprintf(error, error_size, "Could not arm M0 command watchdog");
        return false;
    }
    char command[64];
    snprintf(command, sizeof(command), "v 0 %.6f 0", velocity);
    if (!send_locked(command) || !feed_watchdog_locked(0)) {
        set_idle_locked(0);
        snprintf(error, error_size, "Sensorless velocity write failed");
        return false;
    }
    *minimum_speed = minimum;
    *pole_pairs_out = pole_pairs;
    return true;
}

static esp_err_t status_response(httpd_req_t *request)
{
    odesc_state_t snapshot;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshot = state;
    xSemaphoreGive(state_mutex);
    int64_t voltage_age = snapshot.last_voltage_ms ?
                          now_ms() - snapshot.last_voltage_ms : -1;
    float power = snapshot.current_valid ?
                  snapshot.vbus_voltage * snapshot.ibus_a : 0.0f;
    float no_load_rpm = snapshot.voltage_clipped ? 0.0f :
                        snapshot.vbus_voltage * ODESC_MOTOR_KV_RPM_PER_V;
    float voltage_speed_turns_s = no_load_rpm / 60.0f;
    float voltage_control_turns_s = voltage_speed_turns_s *
                                    ODESC_RECOMMENDED_VOLTAGE_MARGIN;
    float current_ceiling = snapshot.max_allowed_current_a -
                            snapshot.current_limit_margin_a;
    if (current_ceiling < 0.0f) current_ceiling = 0.0f;
    char fet_temperature[24];
    char motor_power[24];
    char motor_voltage[24];
    if (isfinite(snapshot.fet_temperature_c)) {
        snprintf(fet_temperature, sizeof(fet_temperature), "%.3f",
                 snapshot.fet_temperature_c);
    } else {
        snprintf(fet_temperature, sizeof(fet_temperature), "null");
    }
    if (isfinite(snapshot.motor_power_w)) {
        snprintf(motor_power, sizeof(motor_power), "%.3f",
                 snapshot.motor_power_w);
    } else {
        snprintf(motor_power, sizeof(motor_power), "null");
    }
    if (isfinite(snapshot.motor_voltage_v)) {
        snprintf(motor_voltage, sizeof(motor_voltage), "%.3f",
                 snapshot.motor_voltage_v);
    } else {
        snprintf(motor_voltage, sizeof(motor_voltage), "null");
    }
    char body[3400];
    snprintf(body, sizeof(body),
             "{\"connected\":%s,\"detecting\":%s,\"tx\":%d,\"rx\":%d,"
             "\"vbus_voltage\":%.4f,\"voltage_clipped\":%s,\"voltage_age_ms\":%lld,"
             "\"vbus_internal\":%.4f,\"external_vbus_adc\":%.6f,"
             "\"external_vbus_scale\":%.6f,"
             "\"external_vbus_supported\":%s,"
             "\"external_vbus_valid\":%s,\"external_vbus_fault\":%s,"
             "\"external_vbus_status\":%u,"
             "\"current_valid\":%s,\"ibus_a\":%.4f,\"bus_power_w\":%.3f,"
             "\"motion_valid\":%s,\"axis_state\":%d,\"system_error\":%d,"
             "\"axis_error\":%d,"
             "\"motor_error\":%d,\"controller_error\":%d,"
             "\"estimator_error\":%d,\"pole_pairs\":%d,"
             "\"command_velocity_turns_s\":%.4f,"
             "\"sensorless_min_turns_s\":%.4f,"
             "\"sensorless_velocity_turns_s\":%.4f,\"sensorless_rpm\":%.1f,"
             "\"iq_measured_a\":%.4f,\"iq_setpoint_a\":%.4f,"
             "\"id_measured_a\":%.4f,\"id_setpoint_a\":%.4f,"
             "\"fet_temperature_c\":%s,\"motor_power_w\":%s,"
             "\"motor_voltage_v\":%s,"
             "\"config_valid\":%s,\"current_limit_a\":%.3f,"
             "\"current_limit_margin_a\":%.3f,"
             "\"max_allowed_current_a\":%.3f,\"current_ceiling_a\":%.3f,"
             "\"velocity_limit_turns_s\":%.3f,"
             "\"velocity_ramp_turns_s2\":%.3f,\"input_mode\":%d,"
             "\"overvoltage_trip_v\":%.3f,"
             "\"motor_kv_rpm_per_v\":%.1f,\"no_load_rpm\":%.1f,"
             "\"voltage_speed_turns_s\":%.3f,"
             "\"voltage_control_turns_s\":%.3f,"
             "\"absolute_velocity_limit_turns_s\":%.3f,"
             "\"absolute_velocity_limit_rpm\":%.1f,"
             "\"axis_watchdog_seconds\":%.1f,"
             "\"active_axis\":%d,\"reconnects\":%u,\"queries\":%u,"
             "\"failures\":%u,\"uart_fifo_overflows\":%u,"
             "\"uart_buffer_full\":%u,\"uart_frame_errors\":%u,"
             "\"uart_parity_errors\":%u,\"uart_breaks\":%u,"
             "\"odesc_log_supported\":%s,\"odesc_log_records\":%u,"
             "\"odesc_uart_rx_bytes\":%u,\"odesc_dma_restarts\":%u,"
             "\"odesc_uart_errors\":%u,\"odesc_silence_events\":%u,"
             "\"last_command\":\"%.80s\","
             "\"last_reply\":\"%.80s\",\"last_action\":\"%.110s\","
             "\"last_error\":\"%.110s\"}",
             snapshot.connected ? "true" : "false",
             snapshot.detecting ? "true" : "false",
             snapshot.tx_gpio, snapshot.rx_gpio, snapshot.vbus_voltage,
             snapshot.voltage_clipped ? "true" : "false",
             (long long)voltage_age,
             snapshot.vbus_voltage_internal,
             snapshot.external_adc_v, snapshot.external_scale,
             snapshot.external_vbus_supported ? "true" : "false",
             snapshot.external_vbus_valid ? "true" : "false",
             snapshot.external_vbus_fault ? "true" : "false",
             snapshot.external_vbus_status,
             snapshot.current_valid ? "true" : "false", snapshot.ibus_a, power,
             snapshot.motion_telemetry_valid ? "true" : "false",
             snapshot.axis_state, snapshot.system_error, snapshot.axis_error,
             snapshot.motor_error, snapshot.controller_error,
             snapshot.estimator_error, snapshot.pole_pairs,
             snapshot.command_velocity_turns_s, snapshot.sensorless_min_turns_s,
             snapshot.sensorless_velocity_turns_s, snapshot.sensorless_rpm,
             snapshot.iq_measured_a, snapshot.iq_setpoint_a,
             snapshot.id_measured_a, snapshot.id_setpoint_a,
             fet_temperature, motor_power, motor_voltage,
             snapshot.config_valid ? "true" : "false",
             snapshot.current_limit_a, snapshot.current_limit_margin_a,
             snapshot.max_allowed_current_a, current_ceiling,
             snapshot.velocity_limit_turns_s, snapshot.velocity_ramp_turns_s2,
             snapshot.input_mode, snapshot.overvoltage_trip_v,
             ODESC_MOTOR_KV_RPM_PER_V, no_load_rpm,
             voltage_speed_turns_s, voltage_control_turns_s,
             ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S, ODESC_MAX_COMMAND_RPM,
             ODESC_AXIS_WATCHDOG_SECONDS,
             snapshot.active_axis, snapshot.reconnects, snapshot.queries,
             snapshot.failures, snapshot.uart_fifo_overflows,
             snapshot.uart_buffer_full, snapshot.uart_frame_errors,
             snapshot.uart_parity_errors, snapshot.uart_breaks,
             snapshot.odesc_log_supported ? "true" : "false",
             snapshot.odesc_log_records, snapshot.odesc_uart_rx_bytes,
             snapshot.odesc_dma_restarts, snapshot.odesc_uart_errors,
             snapshot.odesc_silence_events,
             snapshot.last_command, snapshot.last_reply,
             snapshot.last_action, snapshot.last_error);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, body);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    return status_response(request);
}

static esp_err_t refresh_handler(httpd_req_t *request)
{
    refresh_voltage(true);
    return status_response(request);
}

static esp_err_t config_handler(httpd_req_t *request)
{
    if (!refresh_limits()) {
        httpd_resp_set_status(request, "503 Service Unavailable");
    }
    return status_response(request);
}

static bool request_float(httpd_req_t *request, const char *key, float *value)
{
    char text[32];
    if (!query_value(request, key, text, sizeof(text))) return false;
    errno = 0;
    char *end = NULL;
    float parsed = strtof(text, &end);
    if (errno || end == text || !end || *end || !isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

static esp_err_t limits_handler(httpd_req_t *request)
{
    float current = 0.0f;
    float margin = 0.0f;
    float velocity = 0.0f;
    float ramp = 0.0f;
    char persist_text[8] = {0};
    bool persist = query_value(request, "persist", persist_text,
                               sizeof(persist_text)) && atoi(persist_text) != 0;
    if (!request_float(request, "current", &current) ||
        !request_float(request, "margin", &margin) ||
        !request_float(request, "velocity", &velocity) ||
        !request_float(request, "ramp", &ramp)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "current, margin, velocity and ramp are required");
    }

    odesc_limits_t limits = {0};
    odesc_limits_t verified = {0};
    int axis_state = 0;
    odesc_vbus_t vbus = {0};
    char error[160] = {0};
    bool okay = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(3500)) == pdTRUE) {
        okay = query_int_locked("axis0.current_state", &axis_state) &&
               query_vbus_locked(&vbus) &&
               read_limits_locked(&limits);
        if (!okay) {
            snprintf(error, sizeof(error), "Could not read ODESC limits");
        } else if (axis_state != AXIS_STATE_IDLE) {
            okay = false;
            snprintf(error, sizeof(error), "STOP M0 before changing limits");
        } else if (voltage_is_clipped(&vbus)) {
            okay = false;
            snprintf(error, sizeof(error), "VBUS measurement is unsafe");
        } else {
            float current_ceiling = limits.max_allowed_current_a -
                                    limits.current_limit_margin_a;
            float speed_ceiling = ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S;
            if (current < ODESC_MIN_CURRENT_LIMIT_A || current > current_ceiling) {
                okay = false;
                snprintf(error, sizeof(error),
                         "Phase-current limit must be %.1f through %.1f A",
                         ODESC_MIN_CURRENT_LIMIT_A, current_ceiling);
            } else if (margin < ODESC_MIN_CURRENT_MARGIN_A ||
                       current + margin > limits.max_allowed_current_a) {
                okay = false;
                snprintf(error, sizeof(error),
                         "Current margin must be %.1f through %.1f A at %.1f A limit",
                         ODESC_MIN_CURRENT_MARGIN_A,
                         limits.max_allowed_current_a - current, current);
            } else if (velocity + 0.001f < limits.sensorless_min_turns_s ||
                       velocity > speed_ceiling) {
                okay = false;
                snprintf(error, sizeof(error),
                         "Speed limit must be %.2f through %.2f turns/s",
                         limits.sensorless_min_turns_s, speed_ceiling);
            } else if (ramp < ODESC_MIN_VELOCITY_RAMP_TURNS_S2 ||
                       ramp > ODESC_MAX_VELOCITY_RAMP_TURNS_S2) {
                okay = false;
                snprintf(error, sizeof(error),
                         "Velocity ramp must be %.1f through %.1f turns/s^2",
                         ODESC_MIN_VELOCITY_RAMP_TURNS_S2,
                         ODESC_MAX_VELOCITY_RAMP_TURNS_S2);
            }
        }
        if (okay) {
            okay = write_property_float_locked(0, "motor.config.current_lim", current) &&
                   write_property_float_locked(0, "motor.config.current_lim_margin", margin) &&
                   write_property_float_locked(0, "controller.config.vel_limit", velocity) &&
                   write_property_float_locked(0, "controller.config.vel_ramp_rate", ramp) &&
                   write_property_locked(0, "controller.config.input_mode",
                                         INPUT_MODE_VEL_RAMP);
            if (!okay) snprintf(error, sizeof(error), "ODESC limit write failed");
        }
        if (okay && persist) {
            okay = send_locked("ss");
            if (!okay) snprintf(error, sizeof(error), "ODESC persistent save failed");
        } else if (okay) {
            vTaskDelay(pdMS_TO_TICKS(30));
            okay = read_limits_locked(&verified);
            if (!okay || fabsf(verified.current_limit_a - current) > 0.02f ||
                fabsf(verified.current_limit_margin_a - margin) > 0.02f ||
                fabsf(verified.velocity_limit_turns_s - velocity) > 0.02f ||
                fabsf(verified.velocity_ramp_turns_s2 - ramp) > 0.02f ||
                verified.input_mode != INPUT_MODE_VEL_RAMP) {
                okay = false;
                snprintf(error, sizeof(error), "ODESC limit readback did not match");
            }
        }
        xSemaphoreGive(io_mutex);
    } else {
        snprintf(error, sizeof(error), "ODESC UART busy");
    }

    if (okay && !persist) store_limits(&verified);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_command, sizeof(state.last_command),
             "limits current %.2f margin %.2f speed %.2f ramp %.2f%s",
             current, margin, velocity, ramp, persist ? " save" : "");
    if (okay) {
        state.last_error[0] = 0;
        if (persist) {
            state.connected = false;
            state.detecting = true;
            state.config_valid = false;
            state.active_axis = -1;
            state.motion_deadline_ms = 0;
            state.command_velocity_turns_s = 0.0f;
            snprintf(state.last_action, sizeof(state.last_action),
                     "Limits saved; ODESC restarting");
        } else {
            snprintf(state.last_action, sizeof(state.last_action),
                     "Runtime limits applied and verified");
        }
    } else {
        snprintf(state.last_error, sizeof(state.last_error), "%s", error);
    }
    xSemaphoreGive(state_mutex);
    blackbox_log("LIMITS %s current=%.2f margin=%.2f speed=%.2f ramp=%.2f %s",
                 okay ? "OK" : "FAILED", current, margin, velocity, ramp,
                 error[0] ? error : "");
    if (!okay) httpd_resp_set_status(request, "409 Conflict");
    return status_response(request);
}

static esp_err_t regen_limit_handler(httpd_req_t *request)
{
    float amps = 0.0f;
    char persist_text[8] = {0};
    bool persist = query_value(request, "persist", persist_text,
                               sizeof(persist_text)) && atoi(persist_text) != 0;
    if (!request_float(request, "amps", &amps)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "amps is required");
    }
    if (amps < ODESC_MIN_REGEN_CURRENT_A || amps > ODESC_MAX_REGEN_CURRENT_A) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "regen allowance must be 0.1 through 10 A");
    }

    int axis_state = 0;
    float verified = 0.0f;
    char error[160] = {0};
    bool okay = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(3500)) == pdTRUE) {
        okay = query_int_locked("axis0.current_state", &axis_state);
        if (!okay) {
            snprintf(error, sizeof(error), "Could not read ODESC axis state");
        } else if (axis_state != AXIS_STATE_IDLE) {
            okay = false;
            snprintf(error, sizeof(error), "STOP M0 before changing regen limit");
        } else {
            okay = write_global_property_float_locked(
                       "config.dc_max_negative_current", -amps);
            if (okay) {
                vTaskDelay(pdMS_TO_TICKS(30));
                okay = query_float_locked("config.dc_max_negative_current",
                                          &verified) &&
                       fabsf(verified + amps) <= 0.02f;
            }
            if (!okay) {
                snprintf(error, sizeof(error),
                         "ODESC regen-limit write/readback failed");
            }
        }
        if (okay && persist) {
            okay = send_locked("ss");
            if (!okay) {
                snprintf(error, sizeof(error),
                         "ODESC regen-limit persistent save failed");
            }
        }
        xSemaphoreGive(io_mutex);
    } else {
        snprintf(error, sizeof(error), "ODESC UART busy");
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_command, sizeof(state.last_command),
             "config.dc_max_negative_current = -%.2f A%s", amps,
             persist ? " save" : "");
    if (okay) {
        state.last_error[0] = 0;
        if (persist) {
            state.connected = false;
            state.detecting = true;
            state.config_valid = false;
            state.active_axis = -1;
            state.motion_deadline_ms = 0;
            state.command_velocity_turns_s = 0.0f;
            snprintf(state.last_action, sizeof(state.last_action),
                     "Regen allowance %.2f A saved; ODESC restarting", amps);
        } else {
            snprintf(state.last_action, sizeof(state.last_action),
                     "Regen allowance %.2f A applied and verified", amps);
        }
    } else {
        snprintf(state.last_error, sizeof(state.last_error), "%s", error);
    }
    xSemaphoreGive(state_mutex);
    blackbox_log("REGEN %s allowance=%.2f A readback=%.3f A %s",
                 okay ? "OK" : "FAILED", amps, verified,
                 error[0] ? error : "");
    if (!okay) httpd_resp_set_status(request, "409 Conflict");
    return status_response(request);
}

static esp_err_t state_handler(httpd_req_t *request)
{
    int axis;
    char value_text[8];
    if (!request_axis(request, &axis) ||
        !query_value(request, "value", value_text, sizeof(value_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad axis/state");
    }
    int value = atoi(value_text);
    if (value != AXIS_STATE_IDLE) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "use sensorless start; direct state must be 1");
    }
    bool sent = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1200)) == pdTRUE) {
        sent = write_property_locked(axis, "requested_state", value);
        xSemaphoreGive(io_mutex);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_command, sizeof(state.last_command),
             "w axis%d.requested_state %d", axis, value);
    if (sent) {
        if (value == AXIS_STATE_IDLE && state.active_axis == axis) state.active_axis = -1;
        state.last_error[0] = '\0';
        snprintf(state.last_action, sizeof(state.last_action),
                 "Axis %d requested state %d", axis, value);
    } else {
        snprintf(state.last_error, sizeof(state.last_error), "ODESC UART write failed");
    }
    xSemaphoreGive(state_mutex);
    return status_response(request);
}

static esp_err_t sensorless_start_handler(httpd_req_t *request)
{
    int axis;
    char velocity_text[32];
    if (!request_axis(request, &axis) ||
        !query_value(request, "velocity", velocity_text, sizeof(velocity_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "bad sensorless start request");
    }
    char *end = NULL;
    float velocity = strtof(velocity_text, &end);
    if (!end || *end || !isfinite(velocity) || fabsf(velocity) < 1.0f ||
        fabsf(velocity) > ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "velocity is outside the controller guard range");
    }
    char error[128] = {0};
    float minimum = 0.0f;
    int pole_pairs = 0;
    bool started = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
        started = start_sensorless_locked(velocity, error, sizeof(error),
                                          &minimum, &pole_pairs);
        xSemaphoreGive(io_mutex);
    } else {
        snprintf(error, sizeof(error), "ODESC UART busy");
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_command, sizeof(state.last_command),
             "sensorless start M0 %.3f turns/s", velocity);
    if (started) {
        state.active_axis = 0;
        state.axis_state = AXIS_STATE_SENSORLESS_CLOSED_LOOP;
        state.pole_pairs = pole_pairs;
        state.command_velocity_turns_s = velocity;
        state.sensorless_min_turns_s = minimum;
        state.motion_deadline_ms = now_ms() + ODESC_MOTION_LEASE_MS;
        state.last_error[0] = 0;
        snprintf(state.last_action, sizeof(state.last_action),
                 "M0 sensorless started %.2f turns/s (minimum %.2f)",
                 velocity, minimum);
    } else {
        state.active_axis = -1;
        state.motion_deadline_ms = 0;
        snprintf(state.last_error, sizeof(state.last_error), "%s", error);
    }
    xSemaphoreGive(state_mutex);
    if (!started) {
        blackbox_log("START failed velocity=%.3f: %s", velocity, error);
        httpd_resp_set_status(request, "409 Conflict");
    } else {
        blackbox_log("START M0 velocity=%.3f", velocity);
    }
    return status_response(request);
}

static esp_err_t velocity_handler(httpd_req_t *request)
{
    int axis;
    char velocity_text[32];
    char brief_text[8] = {0};
    bool brief = query_value(request, "brief", brief_text,
                             sizeof(brief_text)) && atoi(brief_text) != 0;
    if (!request_axis(request, &axis) ||
        !query_value(request, "velocity", velocity_text, sizeof(velocity_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad velocity request");
    }
    char *end = NULL;
    float velocity = strtof(velocity_text, &end);
    if (!end || *end || !isfinite(velocity) ||
        fabsf(velocity) > ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S) {
        blackbox_log("GUARD malformed/absolute velocity request: %.3f", velocity);
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "velocity is outside the controller guard range");
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool active = state.active_axis == axis;
    float minimum = state.sensorless_min_turns_s;
    float previous = state.command_velocity_turns_s;
    float velocity_limit = state.velocity_limit_turns_s;
    bool voltage_clipped = state.voltage_clipped;
    bool config_valid = state.config_valid;
    xSemaphoreGive(state_mutex);
    if (voltage_clipped) {
        blackbox_log("GUARD velocity %.3f rejected: VBUS invalid", velocity);
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "VBUS measurement invalid; motion is locked");
    }
    if (!active || velocity == 0.0f || fabsf(velocity) + 0.001f < minimum ||
        velocity * previous <= 0.0f || !config_valid ||
        fabsf(velocity) > velocity_limit ||
        fabsf(velocity) > ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S) {
        blackbox_log("GUARD velocity=%.2f active=%d previous=%.2f min=%.2f limit=%.2f",
                     velocity, active, previous, minimum, velocity_limit);
        httpd_resp_set_status(request, "409 Conflict");
        return httpd_resp_sendstr(request,
            "sensorless command violates active direction or configured speed limit");
    }
    char command[64];
    snprintf(command, sizeof(command), "v %d %.6f 0", axis, velocity);
    bool sent = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1200)) == pdTRUE) {
        sent = send_locked(command) && feed_watchdog_locked(axis);
        xSemaphoreGive(io_mutex);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_command, sizeof(state.last_command), "%s", command);
    if (sent) {
        state.active_axis = axis;
        state.command_velocity_turns_s = velocity;
        state.motion_deadline_ms = now_ms() + ODESC_MOTION_LEASE_MS;
        state.last_error[0] = '\0';
        snprintf(state.last_action, sizeof(state.last_action),
                 "Axis %d velocity %.3f turns/s", axis, velocity);
    } else {
        snprintf(state.last_error, sizeof(state.last_error), "ODESC UART write failed");
        blackbox_log("VELOCITY write failed %.3f", velocity);
    }
    xSemaphoreGive(state_mutex);
    if (brief) {
        httpd_resp_set_type(request, "text/plain");
        if (!sent) httpd_resp_set_status(request, "409 Conflict");
        return httpd_resp_sendstr(request, sent ? "OK" : "ODESC UART write failed");
    }
    return status_response(request);
}

static esp_err_t stop_handler(httpd_req_t *request)
{
    int axis;
    if (!request_axis(request, &axis)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad axis");
    }
    bool sent = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1200)) == pdTRUE) {
        sent = set_idle_locked(axis);
        xSemaphoreGive(io_mutex);
    }
    if (sent) {
        vTaskDelay(pdMS_TO_TICKS(80));
        refresh_motion_telemetry();
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.active_axis == axis) state.active_axis = -1;
    state.motion_deadline_ms = 0;
    state.command_velocity_turns_s = 0.0f;
    snprintf(state.last_command, sizeof(state.last_command),
             "v %d 0 0; requested_state 1", axis);
    snprintf(state.last_action, sizeof(state.last_action), "Axis %d stopped and idle", axis);
    if (!sent) snprintf(state.last_error, sizeof(state.last_error), "ODESC stop write failed");
    else state.last_error[0] = '\0';
    xSemaphoreGive(state_mutex);
    blackbox_log("STOP M0 %s", sent ? "complete" : "failed");
    return status_response(request);
}

static esp_err_t clear_handler(httpd_req_t *request)
{
    int axis;
    if (!request_axis(request, &axis)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad axis");
    }
    bool sent = false;
    bool captured = false;
    int axis_state = -1;
    int system_error = -1;
    int axis_error = -1;
    int motor_error = -1;
    int controller_error = -1;
    int estimator_error = -1;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
        captured = query_int_quick_locked("error", &system_error) &&
                   query_int_quick_locked("axis0.current_state", &axis_state) &&
                   query_int_quick_locked("axis0.error", &axis_error) &&
                   query_int_quick_locked("axis0.motor.error", &motor_error) &&
                   query_int_quick_locked("axis0.controller.error", &controller_error) &&
                   query_int_quick_locked("axis0.sensorless_estimator.error", &estimator_error);
        blackbox_log("CLEAR snapshot%s state=%d system=%d axis=%d motor=%d",
                     captured ? "" : " INCOMPLETE", axis_state, system_error,
                     axis_error, motor_error);
        sent = set_idle_locked(axis);
        sent = send_locked("w error 0") && sent;
        sent = write_property_locked(axis, "error", 0) && sent;
        sent = write_property_locked(axis, "motor.error", 0) && sent;
        sent = write_property_locked(axis, "encoder.error", 0) && sent;
        sent = write_property_locked(axis, "controller.error", 0) && sent;
        sent = write_property_locked(axis, "sensorless_estimator.error", 0) && sent;
        xSemaphoreGive(io_mutex);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_command, sizeof(state.last_command), "clear axis%d errors", axis);
    snprintf(state.last_action, sizeof(state.last_action), "Axis %d errors cleared", axis);
    if (!sent) snprintf(state.last_error, sizeof(state.last_error), "ODESC clear write failed");
    else state.last_error[0] = '\0';
    xSemaphoreGive(state_mutex);
    blackbox_log("CLEAR M0 %s", sent ? "complete" : "failed");
    return status_response(request);
}

void odesc_link_stop_all(void)
{
    if (!io_mutex || !state_mutex) return;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1200)) == pdTRUE) {
        if (uart_installed) {
            set_idle_locked(0);
        }
        xSemaphoreGive(io_mutex);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.active_axis = -1;
    state.motion_deadline_ms = 0;
    state.command_velocity_turns_s = 0.0f;
    state.controller_requested = false;
    state.controller_owned = false;
    snprintf(state.last_action, sizeof(state.last_action), "ODESC M0 stopped and idle");
    xSemaphoreGive(state_mutex);
}

bool odesc_link_get_power(odesc_power_snapshot_t *snapshot)
{
    if (!snapshot || !state_mutex) return false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshot->connected = state.connected;
    snapshot->voltage_valid = state.connected && !state.voltage_clipped;
    snapshot->voltage_clipped = state.voltage_clipped;
    snapshot->current_valid = state.current_valid;
    snapshot->voltage_v = state.vbus_voltage;
    snapshot->current_a = state.ibus_a;
    snapshot->power_w = state.current_valid ?
                        state.vbus_voltage * state.ibus_a : 0.0f;
    snapshot->voltage_age_ms = state.last_voltage_ms ?
                               now_ms() - state.last_voltage_ms : -1;
    xSemaphoreGive(state_mutex);
    return snapshot->connected && snapshot->voltage_age_ms >= 0;
}

bool odesc_link_get_mower_telemetry(odesc_mower_snapshot_t *snapshot)
{
    if (!snapshot || !state_mutex) return false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshot->connected = state.connected;
    snapshot->voltage_valid = state.connected && !state.voltage_clipped &&
                              state.external_vbus_valid &&
                              !state.external_vbus_fault;
    snapshot->current_valid = state.current_valid;
    snapshot->motion_valid = state.motion_telemetry_valid;
    snapshot->active = state.active_axis == 0 &&
                       state.axis_state == AXIS_STATE_SENSORLESS_CLOSED_LOOP;
    snapshot->faulted = state.system_error || state.axis_error ||
                        state.motor_error || state.controller_error ||
                        state.estimator_error;
    snapshot->telemetry_age_ms = state.last_motion_ms ?
                                 now_ms() - state.last_motion_ms : -1;
    snapshot->voltage_v = state.vbus_voltage;
    snapshot->bus_current_a = state.ibus_a;
    snapshot->bus_power_w = state.current_valid ?
                            state.vbus_voltage * state.ibus_a : 0.0f;
    snapshot->command_turns_s = state.command_velocity_turns_s;
    snapshot->estimated_turns_s = state.sensorless_velocity_turns_s;
    snapshot->estimated_rpm = state.sensorless_rpm;
    snapshot->iq_measured_a = state.iq_measured_a;
    snapshot->iq_setpoint_a = state.iq_setpoint_a;
    snapshot->id_measured_a = state.id_measured_a;
    snapshot->id_setpoint_a = state.id_setpoint_a;
    snapshot->phase_current_magnitude_a =
        hypotf(state.iq_measured_a, state.id_measured_a);
    snapshot->motor_voltage_v = state.motor_voltage_v;
    snapshot->motor_power_w = state.motor_power_w;
    snapshot->fet_temperature_c = state.fet_temperature_c;
    snapshot->axis_state = state.axis_state;
    snapshot->system_error = (uint32_t)state.system_error;
    snapshot->axis_error = (uint32_t)state.axis_error;
    snapshot->motor_error = (uint32_t)state.motor_error;
    snapshot->controller_error = (uint32_t)state.controller_error;
    snapshot->estimator_error = (uint32_t)state.estimator_error;
    xSemaphoreGive(state_mutex);
    return snapshot->connected;
}

void odesc_link_controller_request(bool enabled, float velocity_turns_s)
{
    if (!state_mutex || !isfinite(velocity_turns_s)) return;
    bool deceleration_guard = false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (!enabled) {
        state.controller_rearm_required = false;
    } else if (state.controller_rearm_required) {
        state.controller_requested = false;
        state.controller_deadline_ms = 0;
        xSemaphoreGive(state_mutex);
        return;
    }
    if (enabled && state.voltage_clipped) {
        state.controller_requested = false;
        state.controller_owned = false;
        state.controller_deadline_ms = 0;
        snprintf(state.last_error, sizeof(state.last_error),
                 "Handheld MOWER blocked: VBUS measurement invalid");
        xSemaphoreGive(state_mutex);
        return;
    }
    float minimum = state.sensorless_min_turns_s > 0.0f ?
                    state.sensorless_min_turns_s : 9.1f;
    float maximum = state.velocity_limit_turns_s > minimum ?
                    state.velocity_limit_turns_s : minimum;
    if (maximum > ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S) {
        maximum = ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S;
    }
    if (velocity_turns_s < minimum) velocity_turns_s = minimum;
    if (velocity_turns_s > maximum) velocity_turns_s = maximum;
    if (enabled && state.controller_owned && state.active_axis == 0 &&
        fabsf(velocity_turns_s) + 0.05f <
        fabsf(state.command_velocity_turns_s)) {
        state.controller_velocity_turns_s = velocity_turns_s;
        state.controller_requested = false;
        state.controller_rearm_required = true;
        state.controller_deadline_ms = 0;
        snprintf(state.last_action, sizeof(state.last_action),
                 "MOWER speed reduction guarded; coasting to IDLE");
        deceleration_guard = true;
    } else {
        state.controller_velocity_turns_s = velocity_turns_s;
        state.controller_requested = enabled;
        state.controller_deadline_ms = enabled ? now_ms() + 550 : 0;
    }
    xSemaphoreGive(state_mutex);
    if (deceleration_guard) {
        blackbox_log("GUARD handheld speed reduction; requested %.2f turns/s",
                     velocity_turns_s);
    }
}

bool odesc_link_get_controller(odesc_controller_snapshot_t *snapshot)
{
    if (!snapshot || !state_mutex) return false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshot->connected = state.connected;
    snapshot->voltage_clipped = state.voltage_clipped;
    snapshot->active = state.active_axis == 0 &&
                       state.axis_state == AXIS_STATE_SENSORLESS_CLOSED_LOOP;
    snapshot->controller_owned = state.controller_owned;
    snapshot->current_valid = state.current_valid;
    snapshot->faulted = state.voltage_clipped || state.axis_error ||
                        state.motor_error || state.estimator_error;
    snapshot->target_turns_s = state.controller_velocity_turns_s;
    snapshot->minimum_turns_s = state.sensorless_min_turns_s > 0.0f ?
                                state.sensorless_min_turns_s : 9.1f;
    snapshot->maximum_turns_s = state.velocity_limit_turns_s >
                                snapshot->minimum_turns_s ?
                                state.velocity_limit_turns_s :
                                snapshot->minimum_turns_s;
    if (snapshot->maximum_turns_s > ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S) {
        snapshot->maximum_turns_s = ODESC_ABSOLUTE_VELOCITY_LIMIT_TURNS_S;
    }
    snapshot->estimated_rpm = state.sensorless_rpm;
    snapshot->voltage_v = state.vbus_voltage;
    snapshot->current_a = state.ibus_a;
    snapshot->power_w = state.current_valid ?
                        state.vbus_voltage * state.ibus_a : 0.0f;
    snapshot->axis_state = state.axis_state;
    xSemaphoreGive(state_mutex);
    return snapshot->connected;
}

static esp_err_t stop_all_handler(httpd_req_t *request)
{
    odesc_link_stop_all();
    return status_response(request);
}

static void json_float(char *output, size_t output_size, float value)
{
    if (isfinite(value)) snprintf(output, output_size, "%.4f", value);
    else snprintf(output, output_size, "null");
}

static esp_err_t history_handler(httpd_req_t *request)
{
    odesc_history_record_t *snapshot = malloc(sizeof(history));
    if (!snapshot) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "history snapshot allocation failed");
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    size_t count = history_count;
    size_t oldest = (history_head + ODESC_HISTORY_RECORDS - count) %
                    ODESC_HISTORY_RECORDS;
    for (size_t index = 0; index < count; ++index) {
        snapshot[index] = history[(oldest + index) % ODESC_HISTORY_RECORDS];
    }
    xSemaphoreGive(state_mutex);

    int64_t current_ms = now_ms();
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (httpd_resp_send_chunk(request,
        "{\"window_ms\":60000,\"sample_period_ms\":650,"
        "\"keys\":[\"t_s\",\"rpm\",\"bus_a\",\"bus_w\",\"iq_a\","
        "\"fet_c\",\"motor_w\",\"motor_v\",\"bus_v\"],\"samples\":[",
        HTTPD_RESP_USE_STRLEN) != ESP_OK) {
        free(snapshot);
        return ESP_FAIL;
    }

    bool first = true;
    char line[240];
    for (size_t index = 0; index < count; ++index) {
        odesc_history_record_t *record = &snapshot[index];
        int64_t age_ms = current_ms - record->timestamp_ms;
        if (age_ms < 0 || age_ms > ODESC_HISTORY_WINDOW_MS) continue;
        char bus_current[20], bus_power[20], iq_current[20], fet[20];
        char motor_power[20], motor_voltage[20], bus_voltage[20];
        json_float(bus_current, sizeof(bus_current), record->bus_current_a);
        json_float(bus_power, sizeof(bus_power), record->bus_power_w);
        json_float(iq_current, sizeof(iq_current), record->iq_current_a);
        json_float(fet, sizeof(fet), record->fet_temperature_c);
        json_float(motor_power, sizeof(motor_power), record->motor_power_w);
        json_float(motor_voltage, sizeof(motor_voltage), record->motor_voltage_v);
        json_float(bus_voltage, sizeof(bus_voltage), record->bus_voltage_v);
        int length = snprintf(line, sizeof(line),
            "%s[-%.3f,%.2f,%s,%s,%s,%s,%s,%s,%s]",
            first ? "" : ",", age_ms / 1000.0f, record->speed_rpm,
            bus_current, bus_power, iq_current, fet, motor_power,
            motor_voltage, bus_voltage);
        if (length <= 0 || length >= (int)sizeof(line) ||
            httpd_resp_send_chunk(request, line, length) != ESP_OK) {
            free(snapshot);
            return ESP_FAIL;
        }
        first = false;
    }
    free(snapshot);
    return httpd_resp_send_chunk(request, "]}", 2) == ESP_OK ?
           httpd_resp_send_chunk(request, NULL, 0) : ESP_FAIL;
}

static esp_err_t blackbox_handler(httpd_req_t *request)
{
    odesc_blackbox_record_t *snapshot = malloc(sizeof(blackbox));
    if (!snapshot) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "black box snapshot allocation failed");
    }

    xSemaphoreTake(blackbox_mutex, portMAX_DELAY);
    size_t count = blackbox_count;
    size_t oldest = (blackbox_head + ODESC_BLACKBOX_RECORDS - count) %
                    ODESC_BLACKBOX_RECORDS;
    for (size_t index = 0; index < count; ++index) {
        snapshot[index] = blackbox[(oldest + index) % ODESC_BLACKBOX_RECORDS];
    }
    xSemaphoreGive(blackbox_mutex);

    httpd_resp_set_type(request, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    char line[160];
    for (size_t index = 0; index < count; ++index) {
        int length = snprintf(line, sizeof(line), "[%8lld.%03lld] %s\n",
                              snapshot[index].timestamp_ms / 1000,
                              snapshot[index].timestamp_ms % 1000,
                              snapshot[index].message);
        if (length > 0 && httpd_resp_send_chunk(request, line, length) != ESP_OK) {
            free(snapshot);
            return ESP_FAIL;
        }
    }
    free(snapshot);
    return httpd_resp_send_chunk(request, NULL, 0);
}

static const char page_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>ODESC Control</title><style>"
"*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#18212a;font-family:Arial,sans-serif}header{position:sticky;top:0;z-index:5;background:#17212b;color:white;border-bottom:3px solid #e2a400;padding:8px 12px}header>div,main{max-width:800px;margin:auto}h1{font-size:18px;margin:0 0 3px}nav a{color:white;margin-right:14px;font-size:13px}.link{font-size:12px;color:#ff9189}.up{color:#72dfa7}.liveStrip{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:5px;margin-top:6px;font-size:12px}.liveStrip span{min-width:0;padding:5px 6px;border-left:3px solid #22b89a;background:#ffffff12;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}main{padding:12px}.panel{background:white;border:1px solid #cbd3da;border-radius:8px;padding:12px;margin-bottom:12px}.metrics,.fields,.actions{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.metric{min-width:0;border:1px solid #d4dce3;border-left:4px solid #16806a;padding:9px;border-radius:5px}.metric small{display:block;color:#56636f}.metric strong{font-size:19px;overflow-wrap:anywhere}label{font-size:12px;color:#4d5a65}input,button{min-width:0;width:100%;min-height:46px;margin-top:4px;padding:8px;border:1px solid #aab5bf;border-radius:6px;background:white;font-size:16px}input[type=range]{padding:0;accent-color:#176daf}button{font-weight:700;background:#e7ecf0}.primary{background:#176daf;color:white}.danger{background:#c5251c;color:white}.save{background:#267149;color:white}#negative,#positive{-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;-webkit-tap-highlight-color:transparent;touch-action:none}.note{font-size:12px;color:#53616d;line-height:1.4}.status{min-height:18px;font-size:13px;font-weight:700;color:#267149}.unsaved{color:#a96800}.log{font:12px monospace;white-space:pre-wrap;overflow-wrap:anywhere}.blackbox{max-height:300px;overflow:auto;background:#101820;color:#d7f7e3;padding:9px;border-radius:6px}.charts{display:grid;grid-template-columns:1fr 1fr;gap:8px}.chart{border:1px solid #d4dce3;border-radius:6px;padding:7px}.chart strong{display:block;font-size:13px;margin-bottom:4px}.chart canvas{display:block;width:100%;height:145px;background:#101820;border-radius:4px}h2{font-size:17px;margin:0 0 9px}@media(max-width:560px){.metrics,.fields,.actions,.charts{grid-template-columns:minmax(0,1fr)}.liveStrip{grid-template-columns:1fr 1fr}header{padding-top:6px;padding-bottom:6px}nav{white-space:nowrap;overflow-x:auto}}"
".chartHead{display:flex;justify-content:space-between;gap:8px;font-size:12px;margin-bottom:4px}.chartHead strong{display:inline;margin:0;font-size:13px}.chartHead span{color:#53616d;text-align:right}.chart canvas{touch-action:none;-webkit-user-select:none;user-select:none}.historyPoint{margin:8px 0 0;padding:8px;background:#f3f6f7;border-left:4px solid #16806a;font-size:12px;line-height:1.4}"
"</style></head><body><header><div><h1>ODESC / ODrive Control</h1><div id=link class=link>Detecting GPIO27/47...</div><nav><a href='/'>Motors</a><a href='/steppers'>Steppers</a><a href='/battery'>Battery</a><a href='/mower-logs'>Mower Logs</a><a href='/wifi'>Wi-Fi</a><a href='/update'>Firmware</a></nav><div class=liveStrip><span id=liveRpm>RPM --</span><span id=liveCurrent>A --</span><span id=liveFet>FET --</span><span id=livePower>W --</span></div></div></header><main>"
"<section class=panel><div id=voltageWarning class='status unsaved'></div><div class=metrics><div class=metric><small>Battery bus</small><strong id=vbus>-- V</strong></div><div class=metric><small>Bus current</small><strong id=ibus>-- A</strong></div><div class=metric><small>Bus power</small><strong id=power>-- W</strong></div><div class=metric><small>Sensorless estimate</small><strong id=rpm>-- RPM</strong></div><div class=metric><small>Phase current Iq</small><strong id=iq>-- A</strong></div><div class=metric><small>FET temperature</small><strong id=fet>-- C</strong></div><div class=metric><small>Motor electrical power</small><strong id=motorPower>-- W</strong></div><div class=metric><small>FOC voltage vector</small><strong id=motorVoltage>-- V</strong></div><div class=metric><small>Axis state</small><strong id=axisState>--</strong></div></div></section>"
"<section class=panel><h2>Speed boundaries</h2><div class=metrics><div class=metric><small>170 KV no-load estimate</small><strong id=noLoad>-- RPM</strong></div><div class=metric><small>Recommended 80% headroom</small><strong id=working>-- RPM</strong></div><div class=metric><small>P4 command guard</small><strong id=hardSpeed>7000 RPM</strong></div><div class=metric><small>Current measurement ceiling</small><strong id=currentCeiling>-- A</strong></div></div><p class=note>The 7,000 RPM command guard is fixed; battery sag no longer stops a held command at the moving 80% estimate. ODESC configured limits and all electrical, thermal, watchdog, and sensorless faults remain enforced.</p></section>"
"<section class=panel><h2>Current and speed limits</h2><div class=fields><label>Maximum commanded phase current (A)<input id=currentLimit type=number min=1 step=.5></label><label>Current overshoot fault margin (A)<input id=currentMargin type=number min=2 step=.5></label><label>Maximum speed (turns/s)<input id=speedLimit type=number min=9.1 step=.1></label><label>Acceleration ramp (turns/s^2)<input id=rampLimit type=number min=.5 max=50 step=.5></label></div><div id=limitStatus class=status>Reading ODESC limits...</div><div id=speedHint class=note></div><div class=actions><button class=primary onclick=applyLimits(false)>Apply until restart</button><button class=save onclick=applyLimits(true)>Save to ODESC + restart</button><button onclick=reloadLimits()>Reload saved values</button></div><p class=note>Limits can only change while M0 is IDLE. The overshoot margin changes only the transient fault threshold; it does not raise commanded current. Do not raise or save it merely to mask a recurring CURRENT_LIMIT_VIOLATION; inspect motor phases, loading, current sensing, and power integrity first. The command slider follows the successfully applied speed limit. Persistent save writes ODESC flash and restarts the controller; test runtime values first.</p></section>"
"<section class=panel><h2>M0 sensorless velocity</h2><div class=fields><label>Command speed (turns/s)<input id=velocity type=number min=9.1 max=15 step=.1 value=9.1 oninput=syncCommand(false)></label><label>Command speed<input id=velocitySlider type=range min=9.1 max=15 step=.1 value=9.1 oninput=syncCommand(true)></label><div class=metric><small>Command target</small><strong id=commandRpm>546 RPM</strong></div></div><div class=actions><button class=primary id=negative>Hold negative</button><button class=primary id=positive>Hold positive</button><button class=danger onclick=stopAxis()>STOP + IDLE</button><button onclick=clearErrors()>Clear errors</button></div><p class=note>Hold-to-run uses the configured acceleration ramp, a 2 second P4 command deadman, and a 3 second watchdog inside ODESC. Release commands zero and IDLE. Stop before reversing.</p></section>"
"<section class=panel><h2>Last 60 seconds</h2><div class=charts><div class=chart><div class=chartHead><strong>Speed (RPM)</strong><span id=statRpm>avg -- | peak --</span></div><canvas id=chartRpm></canvas></div><div class=chart><div class=chartHead><strong>Bus current (A)</strong><span id=statCurrent>avg -- | peak --</span></div><canvas id=chartCurrent></canvas></div><div class=chart><div class=chartHead><strong>Bus power (W)</strong><span id=statBusPower>avg -- | peak --</span></div><canvas id=chartBusPower></canvas></div><div class=chart><div class=chartHead><strong>FET temperature (C)</strong><span id=statFet>avg -- | max --</span></div><canvas id=chartFet></canvas></div><div class=chart><div class=chartHead><strong>Motor electrical power (W)</strong><span id=statPower>avg -- | peak --</span></div><canvas id=chartPower></canvas></div><div class=chart><div class=chartHead><strong>FOC voltage vector (V)</strong><span id=statVoltage>avg -- | peak --</span></div><canvas id=chartVoltage></canvas></div></div><div id=historyPoint class=historyPoint>Tap any chart to inspect one synchronized sample.</div></section>"
"<section class=panel><div class=actions><button onclick=api('/api/odrive/refresh')>Refresh telemetry</button><button class=danger onclick=api('/api/odrive/stop_all')>STOP M0</button></div><div id=log class=log>Waiting for ODESC...</div></section>"
"<section class=panel><h2>ODESC diagnostics</h2><div class=actions><button onclick=refreshBlackbox()>Refresh live log</button><button onclick=\"location.href='/api/odrive/blackbox/persistent'\">Download persistent log</button></div><p class=note>The live UART trace is volatile. Fault, reconnect, boot, and ODESC internal diagnostic records are retained in a CRC-protected 256 KiB microSD ring across abrupt power loss.</p><pre id=blackbox class='log blackbox'>Loading...</pre></section></main><script>"
"const e=id=>document.getElementById(id),axis=()=>0,vel=()=>Math.abs(Number(e('velocity').value));let holding=null,keepTimer=null,limitsLoaded=false,limitsDirty=false;"
"function f(v,n=1){return Number(v).toFixed(n)}function shown(v,n,suffix){return v===null||v===undefined||!Number.isFinite(Number(v))?'--':f(v,n)+suffix}function syncCommand(fromSlider){let a=e('velocity'),b=e('velocitySlider');if(fromSlider)a.value=b.value;else b.value=a.value;e('commandRpm').textContent=f(Number(a.value)*60,0)+' RPM'}function markDirty(){limitsDirty=true;e('limitStatus').textContent='Unsaved edits: STOP M0, then Apply until restart';e('limitStatus').className='status unsaved';e('speedHint').textContent='The command slider remains at the last applied limit until Apply succeeds.'}['currentLimit','currentMargin','speedLimit','rampLimit'].forEach(id=>e(id).addEventListener('input',markDirty));"
"function loadLimits(s,force=false){if(!s.config_valid||limitsLoaded&&!force)return;let vmax=Math.min(s.velocity_limit_turns_s,s.absolute_velocity_limit_turns_s);e('currentLimit').value=f(s.current_limit_a,2);e('currentLimit').max=f(s.current_ceiling_a,2);e('currentMargin').value=f(s.current_limit_margin_a,2);e('currentMargin').max=f(s.max_allowed_current_a-s.current_limit_a,2);e('speedLimit').value=f(s.velocity_limit_turns_s,2);e('speedLimit').min=f(s.sensorless_min_turns_s,2);e('speedLimit').max=f(s.absolute_velocity_limit_turns_s,2);e('rampLimit').value=f(s.velocity_ramp_turns_s2,2);e('velocity').min=f(s.sensorless_min_turns_s,2);e('velocitySlider').min=e('velocity').min;e('velocity').max=f(vmax,2);e('velocitySlider').max=e('velocity').max;let c=Math.max(s.sensorless_min_turns_s,Math.min(Number(e('velocity').value),vmax));e('velocity').value=f(c,2);syncCommand(false);limitsLoaded=true;limitsDirty=false;e('limitStatus').textContent='Applied: '+f(s.current_limit_a,1)+' A + '+f(s.current_limit_margin_a,1)+' A transient margin, '+f(s.velocity_limit_turns_s,1)+' turns/s';e('speedHint').textContent=s.voltage_clipped?'Motion locked because VBUS measurement is clipped. Speed limits are shown for diagnostics only.':'Command range: '+f(s.sensorless_min_turns_s,2)+' to '+f(vmax,2)+' turns/s ('+f(vmax*60,0)+' RPM). 80% voltage headroom is informational: '+f(s.voltage_control_turns_s*60,0)+' RPM.';e('limitStatus').className='status'}"
"function faultText(s){let x=[];if(s.voltage_clipped)x.push('VBUS_ADC_CLIPPED');if(s.axis_error&64)x.push('MOTOR_FAILED');if(s.axis_error&2048)x.push('WATCHDOG_EXPIRED');if(s.motor_error&8)x.push('DRV_FAULT');if(s.motor_error&1024)x.push('CURRENT_SENSE_SATURATION');if(s.motor_error&4096)x.push('CURRENT_LIMIT_VIOLATION');return x.length?' ['+x.join(', ')+']':''}"
"function render(s){let l=e('link');l.textContent=s.connected?'ODESC linked | TX GPIO'+s.tx+' RX GPIO'+s.rx:(s.detecting?'Detecting GPIO27/47...':'ODESC not detected');l.className='link '+(s.connected?'up':'');e('voltageWarning').textContent=s.voltage_clipped?'CRITICAL: VBUS sense is clipped. Actual battery voltage is above the measurable range; all ODESC motion is locked.':'';e('vbus').textContent=s.voltage_clipped?'>= '+f(s.vbus_voltage,2)+' V (clipped)':(s.connected?f(s.vbus_voltage,2)+' V':'-- V');e('ibus').textContent=s.current_valid?f(s.ibus_a,2)+' A':'-- A';e('power').textContent=s.current_valid&&!s.voltage_clipped?f(s.bus_power_w,1)+' W':'-- W';e('rpm').textContent=s.motion_valid?f(s.sensorless_rpm,0)+' RPM':'-- RPM';e('iq').textContent=s.motion_valid?f(s.iq_measured_a,2)+' A':'-- A';e('fet').textContent=shown(s.fet_temperature_c,1,' C');e('motorPower').textContent=shown(s.motor_power_w,1,' W');e('motorVoltage').textContent=shown(s.motor_voltage_v,1,' V');e('axisState').textContent=s.motion_valid?String(s.axis_state):'--';e('noLoad').textContent=s.connected&&!s.voltage_clipped?f(s.no_load_rpm,0)+' RPM':'-- RPM';e('working').textContent=s.connected&&!s.voltage_clipped?f(s.no_load_rpm*.8,0)+' RPM':'-- RPM';e('hardSpeed').textContent=f(s.absolute_velocity_limit_rpm,0)+' RPM';e('currentCeiling').textContent=s.config_valid?f(s.current_ceiling_a,1)+' A':'-- A';e('liveRpm').textContent=s.motion_valid?'RPM '+f(s.sensorless_rpm,0):'RPM --';e('liveCurrent').textContent=s.current_valid?'A '+f(s.ibus_a,1):'A --';e('liveFet').textContent='FET '+shown(s.fet_temperature_c,0,' C');e('livePower').textContent='W '+(s.current_valid?f(s.bus_power_w,0):'--');loadLimits(s);e('log').textContent='estimated '+f(s.sensorless_velocity_turns_s,3)+' turns/s | command '+f(s.command_velocity_turns_s,3)+' | sensorless minimum '+f(s.sensorless_min_turns_s,3)+'\\naxis/motor/estimator errors: '+s.axis_error+'/'+s.motor_error+'/'+s.estimator_error+faultText(s)+'\\ninput mode '+s.input_mode+' (2 = velocity ramp) | voltage age '+s.voltage_age_ms+' ms\\nqueries '+s.queries+' | reconnects '+s.reconnects+' | active axis '+s.active_axis+'\\ncommand: '+(s.last_command||'none')+'\\naction: '+(s.last_action||'none')+'\\nreply: '+(s.last_reply||'none')+'\\nerror: '+(s.last_error||'none')}"
"function request(path){return fetch(path,{cache:'no-store'}).then(async r=>{let t=await r.text(),j;try{j=JSON.parse(t)}catch(_){j=null}if(!r.ok)throw Error(j&&j.last_error?j.last_error:t||r.status);return j})}function api(path){return request(path).then(render).catch(x=>e('log').textContent='Request failed: '+x.message)}function applyLimits(persist){if(holding)return;e('limitStatus').textContent=persist?'Saving; ODESC will restart...':'Applying runtime limits...';let p='/api/odrive/limits?current='+encodeURIComponent(e('currentLimit').value)+'&margin='+encodeURIComponent(e('currentMargin').value)+'&velocity='+encodeURIComponent(e('speedLimit').value)+'&ramp='+encodeURIComponent(e('rampLimit').value)+'&persist='+(persist?1:0);request(p).then(s=>{render(s);if(!persist)loadLimits(s,true);else{limitsDirty=false;e('limitStatus').textContent='Saved; waiting for ODESC reconnect';e('limitStatus').className='status'}}).catch(x=>{e('limitStatus').textContent=x.message;e('limitStatus').className='status unsaved'})}function reloadLimits(){request('/api/odrive/config').then(s=>{render(s);loadLimits(s,true)}).catch(x=>e('limitStatus').textContent=x.message)}"
"function push(d){if(holding!==d)return;request('/api/odrive/velocity?axis=0&velocity='+(vel()*d)).then(s=>{render(s);if(holding===d)keepTimer=setTimeout(()=>push(d),260)}).catch(x=>{holding=null;e('log').textContent='Drive failed: '+x.message;stopAxis()})}function begin(d){if(holding!==null||!Number.isFinite(vel()))return;holding=d;request('/api/odrive/sensorless/start?axis=0&velocity='+(vel()*d)).then(s=>{render(s);if(holding===d)keepTimer=setTimeout(()=>push(d),260)}).catch(x=>{holding=null;e('log').textContent='Start failed: '+x.message})}function stopAxis(){holding=null;if(keepTimer){clearTimeout(keepTimer);keepTimer=null}api('/api/odrive/stop?axis='+axis())}function clearErrors(){api('/api/odrive/clear?axis='+axis())}"
"function refreshBlackbox(){fetch('/api/odrive/blackbox',{cache:'no-store'}).then(r=>r.text()).then(t=>{let b=e('blackbox'),atBottom=b.scrollHeight-b.scrollTop-b.clientHeight<24;b.textContent=t||'No ODESC events yet.';if(atBottom)b.scrollTop=b.scrollHeight}).catch(x=>e('blackbox').textContent='Log unavailable: '+x.message)}"
"let historyBusy=false,historySamples=[],historySelected=null;function historyStat(id,p,index,unit,absolute){let v=p.map(x=>Number(x[index])).filter(Number.isFinite);if(absolute)v=v.map(Math.abs);e(id).textContent=v.length?'avg '+f(v.reduce((a,b)=>a+b,0)/v.length,1)+' '+unit+' | peak '+f(Math.max(...v),1)+' '+unit:'avg -- | peak --'}"
"function drawChart(id,samples,index,color,stat,unit,absolute=true){let c=e(id),r=c.getBoundingClientRect(),d=Math.max(1,window.devicePixelRatio||1),w=Math.max(240,Math.round(r.width*d)),h=Math.max(120,Math.round(r.height*d));if(c.width!==w)c.width=w;if(c.height!==h)c.height=h;let g=c.getContext('2d');g.clearRect(0,0,w,h);g.strokeStyle='#52606b';g.lineWidth=d;for(let i=1;i<4;i++){let y=h*i/4;g.beginPath();g.moveTo(0,y);g.lineTo(w,y);g.stroke()}let p=samples.filter(x=>x[index]!==null&&Number.isFinite(Number(x[index])));historyStat(stat,p,index,unit,absolute);if(!p.length)return;let lo=Math.min(...p.map(x=>Number(x[index]))),hi=Math.max(...p.map(x=>Number(x[index])));if(hi-lo<.001){hi+=.5;lo-=.5}let t0=Math.min(...p.map(x=>Number(x[0]))),t1=Math.max(...p.map(x=>Number(x[0])));if(t1-t0<.001)t1=t0+1;let px=t=>(Number(t)-t0)/(t1-t0)*w,py=v=>h-(Number(v)-lo)/(hi-lo)*h;g.strokeStyle=color;g.lineWidth=2*d;g.beginPath();p.forEach((x,i)=>{i?g.lineTo(px(x[0]),py(x[index])):g.moveTo(px(x[0]),py(x[index]))});g.stroke();if(historySelected&&historySelected[0]>=t0&&historySelected[0]<=t1){let sx=px(historySelected[0]);g.strokeStyle='#ffffff';g.lineWidth=d;g.beginPath();g.moveTo(sx,0);g.lineTo(sx,h);g.stroke();if(Number.isFinite(Number(historySelected[index]))){g.fillStyle='#ffffff';g.beginPath();g.arc(sx,py(historySelected[index]),4*d,0,Math.PI*2);g.fill()}}g.fillStyle='#dce7ee';g.font=(11*d)+'px sans-serif';g.fillText(hi.toFixed(1),5,13*d);g.fillText(lo.toFixed(1),5,h-5*d)}"
"function drawHistory(){let s=historySamples;drawChart('chartRpm',s,1,'#30a8ff','statRpm','RPM');drawChart('chartCurrent',s,2,'#f0b429','statCurrent','A');drawChart('chartBusPower',s,3,'#ef7d32','statBusPower','W');drawChart('chartFet',s,5,'#ff6b5e','statFet','C',false);drawChart('chartPower',s,6,'#72dfa7','statPower','W');drawChart('chartVoltage',s,7,'#c899ff','statVoltage','V')}function showHistoryPoint(p){historySelected=p;e('historyPoint').textContent='Selected '+f(Math.abs(p[0]),1)+' s ago | '+shown(p[1],0,' RPM')+' | bus '+shown(p[8],2,' V')+', '+shown(p[2],2,' A')+', '+shown(p[3],1,' W')+' | Iq '+shown(p[4],2,' A')+' | FET '+shown(p[5],1,' C')+' | motor '+shown(p[6],1,' W')+', '+shown(p[7],1,' V');drawHistory()}function selectHistory(ev){if(!historySamples.length)return;ev.preventDefault();let r=ev.currentTarget.getBoundingClientRect(),q=Math.max(0,Math.min(1,(ev.clientX-r.left)/r.width)),t0=Number(historySamples[0][0]),t1=Number(historySamples[historySamples.length-1][0]),target=t0+q*(t1-t0),best=historySamples[0];historySamples.forEach(x=>{if(Math.abs(Number(x[0])-target)<Math.abs(Number(best[0])-target))best=x});showHistoryPoint(best)}"
"function refreshHistory(){if(historyBusy)return;historyBusy=true;request('/api/odrive/history').then(h=>{historySamples=h.samples||[];drawHistory()}).catch(()=>{}).finally(()=>historyBusy=false)}['chartRpm','chartCurrent','chartBusPower','chartFet','chartPower','chartVoltage'].forEach(id=>e(id).addEventListener('pointerdown',selectHistory));"
"function bind(id,d){let b=e(id);let block=x=>x.preventDefault();let start=x=>{x.preventDefault();b.setPointerCapture(x.pointerId);begin(d)};let stop=x=>{x.preventDefault();if(holding!==null)stopAxis()};b.addEventListener('contextmenu',block);b.addEventListener('selectstart',block);b.addEventListener('dragstart',block);b.addEventListener('pointerdown',start);b.addEventListener('pointerup',stop);b.addEventListener('pointercancel',stop);b.addEventListener('lostpointercapture',stop)}bind('negative',-1);bind('positive',1);setInterval(()=>api('/api/odrive/status'),1000);setInterval(refreshHistory,1000);setInterval(refreshBlackbox,5000);api('/api/odrive/config');refreshHistory();refreshBlackbox();syncCommand(false);document.addEventListener('visibilitychange',()=>{if(document.hidden&&holding)stopAxis()});"
"</script></body></html>";

static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, page_html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t odesc_link_start(void)
{
    blackbox_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();
    io_mutex = xSemaphoreCreateMutex();
    if (!blackbox_mutex || !state_mutex || !io_mutex) return ESP_ERR_NO_MEM;
    esp_err_t persistent_result = odesc_persistent_log_start();
    if (persistent_result != ESP_OK) return persistent_result;
    blackbox_log("BOOT ODESC logger initialized (%d-record RAM ring)",
                 ODESC_BLACKBOX_RECORDS);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.detecting = true;
    state.last_error[0] = '\0';
    snprintf(state.last_action, sizeof(state.last_action),
             "Searching for ODESC: TX GPIO27 RX GPIO47");
    xSemaphoreGive(state_mutex);
    return xTaskCreate(service_task, "odesc_link", 4096, NULL, 5, NULL) == pdPASS ?
           ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t odesc_link_register_routes(httpd_handle_t server)
{
    const httpd_uri_t routes[] = {
        {.uri = "/odrive", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/odrive/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/odrive/refresh", .method = HTTP_GET, .handler = refresh_handler},
        {.uri = "/api/odrive/config", .method = HTTP_GET, .handler = config_handler},
        {.uri = "/api/odrive/limits", .method = HTTP_GET, .handler = limits_handler},
        {.uri = "/api/odrive/regen", .method = HTTP_GET,
         .handler = regen_limit_handler},
        {.uri = "/api/odrive/state", .method = HTTP_GET, .handler = state_handler},
        {.uri = "/api/odrive/sensorless/start", .method = HTTP_GET,
         .handler = sensorless_start_handler},
        {.uri = "/api/odrive/velocity", .method = HTTP_GET, .handler = velocity_handler},
        {.uri = "/api/odrive/stop", .method = HTTP_GET, .handler = stop_handler},
        {.uri = "/api/odrive/stop_all", .method = HTTP_GET, .handler = stop_all_handler},
        {.uri = "/api/odrive/clear", .method = HTTP_GET, .handler = clear_handler},
        {.uri = "/api/odrive/history", .method = HTTP_GET,
         .handler = history_handler},
        {.uri = "/api/odrive/blackbox", .method = HTTP_GET,
         .handler = blackbox_handler},
        {.uri = "/api/odrive/blackbox/persistent", .method = HTTP_GET,
         .handler = odesc_persistent_log_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        esp_err_t result = httpd_register_uri_handler(server, &routes[i]);
        if (result != ESP_OK) return result;
    }
    return ESP_OK;
}
