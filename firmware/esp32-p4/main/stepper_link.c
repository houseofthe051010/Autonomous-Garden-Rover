#include "stepper_link.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define GD32_UART UART_NUM_2
#define GD32_BAUD 115200
#define GD32_WIRE_A_GPIO 1
#define GD32_WIRE_B_GPIO 2
#define DETECT_WINDOW_MS 4500
#define LINK_TIMEOUT_MS 8000
#define LINE_CAPACITY 192
#define MAX_PENDING_MOVES 8
#define MAX_RPM 600
#define DIRECT_UPDATE_MS 17
#define DIRECT_LEASE_DEFAULT_MS 400
#define DIRECT_LEASE_MIN_MS 150
#define DIRECT_LEASE_MAX_MS 1000
#define TX_QUEUE_DEPTH 20
#define TX_PAYLOAD_CAPACITY 256

static const char *TAG = "gd32_stepper";
static const char AXIS_NAMES[] = {'X', 'Y', 'Z'};
static const double STEPS_PER_UNIT[] = {80.0, 80.0, 400.0};

typedef struct {
    int64_t position;
    int64_t target;
    int64_t minimum;
    int64_t maximum;
    bool known;
    bool limits_set;
} axis_state_t;

typedef struct {
    bool active;
    uint32_t id;
    uint8_t axis;
    int32_t steps;
} pending_move_t;

typedef struct {
    char payload[TX_PAYLOAD_CAPACITY];
} tx_item_t;

typedef struct {
    bool receiving;
    bool round_trip;
    int tx_gpio;
    int rx_gpio;
    int64_t last_heartbeat_ms;
    int64_t last_round_trip_ms;
    int64_t orientation_started_ms;
    unsigned reconnects;
    unsigned heartbeat_count;
    unsigned ack_count;
    bool drivers_enabled;
    bool switch_known[3];
    bool switches[3];
    axis_state_t axes[3];
    pending_move_t pending[MAX_PENDING_MOVES];
    uint32_t move_sequence;
    bool direct_capable;
    bool direct_probe_sent;
    bool direct_active;
    int direct_mode;
    float direct_target_rpm[3];
    float direct_actual_rpm[3];
    int64_t direct_lease_deadline_ms;
    uint32_t direct_sequence;
    uint32_t direct_last_ack;
    unsigned direct_ack_count;
    bool count_active;
    uint32_t count_id;
    int32_t count_steps[3];
    char last_command[96];
    char last_action[128];
    char last_line[128];
    char last_error[128];
} stepper_state_t;

static stepper_state_t state = {
    .tx_gpio = -1,
    .rx_gpio = -1,
};
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t write_mutex;
static QueueHandle_t tx_queue;
static bool uart_installed;
static char line_buffer[LINE_CAPACITY];
static size_t line_length;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static int axis_index(char axis)
{
    for (int i = 0; i < 3; ++i) {
        if (AXIS_NAMES[i] == axis) return i;
    }
    return -1;
}

static void uart_close(void)
{
    xSemaphoreTake(write_mutex, portMAX_DELAY);
    if (uart_installed) {
        uart_driver_delete(GD32_UART);
        uart_installed = false;
    }
    gpio_reset_pin(GD32_WIRE_A_GPIO);
    gpio_reset_pin(GD32_WIRE_B_GPIO);
    gpio_set_direction(GD32_WIRE_A_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(GD32_WIRE_B_GPIO, GPIO_MODE_INPUT);
    line_length = 0;
    xSemaphoreGive(write_mutex);
}

static bool uart_open_listen(int rx_gpio)
{
    uart_close();
    const uart_config_t config = {
        .baud_rate = GD32_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    xSemaphoreTake(write_mutex, portMAX_DELAY);
    esp_err_t result = uart_driver_install(GD32_UART, 2048, 0, 0, NULL, 0);
    if (result == ESP_OK) result = uart_param_config(GD32_UART, &config);
    if (result == ESP_OK) {
        result = uart_set_pin(GD32_UART, UART_PIN_NO_CHANGE, rx_gpio,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    uart_installed = result == ESP_OK;
    if (uart_installed) {
        gpio_set_pull_mode(rx_gpio, GPIO_PULLUP_ONLY);
        uart_flush_input(GD32_UART);
    }
    xSemaphoreGive(write_mutex);
    return result == ESP_OK;
}

static bool read_line(char *output, size_t output_size, uint32_t timeout_ms)
{
    int64_t deadline = now_ms() + timeout_ms;
    uint8_t byte;
    while (now_ms() < deadline && uart_installed) {
        int remaining = (int)(deadline - now_ms());
        TickType_t wait = pdMS_TO_TICKS(remaining > 20 ? 20 : remaining);
        if (uart_read_bytes(GD32_UART, &byte, 1, wait) != 1) continue;
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

static bool send_payload_owner(const char *payload)
{
    bool sent = false;
    if (xSemaphoreTake(write_mutex, pdMS_TO_TICKS(500)) != pdTRUE) return false;
    if (uart_installed) {
        int length = (int)strlen(payload);
        sent = uart_write_bytes(GD32_UART, payload, length) == length;
        if (sent) uart_wait_tx_done(GD32_UART, pdMS_TO_TICKS(200));
    }
    xSemaphoreGive(write_mutex);
    return sent;
}

static bool send_gcode_owner(const char *command)
{
    char payload[160];
    int length = snprintf(payload, sizeof(payload), "%s\n", command);
    if (length <= 0 || length >= (int)sizeof(payload)) return false;
    ESP_LOGI(TAG, "TX: %s", command);
    return send_payload_owner(payload);
}

static bool queue_payload(const char *payload, bool urgent)
{
    if (!tx_queue || !payload || strlen(payload) >= TX_PAYLOAD_CAPACITY) return false;
    tx_item_t item = {0};
    snprintf(item.payload, sizeof(item.payload), "%s", payload);
    BaseType_t queued = urgent ? xQueueSendToFront(tx_queue, &item, 0)
                               : xQueueSend(tx_queue, &item, 0);
    return queued == pdTRUE;
}

static bool queue_gcode(const char *command, bool urgent)
{
    char payload[TX_PAYLOAD_CAPACITY];
    int length = snprintf(payload, sizeof(payload), "%s\n", command);
    if (length <= 0 || length >= (int)sizeof(payload)) return false;
    ESP_LOGI(TAG, "QUEUE: %s", command);
    return queue_payload(payload, urgent);
}

static unsigned pending_count_locked(void)
{
    unsigned count = 0;
    for (int i = 0; i < MAX_PENDING_MOVES; ++i) count += state.pending[i].active;
    return count;
}

static bool axis_pending_locked(int axis)
{
    for (int i = 0; i < MAX_PENDING_MOVES; ++i) {
        if (state.pending[i].active && state.pending[i].axis == axis) return true;
    }
    return false;
}

static void process_line(const char *line)
{
    if (strncmp(line, "VEL_ACK I", 9) != 0 && strcmp(line, "ok") != 0) {
        ESP_LOGI(TAG, "RX: %s", line);
    }
    int64_t now = now_ms();

    if (strncmp(line, "HB ", 3) == 0) {
        const char *sequence = line + 3;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.receiving = true;
        state.last_heartbeat_ms = now;
        state.heartbeat_count++;
        snprintf(state.last_line, sizeof(state.last_line), "%s", line);
        xSemaphoreGive(state_mutex);
        char ack[96];
        snprintf(ack, sizeof(ack), "M118 HB_ACK %s", sequence);
        queue_gcode(ack, true);
        return;
    }

    bool probe_direct_mode = false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_line, sizeof(state.last_line), "%s", line);
    if (strncmp(line, "HB_ACK_OK", 9) == 0) {
        state.round_trip = true;
        state.last_round_trip_ms = now;
        state.ack_count++;
        state.last_error[0] = '\0';
        if (!state.direct_probe_sent) {
            state.direct_probe_sent = true;
            probe_direct_mode = true;
        }
    } else if (strncmp(line, "SW ", 3) == 0) {
        unsigned x, y, z;
        if (sscanf(line, "SW X%u Y%u Z%u", &x, &y, &z) == 3 &&
            x <= 1 && y <= 1 && z <= 1) {
            state.switches[0] = x;
            state.switches[1] = y;
            state.switches[2] = z;
            state.switch_known[0] = state.switch_known[1] = state.switch_known[2] = true;
        } else {
            snprintf(state.last_error, sizeof(state.last_error), "Malformed switch report");
        }
    } else if (strncmp(line, "DRV_DONE ", 9) == 0) {
        unsigned id;
        char axis_name;
        int steps;
        if (sscanf(line, "DRV_DONE %u %c %d", &id, &axis_name, &steps) == 3) {
            int axis = axis_index(axis_name);
            pending_move_t *match = NULL;
            for (int i = 0; i < MAX_PENDING_MOVES; ++i) {
                if (state.pending[i].active && state.pending[i].id == id) {
                    match = &state.pending[i];
                    break;
                }
            }
            if (match && axis >= 0 && match->axis == axis && match->steps == steps) {
                state.axes[axis].position += steps;
                match->active = false;
                snprintf(state.last_action, sizeof(state.last_action),
                         "%c move %u complete: %+d steps", axis_name, id, steps);
            } else {
                snprintf(state.last_error, sizeof(state.last_error),
                         "Unexpected DRV_DONE %u", id);
            }
        } else {
            snprintf(state.last_error, sizeof(state.last_error), "Malformed DRV_DONE");
        }
    } else if (strncmp(line, "DIRECT_STATUS ", 14) == 0) {
        int mode = 0, armed = 0;
        unsigned id = 0;
        float x = 0, y = 0, z = 0, e = 0;
        if (sscanf(line, "DIRECT_STATUS M%d I%u X%f Y%f Z%f E%f A%d",
                   &mode, &id, &x, &y, &z, &e, &armed) == 7) {
            state.direct_capable = true;
            state.direct_mode = mode;
            state.direct_actual_rpm[0] = x;
            state.direct_actual_rpm[1] = y;
            state.direct_actual_rpm[2] = z;
            state.last_error[0] = '\0';
            snprintf(state.last_action, sizeof(state.last_action),
                     "GD32 direct-motion firmware detected");
        } else {
            snprintf(state.last_error, sizeof(state.last_error),
                     "Malformed DIRECT_STATUS");
        }
    } else if (strncmp(line, "VEL_ACK I", 9) == 0) {
        unsigned id = 0;
        if (sscanf(line, "VEL_ACK I%u", &id) == 1) {
            state.direct_last_ack = id;
            state.direct_ack_count++;
            state.direct_capable = true;
        }
    } else if (strncmp(line, "VEL_TIMEOUT I", 13) == 0) {
        state.direct_active = false;
        state.direct_mode = 0;
        memset(state.direct_target_rpm, 0, sizeof(state.direct_target_rpm));
        snprintf(state.last_error, sizeof(state.last_error),
                 "GD32 direct-motion deadman timeout");
    } else if (strncmp(line, "DIRECT_STOPPED", 14) == 0) {
        state.direct_active = false;
        state.direct_mode = 0;
        memset(state.direct_target_rpm, 0, sizeof(state.direct_target_rpm));
        memset(state.direct_actual_rpm, 0, sizeof(state.direct_actual_rpm));
        snprintf(state.last_action, sizeof(state.last_action), "Direct motion stopped");
    } else if (strncmp(line, "COUNT_ACK I", 11) == 0) {
        unsigned id = 0;
        if (sscanf(line, "COUNT_ACK I%u", &id) == 1 && id == state.count_id) {
            state.count_active = true;
            state.direct_mode = 2;
            snprintf(state.last_action, sizeof(state.last_action),
                     "Simultaneous move %u accepted", id);
        }
    } else if (strncmp(line, "COUNT_DONE I", 12) == 0) {
        unsigned id = 0;
        int x = 0, y = 0, z = 0, e = 0;
        if (sscanf(line, "COUNT_DONE I%u X%d Y%d Z%d E%d",
                   &id, &x, &y, &z, &e) == 5 &&
            state.count_active && id == state.count_id) {
            const int completed[3] = {x, y, z};
            for (int i = 0; i < 3; ++i) {
                if (completed[i] == state.count_steps[i]) {
                    state.axes[i].position += completed[i];
                    state.axes[i].target = state.axes[i].position;
                } else {
                    state.axes[i].known = false;
                }
            }
            state.count_active = false;
            state.direct_mode = 0;
            snprintf(state.last_action, sizeof(state.last_action),
                     "Simultaneous move %u complete", id);
        } else {
            snprintf(state.last_error, sizeof(state.last_error),
                     "Unexpected COUNT_DONE");
        }
    } else if (strncmp(line, "COUNT_ERR I", 11) == 0) {
        if (state.count_active) {
            for (int i = 0; i < 3; ++i) state.axes[i].target -= state.count_steps[i];
        }
        state.count_active = false;
        state.direct_mode = 0;
        snprintf(state.last_error, sizeof(state.last_error), "GD32 rejected: %.100s", line);
    } else if (strncmp(line, "VEL_ERR I", 9) == 0) {
        state.direct_active = false;
        state.direct_mode = 0;
        memset(state.direct_target_rpm, 0, sizeof(state.direct_target_rpm));
        snprintf(state.last_error, sizeof(state.last_error), "GD32 rejected: %.100s", line);
    } else if (strstr(line, "Unknown command") && strstr(line, "M974")) {
        state.direct_capable = false;
        snprintf(state.last_error, sizeof(state.last_error),
                 "GD32 direct-motion firmware update required");
    }
    xSemaphoreGive(state_mutex);
    if (probe_direct_mode) queue_gcode("M974", false);
}

static bool detect_orientation(void)
{
    char line[LINE_CAPACITY];
    const int rx_candidates[] = {GD32_WIRE_B_GPIO, GD32_WIRE_A_GPIO};
    for (size_t candidate = 0;
         candidate < sizeof(rx_candidates) / sizeof(rx_candidates[0]);
         ++candidate) {
        int rx_gpio = rx_candidates[candidate];
        int tx_gpio = rx_gpio == GD32_WIRE_A_GPIO ?
                      GD32_WIRE_B_GPIO : GD32_WIRE_A_GPIO;
        ESP_LOGI(TAG,
                 "Passive GD32 listen on RX GPIO%d; GPIO%d remains input",
                 rx_gpio, tx_gpio);
        if (!uart_open_listen(rx_gpio)) continue;
        int64_t deadline = now_ms() + DETECT_WINDOW_MS;
        while (now_ms() < deadline) {
            if (!read_line(line, sizeof(line), 100)) continue;
            ESP_LOGI(TAG, "GD32 candidate RX GPIO%d: %s", rx_gpio, line);
            if (strncmp(line, "HB ", 3) != 0) continue;
            xSemaphoreTake(write_mutex, portMAX_DELAY);
            esp_err_t result = uart_set_pin(GD32_UART, tx_gpio, rx_gpio,
                                            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            xSemaphoreGive(write_mutex);
            if (result != ESP_OK) break;
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.tx_gpio = tx_gpio;
            state.rx_gpio = rx_gpio;
            state.receiving = true;
            state.round_trip = false;
            state.direct_probe_sent = false;
            state.direct_capable = false;
            state.last_heartbeat_ms = now_ms();
            state.orientation_started_ms = now_ms();
            state.reconnects++;
            xSemaphoreGive(state_mutex);
            process_line(line);
            ESP_LOGI(TAG, "GD32 UART detected: TX GPIO%d RX GPIO%d",
                     tx_gpio, rx_gpio);
            return true;
        }
        uart_close();
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.receiving = false;
    state.round_trip = false;
    state.tx_gpio = -1;
    state.rx_gpio = -1;
    snprintf(state.last_error, sizeof(state.last_error),
             "No GD32 heartbeat on passive GPIO1/GPIO2 inputs");
    xSemaphoreGive(state_mutex);
    return false;
}

static void service_task(void *argument)
{
    (void)argument;
    char line[LINE_CAPACITY];
    int64_t next_direct_update_ms = 0;
    int64_t next_direct_status_ms = 0;
    while (true) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool connected = state.tx_gpio >= 0 && state.rx_gpio >= 0;
        xSemaphoreGive(state_mutex);
        if (!connected) {
            if (!detect_orientation()) vTaskDelay(pdMS_TO_TICKS(750));
            continue;
        }
        if (read_line(line, sizeof(line), 2)) process_line(line);

        tx_item_t queued;
        for (int i = 0; i < 4 && xQueueReceive(tx_queue, &queued, 0) == pdTRUE; ++i) {
            if (!send_payload_owner(queued.payload)) {
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                snprintf(state.last_error, sizeof(state.last_error),
                         "GD32 UART write failed");
                xSemaphoreGive(state_mutex);
                break;
            }
        }

        int64_t now = now_ms();
        bool send_direct = false;
        bool direct_expired = false;
        float rpm[3] = {0};
        uint32_t direct_id = 0;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (state.direct_active && now >= state.direct_lease_deadline_ms) {
            state.direct_active = false;
            state.direct_mode = 0;
            memset(state.direct_target_rpm, 0, sizeof(state.direct_target_rpm));
            snprintf(state.last_action, sizeof(state.last_action),
                     "Direct-motion browser lease expired; stopped");
            snprintf(state.last_error, sizeof(state.last_error),
                     "Direct motion stopped because command refresh ended");
            direct_expired = true;
        } else if (state.direct_active && now >= next_direct_update_ms) {
            memcpy(rpm, state.direct_target_rpm, sizeof(rpm));
            direct_id = ++state.direct_sequence;
            send_direct = true;
            next_direct_update_ms = now + DIRECT_UPDATE_MS;
        }
        xSemaphoreGive(state_mutex);
        if (direct_expired) {
            send_gcode_owner("M970 I0 X0 Y0 Z0");
            send_gcode_owner("M975");
        } else if (send_direct) {
            char payload[128];
            snprintf(payload, sizeof(payload),
                     "M970 I%" PRIu32 " X%.3f Y%.3f Z%.3f\n",
                     direct_id, rpm[0], rpm[1], rpm[2]);
            send_payload_owner(payload);
            if (now >= next_direct_status_ms) {
                send_gcode_owner("M974");
                next_direct_status_ms = now + 500;
            }
        }

        // A received line may refresh the heartbeat or round-trip timestamp.
        // Snapshot after processing so a just-arrived ACK cannot be timed out.
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool round_trip = state.round_trip;
        int64_t last_heartbeat = state.last_heartbeat_ms;
        int64_t last_round_trip = state.last_round_trip_ms;
        int64_t orientation_started = state.orientation_started_ms;
        xSemaphoreGive(state_mutex);
        bool initial_round_trip_failed = !round_trip && orientation_started &&
                                         now_ms() - orientation_started > LINK_TIMEOUT_MS;
        bool heartbeat_failed = last_heartbeat &&
                                now_ms() - last_heartbeat > LINK_TIMEOUT_MS;
        bool established_link_failed = round_trip && last_round_trip &&
                                       now_ms() - last_round_trip > LINK_TIMEOUT_MS;
        if (initial_round_trip_failed || heartbeat_failed || established_link_failed) {
            ESP_LOGW(TAG, "GD32 round trip timed out; restarting passive detection");
            uart_close();
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_PENDING_MOVES; ++i) {
                if (state.pending[i].active) {
                    int axis = state.pending[i].axis;
                    state.axes[axis].target = state.axes[axis].position;
                    state.axes[axis].known = false;
                    state.pending[i].active = false;
                }
            }
            if (state.count_active) {
                for (int i = 0; i < 3; ++i) {
                    if (state.count_steps[i]) {
                        state.axes[i].target = state.axes[i].position;
                        state.axes[i].known = false;
                    }
                }
            }
            state.receiving = false;
            state.round_trip = false;
            state.drivers_enabled = false;
            state.direct_capable = false;
            state.direct_probe_sent = false;
            state.direct_active = false;
            state.direct_mode = 0;
            state.count_active = false;
            memset(state.direct_target_rpm, 0, sizeof(state.direct_target_rpm));
            state.tx_gpio = -1;
            state.rx_gpio = -1;
            snprintf(state.last_error, sizeof(state.last_error), "GD32 link timed out");
            xSemaphoreGive(state_mutex);
        }
    }
}

static bool query_value(httpd_req_t *request, const char *key,
                        char *output, size_t output_size)
{
    char query[256];
    return httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK &&
           httpd_query_key_value(query, key, output, output_size) == ESP_OK;
}

static esp_err_t status_response(httpd_req_t *request)
{
    stepper_state_t snapshot;
    char body[2300];
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshot = state;
    unsigned pending = pending_count_locked();
    xSemaphoreGive(state_mutex);
    int64_t now = now_ms();
    int used = snprintf(body, sizeof(body),
        "{\"alive\":%s,\"receiving\":%s,\"tx\":%d,\"rx\":%d,"
        "\"reconnects\":%u,\"heartbeat_count\":%u,\"ack_count\":%u,"
        "\"heartbeat_age_ms\":%" PRId64 ",\"ack_age_ms\":%" PRId64 ","
        "\"drivers_enabled\":%s,\"pending\":%u,\"switches\":[%d,%d,%d],"
        "\"direct_capable\":%s,\"direct_active\":%s,\"direct_mode\":%d,"
        "\"direct_target\":[%.3f,%.3f,%.3f],\"direct_actual\":[%.3f,%.3f,%.3f],"
        "\"direct_ack_count\":%u,\"direct_last_ack\":%" PRIu32 ","
        "\"count_active\":%s,\"count_id\":%" PRIu32 ",\"axes\":[",
        snapshot.round_trip ? "true" : "false",
        snapshot.receiving ? "true" : "false", snapshot.tx_gpio, snapshot.rx_gpio,
        snapshot.reconnects, snapshot.heartbeat_count, snapshot.ack_count,
        snapshot.last_heartbeat_ms ? now - snapshot.last_heartbeat_ms : -1,
        snapshot.last_round_trip_ms ? now - snapshot.last_round_trip_ms : -1,
        snapshot.drivers_enabled ? "true" : "false",
        pending,
        snapshot.switch_known[0] ? (int)snapshot.switches[0] : -1,
        snapshot.switch_known[1] ? (int)snapshot.switches[1] : -1,
        snapshot.switch_known[2] ? (int)snapshot.switches[2] : -1,
        snapshot.direct_capable ? "true" : "false",
        snapshot.direct_active ? "true" : "false", snapshot.direct_mode,
        snapshot.direct_target_rpm[0], snapshot.direct_target_rpm[1],
        snapshot.direct_target_rpm[2], snapshot.direct_actual_rpm[0],
        snapshot.direct_actual_rpm[1], snapshot.direct_actual_rpm[2],
        snapshot.direct_ack_count, snapshot.direct_last_ack,
        snapshot.count_active ? "true" : "false", snapshot.count_id);
    for (int i = 0; i < 3 && used > 0 && used < (int)sizeof(body); ++i) {
        axis_state_t *axis = &snapshot.axes[i];
        used += snprintf(body + used, sizeof(body) - used,
            "%s{\"name\":\"%c\",\"position\":%" PRId64 ",\"target\":%" PRId64
            ",\"known\":%s,\"limits_set\":%s,\"minimum\":%" PRId64
            ",\"maximum\":%" PRId64 "}",
            i ? "," : "", AXIS_NAMES[i], axis->position, axis->target,
            axis->known ? "true" : "false", axis->limits_set ? "true" : "false",
            axis->minimum, axis->maximum);
    }
    snprintf(body + used, sizeof(body) - used,
             "],\"last_command\":\"%.80s\",\"last_action\":\"%.110s\","
             "\"last_line\":\"%.100s\",\"last_error\":\"%.100s\"}",
             snapshot.last_command, snapshot.last_action,
             snapshot.last_line, snapshot.last_error);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, body);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    return status_response(request);
}

static bool parse_rpm_value(const char *text, float *value)
{
    char *end = NULL;
    float parsed = strtof(text, &end);
    if (!text[0] || !end || *end || !isfinite(parsed) ||
        parsed < -MAX_RPM || parsed > MAX_RPM) return false;
    *value = parsed;
    return true;
}

static esp_err_t velocity_handler(httpd_req_t *request)
{
    char x_text[24], y_text[24], z_text[24], lease_text[16] = {0};
    float rpm[3];
    if (!query_value(request, "x", x_text, sizeof(x_text)) ||
        !query_value(request, "y", y_text, sizeof(y_text)) ||
        !query_value(request, "z", z_text, sizeof(z_text)) ||
        !parse_rpm_value(x_text, &rpm[0]) ||
        !parse_rpm_value(y_text, &rpm[1]) ||
        !parse_rpm_value(z_text, &rpm[2])) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "invalid signed RPM targets");
    }
    int lease_ms = DIRECT_LEASE_DEFAULT_MS;
    if (query_value(request, "lease_ms", lease_text, sizeof(lease_text))) {
        lease_ms = atoi(lease_text);
    }
    if (lease_ms < DIRECT_LEASE_MIN_MS || lease_ms > DIRECT_LEASE_MAX_MS) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid lease");
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (!state.round_trip) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "GD32 round trip is not established");
    } else if (!state.direct_capable) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "GD32 direct-motion firmware update required");
    } else if (pending_count_locked() || state.count_active) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "A finite stepper move is active");
    } else {
        memcpy(state.direct_target_rpm, rpm, sizeof(rpm));
        for (int i = 0; i < 3; ++i) {
            if (fabsf(rpm[i]) > 0.0005f) state.axes[i].known = false;
        }
        state.direct_lease_deadline_ms = now_ms() + lease_ms;
        state.direct_active = true;
        state.direct_mode = 1;
        state.last_error[0] = '\0';
        snprintf(state.last_command, sizeof(state.last_command),
                 "Direct X%.2f Y%.2f Z%.2f RPM", rpm[0], rpm[1], rpm[2]);
        snprintf(state.last_action, sizeof(state.last_action),
                 "Direct velocity stream armed with %d ms host lease", lease_ms);
    }
    xSemaphoreGive(state_mutex);
    return status_response(request);
}

static esp_err_t count_handler(httpd_req_t *request)
{
    char step_text[3][24], rpm_text[3][16];
    const char *step_keys[3] = {"x", "y", "z"};
    const char *rpm_keys[3] = {"xrpm", "yrpm", "zrpm"};
    int32_t steps[3] = {0};
    long rpm[3] = {0};
    bool any_steps = false;
    for (int i = 0; i < 3; ++i) {
        if (!query_value(request, step_keys[i], step_text[i], sizeof(step_text[i])) ||
            !query_value(request, rpm_keys[i], rpm_text[i], sizeof(rpm_text[i]))) {
            return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                       "missing simultaneous move value");
        }
        char *step_end = NULL;
        char *rpm_end = NULL;
        long parsed_steps = strtol(step_text[i], &step_end, 10);
        rpm[i] = strtol(rpm_text[i], &rpm_end, 10);
        if (!step_end || *step_end || parsed_steps < INT32_MIN ||
            parsed_steps > INT32_MAX || !rpm_end || *rpm_end ||
            rpm[i] < 1 || rpm[i] > MAX_RPM) {
            return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                       "invalid simultaneous move");
        }
        steps[i] = (int32_t)parsed_steps;
        any_steps |= steps[i] != 0;
    }
    if (!any_steps) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "all steps are zero");
    }

    uint32_t id = 0;
    bool accepted = false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (!state.round_trip || !state.direct_capable) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "GD32 direct-motion firmware is not ready");
    } else if (state.direct_active || state.count_active || pending_count_locked()) {
        snprintf(state.last_error, sizeof(state.last_error), "Stepper motion is already active");
    } else {
        bool allowed = true;
        for (int i = 0; i < 3; ++i) {
            int64_t target = state.axes[i].target + steps[i];
            if (steps[i] && state.axes[i].limits_set &&
                (!state.axes[i].known || target < state.axes[i].minimum ||
                 target > state.axes[i].maximum)) allowed = false;
        }
        if (!allowed) {
            snprintf(state.last_error, sizeof(state.last_error),
                     "Simultaneous target outside limits or zero unknown");
        } else {
            id = ++state.move_sequence;
            state.count_id = id;
            state.count_active = true;
            memcpy(state.count_steps, steps, sizeof(steps));
            for (int i = 0; i < 3; ++i) state.axes[i].target += steps[i];
            state.last_error[0] = '\0';
            accepted = true;
        }
    }
    xSemaphoreGive(state_mutex);

    if (accepted) {
        char payload[TX_PAYLOAD_CAPACITY];
        int used = snprintf(payload, sizeof(payload),
                            "M975\nM972 X%ld Y%ld Z%ld\nM971 I%" PRIu32,
                            rpm[0], rpm[1], rpm[2], id);
        for (int i = 0; i < 3 && used > 0 && used < (int)sizeof(payload); ++i) {
            if (steps[i]) {
                used += snprintf(payload + used, sizeof(payload) - used,
                                 " %c%" PRId32, AXIS_NAMES[i], steps[i]);
            }
        }
        if (used > 0 && used < (int)sizeof(payload) - 1) {
            payload[used++] = '\n';
            payload[used] = '\0';
        }
        if (used <= 0 || used >= (int)sizeof(payload) || !queue_payload(payload, false)) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.count_active = false;
            for (int i = 0; i < 3; ++i) state.axes[i].target -= steps[i];
            snprintf(state.last_error, sizeof(state.last_error), "GD32 TX queue is full");
            xSemaphoreGive(state_mutex);
        } else {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            snprintf(state.last_command, sizeof(state.last_command),
                     "Count X%" PRId32 " Y%" PRId32 " Z%" PRId32,
                     steps[0], steps[1], steps[2]);
            snprintf(state.last_action, sizeof(state.last_action),
                     "Queued simultaneous move %" PRIu32, id);
            xSemaphoreGive(state_mutex);
        }
    }
    return status_response(request);
}

static esp_err_t move_handler(httpd_req_t *request)
{
    char axis_text[4], steps_text[24], rpm_text[16];
    if (!query_value(request, "axis", axis_text, sizeof(axis_text)) ||
        !query_value(request, "steps", steps_text, sizeof(steps_text)) ||
        !query_value(request, "rpm", rpm_text, sizeof(rpm_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing move value");
    }
    int axis = axis_index(axis_text[0]);
    long steps_long = strtol(steps_text, NULL, 10);
    long rpm = strtol(rpm_text, NULL, 10);
    if (axis < 0 || axis_text[1] || !steps_long || steps_long < INT32_MIN ||
        steps_long > INT32_MAX || rpm < 1 || rpm > MAX_RPM) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid move");
    }
    int32_t steps = (int32_t)steps_long;
    uint32_t move_id = 0;
    int slot = -1;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool linked = state.round_trip;
    if (!linked) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "GD32 round trip is not established");
    } else if (state.direct_active || state.count_active) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "Stop direct or simultaneous motion first");
    } else if (axis_pending_locked(axis)) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "Axis %c is already moving", AXIS_NAMES[axis]);
    } else {
        int64_t target = state.axes[axis].target + steps;
        bool allowed = !state.axes[axis].limits_set ||
                       (state.axes[axis].known && target >= state.axes[axis].minimum &&
                        target <= state.axes[axis].maximum);
        if (allowed) {
            for (int i = 0; i < MAX_PENDING_MOVES; ++i) {
                if (!state.pending[i].active) { slot = i; break; }
            }
            if (slot >= 0) {
                move_id = ++state.move_sequence;
                state.pending[slot] = (pending_move_t){
                    .active = true, .id = move_id, .axis = axis, .steps = steps};
                state.axes[axis].target = target;
                state.last_error[0] = '\0';
            } else {
                snprintf(state.last_error, sizeof(state.last_error),
                         "Stepper move queue is full");
            }
        } else {
            snprintf(state.last_error, sizeof(state.last_error),
                     "Axis %c target outside limits or zero unknown", AXIS_NAMES[axis]);
        }
    }
    xSemaphoreGive(state_mutex);
    if (slot >= 0) {
        double distance = steps / STEPS_PER_UNIT[axis];
        double feedrate = rpm * 3200.0 / STEPS_PER_UNIT[axis];
        char payload[180];
        int length = snprintf(payload, sizeof(payload),
            "M975\nG91\nG0 %c%.5f F%.2f\nM400\nM118 DRV_DONE %" PRIu32 " %c %" PRId32 "\n",
            AXIS_NAMES[axis], distance, feedrate, move_id, AXIS_NAMES[axis], steps);
        if (length <= 0 || length >= (int)sizeof(payload) || !queue_payload(payload, false)) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.pending[slot].active = false;
            state.axes[axis].target -= steps;
            snprintf(state.last_error, sizeof(state.last_error), "GD32 TX queue is full");
            xSemaphoreGive(state_mutex);
        } else {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            snprintf(state.last_command, sizeof(state.last_command),
                     "%c %+" PRId32 " steps at %ld RPM", AXIS_NAMES[axis], steps, rpm);
            snprintf(state.last_action, sizeof(state.last_action),
                     "Queued %c move %" PRIu32, AXIS_NAMES[axis], move_id);
            xSemaphoreGive(state_mutex);
            ESP_LOGI(TAG, "Queued %c move %" PRIu32 ": %" PRId32 " steps at %ld RPM",
                     AXIS_NAMES[axis], move_id, steps, rpm);
        }
    }
    return status_response(request);
}

static esp_err_t zero_handler(httpd_req_t *request)
{
    char axis_text[4];
    if (!query_value(request, "axis", axis_text, sizeof(axis_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing axis");
    }
    int axis = axis_index(axis_text[0]);
    if (axis < 0 || axis_text[1]) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid axis");
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (!axis_pending_locked(axis) && !state.direct_active && !state.count_active) {
        state.axes[axis].position = 0;
        state.axes[axis].target = 0;
        state.axes[axis].known = true;
        state.last_error[0] = '\0';
        snprintf(state.last_action, sizeof(state.last_action),
                 "Axis %c current position set to zero", AXIS_NAMES[axis]);
    } else {
        snprintf(state.last_error, sizeof(state.last_error),
                 "Cannot zero %c during active motion", AXIS_NAMES[axis]);
    }
    xSemaphoreGive(state_mutex);
    return status_response(request);
}

static esp_err_t limits_handler(httpd_req_t *request)
{
    char axis_text[4], min_text[24], max_text[24];
    if (!query_value(request, "axis", axis_text, sizeof(axis_text)) ||
        !query_value(request, "min", min_text, sizeof(min_text)) ||
        !query_value(request, "max", max_text, sizeof(max_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing limit");
    }
    int axis = axis_index(axis_text[0]);
    int64_t minimum = strtoll(min_text, NULL, 10);
    int64_t maximum = strtoll(max_text, NULL, 10);
    if (axis < 0 || axis_text[1] || minimum >= maximum) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid limits");
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.direct_active || state.count_active || axis_pending_locked(axis)) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "Cannot set %c limits during active motion", AXIS_NAMES[axis]);
    } else if (!state.axes[axis].known || state.axes[axis].target < minimum ||
        state.axes[axis].target > maximum) {
        snprintf(state.last_error, sizeof(state.last_error),
                 "Zero %c before setting limits", AXIS_NAMES[axis]);
    } else {
        state.axes[axis].minimum = minimum;
        state.axes[axis].maximum = maximum;
        state.axes[axis].limits_set = true;
        state.last_error[0] = '\0';
        snprintf(state.last_action, sizeof(state.last_action),
                 "Axis %c limits set: %" PRId64 " to %" PRId64,
                 AXIS_NAMES[axis], minimum, maximum);
    }
    xSemaphoreGive(state_mutex);
    return status_response(request);
}

static esp_err_t enable_handler(httpd_req_t *request)
{
    char value[8];
    if (!query_value(request, "value", value, sizeof(value))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing value");
    }
    bool enable = atoi(value) != 0;
    bool sent = queue_gcode(enable ? "M17" : "M18", false);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (sent) {
        state.drivers_enabled = enable;
        snprintf(state.last_command, sizeof(state.last_command), "%s", enable ? "M17" : "M18");
        snprintf(state.last_action, sizeof(state.last_action),
                 "Stepper drivers %s", enable ? "enabled" : "disabled");
        state.last_error[0] = '\0';
    } else {
        snprintf(state.last_error, sizeof(state.last_error), "GD32 TX queue is full");
    }
    xSemaphoreGive(state_mutex);
    return status_response(request);
}

void stepper_link_quick_stop(void)
{
    if (state_mutex) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        for (int i = 0; i < MAX_PENDING_MOVES; ++i) {
            if (state.pending[i].active) {
                int axis = state.pending[i].axis;
                state.axes[axis].target = state.axes[axis].position;
                state.axes[axis].known = false;
                state.pending[i].active = false;
            }
        }
        if (state.count_active) {
            for (int i = 0; i < 3; ++i) {
                if (state.count_steps[i]) {
                    state.axes[i].target = state.axes[i].position;
                    state.axes[i].known = false;
                }
            }
        }
        state.count_active = false;
        state.direct_active = false;
        state.direct_mode = 0;
        memset(state.direct_target_rpm, 0, sizeof(state.direct_target_rpm));
        memset(state.direct_actual_rpm, 0, sizeof(state.direct_actual_rpm));
        snprintf(state.last_command, sizeof(state.last_command), "M410");
        snprintf(state.last_action, sizeof(state.last_action), "Stepper quick stop requested");
        xSemaphoreGive(state_mutex);
    }
    if (tx_queue) {
        xQueueReset(tx_queue);
        queue_gcode("M410", true);
        queue_gcode("M975", false);
    }
}

bool stepper_link_set_direct_rpm(float x_rpm, float y_rpm, float z_rpm,
                                 int lease_ms)
{
    if (!state_mutex || lease_ms < DIRECT_LEASE_MIN_MS ||
        lease_ms > DIRECT_LEASE_MAX_MS) return false;
    const float rpm[3] = {x_rpm, y_rpm, z_rpm};
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(rpm[i]) || fabsf(rpm[i]) > MAX_RPM) return false;
    }

    bool accepted = false;
    bool enable_drivers = false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.round_trip && state.direct_capable && !state.count_active &&
        !pending_count_locked()) {
        memcpy(state.direct_target_rpm, rpm, sizeof(rpm));
        for (int i = 0; i < 3; ++i) {
            if (fabsf(rpm[i]) > 0.0005f) state.axes[i].known = false;
        }
        state.direct_lease_deadline_ms = now_ms() + lease_ms;
        state.direct_active = true;
        state.direct_mode = 1;
        if (!state.drivers_enabled &&
            (fabsf(rpm[0]) > 0.0005f || fabsf(rpm[1]) > 0.0005f ||
             fabsf(rpm[2]) > 0.0005f)) {
            state.drivers_enabled = true;
            enable_drivers = true;
        }
        state.last_error[0] = '\0';
        snprintf(state.last_command, sizeof(state.last_command),
                 "Controller X%.1f Y%.1f Z%.1f RPM", rpm[0], rpm[1], rpm[2]);
        accepted = true;
    }
    xSemaphoreGive(state_mutex);
    if (enable_drivers && !queue_gcode("M17", true)) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.drivers_enabled = false;
        snprintf(state.last_error, sizeof(state.last_error),
                 "Could not queue controller driver enable");
        xSemaphoreGive(state_mutex);
        return false;
    }
    return accepted;
}

static esp_err_t stop_handler(httpd_req_t *request)
{
    stepper_link_quick_stop();
    return status_response(request);
}

static const char page_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>GD32 Steppers</title><style>"
"*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#18212a;font-family:Arial,sans-serif}"
"header{position:sticky;top:0;background:#17212b;color:white;border-bottom:3px solid #e2a400;padding:10px 12px;z-index:2}"
"header div,main{max-width:900px;margin:auto}h1{font-size:19px;margin:0 0 4px}a{color:white;margin-right:14px;font-size:13px}.link{font-size:13px;color:#ff9189}.up{color:#72dfa7}"
"main{padding:12px 12px 36px;width:100%}.toolbar,.axes{display:grid;gap:10px}.toolbar{grid-template-columns:repeat(3,minmax(0,1fr));margin-bottom:10px}.axes{grid-template-columns:repeat(3,minmax(0,1fr))}"
".axis{min-width:0;background:white;border:1px solid #cbd3da;border-radius:8px;padding:11px}h2{font-size:18px;margin:0 0 8px}.metrics,.fields,.actions,.jog{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:7px;margin-top:8px}"
".direct{background:white;border:1px solid #cbd3da;border-radius:8px;padding:11px;margin-bottom:10px}.direct-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.direct-actions{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:9px}.notice{font-size:12px;color:#52606c;margin:3px 0 9px}.ready{color:#16704b;font-weight:700}.needs-update{color:#b42318;font-weight:700}"
".metric{border:1px solid #d4dce3;border-left:4px solid #16806a;padding:7px;border-radius:5px}.metric small{display:block;color:#5b6874}.metric strong{font-size:15px}"
"label{min-width:0;font-size:12px;color:#4c5965}input{min-width:0;width:100%;min-height:40px;margin-top:4px;padding:7px;font-size:15px;border:1px solid #adb8c2;border-radius:5px}"
"button{min-width:0;max-width:100%;min-height:46px;border:1px solid #aab5bf;border-radius:6px;background:#e7ecf0;font-size:14px;font-weight:700;padding:7px;touch-action:none}.primary{background:#176daf;color:white}.danger{background:#c5251c;color:white}.jog button{background:#dce8f1}.log{background:white;border:1px solid #cbd3da;border-radius:8px;padding:10px;margin-top:10px;font:12px monospace;white-space:pre-wrap;overflow-wrap:anywhere}"
"@media(max-width:760px){.axes{grid-template-columns:minmax(0,1fr)}.toolbar{grid-template-columns:repeat(2,minmax(0,1fr))}}@media(max-width:520px){.direct-grid,.direct-actions{grid-template-columns:minmax(0,1fr)}}@media(max-width:420px){.toolbar{grid-template-columns:minmax(0,1fr)}}"
"</style></head><body><header><div><h1>GD32 X/Y/Z Stepper Control</h1><div id=link class=link>Passively detecting GPIO1/GPIO2...</div><a href='/'>Motors</a><a href='/odrive'>ODrive</a><a href='/battery'>Battery</a><a href='/wifi'>Wi-Fi</a><a href='/update'>Firmware</a></div></header><main>"
"<div class=toolbar><button class=primary onclick=enable(1)>Enable drivers</button><button onclick=enable(0)>Disable drivers</button><button class=danger onclick=stopAll()>QUICK STOP</button></div>"
"<section class=direct><h2>Simultaneous continuous RPM</h2><div id=directfw class=notice>Checking direct-motion firmware...</div><div class=direct-grid><label>X signed RPM<input id=vx type=number min=-600 max=600 step=0.5 value=0></label><label>Y signed RPM<input id=vy type=number min=-600 max=600 step=0.5 value=0></label><label>Z signed RPM<input id=vz type=number min=-600 max=600 step=0.5 value=0></label></div><div class=direct-actions><button id=runDirect class=primary onclick=startDirect()>Run / apply all axes</button><button onclick=centerDirect()>Set all to zero</button><button class=danger onclick=stopAll()>Exit and stop</button></div></section>"
"<section class=direct><h2>Simultaneous exact steps</h2><div class=notice>Each nonzero axis starts together and stops at its exact signed count.</div><div class=direct-grid><label>X steps<input id=cx type=number value=0></label><label>Y steps<input id=cy type=number value=0></label><label>Z steps<input id=cz type=number value=0></label><label>X RPM<input id=cxr type=number min=1 max=600 value=30></label><label>Y RPM<input id=cyr type=number min=1 max=600 value=30></label><label>Z RPM<input id=czr type=number min=1 max=600 value=30></label></div><div class=direct-actions><button id=runCount class=primary onclick=countMove()>Run simultaneous steps</button><button class=danger onclick=stopAll()>Abort move</button></div></section>"
"<div id=axes class=axes></div><div id=log class=log>Waiting for GD32 heartbeat...</div></main><script>"
"const names=['X','Y','Z'];const axisHtml=a=>{let k=a.toLowerCase();return `<section class=axis><h2>${a} axis</h2><div class=metrics><div class=metric><small>Position</small><strong id=${k}pos>unknown</strong></div><div class=metric><small>Allowed negative / positive range</small><strong id=${k}range>not set</strong></div></div><div class=fields><label>Steps (signed or magnitude)<input id=${k}steps type=number value=200></label><label>RPM<input id=${k}rpm type=number min=1 max=1000 value=30></label><label>Negative limit<input id=${k}min type=number value=-10000></label><label>Positive limit<input id=${k}max type=number value=10000></label><label>Hold-jog chunk<input id=${k}chunk type=number min=1 value=50></label></div><div class=actions><button onclick=moveDir('${a}',-1)>Move -</button><button class=primary onclick=move('${a}')>Move signed</button><button onclick=moveDir('${a}',1)>Move +</button><button onclick=zero('${a}')>Reset zero</button><button onclick=limits('${a}')>Set limits</button></div><div class=jog><button id=${k}neg>Hold -</button><button id=${k}posbtn>Hold +</button></div></section>`};"
"document.getElementById('axes').innerHTML=names.map(axisHtml).join('');const val=id=>document.getElementById(id).value;let jogging=null,jogBusy=false,directRunning=false;"
"function api(path){return fetch(path,{cache:'no-store'}).then(r=>{if(!r.ok)throw Error(r.status+' '+r.statusText);return r.json()}).then(render).catch(e=>document.getElementById('log').textContent='Request failed: '+e)}"
"function sendMove(a,steps){let k=a.toLowerCase();api('/api/stepper/move?axis='+a+'&steps='+steps+'&rpm='+val(k+'rpm'))}function move(a){let k=a.toLowerCase();sendMove(a,Number(val(k+'steps')))}function moveDir(a,d){let k=a.toLowerCase();sendMove(a,Math.abs(Number(val(k+'steps')))*d)}function zero(a){if(confirm('Define current '+a+' position as zero?'))api('/api/stepper/zero?axis='+a)}"
"function limits(a){let k=a.toLowerCase();api('/api/stepper/limits?axis='+a+'&min='+val(k+'min')+'&max='+val(k+'max'))}function enable(v){api('/api/stepper/enable?value='+v)}function stopAll(){jogging=null;directRunning=false;api('/api/stepper/stop')}"
"function velocity(){return api('/api/stepper/velocity?x='+val('vx')+'&y='+val('vy')+'&z='+val('vz')+'&lease_ms=400')}function startDirect(){directRunning=true;velocity()}function centerDirect(){document.getElementById('vx').value=0;document.getElementById('vy').value=0;document.getElementById('vz').value=0;directRunning=true;velocity()}function countMove(){directRunning=false;api('/api/stepper/count?x='+val('cx')+'&y='+val('cy')+'&z='+val('cz')+'&xrpm='+val('cxr')+'&yrpm='+val('cyr')+'&zrpm='+val('czr'))}"
"async function jogSend(){if(!jogging||jogBusy)return;jogBusy=true;let a=jogging.axis,k=a.toLowerCase(),steps=Math.abs(Number(val(k+'chunk')))*jogging.dir;try{await api('/api/stepper/move?axis='+a+'&steps='+steps+'&rpm='+val(k+'rpm'))}finally{jogBusy=false}}"
"function startJog(a,d){jogging={axis:a,dir:d};jogSend()}function endJog(){jogging=null}function bind(a,d,s){let b=document.getElementById(a.toLowerCase()+s);b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);startJog(a,d)});b.addEventListener('pointerup',endJog);b.addEventListener('pointercancel',endJog);b.addEventListener('lostpointercapture',endJog)}"
"names.forEach(a=>{bind(a,-1,'neg');bind(a,1,'posbtn')});setInterval(()=>{if(jogging)jogSend()},180);setInterval(()=>{if(directRunning)velocity()},100);"
"function render(s){let l=document.getElementById('link');l.textContent=s.alive?'GD32 linked | drivers '+(s.drivers_enabled?'ENABLED':'disabled')+' | TX GPIO'+s.tx+' RX GPIO'+s.rx:(s.receiving?'heartbeat received; waiting round trip':'GD32 heartbeat missing');l.className='link '+(s.alive?'up':'');let fw=document.getElementById('directfw');fw.textContent=s.direct_capable?'Direct-motion firmware ready | mode '+s.direct_mode+' | actual RPM '+s.direct_actual.join(' / '):'Direct-motion firmware not detected';fw.className='notice '+(s.direct_capable?'ready':'needs-update');document.getElementById('runDirect').disabled=!s.direct_capable;document.getElementById('runCount').disabled=!s.direct_capable;s.axes.forEach(a=>{let k=a.name.toLowerCase();document.getElementById(k+'pos').textContent=a.known?a.position+' (target '+a.target+')':'unknown - reset zero before setting limits';document.getElementById(k+'range').textContent=a.limits_set?a.minimum+' .. '+a.maximum:'not set';if(a.limits_set&&document.activeElement!==document.getElementById(k+'min'))document.getElementById(k+'min').value=a.minimum;if(a.limits_set&&document.activeElement!==document.getElementById(k+'max'))document.getElementById(k+'max').value=a.maximum});document.getElementById('log').textContent='switches X/Y/Z: '+s.switches.join(' / ')+' | planner pending '+s.pending+' | counted '+s.count_active+' | direct acks '+s.direct_ack_count+'\\ncommand: '+(s.last_command||'none')+'\\naction: '+(s.last_action||'none')+'\\nGD32: '+s.last_line+'\\nerror: '+(s.last_error||'none')}"
"setInterval(()=>api('/api/stepper/status'),700);api('/api/stepper/status');document.addEventListener('visibilitychange',()=>{if(document.hidden){endJog();directRunning=false;api('/api/stepper/stop')}});"
"</script></body></html>";

static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, page_html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t stepper_link_start(void)
{
    state_mutex = xSemaphoreCreateMutex();
    write_mutex = xSemaphoreCreateMutex();
    tx_queue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_item_t));
    if (!state_mutex || !write_mutex || !tx_queue) return ESP_ERR_NO_MEM;
    BaseType_t created = xTaskCreate(service_task, "gd32_uart", 6144, NULL, 8, NULL);
    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t stepper_link_register_routes(httpd_handle_t server)
{
    const httpd_uri_t routes[] = {
        {.uri = "/steppers", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/stepper/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/stepper/velocity", .method = HTTP_GET, .handler = velocity_handler},
        {.uri = "/api/stepper/count", .method = HTTP_GET, .handler = count_handler},
        {.uri = "/api/stepper/move", .method = HTTP_GET, .handler = move_handler},
        {.uri = "/api/stepper/zero", .method = HTTP_GET, .handler = zero_handler},
        {.uri = "/api/stepper/limits", .method = HTTP_GET, .handler = limits_handler},
        {.uri = "/api/stepper/enable", .method = HTTP_GET, .handler = enable_handler},
        {.uri = "/api/stepper/stop", .method = HTTP_GET, .handler = stop_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        esp_err_t result = httpd_register_uri_handler(server, &routes[i]);
        if (result != ESP_OK) return result;
    }
    return ESP_OK;
}
