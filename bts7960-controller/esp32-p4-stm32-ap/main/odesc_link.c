#include "odesc_link.h"

#include <errno.h>
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
#include "freertos/semphr.h"
#include "freertos/task.h"

#define ODESC_UART UART_NUM_3
#define ODESC_BAUD 115200
#define ODESC_GPIO_A 46
#define ODESC_GPIO_B 33
#define ODESC_QUERY_TIMEOUT_MS 650
#define ODESC_POLL_MS 1000
#define ODESC_LINK_TIMEOUT_MS 4000
#define ODESC_MOTION_LEASE_MS 850
#define ODESC_LINE_CAPACITY 128

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_SENSORLESS_CONTROL 5
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

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
    float ibus_a;
    bool current_valid;
    unsigned reconnects;
    unsigned queries;
    unsigned failures;
    char last_command[96];
    char last_reply[96];
    char last_action[128];
    char last_error[128];
} odesc_state_t;

static const char *TAG = "odesc_uart";
static odesc_state_t state = {
    .tx_gpio = -1,
    .rx_gpio = -1,
    .active_axis = -1,
};
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t io_mutex;
static bool uart_installed;
static char line_buffer[ODESC_LINE_CAPACITY];
static size_t line_length;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static void uart_close_locked(void)
{
    if (uart_installed) {
        uart_driver_delete(ODESC_UART);
        uart_installed = false;
    }
    gpio_reset_pin(ODESC_GPIO_A);
    gpio_reset_pin(ODESC_GPIO_B);
    gpio_set_direction(ODESC_GPIO_A, GPIO_MODE_INPUT);
    gpio_set_direction(ODESC_GPIO_B, GPIO_MODE_INPUT);
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
    esp_err_t result = uart_driver_install(ODESC_UART, 1024, 0, 0, NULL, 0);
    if (result == ESP_OK) result = uart_param_config(ODESC_UART, &config);
    if (result == ESP_OK) {
        result = uart_set_pin(ODESC_UART, tx_gpio, rx_gpio,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    uart_installed = result == ESP_OK;
    if (!uart_installed) {
        uart_close_locked();
        return false;
    }
    gpio_set_pull_mode(rx_gpio, GPIO_PULLUP_ONLY);
    uart_flush_input(ODESC_UART);
    return true;
}

static bool read_line_locked(char *output, size_t output_size, uint32_t timeout_ms)
{
    int64_t deadline = now_ms() + timeout_ms;
    uint8_t byte;
    while (uart_installed && now_ms() < deadline) {
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
    return sent;
}

static bool query_locked(const char *command, char *reply, size_t reply_size,
                         uint32_t timeout_ms)
{
    if (!uart_installed) return false;
    uart_flush_input(ODESC_UART);
    line_length = 0;
    if (!send_locked(command)) return false;
    if (!read_line_locked(reply, reply_size, timeout_ms)) return false;
    ESP_LOGI(TAG, "RX: %s", reply);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.last_reply_ms = now_ms();
    state.queries++;
    snprintf(state.last_reply, sizeof(state.last_reply), "%.95s", reply);
    xSemaphoreGive(state_mutex);
    return true;
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

static bool query_float_locked(const char *property, float *value)
{
    char command[96];
    char reply[ODESC_LINE_CAPACITY];
    snprintf(command, sizeof(command), "r %s", property);
    return query_locked(command, reply, sizeof(reply), ODESC_QUERY_TIMEOUT_MS) &&
           parse_float_reply(reply, value);
}

static bool write_property_locked(int axis, const char *property, int value)
{
    char command[96];
    snprintf(command, sizeof(command), "w axis%d.%s %d", axis, property, value);
    return send_locked(command);
}

static bool set_idle_locked(int axis)
{
    char command[48];
    snprintf(command, sizeof(command), "v %d 0 0", axis);
    bool sent = send_locked(command);
    return write_property_locked(axis, "requested_state", AXIS_STATE_IDLE) && sent;
}

static bool probe_orientation(int tx_gpio, int rx_gpio, float *voltage)
{
    bool ok = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1500)) != pdTRUE) return false;
    if (uart_open_locked(tx_gpio, rx_gpio)) {
        vTaskDelay(pdMS_TO_TICKS(30));
        ok = query_float_locked("vbus_voltage", voltage) &&
             *voltage >= 0.0f && *voltage <= 100.0f;
    }
    if (!ok) uart_close_locked();
    xSemaphoreGive(io_mutex);
    return ok;
}

static bool find_link(void)
{
    const int tx_candidates[] = {ODESC_GPIO_A, ODESC_GPIO_B};
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.detecting = true;
    state.connected = false;
    state.tx_gpio = -1;
    state.rx_gpio = -1;
    xSemaphoreGive(state_mutex);

    for (int i = 0; i < 2; ++i) {
        int tx = tx_candidates[i];
        int rx = tx == ODESC_GPIO_A ? ODESC_GPIO_B : ODESC_GPIO_A;
        float voltage = 0.0f;
        ESP_LOGI(TAG, "Probing ODESC TX GPIO%d RX GPIO%d", tx, rx);
        if (!probe_orientation(tx, rx, &voltage)) continue;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.connected = true;
        state.detecting = false;
        state.tx_gpio = tx;
        state.rx_gpio = rx;
        state.vbus_voltage = voltage;
        state.last_voltage_ms = now_ms();
        state.reconnects++;
        state.failures = 0;
        state.last_error[0] = '\0';
        snprintf(state.last_action, sizeof(state.last_action),
                 "ODESC UART verified at %.2f V", voltage);
        xSemaphoreGive(state_mutex);
        ESP_LOGI(TAG, "ODESC verified: TX GPIO%d RX GPIO%d VBUS %.3f V",
                 tx, rx, voltage);
        return true;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.detecting = false;
    state.connected = false;
    snprintf(state.last_error, sizeof(state.last_error),
             "No ODESC vbus reply on GPIO46/33");
    xSemaphoreGive(state_mutex);
    return false;
}

static bool refresh_voltage(bool include_current)
{
    float voltage = 0.0f;
    float current = 0.0f;
    bool voltage_ok = false;
    bool current_ok = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1500)) != pdTRUE) return false;
    voltage_ok = query_float_locked("vbus_voltage", &voltage) &&
                 voltage >= 0.0f && voltage <= 100.0f;
    if (voltage_ok && include_current) current_ok = query_float_locked("ibus", &current);
    xSemaphoreGive(io_mutex);

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (voltage_ok) {
        state.vbus_voltage = voltage;
        state.last_voltage_ms = now_ms();
        state.failures = 0;
        state.last_error[0] = '\0';
        if (include_current && current_ok) {
            state.ibus_a = current;
            state.current_valid = true;
        }
    } else {
        state.failures++;
        snprintf(state.last_error, sizeof(state.last_error),
                 "No valid ODESC vbus_voltage reply");
    }
    xSemaphoreGive(state_mutex);
    return voltage_ok;
}

static void service_task(void *argument)
{
    (void)argument;
    int64_t next_poll = 0;
    unsigned poll_count = 0;
    while (true) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool connected = state.connected;
        int axis = state.active_axis;
        bool lease_expired = axis >= 0 && state.motion_deadline_ms &&
                             now_ms() >= state.motion_deadline_ms;
        if (lease_expired) state.motion_deadline_ms = 0;
        unsigned failures = state.failures;
        xSemaphoreGive(state_mutex);

        if (!connected) {
            if (!find_link()) vTaskDelay(pdMS_TO_TICKS(1200));
            next_poll = now_ms();
            continue;
        }

        if (lease_expired) {
            if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                set_idle_locked(axis);
                xSemaphoreGive(io_mutex);
            }
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.active_axis = -1;
            snprintf(state.last_action, sizeof(state.last_action),
                     "Motion deadman stopped axis %d", axis);
            xSemaphoreGive(state_mutex);
        }

        if (now_ms() >= next_poll) {
            next_poll = now_ms() + ODESC_POLL_MS;
            refresh_voltage((++poll_count % 2) == 0);
        }

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        failures = state.failures;
        int64_t last_reply = state.last_reply_ms;
        xSemaphoreGive(state_mutex);
        if (failures >= 3 || (last_reply && now_ms() - last_reply > ODESC_LINK_TIMEOUT_MS)) {
            if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                uart_close_locked();
                xSemaphoreGive(io_mutex);
            }
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.connected = false;
            state.tx_gpio = -1;
            state.rx_gpio = -1;
            state.active_axis = -1;
            state.current_valid = false;
            snprintf(state.last_error, sizeof(state.last_error), "ODESC link timed out");
            xSemaphoreGive(state_mutex);
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
    if (!query_value(request, "axis", text, sizeof(text))) return false;
    char *end = NULL;
    long value = strtol(text, &end, 10);
    if (!end || *end || value < 0 || value > 1) return false;
    *axis = (int)value;
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
    char body[1200];
    snprintf(body, sizeof(body),
             "{\"connected\":%s,\"detecting\":%s,\"tx\":%d,\"rx\":%d,"
             "\"vbus_voltage\":%.4f,\"voltage_age_ms\":%lld,"
             "\"current_valid\":%s,\"ibus_a\":%.4f,\"bus_power_w\":%.3f,"
             "\"active_axis\":%d,\"reconnects\":%u,\"queries\":%u,"
             "\"failures\":%u,\"last_command\":\"%.80s\","
             "\"last_reply\":\"%.80s\",\"last_action\":\"%.110s\","
             "\"last_error\":\"%.110s\"}",
             snapshot.connected ? "true" : "false",
             snapshot.detecting ? "true" : "false",
             snapshot.tx_gpio, snapshot.rx_gpio, snapshot.vbus_voltage,
             (long long)voltage_age,
             snapshot.current_valid ? "true" : "false", snapshot.ibus_a, power,
             snapshot.active_axis, snapshot.reconnects, snapshot.queries,
             snapshot.failures, snapshot.last_command, snapshot.last_reply,
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

static esp_err_t state_handler(httpd_req_t *request)
{
    int axis;
    char value_text[8];
    if (!request_axis(request, &axis) ||
        !query_value(request, "value", value_text, sizeof(value_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad axis/state");
    }
    int value = atoi(value_text);
    if (value != AXIS_STATE_IDLE && value != AXIS_STATE_SENSORLESS_CONTROL &&
        value != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "state must be 1, 5, or 8");
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

static esp_err_t velocity_handler(httpd_req_t *request)
{
    int axis;
    char velocity_text[32];
    if (!request_axis(request, &axis) ||
        !query_value(request, "velocity", velocity_text, sizeof(velocity_text))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad velocity request");
    }
    char *end = NULL;
    float velocity = strtof(velocity_text, &end);
    if (!end || *end || !isfinite(velocity) || velocity < -50.0f || velocity > 50.0f) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "velocity must be -50 through 50 turns/s");
    }
    char command[64];
    snprintf(command, sizeof(command), "v %d %.6f 0", axis, velocity);
    bool sent = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1200)) == pdTRUE) {
        sent = send_locked(command);
        xSemaphoreGive(io_mutex);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_command, sizeof(state.last_command), "%s", command);
    if (sent) {
        state.active_axis = axis;
        state.motion_deadline_ms = now_ms() + ODESC_MOTION_LEASE_MS;
        state.last_error[0] = '\0';
        snprintf(state.last_action, sizeof(state.last_action),
                 "Axis %d velocity %.3f turns/s", axis, velocity);
    } else {
        snprintf(state.last_error, sizeof(state.last_error), "ODESC UART write failed");
    }
    xSemaphoreGive(state_mutex);
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
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.active_axis == axis) state.active_axis = -1;
    state.motion_deadline_ms = 0;
    snprintf(state.last_command, sizeof(state.last_command),
             "v %d 0 0; requested_state 1", axis);
    snprintf(state.last_action, sizeof(state.last_action), "Axis %d stopped and idle", axis);
    if (!sent) snprintf(state.last_error, sizeof(state.last_error), "ODESC stop write failed");
    else state.last_error[0] = '\0';
    xSemaphoreGive(state_mutex);
    return status_response(request);
}

static esp_err_t clear_handler(httpd_req_t *request)
{
    int axis;
    if (!request_axis(request, &axis)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad axis");
    }
    bool sent = false;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1500)) == pdTRUE) {
        sent = set_idle_locked(axis);
        sent = write_property_locked(axis, "error", 0) && sent;
        sent = write_property_locked(axis, "motor.error", 0) && sent;
        sent = write_property_locked(axis, "encoder.error", 0) && sent;
        sent = write_property_locked(axis, "controller.error", 0) && sent;
        xSemaphoreGive(io_mutex);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_command, sizeof(state.last_command), "clear axis%d errors", axis);
    snprintf(state.last_action, sizeof(state.last_action), "Axis %d errors cleared", axis);
    if (!sent) snprintf(state.last_error, sizeof(state.last_error), "ODESC clear write failed");
    else state.last_error[0] = '\0';
    xSemaphoreGive(state_mutex);
    return status_response(request);
}

void odesc_link_stop_all(void)
{
    if (!io_mutex || !state_mutex) return;
    if (xSemaphoreTake(io_mutex, pdMS_TO_TICKS(1200)) == pdTRUE) {
        if (uart_installed) {
            set_idle_locked(0);
            set_idle_locked(1);
        }
        xSemaphoreGive(io_mutex);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.active_axis = -1;
    state.motion_deadline_ms = 0;
    snprintf(state.last_action, sizeof(state.last_action), "All ODESC axes stopped and idle");
    xSemaphoreGive(state_mutex);
}

static esp_err_t stop_all_handler(httpd_req_t *request)
{
    odesc_link_stop_all();
    return status_response(request);
}

static const char page_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>ODESC Control</title><style>"
"*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#18212a;font-family:Arial,sans-serif}header{background:#17212b;color:white;border-bottom:3px solid #e2a400;padding:11px 12px}header div,main{max-width:760px;margin:auto}h1{font-size:19px;margin:0 0 4px}nav a{color:white;margin-right:14px;font-size:13px}.link{font-size:13px;color:#ff9189}.up{color:#72dfa7}main{padding:12px}.panel{background:white;border:1px solid #cbd3da;border-radius:8px;padding:12px;margin-bottom:12px}.metrics,.fields,.actions{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.metric{min-width:0;border:1px solid #d4dce3;border-left:4px solid #16806a;padding:9px;border-radius:5px}.metric small{display:block;color:#56636f}.metric strong{font-size:20px}label{font-size:12px;color:#4d5a65}input,select,button{min-width:0;width:100%;min-height:46px;margin-top:4px;padding:8px;border:1px solid #aab5bf;border-radius:6px;background:white;font-size:15px}button{font-weight:700;background:#e7ecf0}.primary{background:#176daf;color:white}.danger{background:#c5251c;color:white}.log{font:12px monospace;white-space:pre-wrap;overflow-wrap:anywhere}h2{font-size:17px;margin:0 0 9px}@media(max-width:560px){.metrics,.fields,.actions{grid-template-columns:minmax(0,1fr)}}"
"</style></head><body><header><div><h1>ODESC / ODrive Control</h1><div id=link class=link>Detecting GPIO46/33...</div><nav><a href='/'>Motors</a><a href='/steppers'>Steppers</a><a href='/wifi'>Wi-Fi</a><a href='/update'>Firmware</a></nav></div></header><main>"
"<section class=panel><div class=metrics><div class=metric><small>Battery bus</small><strong id=vbus>-- V</strong></div><div class=metric><small>Bus current</small><strong id=ibus>-- A</strong></div><div class=metric><small>Bus power</small><strong id=power>-- W</strong></div></div></section>"
"<section class=panel><h2>Axis control</h2><div class=fields><label>Axis<select id=axis><option value=0>Axis 0</option><option value=1>Axis 1</option></select></label><label>Velocity turns/s<input id=velocity type=number min=-50 max=50 step=0.1 value=2></label></div><div class=actions><button onclick=stateCmd(8)>Closed loop state</button><button onclick=stateCmd(5)>Sensorless state</button><button class=primary id=negative>Hold negative</button><button class=primary id=positive>Hold positive</button><button class=danger onclick=stopAxis()>STOP + IDLE</button><button onclick=clearErrors()>Clear errors</button></div></section>"
"<section class=panel><div class=actions><button onclick=api('/api/odrive/refresh')>Refresh telemetry</button><button class=danger onclick=api('/api/odrive/stop_all')>STOP BOTH AXES</button></div><div id=log class=log>Waiting for ODESC...</div></section></main><script>"
"const e=id=>document.getElementById(id),axis=()=>e('axis').value,vel=()=>Math.abs(Number(e('velocity').value));let holding=null,timer=null;"
"function render(s){let l=e('link');l.textContent=s.connected?'ODESC linked | TX GPIO'+s.tx+' RX GPIO'+s.rx:(s.detecting?'Detecting GPIO46/33...':'ODESC not detected');l.className='link '+(s.connected?'up':'');e('vbus').textContent=s.connected?s.vbus_voltage.toFixed(2)+' V':'-- V';e('ibus').textContent=s.current_valid?s.ibus_a.toFixed(2)+' A':'-- A';e('power').textContent=s.current_valid?s.bus_power_w.toFixed(1)+' W':'-- W';e('log').textContent='voltage age '+s.voltage_age_ms+' ms | queries '+s.queries+' | reconnects '+s.reconnects+'\\nactive axis: '+s.active_axis+'\\ncommand: '+(s.last_command||'none')+'\\naction: '+(s.last_action||'none')+'\\nreply: '+(s.last_reply||'none')+'\\nerror: '+(s.last_error||'none')}"
"function api(path){return fetch(path,{cache:'no-store'}).then(r=>{if(!r.ok)throw Error(r.status+' '+r.statusText);return r.json()}).then(render).catch(x=>e('log').textContent='Request failed: '+x)}function stateCmd(v){api('/api/odrive/state?axis='+axis()+'&value='+v)}function push(d){api('/api/odrive/velocity?axis='+axis()+'&velocity='+(vel()*d))}function stopAxis(){holding=null;if(timer){clearInterval(timer);timer=null}api('/api/odrive/stop?axis='+axis())}function clearErrors(){api('/api/odrive/clear?axis='+axis())}"
"function bind(id,d){let b=e(id);let start=x=>{x.preventDefault();holding=d;push(d);timer=setInterval(()=>{if(holding)push(holding)},300)};let stop=x=>{x.preventDefault();stopAxis()};b.addEventListener('pointerdown',start);b.addEventListener('pointerup',stop);b.addEventListener('pointercancel',stop);b.addEventListener('lostpointercapture',stop)}bind('negative',-1);bind('positive',1);setInterval(()=>api('/api/odrive/status'),1000);api('/api/odrive/status');document.addEventListener('visibilitychange',()=>{if(document.hidden&&holding)stopAxis()});"
"</script></body></html>";

static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, page_html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t odesc_link_start(void)
{
    state_mutex = xSemaphoreCreateMutex();
    io_mutex = xSemaphoreCreateMutex();
    if (!state_mutex || !io_mutex) return ESP_ERR_NO_MEM;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_error, sizeof(state.last_error),
             "ODESC UART disabled; controller suspected damaged");
    xSemaphoreGive(state_mutex);
    return ESP_OK;
}

esp_err_t odesc_link_register_routes(httpd_handle_t server)
{
    const httpd_uri_t routes[] = {
        {.uri = "/odrive", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/odrive/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/odrive/refresh", .method = HTTP_GET, .handler = refresh_handler},
        {.uri = "/api/odrive/state", .method = HTTP_GET, .handler = state_handler},
        {.uri = "/api/odrive/velocity", .method = HTTP_GET, .handler = velocity_handler},
        {.uri = "/api/odrive/stop", .method = HTTP_GET, .handler = stop_handler},
        {.uri = "/api/odrive/stop_all", .method = HTTP_GET, .handler = stop_all_handler},
        {.uri = "/api/odrive/clear", .method = HTTP_GET, .handler = clear_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        esp_err_t result = httpd_register_uri_handler(server, &routes[i]);
        if (result != ESP_OK) return result;
    }
    return ESP_OK;
}
