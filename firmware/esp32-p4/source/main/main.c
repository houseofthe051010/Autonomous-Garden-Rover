#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "network_ota.h"
#include "nvs_flash.h"
#include "odesc_link.h"
#include "rover_link.h"
#include "speaker.h"
#include "battery_history.h"
#include "mower_history.h"
#include "bno080_link.h"
#include "stepper_link.h"
#include "drive_control.h"

#define STM32_UART UART_NUM_1
#define STM32_BAUD 115200
#define STM32_TX_GPIO 21
#define STM32_RX_GPIO 22
#define UART_RX_PROBE_MS 2400
#define UART_REPLY_MS 700
#define UART_LINK_TIMEOUT_MS 3500
#define MOTOR_LEASE_MS 800
#define STATUS_INTERVAL_MS 450
#define LINE_CAPACITY 192
#define TANK_DEFAULT_RAMP_PERCENT 80
#define TANK_TARGET_TIMEOUT_MS 450
#define TANK_UPDATE_MS 50
#define DRIVE_BUS_VOLTAGE 8.0f
#define BTS7960_SENSE_AMPS_PER_VOLT 8.5f

static const char *TAG = "robot_ap";

typedef struct {
    char direction;
    unsigned duty;
    unsigned current_a_raw;
    unsigned current_b_raw;
    unsigned current_a_mv;
    unsigned current_b_mv;
} motor_state_t;

typedef struct {
    bool alive;
    int tx_gpio;
    int rx_gpio;
    int64_t last_rx_ms;
    unsigned heartbeat_ms;
    motor_state_t motors[2];
    unsigned encoders[4];
    unsigned encoder_sequence;
    unsigned encoder_rate_hz;
    unsigned watchdog;
    char last_reply[128];
    char last_error[128];
} robot_state_t;

static robot_state_t state = {
    .tx_gpio = -1,
    .rx_gpio = -1,
    .motors = {{.direction = 'S'}, {.direction = 'S'}},
};
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t uart_mutex;
static bool uart_installed;
static char uart_line_buffer[LINE_CAPACITY];
static size_t uart_line_length;
static int64_t motor_lease_deadline_ms;
static int tank_target[2];
static int tank_output[2];
static unsigned tank_ramp_percent = TANK_DEFAULT_RAMP_PERCENT;
static int64_t tank_target_deadline_ms;
static int64_t tank_last_update_ms;
static int64_t tank_last_send_ms;
static bool tank_mode_active;
static bool motor_stop_pending;
static bool imu_calibration_owns_drive;
static bool autonomous_sequence_owns_drive;
static bool autonomous_sequence_started_encoders;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static esp_err_t send_conflict(httpd_req_t *request, const char *message)
{
    httpd_resp_set_status(request, "409 Conflict");
    httpd_resp_set_type(request, "text/plain");
    return httpd_resp_sendstr(request, message);
}

static void state_error(const char *message)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_error, sizeof(state.last_error), "%s", message);
    xSemaphoreGive(state_mutex);
}

static void state_reply(const char *line)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.last_reply, sizeof(state.last_reply), "%s", line);
    if (strncmp(line, "ERR ", 4) == 0 || strncmp(line, "FAULT ", 6) == 0) {
        snprintf(state.last_error, sizeof(state.last_error), "%s", line);
    } else {
        state.last_error[0] = '\0';
    }
    xSemaphoreGive(state_mutex);
}

static void process_stm32_line(const char *line)
{
    ESP_LOGI(TAG, "STM32: %s", line);

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.last_rx_ms = now_ms();

    if (strncmp(line, "HB ", 3) == 0) {
        state.heartbeat_ms = (unsigned)strtoul(line + 3, NULL, 10);
        state.alive = state.tx_gpio >= 0 && state.rx_gpio >= 0;
    } else if (strncmp(line, "READY ", 6) == 0 || strcmp(line, "OK PONG") == 0) {
        state.alive = state.tx_gpio >= 0 && state.rx_gpio >= 0;
        snprintf(state.last_reply, sizeof(state.last_reply), "%s", line);
    } else if (strncmp(line, "MSTAT ", 6) == 0) {
        char d1 = 'S';
        char d2 = 'S';
        unsigned motor1 = 0;
        unsigned motor2 = 0;
        unsigned wd = 0;
        motor_state_t m1 = {0};
        motor_state_t m2 = {0};
        const int fields = sscanf(
            line,
            "MSTAT %u %c %u %u %u %u %u %u %c %u %u %u %u %u WD %u",
            &motor1, &d1, &m1.duty, &m1.current_a_raw, &m1.current_b_raw,
            &m1.current_a_mv, &m1.current_b_mv, &motor2, &d2, &m2.duty,
            &m2.current_a_raw, &m2.current_b_raw, &m2.current_a_mv,
            &m2.current_b_mv, &wd);
        if (fields == 15 && motor1 == 1 && motor2 == 2) {
            m1.direction = d1;
            m2.direction = d2;
            state.motors[0] = m1;
            state.motors[1] = m2;
            state.watchdog = wd;
            snprintf(state.last_reply, sizeof(state.last_reply), "%s", line);
        } else {
            snprintf(state.last_error, sizeof(state.last_error),
                     "Malformed MSTAT (%d fields)", fields);
        }
    } else if (strncmp(line, "ENC ", 4) == 0) {
        unsigned stm32_ms = 0;
        unsigned values[4];
        if (sscanf(line, "ENC %u %u %u %u %u %u", &state.encoder_sequence,
                   &stm32_ms, &values[0], &values[1], &values[2], &values[3]) == 6) {
            memcpy(state.encoders, values, sizeof(values));
        } else {
            snprintf(state.last_error, sizeof(state.last_error), "Malformed ENC");
        }
    } else {
        snprintf(state.last_reply, sizeof(state.last_reply), "%s", line);
        if (strncmp(line, "ERR ", 4) == 0 || strncmp(line, "FAULT ", 6) == 0) {
            snprintf(state.last_error, sizeof(state.last_error), "%s", line);
        }
    }
    xSemaphoreGive(state_mutex);
}

static bool uart_read_line(char *output, size_t output_size, uint32_t timeout_ms)
{
    const int64_t deadline = now_ms() + timeout_ms;
    uint8_t byte;

    while (now_ms() < deadline) {
        int remaining = (int)(deadline - now_ms());
        TickType_t wait = pdMS_TO_TICKS(remaining > 20 ? 20 : remaining);
        if (uart_read_bytes(STM32_UART, &byte, 1, wait) != 1) {
            continue;
        }
        if (byte == '\n') {
            if (uart_line_length == 0) {
                continue;
            }
            uart_line_buffer[uart_line_length] = '\0';
            snprintf(output, output_size, "%s", uart_line_buffer);
            uart_line_length = 0;
            return true;
        }
        if (byte == '\r') {
            continue;
        }
        if (uart_line_length < sizeof(uart_line_buffer) - 1) {
            uart_line_buffer[uart_line_length++] = (char)byte;
        } else {
            uart_line_length = 0;
        }
    }
    return false;
}

static void uart_close(void)
{
    if (uart_installed) {
        uart_driver_delete(STM32_UART);
        uart_installed = false;
    }
    gpio_reset_pin(STM32_TX_GPIO);
    gpio_reset_pin(STM32_RX_GPIO);
    uart_line_length = 0;
}

static bool uart_open_probe(int rx_gpio)
{
    uart_close();
    const uart_config_t config = {
        .baud_rate = STM32_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(STM32_UART, 1024, 0, 0, NULL, 0) != ESP_OK) {
        return false;
    }
    uart_installed = true;
    if (uart_param_config(STM32_UART, &config) != ESP_OK ||
        uart_set_pin(STM32_UART, UART_PIN_NO_CHANGE, rx_gpio,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        uart_close();
        return false;
    }
    gpio_set_pull_mode(rx_gpio, GPIO_PULLUP_ONLY);
    uart_flush_input(STM32_UART);
    return true;
}

static bool uart_probe_rx(int rx_gpio)
{
    char line[LINE_CAPACITY];
    ESP_LOGI(TAG, "Probing STM32 TX on ESP32-P4 GPIO%d", rx_gpio);
    if (!uart_open_probe(rx_gpio)) {
        return false;
    }
    int64_t deadline = now_ms() + UART_RX_PROBE_MS;
    while (now_ms() < deadline) {
        if (!uart_read_line(line, sizeof(line), 100)) {
            continue;
        }
        process_stm32_line(line);
        if (strncmp(line, "HB ", 3) == 0 || strncmp(line, "READY ", 6) == 0) {
            return true;
        }
    }
    return false;
}

static void uart_send(const char *command)
{
    uart_write_bytes(STM32_UART, command, strlen(command));
    uart_write_bytes(STM32_UART, "\n", 1);
    uart_wait_tx_done(STM32_UART, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "TX: %s", command);
}

static bool uart_command_locked(const char *command, const char *expected,
                                uint32_t timeout_ms)
{
    char line[LINE_CAPACITY];
    if (!uart_installed) {
        return false;
    }
    uart_send(command);
    int64_t deadline = now_ms() + timeout_ms;
    while (now_ms() < deadline) {
        if (!uart_read_line(line, sizeof(line), 80)) {
            continue;
        }
        process_stm32_line(line);
        if (expected == NULL || strncmp(line, expected, strlen(expected)) == 0) {
            state_reply(line);
            return true;
        }
        if (strncmp(line, "ERR ", 4) == 0) {
            state_reply(line);
            return false;
        }
    }
    char error[96];
    snprintf(error, sizeof(error), "No STM32 reply to %s", command);
    state_error(error);
    return false;
}

static bool uart_command(const char *command, const char *expected,
                         uint32_t timeout_ms)
{
    bool result;
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(timeout_ms + 300)) != pdTRUE) {
        state_error("STM32 UART busy");
        return false;
    }
    result = uart_command_locked(command, expected, timeout_ms);
    xSemaphoreGive(uart_mutex);
    return result;
}

static bool uart_find_link(void)
{
    char line[LINE_CAPACITY];

    xSemaphoreTake(uart_mutex, portMAX_DELAY);
    if (uart_probe_rx(STM32_RX_GPIO) &&
        uart_set_pin(STM32_UART, STM32_TX_GPIO, STM32_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK) {
        uart_send("MSTOP ALL");
        uart_send("PING");
        int64_t deadline = now_ms() + 1200;
        while (now_ms() < deadline) {
            if (!uart_read_line(line, sizeof(line), 100)) continue;
            process_stm32_line(line);
            if (strcmp(line, "OK PONG") != 0) continue;
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.alive = true;
            state.tx_gpio = STM32_TX_GPIO;
            state.rx_gpio = STM32_RX_GPIO;
            state.last_rx_ms = now_ms();
            xSemaphoreGive(state_mutex);
            ESP_LOGI(TAG, "STM32 fixed UART verified: TX GPIO%d RX GPIO%d",
                     STM32_TX_GPIO, STM32_RX_GPIO);
            xSemaphoreGive(uart_mutex);
            return true;
        }
    }
    uart_close();
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.alive = false;
    state.tx_gpio = -1;
    state.rx_gpio = -1;
    xSemaphoreGive(state_mutex);
    xSemaphoreGive(uart_mutex);
    return false;
}

static void set_tank_target(int left, int right, unsigned ramp_percent,
                            bool immediate_stop)
{
    if (left > 4095) left = 4095;
    if (left < -4095) left = -4095;
    if (right > 4095) right = 4095;
    if (right < -4095) right = -4095;
    if (ramp_percent < 10) ramp_percent = 10;
    if (ramp_percent > 300) ramp_percent = 300;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (!tank_mode_active) {
        tank_last_update_ms = now_ms();
        tank_last_send_ms = 0;
    }
    tank_target[0] = left;
    tank_target[1] = right;
    tank_ramp_percent = ramp_percent;
    tank_target_deadline_ms = now_ms() + TANK_TARGET_TIMEOUT_MS;
    tank_mode_active = true;
    if (immediate_stop) {
        tank_target[0] = tank_target[1] = 0;
        tank_output[0] = tank_output[1] = 0;
        tank_mode_active = false;
        tank_target_deadline_ms = 0;
        motor_lease_deadline_ms = 0;
        motor_stop_pending = true;
    }
    xSemaphoreGive(state_mutex);
}

static void set_controller_targets(int left, int right, unsigned ramp_percent,
                                   bool immediate_stop, int16_t x_rpm,
                                   int16_t y_rpm, int16_t z_rpm)
{
    static bool controller_stepper_active;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool imu_owns_drive = imu_calibration_owns_drive;
    bool autonomous_owns_drive = autonomous_sequence_owns_drive;
    xSemaphoreGive(state_mutex);
    if (imu_owns_drive || autonomous_owns_drive) {
        if (immediate_stop) {
            bno080_link_abort_calibration();
            set_tank_target(0, 0, 300, true);
        }
        return;
    }
    set_tank_target(left, right, ramp_percent, immediate_stop);
    if (immediate_stop) {
        if (controller_stepper_active) stepper_link_quick_stop();
        controller_stepper_active = false;
        return;
    }
    if (stepper_link_set_direct_rpm(x_rpm, y_rpm, z_rpm, 400)) {
        controller_stepper_active = true;
    } else {
        ESP_LOGD(TAG, "Controller stepper command ignored until GD32 direct link is ready");
    }
}

static bool imu_motion_begin(void)
{
    rover_link_status_t controller = {0};
    rover_link_get_status(&controller);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool allowed = state.alive && !controller.active && !imu_calibration_owns_drive;
    if (allowed) imu_calibration_owns_drive = true;
    xSemaphoreGive(state_mutex);
    if (allowed) set_tank_target(0, 0, 300, true);
    return allowed;
}

static bool imu_motion_drive(int left, int right, unsigned ramp_percent,
                             bool immediate_stop)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool owns_drive = imu_calibration_owns_drive && state.alive;
    xSemaphoreGive(state_mutex);
    if (owns_drive) set_tank_target(left, right, ramp_percent, immediate_stop);
    return owns_drive;
}

static void imu_motion_end(void)
{
    set_tank_target(0, 0, 300, true);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    imu_calibration_owns_drive = false;
    xSemaphoreGive(state_mutex);
}

bool drive_control_autonomous_begin(void)
{
    rover_link_status_t controller = {0};
    rover_link_get_status(&controller);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool allowed = state.alive && !controller.active &&
                   !imu_calibration_owns_drive &&
                   !autonomous_sequence_owns_drive;
    bool encoders_running = state.encoder_rate_hz != 0;
    if (allowed) autonomous_sequence_owns_drive = true;
    xSemaphoreGive(state_mutex);
    if (!allowed) return false;

    set_tank_target(0, 0, 300, true);
    if (!encoders_running &&
        uart_command("ENCON 50", "OK ENCON", UART_REPLY_MS)) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.encoder_rate_hz = 50;
        autonomous_sequence_started_encoders = true;
        xSemaphoreGive(state_mutex);
    } else if (!encoders_running) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        autonomous_sequence_owns_drive = false;
        xSemaphoreGive(state_mutex);
        return false;
    }
    return true;
}

bool drive_control_autonomous_set_percent(int left_percent, int right_percent)
{
    if (left_percent < -100 || left_percent > 100 ||
        right_percent < -100 || right_percent > 100) return false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool owns_drive = autonomous_sequence_owns_drive && state.alive;
    xSemaphoreGive(state_mutex);
    if (!owns_drive) return false;
    set_tank_target(left_percent * 4095 / 100,
                    right_percent * 4095 / 100, 200, false);
    return true;
}

bool drive_control_autonomous_get_encoders(uint16_t adc[4], uint32_t *sequence)
{
    if (!adc) return false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool valid = autonomous_sequence_owns_drive && state.alive &&
                 state.encoder_rate_hz != 0;
    for (int i = 0; i < 4; ++i) adc[i] = (uint16_t)state.encoders[i];
    if (sequence) *sequence = state.encoder_sequence;
    xSemaphoreGive(state_mutex);
    return valid;
}

void drive_control_autonomous_end(void)
{
    set_tank_target(0, 0, 300, true);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool stop_encoders = autonomous_sequence_started_encoders;
    autonomous_sequence_started_encoders = false;
    autonomous_sequence_owns_drive = false;
    xSemaphoreGive(state_mutex);
    if (stop_encoders && uart_command("ENCOFF", "OK ENCOFF", UART_REPLY_MS)) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.encoder_rate_hz = 0;
        xSemaphoreGive(state_mutex);
    }
}

static int ramp_value(int current, int target, int step)
{
    if ((current < 0 && target > 0) || (current > 0 && target < 0)) target = 0;
    if (current < target) return current + step > target ? target : current + step;
    if (current > target) return current - step < target ? target : current - step;
    return current;
}

static bool send_track_output(int motor, int signed_duty)
{
    if (signed_duty == 0) {
        char command[20];
        snprintf(command, sizeof(command), "MSTOP %d", motor);
        return uart_command(command, "OK MSTOP", UART_REPLY_MS);
    }
    // Both installed tracks use BTS7960 direction B for rover-forward.
    char direction = signed_duty > 0 ? 'B' : 'A';
    unsigned duty = (unsigned)(signed_duty > 0 ? signed_duty : -signed_duty);
    char command[40];
    snprintf(command, sizeof(command), "MOTOR %d %c %u", motor, direction, duty);
    return uart_command(command, "OK MOTOR", UART_REPLY_MS);
}

static void service_tank_ramp(void)
{
    int next[2] = {0};
    int previous[2] = {0};
    bool stop_now = false;
    bool refresh_outputs = false;
    int64_t now = now_ms();

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (motor_stop_pending) {
        motor_stop_pending = false;
        stop_now = true;
    } else if (tank_mode_active && now >= tank_target_deadline_ms) {
        tank_target[0] = tank_target[1] = 0;
        tank_output[0] = tank_output[1] = 0;
        tank_mode_active = false;
        tank_target_deadline_ms = 0;
        motor_lease_deadline_ms = 0;
        stop_now = true;
    } else if (tank_mode_active && now - tank_last_update_ms >= TANK_UPDATE_MS) {
        int elapsed = tank_last_update_ms ? (int)(now - tank_last_update_ms) : TANK_UPDATE_MS;
        if (elapsed > 200) elapsed = 200;
        int step = (4095 * (int)tank_ramp_percent * elapsed) / 100000;
        if (step < 1) step = 1;
        tank_last_update_ms = now;
        for (int i = 0; i < 2; ++i) {
            previous[i] = tank_output[i];
            tank_output[i] = ramp_value(tank_output[i], tank_target[i], step);
            next[i] = tank_output[i];
        }
        motor_lease_deadline_ms = now + MOTOR_LEASE_MS;
        refresh_outputs = now - tank_last_send_ms >= 250;
        if (refresh_outputs) tank_last_send_ms = now;
    }
    xSemaphoreGive(state_mutex);

    if (stop_now) {
        uart_command("MSTOP ALL", "OK MSTOP ALL", UART_REPLY_MS);
        return;
    }
    for (int i = 0; i < 2; ++i) {
        // tank_output[0] is the physical left track (Motor 2); slot 1 is right (Motor 1).
        const int motor = i == 0 ? 2 : 1;
        if (next[i] != previous[i] || refresh_outputs) send_track_output(motor, next[i]);
    }
}

static void uart_service_task(void *argument)
{
    (void)argument;
    int64_t next_status = 0;
    char line[LINE_CAPACITY];

    while (true) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool alive = state.alive;
        int64_t last_rx = state.last_rx_ms;
        xSemaphoreGive(state_mutex);

        if (!alive) {
            if (!uart_find_link()) {
                state_error("STM32 heartbeat missing on fixed TX21/RX22");
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            continue;
        }

        service_tank_ramp();

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool lease_expired = motor_lease_deadline_ms &&
                             now_ms() >= motor_lease_deadline_ms;
        if (lease_expired) {
            motor_lease_deadline_ms = 0;
        }
        xSemaphoreGive(state_mutex);
        if (lease_expired) {
            uart_command("MSTOP ALL", "OK MSTOP ALL", UART_REPLY_MS);
        }

        if (now_ms() >= next_status) {
            next_status = now_ms() + STATUS_INTERVAL_MS;
            uart_command("MSTATUS", "MSTAT ", UART_REPLY_MS);
        } else if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (uart_read_line(line, sizeof(line), 20)) {
                process_stm32_line(line);
            }
            xSemaphoreGive(uart_mutex);
        }

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        last_rx = state.last_rx_ms;
        xSemaphoreGive(state_mutex);
        if (now_ms() - last_rx > UART_LINK_TIMEOUT_MS) {
            ESP_LOGE(TAG, "STM32 link timed out");
            xSemaphoreTake(uart_mutex, portMAX_DELAY);
            uart_close();
            xSemaphoreGive(uart_mutex);
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.alive = false;
            state.tx_gpio = -1;
            state.rx_gpio = -1;
            state.motors[0].direction = 'S';
            state.motors[0].duty = 0;
            state.motors[1].direction = 'S';
            state.motors[1].duty = 0;
            xSemaphoreGive(state_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static const char index_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>STM32 Motor Test</title><style>"
"*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#17202a;font-family:Arial,sans-serif}"
"header{position:sticky;top:0;z-index:2;background:#17212b;color:#fff;border-bottom:3px solid #e2a400;padding:10px 12px}"
".top,main{max-width:760px;margin:auto}.top{display:flex;justify-content:space-between;align-items:center;gap:10px}"
"h1{font-size:18px;margin:0 0 4px}.link{font-size:13px;color:#ff9189}.link.ok{color:#72dfa7}.nav{font-size:13px;margin-top:5px}.nav a{color:#fff;margin-right:14px}"
"button{min-height:48px;border:1px solid #aab5bf;border-radius:6px;background:#e7ecf0;font-size:15px;font-weight:700;padding:8px;touch-action:manipulation}"
".estop{background:#c5251c;color:#fff;border:0;min-width:110px}main{padding:12px 12px 36px}"
".motor{background:#fff;border:1px solid #cbd3da;border-radius:8px;padding:12px;margin-bottom:14px}"
".head,.speed{display:flex;justify-content:space-between;align-items:center}.head h2{font-size:18px;margin:0}.state{font-size:13px;background:#edf1f4;padding:6px 8px;border-radius:5px}"
".sense,.enc{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;margin:10px 0}.reading{border:1px solid #d3dbe2;border-left:4px solid #15806a;padding:8px;border-radius:5px;min-width:0;overflow-wrap:anywhere}"
".reading small{display:block;color:#596673}.reading strong{font-size:18px}.speed{font-size:13px;font-weight:700;margin-top:12px}"
"input[type=range]{width:100%;height:42px;accent-color:#176ab0}.drive{display:grid;grid-template-columns:1fr 86px 1fr;gap:8px}"
".a{background:#176daf;color:#fff;border-color:#176daf}.b{background:#55772c;color:#fff;border-color:#55772c}.stop{background:#c5251c;color:#fff;border-color:#c5251c}"
".panel{background:#fff;border:1px solid #cbd3da;border-radius:8px;padding:12px}.actions{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}"
".power{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-bottom:14px}.power .reading{background:#fff}.estimate{font-size:11px;color:#68747d;margin:7px 0 0}"
".enc{grid-template-columns:repeat(4,1fr)}.log{font:12px monospace;white-space:pre-wrap;word-break:break-word;margin-top:10px;color:#4c5965}"
"@media(max-width:600px){.top{display:grid;grid-template-columns:minmax(0,1fr) auto}.top>div{min-width:0}.nav{display:flex;flex-wrap:wrap;gap:4px 12px}.nav a{margin:0}.drive{grid-template-columns:minmax(0,1fr) 74px minmax(0,1fr)}.actions{grid-template-columns:1fr}.enc,.power{grid-template-columns:repeat(2,minmax(0,1fr))}.power .reading:last-child{grid-column:1/-1}}"
"</style></head><body><header><div class=top><div><h1>STM32 Dual Motor Test</h1><div id=link class=link>Connecting...</div><div class=nav><a href='/mobile'>Tank drive</a><a href='/steppers'>Steppers</a><a href='/odrive'>ODrive</a><a href='/mower-logs'>Mower Logs</a><a href='/sensors'>Sensors</a><a href='/battery'>Battery</a><a href='/speaker'>Speaker</a><a href='/mic'>Mic</a><a href='/wifi'>Wi-Fi</a><a href='/update'>Firmware</a></div></div>"
"<button class=estop onclick=stopAll()>STOP ALL</button></div></header><main>"
"<section class=power><div class=reading><small>Drivetrain current (I)</small><strong id=totalI>0.00 A</strong></div><div class=reading><small>Drivetrain power (P)</small><strong id=totalP>0.0 W</strong></div><div class=reading><small>Motor bus</small><strong id=busV>8.0 V</strong></div></section>"
"<div id=motors></div><section class=panel><h2>Encoder ADC</h2><div class=enc>"
"<div class=reading><small>PA0</small><strong id=e0>0</strong></div><div class=reading><small>PA1</small><strong id=e1>0</strong></div>"
"<div class=reading><small>PA2</small><strong id=e2>0</strong></div><div class=reading><small>PA3</small><strong id=e3>0</strong></div></div>"
"<div class=actions><button onclick=enc('read')>Read once</button><button onclick=enc('start')>Start 20 Hz</button><button onclick=enc('stop')>Stop stream</button></div>"
"<div id=log class=log>Waiting for status...</div></section></main><script>"
"const motorHtml=n=>`<section class=motor><div class=head><h2>Motor ${n}</h2><div id=s${n} class=state>Stopped</div></div>"
"<div class=sense><div class=reading><small>R_IS</small><strong id=a${n}>0 mV</strong><small id=ar${n}>raw 0</small></div>"
"<div class=reading><small>L_IS</small><strong id=b${n}>0 mV</strong><small id=br${n}>raw 0</small></div></div>"
"<div class=sense><div class=reading><small>Motor current (I)</small><strong id=i${n}>0.00 A</strong></div><div class=reading><small>Motor power (P)</small><strong id=p${n}>0.0 W</strong></div></div><p class=estimate>Estimated from averaged IS voltage, nominal kILIS, a 1 kOhm sense resistor, and the 8 V motor bus.</p>"
"<div class=speed><span>Duty</span><output id=o${n}>1024 / 4095</output></div><input id=d${n} type=range min=0 max=4095 value=1024 oninput=show(${n})>"
"<div class=drive><button class=a id=aBtn${n}>Hold A</button><button class=stop onclick=stopMotor(${n})>STOP</button><button class=b id=bBtn${n}>Hold B</button></div></section>`;"
"document.getElementById('motors').innerHTML=motorHtml(1)+motorHtml(2);"
"function show(n){document.getElementById('o'+n).textContent=document.getElementById('d'+n).value+' / 4095'}"
"function api(path){return fetch(path,{cache:'no-store'}).then(r=>r.json()).then(render).catch(e=>document.getElementById('log').textContent='Request failed: '+e)}"
"function hold(n,d){let timer=null;const send=()=>api('/api/motor?m='+n+'&d='+d+'&duty='+document.getElementById('d'+n).value);"
"const start=e=>{e.preventDefault();send();timer=setInterval(send,300)};const stop=e=>{e.preventDefault();if(timer){clearInterval(timer);timer=null}stopMotor(n)};"
"let b=document.getElementById(d.toLowerCase()+'Btn'+n);b.addEventListener('pointerdown',start);b.addEventListener('pointerup',stop);b.addEventListener('pointercancel',stop);b.addEventListener('pointerleave',stop)}"
"function stopMotor(n){api('/api/stop?m='+n)}function stopAll(){api('/api/stop?m=all')}function enc(c){api('/api/encoder?cmd='+c+'&hz=20')}"
"function render(s){let l=document.getElementById('link');l.textContent=s.alive?'STM32 alive | TX '+s.tx+' RX '+s.rx:'STM32 heartbeat missing';l.className='link '+(s.alive?'ok':'');"
"s.motors.forEach((m,i)=>{let n=i+1;document.getElementById('s'+n).textContent=m.direction==='S'?'Stopped':'Direction '+m.direction+' | '+Math.round(m.duty*100/4095)+'%';"
"document.getElementById('a'+n).textContent=m.a_mv+' mV';document.getElementById('b'+n).textContent=m.b_mv+' mV';document.getElementById('ar'+n).textContent='raw '+m.a_raw;document.getElementById('br'+n).textContent='raw '+m.b_raw;document.getElementById('i'+n).textContent=m.current_a.toFixed(2)+' A';document.getElementById('p'+n).textContent=m.power_w.toFixed(1)+' W'});document.getElementById('totalI').textContent=s.drivetrain.current_a.toFixed(2)+' A';document.getElementById('totalP').textContent=s.drivetrain.power_w.toFixed(1)+' W';document.getElementById('busV').textContent=s.drivetrain.bus_v.toFixed(1)+' V';"
"s.encoders.forEach((v,i)=>document.getElementById('e'+i).textContent=v);let c=s.controller;document.getElementById('log').textContent='CONTROLLER BLACK BOX\\nactive '+c.active+' | age '+c.age_ms+' ms | datagrams '+c.datagrams+' | valid '+c.valid+' | rejected '+c.rejected+'\\nwrong size '+c.wrong_size+' | auth '+c.auth_rejects+' | range '+c.range_rejects+' | stale '+c.stale+' | ACK '+c.acks+'/'+c.ack_failures+'\\n\\nSTM32\\nAP: '+location.host+' | encoder '+s.encoder_hz+' Hz | watchdog '+s.watchdog+'\\nlast: '+s.last_reply+'\\nerror: '+s.last_error}"
"hold(1,'A');hold(1,'B');hold(2,'A');hold(2,'B');setInterval(()=>api('/api/status'),700);api('/api/status');"
"</script></body></html>";

static const char mobile_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'>"
"<title>Rover Drive</title><style>"
"*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#10161b;color:#eef3f6;font-family:Arial,sans-serif;touch-action:manipulation}"
"header{position:sticky;top:0;z-index:3;background:#17212b;border-bottom:3px solid #e2a400;padding:10px 12px}.top,main{max-width:620px;margin:auto}"
".top{display:flex;align-items:center;justify-content:space-between;gap:10px}h1{font-size:18px;margin:0}.link{font-size:12px;color:#ff9189}.ok{color:#72dfa7}"
"button{border:1px solid #65727d;border-radius:7px;background:#26333d;color:#fff;font-weight:750;font-size:16px;min-height:58px;touch-action:none}.stop{background:#c5251c;border-color:#c5251c;padding:8px 16px}"
"main{padding:12px}.panel{background:#1b242b;border:1px solid #35434d;border-radius:8px;padding:12px;margin-bottom:12px}"
"#joy{position:relative;width:min(84vw,360px);aspect-ratio:1;margin:auto;border-radius:50%;background:#0c1115;border:2px solid #65727d;touch-action:none}"
"#joy:before,#joy:after{content:'';position:absolute;background:#35434d}#joy:before{left:50%;top:7%;bottom:7%;width:1px}#joy:after{top:50%;left:7%;right:7%;height:1px}"
"#knob{position:absolute;width:76px;height:76px;border-radius:50%;background:#2380c3;border:3px solid #9bd2f5;left:50%;top:50%;transform:translate(-50%,-50%)}"
".readout{text-align:center;font:13px monospace;margin-top:8px}.pad{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,64px);gap:7px}.f{grid-column:2}.l{grid-row:2;grid-column:1}.r{grid-row:2;grid-column:3}.b{grid-row:3;grid-column:2}"
"label{display:grid;gap:6px;font-size:13px;font-weight:700}input{width:100%;height:42px}.note{font-size:12px;color:#b9c4cb;margin:8px 0 0}a{color:#8fcfff}"
"</style></head><body><header><div class=top><div><h1>Rover Tank Drive</h1><div id=link class=link>Connecting...</div></div><button class=stop onclick=estop()>STOP</button></div></header><main>"
"<section class=panel><div id=joy><div id=knob></div></div><div id=out class=readout>left 0% | right 0%</div></section>"
"<section class='panel pad'><button class=f data-l=4095 data-r=4095>Forward</button><button class=l data-l=-4095 data-r=4095>Left</button><button class=r data-l=4095 data-r=-4095>Right</button><button class=b data-l=-4095 data-r=-4095>Backward</button></section>"
"<section class=panel><label>Ramp rate <output id=rampOut>80% per second</output><input id=ramp type=range min=10 max=300 value=80 oninput=showRamp()></label>"
"<p class=note>Direction B is forward on both installed tracks. Releasing a control ramps to zero; STOP is immediate.</p><p><a href='/'>Motor diagnostics</a> | <a href='/speaker'>Speaker</a></p></section>"
"</main><script>let active=false,lastSend=0,left=0,right=0;const joy=document.getElementById('joy'),knob=document.getElementById('knob');"
"function ramp(){return document.getElementById('ramp').value}function showRamp(){document.getElementById('rampOut').textContent=ramp()+'% per second'}"
"function command(l,r,force=false){left=Math.round(l);right=Math.round(r);let now=Date.now();if(!force&&now-lastSend<70)return;lastSend=now;document.getElementById('out').textContent='left '+Math.round(left*100/4095)+'% | right '+Math.round(right*100/4095)+'%';fetch('/api/tank?left='+left+'&right='+right+'&ramp='+ramp(),{cache:'no-store'}).then(x=>x.json()).then(render).catch(()=>{})}"
"function move(e){if(!active)return;let q=joy.getBoundingClientRect(),x=(e.clientX-q.left-q.width/2)/(q.width*.42),y=(e.clientY-q.top-q.height/2)/(q.height*.42),m=Math.hypot(x,y);if(m>1){x/=m;y/=m}knob.style.left=(50+x*42)+'%';knob.style.top=(50+y*42)+'%';let f=-y,t=x,l=f+t,r=f-t,z=Math.max(1,Math.abs(l),Math.abs(r));command(l/z*4095,r/z*4095)}"
"function release(){if(!active)return;active=false;knob.style.left='50%';knob.style.top='50%';command(0,0,true)}"
"joy.onpointerdown=e=>{active=true;joy.setPointerCapture(e.pointerId);move(e)};joy.onpointermove=move;joy.onpointerup=release;joy.onpointercancel=release;"
"document.querySelectorAll('.pad button').forEach(b=>{let go=()=>command(0,0,true);b.onpointerdown=e=>{e.preventDefault();b.setPointerCapture(e.pointerId);command(+b.dataset.l,+b.dataset.r,true)};b.onpointerup=go;b.onpointercancel=go;b.onpointerleave=e=>{if(e.buttons)go()}});"
"function estop(){fetch('/api/stop?m=all',{cache:'no-store'}).then(x=>x.json()).then(render)}function render(s){let e=document.getElementById('link');e.textContent=s.alive?(s.controller.active?'Handheld active | STM32 alive':'STM32 alive'):'STM32 heartbeat missing';e.className='link '+(s.alive?'ok':'')}"
"setInterval(()=>{if(active)command(left,right,true);else fetch('/api/status',{cache:'no-store'}).then(x=>x.json()).then(render).catch(()=>{})},150);showRamp();</script></body></html>";

static void json_escape(char *output, size_t output_size, const char *input)
{
    size_t used = 0;
    for (size_t i = 0; input[i] && used + 2 < output_size; ++i) {
        char c = input[i];
        if (c == '"' || c == '\\') {
            output[used++] = '\\';
        }
        output[used++] = (c == '\n' || c == '\r') ? ' ' : c;
    }
    output[used] = '\0';
}

static float estimated_motor_current_a(const motor_state_t *motor)
{
    if (motor->direction == 'S' || motor->duty == 0) return 0.0f;
    unsigned sense_mv = motor->current_a_mv > motor->current_b_mv
                            ? motor->current_a_mv
                            : motor->current_b_mv;
    return ((float)sense_mv / 1000.0f) * BTS7960_SENSE_AMPS_PER_VOLT;
}

static esp_err_t send_status_json(httpd_req_t *request)
{
    robot_state_t snapshot;
    rover_link_status_t controller = {0};
    char reply[260];
    char error[260];
    char body[1900];
    int tank_snapshot[4];
    unsigned ramp_snapshot;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshot = state;
    tank_snapshot[0] = tank_target[0];
    tank_snapshot[1] = tank_target[1];
    tank_snapshot[2] = tank_output[0];
    tank_snapshot[3] = tank_output[1];
    ramp_snapshot = tank_ramp_percent;
    xSemaphoreGive(state_mutex);
    json_escape(reply, sizeof(reply), snapshot.last_reply);
    json_escape(error, sizeof(error), snapshot.last_error);
    rover_link_get_status(&controller);
    float motor_current[2] = {
        estimated_motor_current_a(&snapshot.motors[0]),
        estimated_motor_current_a(&snapshot.motors[1]),
    };
    float motor_power[2] = {
        motor_current[0] * DRIVE_BUS_VOLTAGE,
        motor_current[1] * DRIVE_BUS_VOLTAGE,
    };
    float drivetrain_current = motor_current[0] + motor_current[1];
    float drivetrain_power = motor_power[0] + motor_power[1];

    snprintf(body, sizeof(body),
             "{\"alive\":%s,\"tx\":%d,\"rx\":%d,\"heartbeat_ms\":%u,"
             "\"watchdog\":%u,\"last_reply\":\"%s\",\"last_error\":\"%s\","
             "\"encoder_hz\":%u,\"encoders\":[%u,%u,%u,%u],"
             "\"controller\":{\"active\":%s,\"age_ms\":%lld,\"sequence\":%lu,"
             "\"right_x\":%u,\"right_y\":%u,\"buttons\":%u,"
             "\"stepper_rpm\":[%d,%d,%d],\"valid\":%lu,\"rejected\":%lu,"
             "\"datagrams\":%lu,\"wrong_size\":%lu,\"auth_rejects\":%lu,"
             "\"range_rejects\":%lu,\"stale\":%lu,\"acks\":%lu,\"ack_failures\":%lu},"
             "\"drivetrain\":{\"bus_v\":%.2f,\"current_a\":%.4f,\"power_w\":%.3f},"
             "\"tank\":{\"left_target\":%d,\"right_target\":%d,\"left_output\":%d,\"right_output\":%d,\"ramp\":%u},\"motors\":["
             "{\"direction\":\"%c\",\"duty\":%u,\"a_raw\":%u,\"b_raw\":%u,\"a_mv\":%u,\"b_mv\":%u,\"current_a\":%.4f,\"power_w\":%.3f},"
             "{\"direction\":\"%c\",\"duty\":%u,\"a_raw\":%u,\"b_raw\":%u,\"a_mv\":%u,\"b_mv\":%u,\"current_a\":%.4f,\"power_w\":%.3f}]}",
             snapshot.alive ? "true" : "false", snapshot.tx_gpio,
             snapshot.rx_gpio, snapshot.heartbeat_ms, snapshot.watchdog,
             reply, error, snapshot.encoder_rate_hz, snapshot.encoders[0],
             snapshot.encoders[1], snapshot.encoders[2], snapshot.encoders[3],
             controller.active ? "true" : "false", (long long)controller.age_ms,
             (unsigned long)controller.sequence, controller.right_x,
             controller.right_y, controller.button_mask,
             controller.stepper_rpm[0], controller.stepper_rpm[1],
             controller.stepper_rpm[2],
             (unsigned long)controller.valid_packets,
             (unsigned long)controller.rejected_packets,
             (unsigned long)controller.datagrams,
             (unsigned long)controller.wrong_size_packets,
             (unsigned long)controller.auth_rejects,
             (unsigned long)controller.range_rejects,
             (unsigned long)controller.stale_packets,
             (unsigned long)controller.acks_sent,
             (unsigned long)controller.ack_failures,
             DRIVE_BUS_VOLTAGE, drivetrain_current, drivetrain_power,
             tank_snapshot[0], tank_snapshot[1], tank_snapshot[2], tank_snapshot[3],
             ramp_snapshot,
             snapshot.motors[0].direction, snapshot.motors[0].duty,
             snapshot.motors[0].current_a_raw, snapshot.motors[0].current_b_raw,
             snapshot.motors[0].current_a_mv, snapshot.motors[0].current_b_mv,
             motor_current[0], motor_power[0],
             snapshot.motors[1].direction, snapshot.motors[1].duty,
             snapshot.motors[1].current_a_raw, snapshot.motors[1].current_b_raw,
             snapshot.motors[1].current_a_mv, snapshot.motors[1].current_b_mv,
             motor_current[1], motor_power[1]);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, body);
}

static int query_int(httpd_req_t *request, const char *key, int fallback)
{
    char query[160];
    char value[24];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) {
        return fallback;
    }
    return atoi(value);
}

static bool query_string(httpd_req_t *request, const char *key,
                         char *output, size_t output_size)
{
    char query[160];
    return httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK &&
           httpd_query_key_value(query, key, output, output_size) == ESP_OK;
}

static esp_err_t root_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t mobile_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, mobile_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    return send_status_json(request);
}

static esp_err_t motor_handler(httpd_req_t *request)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool imu_owns_drive = imu_calibration_owns_drive;
    bool autonomous_owns_drive = autonomous_sequence_owns_drive;
    xSemaphoreGive(state_mutex);
    if (imu_owns_drive) {
        return send_conflict(request, "IMU forward calibration owns drivetrain");
    }
    if (autonomous_owns_drive) {
        return send_conflict(request, "Autonomous sequence owns drivetrain");
    }
    int motor = query_int(request, "m", 0);
    int duty = query_int(request, "duty", 0);
    char direction[4] = {0};
    if (motor < 1 || motor > 2 || duty < 0 || duty > 4095 ||
        !query_string(request, "d", direction, sizeof(direction)) ||
        !((direction[0] == 'A' || direction[0] == 'B') && direction[1] == '\0')) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad motor command");
        return ESP_FAIL;
    }
    char command[40];
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    tank_mode_active = false;
    tank_target[0] = tank_target[1] = 0;
    tank_output[0] = tank_output[1] = 0;
    xSemaphoreGive(state_mutex);
    snprintf(command, sizeof(command), "MOTOR %d %c %d", motor, direction[0], duty);
    if (uart_command(command, "OK MOTOR", UART_REPLY_MS)) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        motor_lease_deadline_ms = now_ms() + MOTOR_LEASE_MS;
        xSemaphoreGive(state_mutex);
    }
    return send_status_json(request);
}

static esp_err_t tank_handler(httpd_req_t *request)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool imu_owns_drive = imu_calibration_owns_drive;
    bool autonomous_owns_drive = autonomous_sequence_owns_drive;
    xSemaphoreGive(state_mutex);
    if (imu_owns_drive) {
        return send_conflict(request, "IMU forward calibration owns drivetrain");
    }
    if (autonomous_owns_drive) {
        return send_conflict(request, "Autonomous sequence owns drivetrain");
    }
    int left = query_int(request, "left", 0);
    int right = query_int(request, "right", 0);
    int ramp = query_int(request, "ramp", TANK_DEFAULT_RAMP_PERCENT);
    if (left < -4095 || left > 4095 || right < -4095 || right > 4095 ||
        ramp < 10 || ramp > 300) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad tank target");
        return ESP_FAIL;
    }
    set_tank_target(left, right, (unsigned)ramp, false);
    return send_status_json(request);
}

static esp_err_t stop_handler(httpd_req_t *request)
{
    bno080_link_abort_calibration();
    char motor[8] = {0};
    query_string(request, "m", motor, sizeof(motor));
    char command[24];
    const char *expected;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool was_tank_mode = tank_mode_active;
    if (was_tank_mode) {
        tank_mode_active = false;
        tank_target[0] = tank_target[1] = 0;
        tank_output[0] = tank_output[1] = 0;
        tank_target_deadline_ms = 0;
        motor_lease_deadline_ms = 0;
    }
    xSemaphoreGive(state_mutex);
    if (was_tank_mode) {
        uart_command("MSTOP ALL", "OK MSTOP ALL", UART_REPLY_MS);
        return send_status_json(request);
    }
    if (strcmp(motor, "all") == 0) {
        snprintf(command, sizeof(command), "MSTOP ALL");
        expected = "OK MSTOP ALL";
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        motor_lease_deadline_ms = 0;
        tank_mode_active = false;
        tank_target[0] = tank_target[1] = 0;
        tank_output[0] = tank_output[1] = 0;
        tank_target_deadline_ms = 0;
        xSemaphoreGive(state_mutex);
    } else {
        int number = atoi(motor);
        if (number < 1 || number > 2) {
            httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad motor");
            return ESP_FAIL;
        }
        snprintf(command, sizeof(command), "MSTOP %d", number);
        expected = "OK MSTOP";
    }
    uart_command(command, expected, UART_REPLY_MS);
    return send_status_json(request);
}

static esp_err_t encoder_handler(httpd_req_t *request)
{
    char action[12] = {0};
    if (!query_string(request, "cmd", action, sizeof(action))) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing command");
        return ESP_FAIL;
    }
    if (strcmp(action, "start") == 0) {
        int hz = query_int(request, "hz", 20);
        if (!(hz == 10 || hz == 20 || hz == 25 || hz == 50 || hz == 100)) {
            hz = 20;
        }
        char command[24];
        snprintf(command, sizeof(command), "ENCON %d", hz);
        if (uart_command(command, "OK ENCON", UART_REPLY_MS)) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.encoder_rate_hz = (unsigned)hz;
            xSemaphoreGive(state_mutex);
        }
    } else if (strcmp(action, "stop") == 0) {
        if (uart_command("ENCOFF", "OK ENCOFF", UART_REPLY_MS)) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.encoder_rate_hz = 0;
            xSemaphoreGive(state_mutex);
        }
    } else if (strcmp(action, "read") == 0) {
        uart_command("ENCREAD", "ENC ", UART_REPLY_MS);
    } else {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "bad command");
        return ESP_FAIL;
    }
    return send_status_json(request);
}

static void ota_safe_stop(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    motor_lease_deadline_ms = 0;
    tank_mode_active = false;
    tank_target[0] = tank_target[1] = 0;
    tank_output[0] = tank_output[1] = 0;
    xSemaphoreGive(state_mutex);
    uart_command("MSTOP ALL", "OK MSTOP ALL", UART_REPLY_MS);
    stepper_link_quick_stop();
    odesc_link_stop_all();
}

static httpd_handle_t start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.max_open_sockets = 12;
    config.max_uri_handlers = 80;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        return NULL;
    }
    const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root_handler},
        {.uri = "/mobile", .method = HTTP_GET, .handler = mobile_handler},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/motor", .method = HTTP_GET, .handler = motor_handler},
        {.uri = "/api/tank", .method = HTTP_GET, .handler = tank_handler},
        {.uri = "/api/stop", .method = HTTP_GET, .handler = stop_handler},
        {.uri = "/api/encoder", .method = HTTP_GET, .handler = encoder_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }
    ESP_ERROR_CHECK(network_ota_register_routes(server, ota_safe_stop));
    ESP_ERROR_CHECK(stepper_link_register_routes(server));
    ESP_ERROR_CHECK(odesc_link_register_routes(server));
    ESP_ERROR_CHECK(speaker_register_routes(server));
    ESP_ERROR_CHECK(battery_history_register_routes(server));
    ESP_ERROR_CHECK(mower_history_register_routes(server));
    ESP_ERROR_CHECK(bno080_link_register_routes(server));
    return server;
}

void app_main(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);

    state_mutex = xSemaphoreCreateMutex();
    uart_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(state_mutex && uart_mutex ? ESP_OK : ESP_ERR_NO_MEM);

    xTaskCreate(uart_service_task, "stm32_uart", 6144, NULL, 8, NULL);
    ESP_ERROR_CHECK(stepper_link_start());
    ESP_ERROR_CHECK(odesc_link_start());
    ESP_ERROR_CHECK(network_ota_start());
    result = speaker_start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Speaker unavailable: %s", esp_err_to_name(result));
    }
    result = battery_history_start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Battery history unavailable: %s", esp_err_to_name(result));
    }
    result = mower_history_start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Mower history unavailable: %s", esp_err_to_name(result));
    }
    result = bno080_link_start(imu_motion_begin, imu_motion_drive, imu_motion_end);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "BNO080 unavailable: %s", esp_err_to_name(result));
    }
    ESP_ERROR_CHECK(rover_link_start(set_controller_targets));
    ESP_ERROR_CHECK(start_web_server() ? ESP_OK : ESP_FAIL);
    network_ota_mark_running_app_valid();
}
