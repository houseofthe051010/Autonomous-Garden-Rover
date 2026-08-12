#include "bno080_link.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_http_server.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define BNO_UART UART_NUM_4
#define BNO_BAUD 3000000
#define BNO_PIN_A GPIO_NUM_5
#define BNO_PIN_B GPIO_NUM_6
#define BNO_RX_BUFFER 4096
#define BNO_FRAME_MAX 384
#define BNO_PASSIVE_PROBE_MS 1400
#define BNO_RETRY_MS 4000
#define BNO_REPORT_TIMEOUT_MS 2500
#define CALIBRATION_NAMESPACE "imu_cal"
#define CALIBRATION_KEY "mount_v1"
#define CALIBRATION_MAGIC 0x494d5531u
#define LEVEL_SAMPLE_MS 1200
#define FORWARD_COUNTDOWN_MS 3000
#define FORWARD_RUN_MS 3000
#define FORWARD_SAMPLE_START_MS 100
#define FORWARD_SAMPLE_END_MS 650
#define FORWARD_RAMP_PERCENT_PER_SECOND 100
#define FORWARD_LOG_CAPACITY 160

#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_INPUT_NORMAL 3
#define CHANNEL_INPUT_WAKE 4

#define REPORT_ACCELEROMETER 0x01
#define REPORT_GYROSCOPE 0x02
#define REPORT_MAGNETOMETER 0x03
#define REPORT_LINEAR_ACCELERATION 0x04
#define REPORT_ROTATION_VECTOR 0x05
#define REPORT_BASE_TIMESTAMP 0xfb
#define SET_FEATURE_COMMAND 0xfd
#define COMMAND_REQUEST 0xf2
#define ME_CALIBRATE_COMMAND 0x07

static const char *TAG = "bno080";

typedef struct {
    bool uart_ready;
    bool orientation_found;
    bool round_trip;
    bool connected;
    int tx_gpio;
    int rx_gpio;
    uint16_t host_buffer;
    uint64_t frames;
    uint64_t reports;
    int64_t last_frame_ms;
    float heading_deg;
    float roll_deg;
    float pitch_deg;
    float corrected_roll_deg;
    float corrected_pitch_deg;
    float corrected_heading_deg;
    float quaternion[4];
    float magnetic_ut[3];
    float gyro_rad_s[3];
    float acceleration_m_s2[3];
    float linear_acceleration_m_s2[3];
    uint8_t accuracy[6];
    bool have_rotation;
    bool have_magnetic;
    bool have_gyro;
    bool have_acceleration;
    bool have_linear;
    bool level_calibrated;
    bool heading_calibrated;
    float level_roll_offset_deg;
    float level_pitch_offset_deg;
    float heading_offset_deg;
    bool calibration_active;
    bool calibration_cancel;
    unsigned calibration_progress;
    char calibration_phase[64];
    char calibration_error[160];
    char phase[64];
    char error[160];
} bno_state_t;

typedef struct {
    bool in_frame;
    bool escaped;
    uint8_t data[BNO_FRAME_MAX];
    size_t length;
} frame_reader_t;

typedef struct {
    uint32_t magic;
    float level_roll_offset_deg;
    float level_pitch_offset_deg;
    float heading_offset_deg;
    uint8_t level_calibrated;
    uint8_t heading_calibrated;
    uint8_t reserved[2];
} calibration_blob_t;

typedef struct {
    uint16_t elapsed_ms;
    uint8_t accepted;
    float heading_deg;
    float roll_deg;
    float pitch_deg;
    float acceleration[3];
    float linear[3];
    float gyro[3];
    float magnetic[3];
} forward_log_sample_t;

static bno_state_t state = {.tx_gpio = -1, .rx_gpio = -1};
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t uart_mutex;
static uint8_t tx_sequence[6];
static uint8_t command_sequence;
static bno080_motion_begin_fn motion_begin;
static bno080_motion_drive_fn motion_drive;
static bno080_motion_end_fn motion_end;
static forward_log_sample_t forward_log[FORWARD_LOG_CAPACITY];
static unsigned forward_log_count;
static unsigned forward_log_accepted;
static unsigned forward_log_attempt;
static float forward_log_vector_x;
static float forward_log_vector_y;
static float forward_log_strength;
static float forward_log_offset_deg;
static float forward_log_baseline_x;
static float forward_log_baseline_y;
static char forward_log_result[96] = "no forward calibration recorded";

static float wrap_degrees(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float wrap_signed_degrees(float angle)
{
    angle = wrap_degrees(angle);
    return angle > 180.0f ? angle - 360.0f : angle;
}

static void quaternion_euler(const float q[4], float *roll, float *pitch,
                             float *yaw)
{
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    *roll = atan2f(2.0f * (w * x + y * z),
                   1.0f - 2.0f * (x * x + y * y)) * (180.0f / (float)M_PI);
    float sin_pitch = 2.0f * (w * y - z * x);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    *pitch = asinf(sin_pitch) * (180.0f / (float)M_PI);
    *yaw = wrap_degrees(atan2f(2.0f * (w * z + x * y),
                               1.0f - 2.0f * (y * y + z * z)) *
                        (180.0f / (float)M_PI));
}

static void save_calibration(void)
{
    calibration_blob_t blob = {.magic = CALIBRATION_MAGIC};
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    blob.level_roll_offset_deg = state.level_roll_offset_deg;
    blob.level_pitch_offset_deg = state.level_pitch_offset_deg;
    blob.heading_offset_deg = state.heading_offset_deg;
    blob.level_calibrated = state.level_calibrated;
    blob.heading_calibrated = state.heading_calibrated;
    xSemaphoreGive(state_mutex);
    nvs_handle_t handle;
    if (nvs_open(CALIBRATION_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        if (nvs_set_blob(handle, CALIBRATION_KEY, &blob, sizeof(blob)) == ESP_OK) {
            nvs_commit(handle);
        }
        nvs_close(handle);
    }
}

static void load_calibration(void)
{
    calibration_blob_t blob = {0};
    size_t size = sizeof(blob);
    nvs_handle_t handle;
    if (nvs_open(CALIBRATION_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return;
    esp_err_t result = nvs_get_blob(handle, CALIBRATION_KEY, &blob, &size);
    nvs_close(handle);
    if (result != ESP_OK || size != sizeof(blob) || blob.magic != CALIBRATION_MAGIC) return;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.level_roll_offset_deg = blob.level_roll_offset_deg;
    state.level_pitch_offset_deg = blob.level_pitch_offset_deg;
    state.heading_offset_deg = blob.heading_offset_deg;
    state.level_calibrated = blob.level_calibrated != 0;
    state.heading_calibrated = blob.heading_calibrated != 0;
    xSemaphoreGive(state_mutex);
}

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

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16(const uint8_t *data)
{
    return (int16_t)read_u16(data);
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static void set_phase(const char *phase, const char *error)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.phase, sizeof(state.phase), "%s", phase ? phase : "");
    snprintf(state.error, sizeof(state.error), "%s", error ? error : "");
    xSemaphoreGive(state_mutex);
}

static void clear_measurements(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.connected = false;
    state.have_rotation = false;
    state.have_magnetic = false;
    state.have_gyro = false;
    state.have_acceleration = false;
    state.have_linear = false;
    xSemaphoreGive(state_mutex);
}

static bool frame_reader_push(frame_reader_t *reader, uint8_t value,
                              uint8_t *frame, size_t *frame_length)
{
    if (value == 0x7e) {
        if (reader->in_frame && reader->length > 0) {
            memcpy(frame, reader->data, reader->length);
            *frame_length = reader->length;
            reader->length = 0;
            reader->escaped = false;
            return true;
        }
        reader->in_frame = true;
        reader->escaped = false;
        reader->length = 0;
        return false;
    }
    if (!reader->in_frame) return false;
    if (reader->escaped) {
        value ^= 0x20;
        reader->escaped = false;
    } else if (value == 0x7d) {
        reader->escaped = true;
        return false;
    }
    if (reader->length < sizeof(reader->data)) {
        reader->data[reader->length++] = value;
    } else {
        reader->in_frame = false;
        reader->length = 0;
        reader->escaped = false;
    }
    return false;
}

static bool read_frame(frame_reader_t *reader, uint8_t *frame,
                       size_t *frame_length, uint32_t timeout_ms)
{
    const int64_t deadline = now_ms() + timeout_ms;
    uint8_t byte;
    while (now_ms() < deadline) {
        int remaining = (int)(deadline - now_ms());
        TickType_t wait = pdMS_TO_TICKS(remaining > 20 ? 20 : remaining);
        if (uart_read_bytes(BNO_UART, &byte, 1, wait) == 1 &&
            frame_reader_push(reader, byte, frame, frame_length)) {
            return true;
        }
    }
    return false;
}

static void parse_sensor_report(const uint8_t *report, size_t length)
{
    const uint8_t id = report[0];
    int count = 0;
    float scale = 1.0f;
    float values[4] = {0};
    if (id == REPORT_ROTATION_VECTOR && length >= 14) {
        count = 4;
        scale = 1.0f / 16384.0f;
    } else if (id == REPORT_GYROSCOPE && length >= 10) {
        count = 3;
        scale = 1.0f / 512.0f;
    } else if ((id == REPORT_ACCELEROMETER || id == REPORT_LINEAR_ACCELERATION) &&
               length >= 10) {
        count = 3;
        scale = 1.0f / 256.0f;
    } else if (id == REPORT_MAGNETOMETER && length >= 10) {
        count = 3;
        scale = 1.0f / 16.0f;
    } else {
        return;
    }
    for (int i = 0; i < count; ++i) {
        values[i] = (float)read_i16(report + 4 + i * 2) * scale;
    }

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.accuracy[id] = report[2] & 0x03;
    if (id == REPORT_ROTATION_VECTOR) {
        memcpy(state.quaternion, values, sizeof(state.quaternion));
        quaternion_euler(values, &state.roll_deg, &state.pitch_deg,
                         &state.heading_deg);
        state.corrected_roll_deg =
            wrap_signed_degrees(state.roll_deg - state.level_roll_offset_deg);
        state.corrected_pitch_deg =
            wrap_signed_degrees(state.pitch_deg - state.level_pitch_offset_deg);
        state.corrected_heading_deg =
            wrap_degrees(state.heading_deg + state.heading_offset_deg);
        state.have_rotation = true;
    } else if (id == REPORT_MAGNETOMETER) {
        memcpy(state.magnetic_ut, values, sizeof(state.magnetic_ut));
        state.have_magnetic = true;
    } else if (id == REPORT_GYROSCOPE) {
        memcpy(state.gyro_rad_s, values, sizeof(state.gyro_rad_s));
        state.have_gyro = true;
    } else if (id == REPORT_ACCELEROMETER) {
        memcpy(state.acceleration_m_s2, values, sizeof(state.acceleration_m_s2));
        state.have_acceleration = true;
    } else if (id == REPORT_LINEAR_ACCELERATION) {
        memcpy(state.linear_acceleration_m_s2, values,
               sizeof(state.linear_acceleration_m_s2));
        state.have_linear = true;
    }
    state.reports++;
    state.connected = state.have_rotation && state.have_magnetic &&
                      state.have_gyro && state.have_acceleration && state.have_linear;
    if (state.connected) {
        snprintf(state.phase, sizeof(state.phase), "streaming fused telemetry");
        state.error[0] = '\0';
    }
    xSemaphoreGive(state_mutex);
}

static void parse_report_batch(const uint8_t *payload, size_t length)
{
    size_t offset = 0;
    while (offset < length) {
        uint8_t id = payload[offset];
        size_t report_length;
        if (id == REPORT_BASE_TIMESTAMP) {
            report_length = 5;
        } else if (id == REPORT_ROTATION_VECTOR) {
            report_length = 14;
        } else if (id >= REPORT_ACCELEROMETER && id <= REPORT_LINEAR_ACCELERATION) {
            report_length = 10;
        } else {
            return;
        }
        if (offset + report_length > length) return;
        if (id != REPORT_BASE_TIMESTAMP) {
            parse_sensor_report(payload + offset, report_length);
        }
        offset += report_length;
    }
}

static void handle_shtp(const uint8_t *cargo, size_t length)
{
    if (length < 4) return;
    size_t packet_length = read_u16(cargo) & 0x7fff;
    uint8_t channel = cargo[2];
    if (packet_length < 4 || packet_length > length) return;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.frames++;
    state.last_frame_ms = now_ms();
    xSemaphoreGive(state_mutex);
    if (channel == CHANNEL_INPUT_NORMAL || channel == CHANNEL_INPUT_WAKE) {
        parse_report_batch(cargo + 4, packet_length - 4);
    }
}

static bool valid_bno_frame(const uint8_t *frame, size_t length)
{
    if (length < 1 || (frame[0] != 0x00 && frame[0] != 0x01)) return false;
    if (frame[0] == 0x00) return length >= 3;
    if (length < 5) return false;
    size_t packet_length = read_u16(frame + 1) & 0x7fff;
    return packet_length >= 4 && packet_length <= length - 1 && frame[3] <= 5;
}

static int passive_probe_pin(int gpio)
{
    ESP_LOGI(TAG, "Passive RX probe on GPIO%d; TX remains disabled", gpio);
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_pin(
        BNO_UART, UART_PIN_NO_CHANGE, gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush_input(BNO_UART);
    frame_reader_t reader = {0};
    uint8_t frame[BNO_FRAME_MAX];
    size_t length = 0;
    const int64_t deadline = now_ms() + BNO_PASSIVE_PROBE_MS;
    int frames = 0;
    while (now_ms() < deadline) {
        if (!read_frame(&reader, frame, &length, 100)) continue;
        if (valid_bno_frame(frame, length)) {
            frames++;
            ESP_LOGI(TAG, "Valid passive BNO080 frame on GPIO%d (%u bytes)",
                     gpio, (unsigned)length);
            break;
        }
    }
    return frames;
}

static bool apply_orientation(int rx)
{
    int tx = rx == BNO_PIN_A ? BNO_PIN_B : BNO_PIN_A;
    gpio_pullup_dis(BNO_PIN_A);
    gpio_pullup_dis(BNO_PIN_B);
    gpio_pulldown_dis(BNO_PIN_A);
    gpio_pulldown_dis(BNO_PIN_B);
    if (uart_set_pin(BNO_UART, tx, rx, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK) {
        return false;
    }
    uart_flush_input(BNO_UART);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.orientation_found = true;
    state.tx_gpio = tx;
    state.rx_gpio = rx;
    snprintf(state.phase, sizeof(state.phase), "orientation found; starting SHTP");
    state.error[0] = '\0';
    xSemaphoreGive(state_mutex);
    ESP_LOGI(TAG, "BNO080 wiring identified: TX GPIO%d -> BNO RX, RX GPIO%d <- BNO TX",
             tx, rx);
    return true;
}

static int passive_idle_high_pin(void)
{
    gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << BNO_PIN_A) | (1ULL << BNO_PIN_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&inputs) != ESP_OK) return -1;
    vTaskDelay(pdMS_TO_TICKS(20));
    int a_high = 0;
    int b_high = 0;
    for (int i = 0; i < 16; ++i) {
        a_high += gpio_get_level(BNO_PIN_A) != 0;
        b_high += gpio_get_level(BNO_PIN_B) != 0;
        esp_rom_delay_us(100);
    }
    gpio_pulldown_dis(BNO_PIN_A);
    gpio_pulldown_dis(BNO_PIN_B);
    ESP_LOGI(TAG, "Passive idle levels with weak pulldowns: GPIO5=%d/16 GPIO6=%d/16",
             a_high, b_high);
    if (a_high >= 14 && b_high <= 2) return BNO_PIN_A;
    if (b_high >= 14 && a_high <= 2) return BNO_PIN_B;
    return -1;
}

static bool find_orientation_passively(void)
{
    gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << BNO_PIN_A) | (1ULL << BNO_PIN_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&inputs);

    int biased_rx = passive_idle_high_pin();
    if (biased_rx >= 0) {
        ESP_LOGI(TAG, "Passive idle bias identifies BNO080 TX on GPIO%d", biased_rx);
        return apply_orientation(biased_rx);
    }

    int on_a = passive_probe_pin(BNO_PIN_A);
    int on_b = passive_probe_pin(BNO_PIN_B);
    if ((on_a > 0) == (on_b > 0)) {
        return false;
    }
    return apply_orientation(on_a ? BNO_PIN_A : BNO_PIN_B);
}

static esp_err_t write_uart_frame(uint8_t protocol_id, const uint8_t *payload,
                                  size_t payload_length)
{
    uint8_t delimiter = 0x7e;
    if (uart_write_bytes(BNO_UART, &delimiter, 1) != 1) return ESP_FAIL;
    if (uart_wait_tx_done(BNO_UART, pdMS_TO_TICKS(20)) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(1));
    uint8_t value = protocol_id;
    for (size_t i = 0; i <= payload_length; ++i) {
        if (i > 0) value = payload[i - 1];
        uint8_t encoded[2];
        size_t count = 1;
        if (value == 0x7d || value == 0x7e) {
            encoded[0] = 0x7d;
            encoded[1] = value ^ 0x20;
            count = 2;
        } else {
            encoded[0] = value;
        }
        if (uart_write_bytes(BNO_UART, encoded, count) != (int)count) return ESP_FAIL;
        if (uart_wait_tx_done(BNO_UART, pdMS_TO_TICKS(20)) != ESP_OK) return ESP_FAIL;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (uart_write_bytes(BNO_UART, &delimiter, 1) != 1) return ESP_FAIL;
    return uart_wait_tx_done(BNO_UART, pdMS_TO_TICKS(50));
}

static int request_buffer(frame_reader_t *reader, uint32_t timeout_ms)
{
    if (write_uart_frame(0x00, NULL, 0) != ESP_OK) return -1;
    const int64_t deadline = now_ms() + timeout_ms;
    uint8_t frame[BNO_FRAME_MAX];
    size_t length = 0;
    while (now_ms() < deadline) {
        if (!read_frame(reader, frame, &length, 60)) continue;
        if (length >= 3 && frame[0] == 0x00) {
            return read_u16(frame + 1);
        }
        if (length >= 2 && frame[0] == 0x01) {
            handle_shtp(frame + 1, length - 1);
        }
    }
    return -1;
}

static esp_err_t send_shtp(frame_reader_t *reader, uint8_t channel,
                           const uint8_t *payload, size_t payload_length)
{
    if (channel >= sizeof(tx_sequence)) return ESP_ERR_INVALID_ARG;
    size_t cargo_length = payload_length + 4;
    int available = request_buffer(reader, 1000);
    if (available < 0 || (size_t)available < cargo_length) return ESP_ERR_TIMEOUT;
    uint8_t cargo[BNO_FRAME_MAX];
    if (cargo_length > sizeof(cargo)) return ESP_ERR_INVALID_SIZE;
    write_u16(cargo, (uint16_t)cargo_length);
    cargo[2] = channel;
    cargo[3] = tx_sequence[channel]++;
    memcpy(cargo + 4, payload, payload_length);
    return write_uart_frame(0x01, cargo, cargo_length);
}

static esp_err_t enable_report(frame_reader_t *reader, uint8_t report_id,
                               uint32_t interval_us)
{
    uint8_t payload[17] = {0};
    payload[0] = SET_FEATURE_COMMAND;
    payload[1] = report_id;
    write_u32(payload + 5, interval_us);
    return send_shtp(reader, CHANNEL_CONTROL, payload, sizeof(payload));
}

static esp_err_t initialize_sensor(frame_reader_t *reader)
{
    set_phase("checking UART-SHTP buffer", NULL);
    int available = request_buffer(reader, 1600);
    if (available < 0) return ESP_ERR_TIMEOUT;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.round_trip = true;
    state.host_buffer = (uint16_t)available;
    xSemaphoreGive(state_mutex);
    ESP_LOGI(TAG, "UART-SHTP round trip verified: host buffer %d bytes", available);

    uint8_t reset = 0x01;
    set_phase("sending first SH-2 reset", NULL);
    ESP_RETURN_ON_ERROR(send_shtp(reader, CHANNEL_EXECUTABLE, &reset, 1), TAG,
                        "first reset");
    vTaskDelay(pdMS_TO_TICKS(500));
    set_phase("sending second SH-2 reset", NULL);
    ESP_RETURN_ON_ERROR(send_shtp(reader, CHANNEL_EXECUTABLE, &reset, 1), TAG,
                        "second reset");
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_flush_input(BNO_UART);
    memset(reader, 0, sizeof(*reader));
    memset(tx_sequence, 0, sizeof(tx_sequence));
    command_sequence = 0;
    clear_measurements();

    const struct { uint8_t id; uint32_t interval_us; } reports[] = {
        {REPORT_ROTATION_VECTOR, 20000},
        {REPORT_MAGNETOMETER, 20000},
        {REPORT_ACCELEROMETER, 20000},
        {REPORT_LINEAR_ACCELERATION, 20000},
        {REPORT_GYROSCOPE, 10000},
    };
    for (size_t i = 0; i < sizeof(reports) / sizeof(reports[0]); ++i) {
        char phase[64];
        snprintf(phase, sizeof(phase), "enabling report 0x%02x", reports[i].id);
        set_phase(phase, NULL);
        ESP_RETURN_ON_ERROR(enable_report(reader, reports[i].id,
                                          reports[i].interval_us), TAG,
                            "enable report 0x%02x", reports[i].id);
    }

    uint8_t calibration[12] = {0};
    set_phase("starting dynamic calibration", NULL);
    calibration[0] = COMMAND_REQUEST;
    calibration[1] = command_sequence++;
    calibration[2] = ME_CALIBRATE_COMMAND;
    calibration[3] = 1;
    calibration[4] = 1;
    calibration[5] = 1;
    ESP_RETURN_ON_ERROR(send_shtp(reader, CHANNEL_CONTROL, calibration,
                                  sizeof(calibration)), TAG, "start calibration");
    set_phase("waiting for fused reports", NULL);
    return ESP_OK;
}

static void receive_for(frame_reader_t *reader, uint32_t duration_ms)
{
    const int64_t deadline = now_ms() + duration_ms;
    uint8_t frame[BNO_FRAME_MAX];
    size_t length = 0;
    while (now_ms() < deadline) {
        if (!read_frame(reader, frame, &length, 50)) continue;
        if (length >= 2 && frame[0] == 0x01) {
            handle_shtp(frame + 1, length - 1);
        }
    }
}

static void bno_task(void *argument)
{
    (void)argument;
    uart_config_t config = {
        .baud_rate = BNO_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(BNO_UART, BNO_RX_BUFFER, 0, 0, NULL, 0) != ESP_OK ||
        uart_param_config(BNO_UART, &config) != ESP_OK) {
        set_phase("UART unavailable", "Could not initialize UART4 at 3 Mbps");
        vTaskDelete(NULL);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.uart_ready = true;
    xSemaphoreGive(state_mutex);

    frame_reader_t reader = {0};
    for (;;) {
        if (!state.orientation_found) {
            set_phase("passive RX orientation probe",
                      "Power-cycle BNO080 if GPIO5/GPIO6 remain silent");
            if (!find_orientation_passively()) {
                vTaskDelay(pdMS_TO_TICKS(BNO_RETRY_MS));
                continue;
            }
            memset(&reader, 0, sizeof(reader));
        }

        xSemaphoreTake(uart_mutex, portMAX_DELAY);
        esp_err_t result = initialize_sensor(&reader);
        xSemaphoreGive(uart_mutex);
        if (result != ESP_OK) {
            char error[128];
            snprintf(error, sizeof(error), "UART-SHTP initialization failed: %s",
                     esp_err_to_name(result));
            set_phase("retrying BNO080 initialization", error);
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.round_trip = false;
            xSemaphoreGive(state_mutex);
            vTaskDelay(pdMS_TO_TICKS(BNO_RETRY_MS));
            memset(&reader, 0, sizeof(reader));
            continue;
        }

        int64_t report_deadline = now_ms() + 6000;
        while (now_ms() < report_deadline) {
            receive_for(&reader, 150);
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            bool connected = state.connected;
            xSemaphoreGive(state_mutex);
            if (connected) break;
        }
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool connected = state.connected;
        xSemaphoreGive(state_mutex);
        if (!connected) {
            set_phase("retrying report setup",
                      "UART-SHTP responds but fused reports did not arrive");
            vTaskDelay(pdMS_TO_TICKS(BNO_RETRY_MS));
            continue;
        }

        for (;;) {
            receive_for(&reader, 200);
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            int64_t age = state.last_frame_ms ? now_ms() - state.last_frame_ms : INT64_MAX;
            if (age > BNO_REPORT_TIMEOUT_MS) state.connected = false;
            connected = state.connected;
            xSemaphoreGive(state_mutex);
            if (!connected) {
                set_phase("telemetry timed out", "Restarting UART-SHTP report setup");
                break;
            }
        }
    }
}

static bool begin_calibration(const char *phase)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool allowed = !state.calibration_active;
    if (allowed) {
        state.calibration_active = true;
        state.calibration_cancel = false;
        state.calibration_progress = 0;
        snprintf(state.calibration_phase, sizeof(state.calibration_phase), "%s", phase);
        state.calibration_error[0] = '\0';
    }
    xSemaphoreGive(state_mutex);
    return allowed;
}

static bool calibration_cancelled(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool cancelled = state.calibration_cancel;
    xSemaphoreGive(state_mutex);
    return cancelled;
}

static void calibration_status(const char *phase, unsigned progress,
                               const char *error, bool finished)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.calibration_phase, sizeof(state.calibration_phase), "%s",
             phase ? phase : "");
    snprintf(state.calibration_error, sizeof(state.calibration_error), "%s",
             error ? error : "");
    state.calibration_progress = progress > 100 ? 100 : progress;
    if (finished) state.calibration_active = false;
    xSemaphoreGive(state_mutex);
}

static bool fresh_sensor_snapshot(bno_state_t *snapshot)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    *snapshot = state;
    xSemaphoreGive(state_mutex);
    return snapshot->connected && snapshot->last_frame_ms &&
           now_ms() - snapshot->last_frame_ms < 300;
}

static void forward_log_reset(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    forward_log_count = 0;
    forward_log_accepted = 0;
    forward_log_vector_x = 0.0f;
    forward_log_vector_y = 0.0f;
    forward_log_strength = 0.0f;
    forward_log_offset_deg = 0.0f;
    forward_log_baseline_x = 0.0f;
    forward_log_baseline_y = 0.0f;
    forward_log_attempt++;
    snprintf(forward_log_result, sizeof(forward_log_result), "attempt in progress");
    xSemaphoreGive(state_mutex);
}

static void forward_log_set_baseline(float x, float y)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    forward_log_baseline_x = x;
    forward_log_baseline_y = y;
    xSemaphoreGive(state_mutex);
}

static void forward_log_append(uint16_t elapsed_ms, const bno_state_t *sample,
                               bool accepted)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (forward_log_count < FORWARD_LOG_CAPACITY) {
        forward_log_sample_t *entry = &forward_log[forward_log_count++];
        entry->elapsed_ms = elapsed_ms;
        entry->accepted = accepted;
        entry->heading_deg = sample->heading_deg;
        entry->roll_deg = sample->roll_deg;
        entry->pitch_deg = sample->pitch_deg;
        memcpy(entry->acceleration, sample->acceleration_m_s2,
               sizeof(entry->acceleration));
        memcpy(entry->linear, sample->linear_acceleration_m_s2,
               sizeof(entry->linear));
        memcpy(entry->gyro, sample->gyro_rad_s, sizeof(entry->gyro));
        memcpy(entry->magnetic, sample->magnetic_ut, sizeof(entry->magnetic));
    }
    if (accepted) forward_log_accepted++;
    xSemaphoreGive(state_mutex);
}

static void forward_log_finish(const char *result, float vector_x, float vector_y,
                               float strength, float offset_deg)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    forward_log_vector_x = vector_x;
    forward_log_vector_y = vector_y;
    forward_log_strength = strength;
    forward_log_offset_deg = offset_deg;
    snprintf(forward_log_result, sizeof(forward_log_result), "%s", result);
    xSemaphoreGive(state_mutex);
}

static void level_calibration_task(void *argument)
{
    (void)argument;
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;
    unsigned accepted = 0;
    const int64_t start = now_ms();
    while (now_ms() - start < LEVEL_SAMPLE_MS) {
        if (calibration_cancelled()) {
            calibration_status("level calibration cancelled", 0, "Cancelled", true);
            vTaskDelete(NULL);
        }
        bno_state_t sample;
        if (!fresh_sensor_snapshot(&sample)) {
            calibration_status("level calibration failed", 0,
                               "BNO080 telemetry became stale", true);
            vTaskDelete(NULL);
        }
        float gyro = sqrtf(sample.gyro_rad_s[0] * sample.gyro_rad_s[0] +
                           sample.gyro_rad_s[1] * sample.gyro_rad_s[1] +
                           sample.gyro_rad_s[2] * sample.gyro_rad_s[2]);
        float accel = sqrtf(sample.acceleration_m_s2[0] * sample.acceleration_m_s2[0] +
                            sample.acceleration_m_s2[1] * sample.acceleration_m_s2[1] +
                            sample.acceleration_m_s2[2] * sample.acceleration_m_s2[2]);
        if (gyro < 0.12f && accel > 8.6f && accel < 11.0f) {
            roll_sum += sample.roll_deg;
            pitch_sum += sample.pitch_deg;
            accepted++;
        }
        calibration_status("keep rover level and motionless",
                           (unsigned)((now_ms() - start) * 100 / LEVEL_SAMPLE_MS),
                           NULL, false);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (accepted < 35) {
        calibration_status("level calibration rejected", 100,
                           "Too much motion or acceleration; keep rover still", true);
        vTaskDelete(NULL);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.level_roll_offset_deg = roll_sum / accepted;
    state.level_pitch_offset_deg = pitch_sum / accepted;
    state.level_calibrated = true;
    xSemaphoreGive(state_mutex);
    save_calibration();
    calibration_status("level calibration saved", 100, NULL, true);
    vTaskDelete(NULL);
}

static void forward_calibration_task(void *argument)
{
    const unsigned throttle_percent = (unsigned)(uintptr_t)argument;
    bool motion_owned = false;
    float vector_x = 0.0f;
    float vector_y = 0.0f;
    float magnitude_sum = 0.0f;
    float baseline_sum_x = 0.0f;
    float baseline_sum_y = 0.0f;
    float baseline_x = 0.0f;
    float baseline_y = 0.0f;
    unsigned baseline_count = 0;
    unsigned accepted = 0;
    if (!motion_begin || !motion_drive || !motion_end || !motion_begin()) {
        calibration_status("forward calibration blocked", 0,
                           "STM32 must be alive and handheld drive disconnected", true);
        vTaskDelete(NULL);
    }
    motion_owned = true;
    forward_log_reset();
    for (int remaining = 3; remaining > 0; --remaining) {
        char phase[64];
        snprintf(phase, sizeof(phase), "clear path: moving in %d", remaining);
        calibration_status(phase, (unsigned)((3 - remaining) * 10), NULL, false);
        for (int elapsed = 0; elapsed < 1000; elapsed += 50) {
            if (calibration_cancelled()) goto cancelled;
            bno_state_t sample;
            if (!fresh_sensor_snapshot(&sample)) goto stale;
            if (remaining == 1) {
                baseline_sum_x += sample.linear_acceleration_m_s2[0];
                baseline_sum_y += sample.linear_acceleration_m_s2[1];
                baseline_count++;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    if (baseline_count > 0) {
        baseline_x = baseline_sum_x / baseline_count;
        baseline_y = baseline_sum_y / baseline_count;
    }
    forward_log_set_baseline(baseline_x, baseline_y);

    const int duty = (int)((4095u * throttle_percent) / 100u);
    int64_t started = now_ms();
    int64_t next_drive = 0;
    while (now_ms() - started < FORWARD_RUN_MS) {
        if (calibration_cancelled()) goto cancelled;
        int64_t elapsed = now_ms() - started;
        if (now_ms() >= next_drive) {
            if (!motion_drive(duty, duty, FORWARD_RAMP_PERCENT_PER_SECOND, false)) {
                goto link_lost;
            }
            next_drive = now_ms() + 100;
        }
        bno_state_t sample;
        if (!fresh_sensor_snapshot(&sample)) goto stale;
        bool use_sample = false;
        if (elapsed >= FORWARD_SAMPLE_START_MS && elapsed <= FORWARD_SAMPLE_END_MS) {
            float linear_x = sample.linear_acceleration_m_s2[0] - baseline_x;
            float linear_y = sample.linear_acceleration_m_s2[1] - baseline_y;
            float horizontal = hypotf(linear_x, linear_y);
            float turn_rate = fabsf(sample.gyro_rad_s[2]);
            if (horizontal >= 0.04f && turn_rate < 0.80f) {
                vector_x += linear_x;
                vector_y += linear_y;
                magnitude_sum += horizontal;
                accepted++;
                use_sample = true;
            }
        }
        forward_log_append((uint16_t)elapsed, &sample, use_sample);
        calibration_status("driving straight; estimating forward axis",
                           30u + (unsigned)(elapsed * 65 / FORWARD_RUN_MS), NULL, false);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    motion_end();
    motion_owned = false;
    float strength = hypotf(vector_x, vector_y) / (accepted ? accepted : 1);
    float coherence = hypotf(vector_x, vector_y) / (magnitude_sum > 0.0f ? magnitude_sum : 1.0f);
    float offset = wrap_signed_degrees(
        atan2f(vector_y, vector_x) * (180.0f / (float)M_PI));
    if (accepted < 10 || strength < 0.06f || coherence < 0.30f) {
        char result[96];
        snprintf(result, sizeof(result),
                 "rejected: accepted=%u strength=%.3f coherence=%.3f",
                 accepted, strength, coherence);
        forward_log_finish(result, vector_x, vector_y, strength, offset);
        calibration_status("forward calibration rejected", 100,
                           "Forward vector was weak or inconsistent; download black box CSV",
                           true);
        vTaskDelete(NULL);
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.heading_offset_deg = wrap_signed_degrees(offset);
    state.heading_calibrated = true;
    xSemaphoreGive(state_mutex);
    save_calibration();
    char result[96];
    snprintf(result, sizeof(result),
             "saved: accepted=%u strength=%.3f coherence=%.3f offset=%.2f",
             accepted, strength, coherence, offset);
    forward_log_finish(result, vector_x, vector_y, strength, offset);
    calibration_status("forward heading offset saved", 100, NULL, true);
    vTaskDelete(NULL);

stale:
    if (motion_owned) motion_end();
    forward_log_finish("stopped: BNO080 telemetry stale", vector_x, vector_y,
                       0.0f, 0.0f);
    calibration_status("forward calibration stopped", 0,
                       "BNO080 telemetry became stale", true);
    vTaskDelete(NULL);
link_lost:
    if (motion_owned) motion_end();
    forward_log_finish("stopped: STM32 heartbeat lost", vector_x, vector_y,
                       0.0f, 0.0f);
    calibration_status("forward calibration stopped", 0,
                       "STM32 heartbeat was lost", true);
    vTaskDelete(NULL);
cancelled:
    if (motion_owned) motion_end();
    forward_log_finish("cancelled", vector_x, vector_y, 0.0f, 0.0f);
    calibration_status("forward calibration cancelled", 0, "Cancelled", true);
    vTaskDelete(NULL);
}

void bno080_link_abort_calibration(void)
{
    if (!state_mutex) return;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool active = state.calibration_active;
    state.calibration_cancel = true;
    xSemaphoreGive(state_mutex);
    if (active && motion_drive) (void)motion_drive(0, 0, 300, true);
}

static esp_err_t send_status_json(httpd_req_t *request)
{
    bno_state_t snapshot;
    unsigned log_count;
    unsigned log_accepted;
    unsigned log_attempt;
    float log_vector_x;
    float log_vector_y;
    float log_strength;
    float log_offset;
    float log_baseline_x;
    float log_baseline_y;
    char log_result[sizeof(forward_log_result)];
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshot = state;
    log_count = forward_log_count;
    log_accepted = forward_log_accepted;
    log_attempt = forward_log_attempt;
    log_vector_x = forward_log_vector_x;
    log_vector_y = forward_log_vector_y;
    log_strength = forward_log_strength;
    log_offset = forward_log_offset_deg;
    log_baseline_x = forward_log_baseline_x;
    log_baseline_y = forward_log_baseline_y;
    snprintf(log_result, sizeof(log_result), "%s", forward_log_result);
    xSemaphoreGive(state_mutex);
    int64_t age = snapshot.last_frame_ms ? now_ms() - snapshot.last_frame_ms : -1;
    char json[2800];
    int count = snprintf(
        json, sizeof(json),
        "{\"connected\":%s,\"uart_ready\":%s,\"orientation_found\":%s,"
        "\"round_trip\":%s,\"tx_gpio\":%d,\"rx_gpio\":%d,"
        "\"baud\":%d,\"host_buffer\":%u,\"age_ms\":%lld,"
        "\"frames\":%llu,\"reports\":%llu,\"phase\":\"%s\",\"error\":\"%s\","
        "\"heading_deg\":%.3f,\"roll_deg\":%.3f,\"pitch_deg\":%.3f,"
        "\"corrected_heading_deg\":%.3f,\"corrected_roll_deg\":%.3f,"
        "\"corrected_pitch_deg\":%.3f,\"magnetometer_accuracy\":%u,"
        "\"level_calibrated\":%s,\"heading_calibrated\":%s,"
        "\"level_roll_offset_deg\":%.3f,\"level_pitch_offset_deg\":%.3f,"
        "\"heading_offset_deg\":%.3f,\"calibration_active\":%s,"
        "\"calibration_progress\":%u,\"calibration_phase\":\"%s\","
        "\"calibration_error\":\"%s\","
        "\"forward_log\":{\"attempt\":%u,\"samples\":%u,\"accepted\":%u,"
        "\"vector_x\":%.5f,\"vector_y\":%.5f,\"strength\":%.5f,"
        "\"baseline_x\":%.5f,\"baseline_y\":%.5f,"
        "\"offset_deg\":%.3f,\"result\":\"%s\"},"
        "\"quaternion\":[%.6f,%.6f,%.6f,%.6f],"
        "\"magnetic_uT\":[%.4f,%.4f,%.4f],"
        "\"gyro_rad_s\":[%.5f,%.5f,%.5f],"
        "\"acceleration_m_s2\":[%.4f,%.4f,%.4f],"
        "\"linear_acceleration_m_s2\":[%.4f,%.4f,%.4f]}",
        snapshot.connected ? "true" : "false",
        snapshot.uart_ready ? "true" : "false",
        snapshot.orientation_found ? "true" : "false",
        snapshot.round_trip ? "true" : "false", snapshot.tx_gpio,
        snapshot.rx_gpio, BNO_BAUD, snapshot.host_buffer, (long long)age,
        (unsigned long long)snapshot.frames, (unsigned long long)snapshot.reports,
        snapshot.phase, snapshot.error, snapshot.heading_deg,
        snapshot.roll_deg, snapshot.pitch_deg, snapshot.corrected_heading_deg,
        snapshot.corrected_roll_deg, snapshot.corrected_pitch_deg,
        snapshot.accuracy[REPORT_MAGNETOMETER],
        snapshot.level_calibrated ? "true" : "false",
        snapshot.heading_calibrated ? "true" : "false",
        snapshot.level_roll_offset_deg, snapshot.level_pitch_offset_deg,
        snapshot.heading_offset_deg, snapshot.calibration_active ? "true" : "false",
        snapshot.calibration_progress, snapshot.calibration_phase,
        snapshot.calibration_error, log_attempt, log_count, log_accepted,
        log_vector_x, log_vector_y, log_strength, log_baseline_x, log_baseline_y,
        log_offset, log_result,
        snapshot.quaternion[0],
        snapshot.quaternion[1], snapshot.quaternion[2], snapshot.quaternion[3],
        snapshot.magnetic_ut[0], snapshot.magnetic_ut[1], snapshot.magnetic_ut[2],
        snapshot.gyro_rad_s[0], snapshot.gyro_rad_s[1], snapshot.gyro_rad_s[2],
        snapshot.acceleration_m_s2[0], snapshot.acceleration_m_s2[1],
        snapshot.acceleration_m_s2[2], snapshot.linear_acceleration_m_s2[0],
        snapshot.linear_acceleration_m_s2[1], snapshot.linear_acceleration_m_s2[2]);
    if (count < 0 || count >= (int)sizeof(json)) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "telemetry JSON overflow");
    }
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, json, count);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    return send_status_json(request);
}

static esp_err_t forward_log_handler(httpd_req_t *request)
{
    unsigned count;
    unsigned attempt;
    unsigned accepted;
    float vector_x;
    float vector_y;
    float strength;
    float offset;
    float baseline_x;
    float baseline_y;
    char result[sizeof(forward_log_result)];
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    count = forward_log_count;
    attempt = forward_log_attempt;
    accepted = forward_log_accepted;
    vector_x = forward_log_vector_x;
    vector_y = forward_log_vector_y;
    strength = forward_log_strength;
    offset = forward_log_offset_deg;
    baseline_x = forward_log_baseline_x;
    baseline_y = forward_log_baseline_y;
    snprintf(result, sizeof(result), "%s", forward_log_result);
    xSemaphoreGive(state_mutex);

    httpd_resp_set_type(request, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "Content-Disposition",
                       "attachment; filename=imu_forward_blackbox.csv");
    char line[512];
    int length = snprintf(
        line, sizeof(line),
        "# attempt=%u,samples=%u,accepted=%u,vector_x=%.6f,vector_y=%.6f,"
        "strength=%.6f,baseline_x=%.6f,baseline_y=%.6f,offset_deg=%.3f,result=%s\n"
        "elapsed_ms,accepted,heading_deg,roll_deg,pitch_deg,"
        "accel_x,accel_y,accel_z,linear_x,linear_y,linear_z,"
        "gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z\n",
        attempt, count, accepted, vector_x, vector_y, strength, baseline_x,
        baseline_y, offset, result);
    if (httpd_resp_send_chunk(request, line, length) != ESP_OK) return ESP_FAIL;
    for (unsigned i = 0; i < count; ++i) {
        forward_log_sample_t sample;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        sample = forward_log[i];
        xSemaphoreGive(state_mutex);
        length = snprintf(
            line, sizeof(line),
            "%u,%u,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
            "%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n",
            sample.elapsed_ms, sample.accepted, sample.heading_deg,
            sample.roll_deg, sample.pitch_deg, sample.acceleration[0],
            sample.acceleration[1], sample.acceleration[2], sample.linear[0],
            sample.linear[1], sample.linear[2], sample.gyro[0], sample.gyro[1],
            sample.gyro[2], sample.magnetic[0], sample.magnetic[1],
            sample.magnetic[2]);
        if (httpd_resp_send_chunk(request, line, length) != ESP_OK) return ESP_FAIL;
    }
    return httpd_resp_send_chunk(request, NULL, 0);
}

static esp_err_t level_calibration_handler(httpd_req_t *request)
{
    bno_state_t snapshot;
    if (!fresh_sensor_snapshot(&snapshot)) {
        return send_conflict(request, "BNO080 telemetry is not fresh");
    }
    if (!begin_calibration("starting level calibration")) {
        return send_conflict(request, "calibration already active");
    }
    if (xTaskCreate(level_calibration_task, "imu_level_cal", 4096, NULL, 6, NULL) !=
        pdPASS) {
        calibration_status("level calibration failed", 0,
                           "Could not start calibration task", true);
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "task allocation failed");
    }
    return send_status_json(request);
}

static esp_err_t forward_calibration_handler(httpd_req_t *request)
{
    int throttle = 70;
    char query[96];
    char value[12];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "throttle", value, sizeof(value)) == ESP_OK) {
        throttle = atoi(value);
    }
    if (throttle < 20 || throttle > 100) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "throttle must be 20 to 100 percent");
    }
    bno_state_t snapshot;
    if (!fresh_sensor_snapshot(&snapshot)) {
        return send_conflict(request, "BNO080 telemetry is not fresh");
    }
    if (!begin_calibration("arming forward calibration")) {
        return send_conflict(request, "calibration already active");
    }
    if (xTaskCreate(forward_calibration_task, "imu_forward_cal", 6144,
                    (void *)(uintptr_t)throttle, 7, NULL) != pdPASS) {
        calibration_status("forward calibration failed", 0,
                           "Could not start calibration task", true);
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "task allocation failed");
    }
    return send_status_json(request);
}

static esp_err_t abort_calibration_handler(httpd_req_t *request)
{
    bno080_link_abort_calibration();
    return send_status_json(request);
}

static esp_err_t reset_calibration_handler(httpd_req_t *request)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.calibration_active) {
        xSemaphoreGive(state_mutex);
        return send_conflict(request, "abort active calibration first");
    }
    state.level_calibrated = false;
    state.heading_calibrated = false;
    state.level_roll_offset_deg = 0.0f;
    state.level_pitch_offset_deg = 0.0f;
    state.heading_offset_deg = 0.0f;
    snprintf(state.calibration_phase, sizeof(state.calibration_phase),
             "mounting calibration reset");
    state.calibration_error[0] = '\0';
    xSemaphoreGive(state_mutex);
    save_calibration();
    return send_status_json(request);
}

static esp_err_t heading_offset_handler(httpd_req_t *request)
{
    char query[96];
    char value[24];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "degrees", value, sizeof(value)) != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "missing degrees");
    }
    char *end = NULL;
    float offset = strtof(value, &end);
    if (!end || *end != '\0' || !isfinite(offset) || offset < -180.0f ||
        offset > 180.0f) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "degrees must be -180 to 180");
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.calibration_active) {
        xSemaphoreGive(state_mutex);
        return send_conflict(request, "abort active calibration first");
    }
    state.heading_offset_deg = offset;
    state.heading_calibrated = true;
    snprintf(state.calibration_phase, sizeof(state.calibration_phase),
             "manual heading offset saved");
    state.calibration_error[0] = '\0';
    xSemaphoreGive(state_mutex);
    save_calibration();
    return send_status_json(request);
}

#include "sensors_page.h"

static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, SENSOR_PAGE, HTTPD_RESP_USE_STRLEN);
}

esp_err_t bno080_link_start(bno080_motion_begin_fn begin_motion,
                            bno080_motion_drive_fn drive,
                            bno080_motion_end_fn end_motion)
{
    state_mutex = xSemaphoreCreateMutex();
    uart_mutex = xSemaphoreCreateMutex();
    if (!state_mutex || !uart_mutex) return ESP_ERR_NO_MEM;
    motion_begin = begin_motion;
    motion_drive = drive;
    motion_end = end_motion;
    load_calibration();
    snprintf(state.calibration_phase, sizeof(state.calibration_phase),
             "calibration idle");
    if (xTaskCreate(bno_task, "bno080_uart", 7168, NULL, 7, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t bno080_link_register_routes(httpd_handle_t server)
{
    const httpd_uri_t routes[] = {
        {.uri = "/sensors", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/sensors", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/sensors/calibration/log", .method = HTTP_GET,
         .handler = forward_log_handler},
        {.uri = "/api/sensors/calibration/level", .method = HTTP_POST,
         .handler = level_calibration_handler},
        {.uri = "/api/sensors/calibration/forward", .method = HTTP_POST,
         .handler = forward_calibration_handler},
        {.uri = "/api/sensors/calibration/abort", .method = HTTP_POST,
         .handler = abort_calibration_handler},
        {.uri = "/api/sensors/calibration/reset", .method = HTTP_POST,
         .handler = reset_calibration_handler},
        {.uri = "/api/sensors/calibration/heading-offset", .method = HTTP_POST,
         .handler = heading_offset_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        esp_err_t result = httpd_register_uri_handler(server, &routes[i]);
        if (result != ESP_OK) return result;
    }
    return ESP_OK;
}
