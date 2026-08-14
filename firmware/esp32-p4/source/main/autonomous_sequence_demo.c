#include "autonomous_sequence_demo.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bno080_link.h"
#include "drive_control.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "odesc_link.h"
#include "stepper_link.h"

#define PI_F                              3.14159265358979323846f
#define WHEEL_DIAMETER_IN                 10.0f
#define MOTOR_TO_WHEEL_REDUCTION           2.5f
#define POTENTIOMETER_TRAVEL_DEG          220.0f
#define ENCODER_ADC_MAX                  4095
#define MOW_HEADING_DEG                   188.0f
#define WATER_HEADING_DEG                  22.0f
#define DRIVE_SPEED_PERCENT               50
#define MOWER_SPEED_RPM                  3000.0f
#define HEADING_KP                          1.20f
#define HEADING_KI                          0.04f
#define HEADING_KD                          0.30f
#define HEADING_MAX_CORRECTION             35.0f
#define HEADING_INTEGRAL_LIMIT            100.0f
#define CONTROL_PERIOD_MS                  20
#define DRIVE_TIMEOUT_MS               120000
#define HOSE_PORT                        4211
#define HOSE_PWM_US                       750
#define HOSE_TARGET_MILLIDEG           720000U
#define HOSE_TIMEOUT_MS                 45000
#define STEPPER_TIMEOUT_MS             90000
#define X_SWEEP_STEPS                    2000
#define Y_DEPLOY_STEPS                  92000
#define X_SWEEP_RPM                       300
#define Y_DEPLOY_RPM                      100
#define WATER_ADVANCE_COUNT                10
#define WATER_ADVANCE_PERIOD_MS          3000

static const char *TAG = "auto_demo";

typedef struct __attribute__((packed)) {
    char magic[4];
    uint8_t version;
    uint8_t command;
    uint16_t pwm_us_be;
    uint32_t target_millideg_be;
    uint32_t sequence_be;
    uint16_t crc_be;
} hose_command_packet_t;

typedef struct __attribute__((packed)) {
    char magic[4];
    uint8_t version;
    uint8_t state;
    uint32_t sequence_be;
    uint32_t measured_millideg_be;
    uint16_t crc_be;
} hose_ack_packet_t;

typedef struct {
    bool initialized;
    uint16_t previous[4];
    float accumulated_deg[4];
} encoder_tracker_t;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float heading_error(float target, float measured)
{
    float error = target - measured;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static float motor_degrees_for_feet(float feet)
{
    /* Ten feet produces 3.8197 wheel turns and 9.5493 motor turns,
     * giving a 3437.75-degree motor-encoder target at 2.5:1 reduction. */
    float distance_inches = feet * 12.0f;
    float wheel_turns = distance_inches / (PI_F * WHEEL_DIAMETER_IN);
    return wheel_turns * MOTOR_TO_WHEEL_REDUCTION * 360.0f;
}

static void encoder_tracker_update(encoder_tracker_t *tracker,
                                   const uint16_t adc[4])
{
    if (!tracker->initialized) {
        memcpy(tracker->previous, adc, sizeof(tracker->previous));
        tracker->initialized = true;
        return;
    }
    for (int i = 0; i < 4; ++i) {
        int delta = (int)adc[i] - (int)tracker->previous[i];
        if (delta > ENCODER_ADC_MAX / 2) delta -= ENCODER_ADC_MAX + 1;
        if (delta < -ENCODER_ADC_MAX / 2) delta += ENCODER_ADC_MAX + 1;
        tracker->accumulated_deg[i] +=
            delta * POTENTIOMETER_TRAVEL_DEG / ENCODER_ADC_MAX;
        tracker->previous[i] = adc[i];
    }
}

static float right_motor_travel(const encoder_tracker_t *tracker)
{
    /* STM32 PA0 and PA1 are the two right-wheel potentiometers. */
    return (fabsf(tracker->accumulated_deg[0]) +
            fabsf(tracker->accumulated_deg[1])) * 0.5f;
}

static float left_motor_travel(const encoder_tracker_t *tracker)
{
    /* STM32 PA2 and PA3 are the two left-wheel potentiometers. */
    return (fabsf(tracker->accumulated_deg[2]) +
            fabsf(tracker->accumulated_deg[3])) * 0.5f;
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t hose_open_two_turns(void)
{
    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (socket_fd < 0) return ESP_FAIL;
    int enabled = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &enabled,
                   sizeof(enabled)) < 0) {
        close(socket_fd);
        return ESP_FAIL;
    }
    struct timeval receive_timeout = {.tv_sec = 0, .tv_usec = 250000};
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout,
               sizeof(receive_timeout));

    const uint32_t sequence = esp_random();
    hose_command_packet_t command = {0};
    memcpy(command.magic, "HSC1", 4);
    command.version = 1;
    command.command = 1;
    command.pwm_us_be = htons(HOSE_PWM_US);
    command.target_millideg_be = htonl(HOSE_TARGET_MILLIDEG);
    command.sequence_be = htonl(sequence);
    command.crc_be = htons(crc16_ccitt((const uint8_t *)&command,
                                       sizeof(command) - sizeof(command.crc_be)));

    struct sockaddr_in destination = {
        .sin_family = AF_INET,
        .sin_port = htons(HOSE_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    const int64_t deadline = now_ms() + HOSE_TIMEOUT_MS;
    int64_t next_send = 0;
    while (now_ms() < deadline) {
        if (now_ms() >= next_send) {
            sendto(socket_fd, &command, sizeof(command), 0,
                   (const struct sockaddr *)&destination, sizeof(destination));
            next_send = now_ms() + 250;
        }
        hose_ack_packet_t ack = {0};
        int received = recvfrom(socket_fd, &ack, sizeof(ack), 0, NULL, NULL);
        if (received != (int)sizeof(ack)) continue;
        uint16_t expected_crc = crc16_ccitt(
            (const uint8_t *)&ack, sizeof(ack) - sizeof(ack.crc_be));
        if (memcmp(ack.magic, "HSA1", 4) == 0 && ack.version == 1 &&
            ntohl(ack.sequence_be) == sequence &&
            ntohs(ack.crc_be) == expected_crc && ack.state == 2 &&
            ntohl(ack.measured_millideg_be) >= HOSE_TARGET_MILLIDEG) {
            close(socket_fd);
            return ESP_OK;
        }
    }
    close(socket_fd);
    return ESP_ERR_TIMEOUT;
}

static bool wait_for_counted_stepper(int timeout_ms)
{
    int64_t deadline = now_ms() + timeout_ms;
    while (stepper_link_counted_active() && now_ms() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return !stepper_link_counted_active();
}

static bool service_x_sweep(int32_t *next_steps)
{
    if (stepper_link_counted_active()) return true;
    if (!stepper_link_move_counted(*next_steps, 0, 0,
                                    X_SWEEP_RPM, X_SWEEP_RPM, X_SWEEP_RPM)) {
        return false;
    }
    *next_steps = -*next_steps;
    return true;
}

static esp_err_t drive_forward(float feet, int speed_percent,
                               float target_heading, int32_t *x_sweep)
{
    encoder_tracker_t encoders = {0};
    uint16_t adc[4] = {0};
    uint32_t encoder_sequence = 0;
    if (!drive_control_autonomous_get_encoders(adc, &encoder_sequence)) {
        return ESP_ERR_INVALID_STATE;
    }
    encoder_tracker_update(&encoders, adc);

    float heading = 0.0f;
    if (!bno080_link_get_magnetic_heading(&heading)) return ESP_ERR_INVALID_STATE;
    float previous_error = heading_error(target_heading, heading);
    float integral = 0.0f;
    const float target_motor_degrees = motor_degrees_for_feet(feet);
    int64_t previous_control = now_ms();
    int64_t deadline = now_ms() + DRIVE_TIMEOUT_MS;

    while (now_ms() < deadline) {
        uint32_t new_sequence = 0;
        if (!drive_control_autonomous_get_encoders(adc, &new_sequence) ||
            !bno080_link_get_magnetic_heading(&heading)) break;
        if (new_sequence != encoder_sequence) {
            encoder_tracker_update(&encoders, adc);
            encoder_sequence = new_sequence;
        }
        if (left_motor_travel(&encoders) >= target_motor_degrees &&
            right_motor_travel(&encoders) >= target_motor_degrees) {
            drive_control_autonomous_set_percent(0, 0);
            return ESP_OK;
        }

        int64_t current = now_ms();
        float dt = (current - previous_control) / 1000.0f;
        if (dt >= CONTROL_PERIOD_MS / 1000.0f) {
            float error = heading_error(target_heading, heading);
            integral = clampf(integral + error * dt,
                              -HEADING_INTEGRAL_LIMIT,
                              HEADING_INTEGRAL_LIMIT);
            float derivative = (error - previous_error) / dt;
            float correction = clampf(HEADING_KP * error +
                                      HEADING_KI * integral +
                                      HEADING_KD * derivative,
                                      -HEADING_MAX_CORRECTION,
                                      HEADING_MAX_CORRECTION);
            int left = (int)clampf(speed_percent + correction, -100.0f, 100.0f);
            int right = (int)clampf(speed_percent - correction, -100.0f, 100.0f);
            if (!drive_control_autonomous_set_percent(left, right)) break;
            previous_error = error;
            previous_control = current;
        }
        if (x_sweep && !service_x_sweep(x_sweep)) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    drive_control_autonomous_set_percent(0, 0);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t initialize_odrive_mower(void)
{
    odesc_controller_snapshot_t mower = {0};
    if (!odesc_link_get_controller(&mower) || !mower.connected || mower.faulted) {
        return ESP_ERR_INVALID_STATE;
    }
    odesc_link_controller_request(true, MOWER_SPEED_RPM / 60.0f);
    int64_t deadline = now_ms() + 5000;
    do {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (odesc_link_get_controller(&mower) && mower.active && !mower.faulted) {
            return ESP_OK;
        }
    } while (now_ms() < deadline);
    odesc_link_controller_request(false, 0.0f);
    return ESP_ERR_TIMEOUT;
}

esp_err_t mow_10_feet(void)
{
    if (!drive_control_autonomous_begin()) return ESP_ERR_INVALID_STATE;
    esp_err_t result = initialize_odrive_mower();
    if (result == ESP_OK) {
        result = drive_forward(10.0f, DRIVE_SPEED_PERCENT,
                               MOW_HEADING_DEG, NULL);
    }
    drive_control_autonomous_end();
    odesc_link_controller_request(false, 0.0f);
    ESP_LOGI(TAG, "mow_10_feet finished: %s", esp_err_to_name(result));
    return result;
}

esp_err_t water_garden(void)
{
    if (!drive_control_autonomous_begin()) return ESP_ERR_INVALID_STATE;
    esp_err_t result = hose_open_two_turns();
    if (result != ESP_OK) goto finished;
    if (!stepper_link_move_counted(0, Y_DEPLOY_STEPS, 0,
                                   Y_DEPLOY_RPM, Y_DEPLOY_RPM, Y_DEPLOY_RPM) ||
        !wait_for_counted_stepper(STEPPER_TIMEOUT_MS)) {
        result = ESP_ERR_TIMEOUT;
        goto finished;
    }

    int32_t next_x_steps = X_SWEEP_STEPS;
    int64_t next_advance = now_ms() + WATER_ADVANCE_PERIOD_MS;
    for (int advance = 0; advance < WATER_ADVANCE_COUNT; ++advance) {
        while (now_ms() < next_advance) {
            if (!service_x_sweep(&next_x_steps)) {
                result = ESP_FAIL;
                goto finished;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        result = drive_forward(0.5f, DRIVE_SPEED_PERCENT,
                               WATER_HEADING_DEG, &next_x_steps);
        if (result != ESP_OK) goto finished;
        next_advance = now_ms() + WATER_ADVANCE_PERIOD_MS;
    }
    result = ESP_OK;

finished:
    drive_control_autonomous_end();
    stepper_link_quick_stop();
    ESP_LOGI(TAG, "water_garden finished: %s", esp_err_to_name(result));
    return result;
}
