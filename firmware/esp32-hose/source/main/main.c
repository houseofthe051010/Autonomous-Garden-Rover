#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define WIFI_CHANNEL 6
#define SERVO_GPIO 14
#define POT_GPIO 35
#define POT_CHANNEL ADC_CHANNEL_7

#define SERVO_NEUTRAL_US 1500
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_PERIOD_US 20000
#define CONTROL_TIMEOUT_US 350000
#define TELEMETRY_PERIOD_US 100000
#define ADC_PERIOD_MS 20
#define ADC_FILTER_SAMPLES 5

#define CONTROL_PACKET_SIZE 16
#define TELEMETRY_PACKET_SIZE 24
#define CONTROL_MAGIC 0xC1
#define TELEMETRY_MAGIC 0xC2
#define PROTOCOL_VERSION 1
#define FLAG_ENABLE (1U << 0)
#define FLAG_RESET_ZERO (1U << 1)
#define TELEMETRY_ENABLED (1U << 0)
#define TELEMETRY_WATCHDOG (1U << 1)
#define TELEMETRY_DEAD_ZONE (1U << 2)

// The measured potentiometer falls as pulse width rises above neutral.
#define POT_DEAD_MAX 80
#define POT_WRAP_MIN_DELTA 300

static const char *TAG = "hose_espnow";
static const uint8_t controller_mac[ESP_NOW_ETH_ALEN] = {
    0x68, 0x09, 0x47, 0x5c, 0x04, 0xc4,
};

static adc_oneshot_unit_handle_t adc_handle;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static int8_t requested_direction;
static uint16_t requested_speed = 100;
static bool requested_enable;
static bool reset_pending;
static int64_t last_control_us;
static uint32_t last_control_sequence;
static bool sequence_valid;
static uint32_t valid_controls;
static uint32_t rejected_controls;
static uint32_t telemetry_sent;
static uint32_t telemetry_delivered;

static uint16_t adc_raw;
static uint16_t adc_filtered;
static int32_t wrap_count;
static int32_t zero_wrap_count;
static int32_t turns_milli;
static uint16_t last_valid_adc;
static uint16_t dead_entry_adc;
static bool have_valid_adc;
static bool dead_entry_valid;
static bool in_dead_zone;
static int8_t last_motion_direction;
static int64_t last_motion_us;
static int current_pulse_us = SERVO_NEUTRAL_US;
static bool watchdog_stopped = true;

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
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

static uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t index = 0; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                    : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t pulse_to_duty(int pulse_us)
{
    return (uint32_t)(((uint64_t)pulse_us * ((1U << 16) - 1U))
        / SERVO_PERIOD_US);
}

static void set_servo_pulse(int pulse_us)
{
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;
    current_pulse_us = pulse_us;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
        pulse_to_duty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static int compare_int(const void *left, const void *right)
{
    const int a = *(const int *)left;
    const int b = *(const int *)right;
    return (a > b) - (a < b);
}

static uint16_t read_adc_median(void)
{
    int values[ADC_FILTER_SAMPLES];
    for (int index = 0; index < ADC_FILTER_SAMPLES; ++index) {
        if (adc_oneshot_read(adc_handle, POT_CHANNEL, &values[index]) != ESP_OK) {
            values[index] = adc_raw;
        }
    }
    qsort(values, ADC_FILTER_SAMPLES, sizeof(values[0]), compare_int);
    return (uint16_t)values[ADC_FILTER_SAMPLES / 2];
}

static void update_position(uint16_t raw, int64_t now)
{
    adc_raw = raw;
    adc_filtered = adc_filtered == 0 ? raw
        : (uint16_t)(((uint32_t)adc_filtered * 3U + raw + 2U) / 4U);
    const bool dead = raw <= POT_DEAD_MAX;
    int8_t direction = requested_direction;
    if (direction == 0 && now - last_motion_us <= 1000000) {
        direction = last_motion_direction;
    }

    if (dead) {
        if (!in_dead_zone && have_valid_adc) {
            dead_entry_adc = last_valid_adc;
            dead_entry_valid = true;
        }
        in_dead_zone = true;
        return;
    }

    if (in_dead_zone && dead_entry_valid) {
        if (direction > 0
            && raw > (uint16_t)(dead_entry_adc + POT_WRAP_MIN_DELTA)) {
            wrap_count++;
        } else if (direction < 0
                   && dead_entry_adc > raw + POT_WRAP_MIN_DELTA) {
            wrap_count--;
        }
    }
    in_dead_zone = false;
    dead_entry_valid = false;
    have_valid_adc = true;
    last_valid_adc = raw;
    turns_milli = (wrap_count - zero_wrap_count) * 1000;
}

static bool valid_control_packet(const uint8_t *data, int length,
    uint32_t *sequence)
{
    if (length != CONTROL_PACKET_SIZE || data[0] != CONTROL_MAGIC
        || data[1] != PROTOCOL_VERSION || data[15] != PROTOCOL_VERSION
        || read_u16(data + 12) != crc16_ccitt(data, 12)) {
        return false;
    }
    const int8_t direction = (int8_t)data[4];
    const uint16_t speed = read_u16(data + 6);
    if (direction < -1 || direction > 1 || speed < 100 || speed > 1000
        || speed % 100 != 0 || (data[5] & ~(FLAG_ENABLE | FLAG_RESET_ZERO))) {
        return false;
    }
    *sequence = read_u32(data + 8);
    return true;
}

static void receive_callback(const esp_now_recv_info_t *info,
    const uint8_t *data, int length)
{
    uint32_t sequence = 0;
    if (!info || !data || memcmp(info->src_addr, controller_mac, ESP_NOW_ETH_ALEN)
        || !valid_control_packet(data, length, &sequence)) {
        rejected_controls++;
        return;
    }

    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&state_mux);
    const bool fresh = !sequence_valid
        || (int32_t)(sequence - last_control_sequence) > 0
        || now - last_control_us > CONTROL_TIMEOUT_US;
    if (fresh) {
        last_control_sequence = sequence;
        sequence_valid = true;
        last_control_us = now;
        requested_enable = (data[5] & FLAG_ENABLE) != 0;
        requested_direction = requested_enable ? (int8_t)data[4] : 0;
        requested_speed = read_u16(data + 6);
        if (data[5] & FLAG_RESET_ZERO) reset_pending = true;
        valid_controls++;
    }
    portEXIT_CRITICAL(&state_mux);
}

static void send_callback(const esp_now_send_info_t *info,
    esp_now_send_status_t status)
{
    (void)info;
    if (status == ESP_NOW_SEND_SUCCESS) telemetry_delivered++;
}

static void send_telemetry(int64_t now)
{
    uint8_t packet[TELEMETRY_PACKET_SIZE] = {0};
    uint32_t sequence;
    int32_t position;
    uint16_t raw;
    uint16_t filtered;
    uint16_t speed;
    uint16_t pulse;
    int8_t direction;
    uint8_t flags = 0;
    int64_t control_us;

    portENTER_CRITICAL(&state_mux);
    sequence = last_control_sequence;
    position = turns_milli;
    raw = adc_raw;
    filtered = adc_filtered;
    speed = requested_speed;
    pulse = (uint16_t)current_pulse_us;
    direction = requested_direction;
    control_us = last_control_us;
    if (requested_enable) flags |= TELEMETRY_ENABLED;
    if (watchdog_stopped) flags |= TELEMETRY_WATCHDOG;
    if (in_dead_zone) flags |= TELEMETRY_DEAD_ZONE;
    portEXIT_CRITICAL(&state_mux);

    packet[0] = TELEMETRY_MAGIC;
    packet[1] = PROTOCOL_VERSION;
    write_u16(packet + 2, (uint16_t)sequence);
    write_u16(packet + 4, raw);
    write_u16(packet + 6, filtered);
    write_u32(packet + 8, (uint32_t)position);
    write_u16(packet + 12, pulse);
    packet[14] = (uint8_t)direction;
    packet[15] = flags;
    write_u16(packet + 16, speed);
    uint32_t age_ms = control_us ? (uint32_t)((now - control_us) / 1000) : 65535;
    if (age_ms > 65535) age_ms = 65535;
    write_u16(packet + 18, (uint16_t)age_ms);
    write_u16(packet + 20, crc16_ccitt(packet, 20));
    packet[23] = PROTOCOL_VERSION;
    if (esp_now_send(controller_mac, packet, sizeof(packet)) == ESP_OK) {
        telemetry_sent++;
    }
}

static void control_task(void *argument)
{
    (void)argument;
    int64_t last_telemetry_us = 0;
    int64_t last_log_us = 0;
    while (true) {
        const int64_t now = esp_timer_get_time();
        const uint16_t raw = read_adc_median();

        portENTER_CRITICAL(&state_mux);
        bool enabled = requested_enable && last_control_us
            && now - last_control_us <= CONTROL_TIMEOUT_US;
        int8_t direction = enabled ? requested_direction : 0;
        uint16_t speed = requested_speed;
        if (reset_pending) {
            zero_wrap_count = wrap_count;
            turns_milli = 0;
            reset_pending = false;
        }
        if (direction != 0) {
            last_motion_direction = direction;
            last_motion_us = now;
        }
        watchdog_stopped = !enabled;
        update_position(raw, now);
        portEXIT_CRITICAL(&state_mux);

        const int pulse = direction == 0 ? SERVO_NEUTRAL_US
            : SERVO_NEUTRAL_US + direction * (int)speed;
        set_servo_pulse(pulse);

        if (now - last_telemetry_us >= TELEMETRY_PERIOD_US) {
            send_telemetry(now);
            last_telemetry_us = now;
        }
        if (now - last_log_us >= 1000000) {
            ESP_LOGI(TAG,
                "enabled=%d dir=%d speed=%u pulse=%d adc=%u filtered=%u turns=%+.3f dead=%d controls=%lu rejected=%lu telemetry=%lu/%lu",
                enabled, direction, speed, current_pulse_us, adc_raw, adc_filtered,
                turns_milli / 1000.0, in_dead_zone,
                (unsigned long)valid_controls, (unsigned long)rejected_controls,
                (unsigned long)telemetry_delivered, (unsigned long)telemetry_sent);
            last_log_us = now;
        }
        vTaskDelay(pdMS_TO_TICKS(ADC_PERIOD_MS));
    }
}

static void initialize_servo(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    const ledc_channel_config_t channel = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = pulse_to_duty(SERVO_NEUTRAL_US),
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    set_servo_pulse(SERVO_NEUTRAL_US);
}

static void initialize_adc(void)
{
    const adc_oneshot_unit_init_cfg_t unit = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc_handle));
    const adc_oneshot_chan_cfg_t channel = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_CHANNEL, &channel));
}

static void initialize_radio(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(receive_callback));
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_callback));
    esp_now_peer_info_t peer = {
        .channel = WIFI_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, controller_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    esp_now_rate_config_t rate = {
        .phymode = WIFI_PHY_MODE_LR,
        .rate = WIFI_PHY_RATE_LORA_250K,
        .ersu = false,
        .dcm = false,
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(controller_mac, &rate));
}

void app_main(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);

    initialize_servo();
    initialize_adc();
    initialize_radio();
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG,
        "ready MAC=%02X:%02X:%02X:%02X:%02X:%02X channel=%d GPIO%d servo neutral; GPIO%d ADC",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], WIFI_CHANNEL,
        SERVO_GPIO, POT_GPIO);
    ESP_LOGI(TAG, "controller=%02X:%02X:%02X:%02X:%02X:%02X watchdog=%d ms",
        controller_mac[0], controller_mac[1], controller_mac[2],
        controller_mac[3], controller_mac[4], controller_mac[5],
        CONTROL_TIMEOUT_US / 1000);

    BaseType_t created = xTaskCreatePinnedToCore(
        control_task, "hose_control", 4096, NULL, 10, NULL, 1);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
