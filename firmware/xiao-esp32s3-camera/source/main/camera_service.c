#include "camera_service.h"

#include <string.h>

#include "board_config.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "camera";
static portMUX_TYPE s_camera_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_stream_fps = 20;
static int s_led_duty;
static int s_stream_clients;

static void apply_led(int duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

esp_err_t camera_service_init(void)
{
    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_UXGA,
        .jpeg_quality = 14,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = -1,
    };

    if (esp_psram_is_initialized()) {
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        ESP_LOGW(TAG, "PSRAM unavailable; using one DRAM frame buffer");
    }

    ESP_RETURN_ON_ERROR(esp_camera_init(&config), TAG, "camera initialization failed");
    sensor_t *sensor = esp_camera_sensor_get();
    ESP_RETURN_ON_FALSE(sensor, ESP_FAIL, TAG, "camera sensor unavailable");
    if (sensor->id.PID == OV3660_PID) {
        sensor->set_vflip(sensor, 1);
        sensor->set_brightness(sensor, 1);
        sensor->set_saturation(sensor, -2);
    }
    sensor->set_framesize(sensor, FRAMESIZE_QVGA);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel = {
        .gpio_num = CAM_PIN_LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "LED timer failed");
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "LED channel failed");
    ESP_LOGI(TAG, "OV%04x ready at QVGA, JPEG quality 14", sensor->id.PID);
    return ESP_OK;
}

sensor_t *camera_service_sensor(void)
{
    return esp_camera_sensor_get();
}

int camera_service_get_fps(void)
{
    portENTER_CRITICAL(&s_camera_lock);
    int fps = s_stream_fps;
    portEXIT_CRITICAL(&s_camera_lock);
    return fps;
}

void camera_service_set_fps(int fps)
{
    if (fps != 0 && fps < 5) fps = 5;
    if (fps > 30) fps = 30;
    portENTER_CRITICAL(&s_camera_lock);
    s_stream_fps = fps;
    portEXIT_CRITICAL(&s_camera_lock);
}

void camera_service_set_led(int duty)
{
    if (duty < 0) duty = 0;
    if (duty > 255) duty = 255;
    portENTER_CRITICAL(&s_camera_lock);
    s_led_duty = duty;
    int applied_duty = s_stream_clients > 0 ? s_led_duty : 0;
    portEXIT_CRITICAL(&s_camera_lock);
    apply_led(applied_duty);
}

int camera_service_get_led(void)
{
    portENTER_CRITICAL(&s_camera_lock);
    int duty = s_led_duty;
    portEXIT_CRITICAL(&s_camera_lock);
    return duty;
}

void camera_service_stream_started(void)
{
    portENTER_CRITICAL(&s_camera_lock);
    ++s_stream_clients;
    int applied_duty = s_led_duty;
    portEXIT_CRITICAL(&s_camera_lock);
    apply_led(applied_duty);
}

void camera_service_stream_stopped(void)
{
    portENTER_CRITICAL(&s_camera_lock);
    if (s_stream_clients > 0) --s_stream_clients;
    int applied_duty = s_stream_clients > 0 ? s_led_duty : 0;
    portEXIT_CRITICAL(&s_camera_lock);
    apply_led(applied_duty);
}

bool camera_service_parse_framesize(const char *name, framesize_t *result)
{
    if (!name || !result) return false;
    if (!strcmp(name, "qvga")) *result = FRAMESIZE_QVGA;
    else if (!strcmp(name, "vga")) *result = FRAMESIZE_VGA;
    else if (!strcmp(name, "svga")) *result = FRAMESIZE_SVGA;
    else if (!strcmp(name, "xga")) *result = FRAMESIZE_XGA;
    else if (!strcmp(name, "sxga")) *result = FRAMESIZE_SXGA;
    else if (!strcmp(name, "uxga")) *result = FRAMESIZE_UXGA;
    else if (!strcmp(name, "qxga")) *result = FRAMESIZE_QXGA;
    else return false;
    return true;
}

const char *camera_service_framesize_name(framesize_t size)
{
    switch (size) {
        case FRAMESIZE_QVGA: return "qvga";
        case FRAMESIZE_VGA: return "vga";
        case FRAMESIZE_SVGA: return "svga";
        case FRAMESIZE_XGA: return "xga";
        case FRAMESIZE_SXGA: return "sxga";
        case FRAMESIZE_UXGA: return "uxga";
        case FRAMESIZE_QXGA: return "qxga";
        default: return "qvga";
    }
}
