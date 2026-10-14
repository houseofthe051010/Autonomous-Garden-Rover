#include "audio_service.h"

#include <math.h>
#include <string.h>

#include "driver/i2s_pdm.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

#define AUDIO_QUEUE_BLOCKS 8
#define AUDIO_GAIN_MIN_DB (-12.0f)
#define AUDIO_GAIN_MAX_DB 30.0f

static const char *TAG = "microphone";
static QueueHandle_t s_audio_queue;
static i2s_chan_handle_t s_rx_channel;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static audio_status_t s_status = {.gain_db = 0.0f};

static float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void load_gain(void)
{
    nvs_handle_t nvs;
    int32_t gain_tenths = 0;
    if (nvs_open("camera", NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_i32(nvs, "mic_gain10", &gain_tenths) == ESP_OK) {
            s_status.gain_db = clamp_float(gain_tenths / 10.0f,
                                           AUDIO_GAIN_MIN_DB, AUDIO_GAIN_MAX_DB);
        }
        nvs_close(nvs);
    }
}

static void microphone_task(void *arg)
{
    (void)arg;
    int16_t raw[MIC_BLOCK_SAMPLES];
    uint32_t level_samples = 0;
    uint32_t level_clipped = 0;
    double level_energy = 0.0;
    int32_t level_peak = 0;

    while (true) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(s_rx_channel, raw, sizeof(raw),
                                         &bytes_read, pdMS_TO_TICKS(1000));
        if (err != ESP_OK || bytes_read == 0) {
            ESP_LOGW(TAG, "PDM read failed: %s", esp_err_to_name(err));
            continue;
        }

        audio_block_t block = {.count = bytes_read / sizeof(int16_t)};
        float gain_db;
        bool listener;
        portENTER_CRITICAL(&s_status_lock);
        gain_db = s_status.gain_db;
        listener = s_status.listener_active;
        block.sequence = ++s_status.sequence;
        portEXIT_CRITICAL(&s_status_lock);

        const float gain = powf(10.0f, gain_db / 20.0f);
        for (size_t i = 0; i < block.count; ++i) {
            int32_t value = lroundf(raw[i] * gain);
            if (value > INT16_MAX) {
                value = INT16_MAX;
                ++level_clipped;
            } else if (value < INT16_MIN) {
                value = INT16_MIN;
                ++level_clipped;
            }
            block.samples[i] = (int16_t)value;
            const int32_t magnitude = value < 0 ? -value : value;
            if (magnitude > level_peak) level_peak = magnitude;
            level_energy += (double)value * (double)value;
        }
        level_samples += block.count;

        if (level_samples >= MIC_SAMPLE_RATE / 4) {
            const double rms = sqrt(level_energy / level_samples);
            const float rms_dbfs = rms > 0.5 ? 20.0f * log10f((float)(rms / 32768.0)) : -96.0f;
            const float peak_dbfs = level_peak > 0 ? 20.0f * log10f(level_peak / 32768.0f) : -96.0f;
            portENTER_CRITICAL(&s_status_lock);
            s_status.rms_dbfs = rms_dbfs;
            s_status.peak_dbfs = peak_dbfs;
            s_status.clipped_samples += level_clipped;
            portEXIT_CRITICAL(&s_status_lock);
            level_samples = 0;
            level_clipped = 0;
            level_energy = 0.0;
            level_peak = 0;
        }

        if (!listener) continue;
        if (xQueueSend(s_audio_queue, &block, 0) != pdTRUE) {
            audio_block_t discarded;
            (void)xQueueReceive(s_audio_queue, &discarded, 0);
            (void)xQueueSend(s_audio_queue, &block, 0);
            portENTER_CRITICAL(&s_status_lock);
            ++s_status.dropped_blocks;
            portEXIT_CRITICAL(&s_status_lock);
        }
    }
}

esp_err_t audio_service_init(void)
{
    load_gain();
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_BLOCKS, sizeof(audio_block_t));
    ESP_RETURN_ON_FALSE(s_audio_queue, ESP_ERR_NO_MEM, TAG, "audio queue allocation failed");

    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = 6;
    channel_config.dma_frame_num = MIC_BLOCK_SAMPLES;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&channel_config, NULL, &s_rx_channel), TAG,
                        "could not allocate PDM RX channel");

    i2s_pdm_rx_config_t pdm_config = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = MIC_PIN_CLK,
            .din = MIC_PIN_DATA,
            .invert_flags = {.clk_inv = false},
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(s_rx_channel, &pdm_config), TAG,
                        "PDM RX configuration failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_channel), TAG, "PDM RX enable failed");

    portENTER_CRITICAL(&s_status_lock);
    s_status.ready = true;
    portEXIT_CRITICAL(&s_status_lock);
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(microphone_task, "pdm_capture", 4096, NULL,
                                                6, NULL, 1) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "microphone task allocation failed");
    ESP_LOGI(TAG, "PDM PCM: %d Hz, 16-bit mono, CLK GPIO%d, DATA GPIO%d",
             MIC_SAMPLE_RATE, MIC_PIN_CLK, MIC_PIN_DATA);
    return ESP_OK;
}

bool audio_service_take(audio_block_t *block, uint32_t timeout_ms)
{
    return block && xQueueReceive(s_audio_queue, block, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void audio_service_set_listener(bool active)
{
    portENTER_CRITICAL(&s_status_lock);
    s_status.listener_active = active;
    portEXIT_CRITICAL(&s_status_lock);
    xQueueReset(s_audio_queue);
}

esp_err_t audio_service_set_gain(float gain_db)
{
    gain_db = clamp_float(gain_db, AUDIO_GAIN_MIN_DB, AUDIO_GAIN_MAX_DB);
    portENTER_CRITICAL(&s_status_lock);
    s_status.gain_db = gain_db;
    s_status.clipped_samples = 0;
    portEXIT_CRITICAL(&s_status_lock);

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("camera", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_i32(nvs, "mic_gain10", lroundf(gain_db * 10.0f));
        if (err == ESP_OK) err = nvs_commit(nvs);
        nvs_close(nvs);
    }
    return err;
}

void audio_service_get_status(audio_status_t *status)
{
    if (!status) return;
    portENTER_CRITICAL(&s_status_lock);
    *status = s_status;
    portEXIT_CRITICAL(&s_status_lock);
}
