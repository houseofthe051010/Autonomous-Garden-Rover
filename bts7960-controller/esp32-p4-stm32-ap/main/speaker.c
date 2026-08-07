#include "speaker.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/sdmmc_host.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "es8311_codec.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "sdmmc_cmd.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "wear_levelling.h"

#define AUDIO_I2C_SDA GPIO_NUM_7
#define AUDIO_I2C_SCL GPIO_NUM_8
#define AUDIO_I2S_DOUT GPIO_NUM_9
#define AUDIO_I2S_WS GPIO_NUM_10
#define AUDIO_I2S_DIN GPIO_NUM_11
#define AUDIO_I2S_BCLK GPIO_NUM_12
#define AUDIO_I2S_MCLK GPIO_NUM_13
#define AUDIO_PA_ENABLE GPIO_NUM_53

#define SD_CLK GPIO_NUM_43
#define SD_CMD GPIO_NUM_44
#define SD_D0 GPIO_NUM_39
#define SD_D1 GPIO_NUM_40
#define SD_D2 GPIO_NUM_41
#define SD_D3 GPIO_NUM_42

#define MOUNT_POINT "/storage"
#define SOUND_DIR MOUNT_POINT "/sounds"
#define RECORDING_DIR MOUNT_POINT "/recordings"
#define ASSIGNMENT_MAX 63
#define MAX_UPLOAD_BYTES (64U * 1024U * 1024U)
#define TONE_DURATION_MS 700
#define TONE_SAMPLE_RATE 16000
#define RECORD_SAMPLE_RATE 16000
#define RECORD_CHANNELS 2
#define RECORD_MAX_SECONDS 600
#define DEFAULT_MIC_GAIN_DB 24
#define MAX_MIC_GAIN_DB 42
#define DEFAULT_SPEAKER_VOLUME 100
#define MAX_SPEAKER_BOOST_DB 6

#define AUDIO_KIND_NONE 0
#define AUDIO_KIND_TONE 1
#define AUDIO_KIND_FILE 2
#define AUDIO_KIND_RECORDING 3

#define PLAY_SOURCE_LIBRARY 0
#define PLAY_SOURCE_RECORDING 1

typedef struct {
    uint8_t button;
    uint8_t source;
    char value[64];
} play_request_t;

typedef struct {
    char name[64];
} record_request_t;

typedef struct {
    uint32_t rate;
    uint16_t channels;
    uint32_t data_size;
    long data_offset;
} wav_info_t;

static const char *TAG = "speaker";
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t storage_mutex;
static SemaphoreHandle_t audio_mutex;
static QueueHandle_t play_queue;
static QueueHandle_t record_queue;
static speaker_status_t state;
static uint16_t previous_buttons;
static i2c_master_bus_handle_t i2c_bus;
static i2s_chan_handle_t i2s_tx;
static i2s_chan_handle_t i2s_rx;
static esp_codec_dev_handle_t codec;
static sdmmc_card_t *sd_card;
static sd_pwr_ctrl_handle_t sd_power;
static wl_handle_t wl_handle = WL_INVALID_HANDLE;
static bool playback_stop_requested;
static bool recording_stop_requested;
static uint8_t current_source;
static uint8_t mic_gain_db = DEFAULT_MIC_GAIN_DB;
static uint8_t speaker_volume = DEFAULT_SPEAKER_VOLUME;
static uint8_t speaker_boost_db;
static int applied_speaker_volume = -1;

static uint16_t read_u16(FILE *file)
{
    uint8_t value[2];
    return fread(value, 1, sizeof(value), file) == sizeof(value)
        ? (uint16_t)value[0] | ((uint16_t)value[1] << 8) : 0;
}

static uint32_t read_u32(FILE *file)
{
    uint8_t value[4];
    return fread(value, 1, sizeof(value), file) == sizeof(value)
        ? (uint32_t)value[0] | ((uint32_t)value[1] << 8) |
          ((uint32_t)value[2] << 16) | ((uint32_t)value[3] << 24) : 0;
}

static void set_error(const char *message)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.error, sizeof(state.error), "%s", message ? message : "");
    xSemaphoreGive(state_mutex);
    if (message && message[0]) ESP_LOGE(TAG, "%s", message);
}

static void refresh_storage_space(void)
{
    uint64_t total = 0;
    uint64_t free = 0;
    if (!state.storage_mounted ||
        esp_vfs_fat_info(MOUNT_POINT, &total, &free) != ESP_OK) {
        return;
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.total_bytes = total;
    state.free_bytes = free;
    xSemaphoreGive(state_mutex);
}

static bool valid_filename(const char *name)
{
    size_t length = name ? strlen(name) : 0;
    if (length < 5 || length > ASSIGNMENT_MAX || name[0] == '.') return false;
    if (strcasecmp(name + length - 4, ".wav") != 0) return false;
    for (size_t i = 0; i < length; ++i) {
        char c = name[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.')) {
            return false;
        }
    }
    return true;
}

static bool valid_assignment(const char *value)
{
    if (!value) return false;
    if (strcmp(value, "tone:150") == 0 || strcmp(value, "tone:600") == 0) return true;
    return valid_filename(value);
}

static void file_path(char *output, size_t output_size, const char *name)
{
    snprintf(output, output_size, SOUND_DIR "/%s", name);
}

static void recording_path(char *output, size_t output_size, const char *name)
{
    snprintf(output, output_size, RECORDING_DIR "/%s", name);
}

static void put_u16(uint8_t *output, uint16_t value)
{
    output[0] = value & 0xff;
    output[1] = value >> 8;
}

static void put_u32(uint8_t *output, uint32_t value)
{
    output[0] = value & 0xff;
    output[1] = (value >> 8) & 0xff;
    output[2] = (value >> 16) & 0xff;
    output[3] = value >> 24;
}

static bool write_wav_header(FILE *file, uint32_t rate, uint16_t channels,
                             uint32_t data_size)
{
    uint8_t header[44] = {0};
    memcpy(header, "RIFF", 4);
    put_u32(header + 4, 36 + data_size);
    memcpy(header + 8, "WAVEfmt ", 8);
    put_u32(header + 16, 16);
    put_u16(header + 20, 1);
    put_u16(header + 22, channels);
    put_u32(header + 24, rate);
    put_u32(header + 28, rate * channels * 2);
    put_u16(header + 32, channels * 2);
    put_u16(header + 34, 16);
    memcpy(header + 36, "data", 4);
    put_u32(header + 40, data_size);
    return fseek(file, 0, SEEK_SET) == 0 &&
           fwrite(header, 1, sizeof(header), file) == sizeof(header);
}

static bool parse_wav(FILE *file, wav_info_t *info)
{
    char tag[4];
    if (!file || !info || fread(tag, 1, 4, file) != 4 || memcmp(tag, "RIFF", 4) != 0) {
        return false;
    }
    (void)read_u32(file);
    if (fread(tag, 1, 4, file) != 4 || memcmp(tag, "WAVE", 4) != 0) return false;

    bool have_format = false;
    bool have_data = false;
    memset(info, 0, sizeof(*info));
    for (unsigned chunk = 0; chunk < 32 && !have_data; ++chunk) {
        if (fread(tag, 1, 4, file) != 4) break;
        uint32_t size = read_u32(file);
        long chunk_start = ftell(file);
        if (memcmp(tag, "fmt ", 4) == 0 && size >= 16) {
            uint16_t format = read_u16(file);
            info->channels = read_u16(file);
            info->rate = read_u32(file);
            (void)read_u32(file);
            (void)read_u16(file);
            uint16_t bits = read_u16(file);
            have_format = format == 1 && (info->channels == 1 || info->channels == 2) &&
                bits == 16 && info->rate >= 8000 && info->rate <= 48000;
        } else if (memcmp(tag, "data", 4) == 0) {
            info->data_offset = chunk_start;
            info->data_size = size;
            have_data = true;
            break;
        }
        if (fseek(file, chunk_start + size + (size & 1U), SEEK_SET) != 0) break;
    }
    if (!have_format || !have_data || info->data_size == 0) return false;
    return fseek(file, info->data_offset, SEEK_SET) == 0;
}

static esp_err_t sdmmc_init_already_done(void)
{
    return ESP_OK;
}

static esp_err_t sdmmc_deinit_keep_host(void)
{
    return ESP_OK;
}

static esp_err_t mount_storage(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t sd_mount = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    sd_pwr_ctrl_ldo_config_t power_config = {.ldo_chan_id = 4};
    esp_err_t power_result = sd_pwr_ctrl_new_on_chip_ldo(&power_config, &sd_power);
    if (power_result == ESP_OK) {
        host.pwr_ctrl_handle = sd_power;
    } else {
        ESP_LOGW(TAG, "TF card LDO unavailable: %s", esp_err_to_name(power_result));
    }
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    host.init = sdmmc_init_already_done;
    host.deinit = sdmmc_deinit_keep_host;
#endif
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    slot.clk = SD_CLK;
    slot.cmd = SD_CMD;
    slot.d0 = SD_D0;
    slot.d1 = SD_D1;
    slot.d2 = SD_D2;
    slot.d3 = SD_D3;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t result = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &sd_mount, &sd_card);
    if (result == ESP_OK) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.storage_mounted = true;
        state.storage_is_sd = true;
        xSemaphoreGive(state_mutex);
        ESP_LOGI(TAG, "TF card mounted at %s", MOUNT_POINT);
        return ESP_OK;
    }

    char storage_warning[128];
    snprintf(storage_warning, sizeof(storage_warning),
             "microSD unavailable (%s); using internal flash", esp_err_to_name(result));
    ESP_LOGW(TAG, "%s", storage_warning);
    if (sd_power) {
        sd_pwr_ctrl_del_on_chip_ldo(sd_power);
        sd_power = NULL;
    }
    const esp_vfs_fat_mount_config_t flash_mount = {
        .format_if_mount_failed = true,
        .max_files = 8,
        .allocation_unit_size = 4096,
    };
    result = esp_vfs_fat_spiflash_mount_rw_wl(
        MOUNT_POINT, "storage", &flash_mount, &wl_handle);
    if (result == ESP_OK) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.storage_mounted = true;
        state.storage_is_sd = false;
        xSemaphoreGive(state_mutex);
        ESP_LOGI(TAG, "Internal FAT storage mounted at %s", MOUNT_POINT);
        set_error(storage_warning);
    }
    return result;
}

static esp_err_t initialize_codec(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = AUDIO_I2C_SCL,
        .sda_io_num = AUDIO_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus), TAG, "audio I2C");

    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_config.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&channel_config, &i2s_tx, &i2s_rx), TAG,
                        "audio I2S channels");
    i2s_std_config_t i2s_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(TONE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = AUDIO_I2S_MCLK,
            .bclk = AUDIO_I2S_BCLK,
            .ws = AUDIO_I2S_WS,
            .dout = AUDIO_I2S_DOUT,
            .din = AUDIO_I2S_DIN,
        },
    };
    i2s_config.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(i2s_tx, &i2s_config), TAG, "audio I2S mode");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(i2s_rx, &i2s_config), TAG,
                        "microphone I2S mode");

    audio_codec_i2s_cfg_t data_config = {.tx_handle = i2s_tx, .rx_handle = i2s_rx};
    const audio_codec_data_if_t *data = audio_codec_new_i2s_data(&data_config);
    audio_codec_i2c_cfg_t control_config = {
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus,
        .clock_speed_hz = 400000,
    };
    const audio_codec_ctrl_if_t *control = audio_codec_new_i2c_ctrl(&control_config);
    const audio_codec_gpio_if_t *gpio = audio_codec_new_gpio();
    if (!data || !control || !gpio) return ESP_ERR_NO_MEM;

    es8311_codec_cfg_t codec_config = {
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .ctrl_if = control,
        .gpio_if = gpio,
        .pa_pin = AUDIO_PA_ENABLE,
        .pa_reverted = false,
        .use_mclk = true,
    };
    const audio_codec_if_t *codec_interface = es8311_codec_new(&codec_config);
    if (!codec_interface) return ESP_ERR_NO_MEM;
    esp_codec_dev_cfg_t device_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec_interface,
        .data_if = data,
    };
    codec = esp_codec_dev_new(&device_config);
    if (!codec) return ESP_ERR_NO_MEM;
    if (esp_codec_dev_set_out_vol(codec, speaker_volume) != ESP_CODEC_DEV_OK) return ESP_FAIL;
    if (esp_codec_dev_set_in_gain(codec, mic_gain_db) != ESP_CODEC_DEV_OK) return ESP_FAIL;
    ESP_LOGI(TAG, "ES8311 speaker and microphone ready");
    return ESP_OK;
}

static bool open_codec(uint32_t rate)
{
    esp_codec_dev_sample_info_t format = {
        .sample_rate = rate,
        .channel = 2,
        .bits_per_sample = 16,
        .mclk_multiple = I2S_MCLK_MULTIPLE_384,
    };
    if (esp_codec_dev_open(codec, &format) != ESP_CODEC_DEV_OK) return false;
    if (esp_codec_dev_set_out_vol(codec, speaker_volume) != ESP_CODEC_DEV_OK) return false;
    applied_speaker_volume = speaker_volume;
    if (esp_codec_dev_set_in_gain(codec, mic_gain_db) != ESP_CODEC_DEV_OK) return false;
    esp_err_t tx_enabled = i2s_channel_enable(i2s_tx);
    esp_err_t rx_enabled = i2s_channel_enable(i2s_rx);
    return (tx_enabled == ESP_OK || tx_enabled == ESP_ERR_INVALID_STATE) &&
           (rx_enabled == ESP_OK || rx_enabled == ESP_ERR_INVALID_STATE);
}

static bool write_audio(const void *data, size_t size)
{
    const uint8_t *cursor = data;
    while (size) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        unsigned volume = speaker_volume;
        unsigned boost = speaker_boost_db;
        xSemaphoreGive(state_mutex);
        if (applied_speaker_volume != (int)volume) {
            if (esp_codec_dev_set_out_vol(codec, (int)volume) != ESP_CODEC_DEV_OK) {
                return false;
            }
            applied_speaker_volume = (int)volume;
        }
        size_t chunk = size > 2048 ? 2048 : size;
        const uint8_t *write_data = cursor;
        int16_t boosted[1024];
        if (boost) {
            int scale = boost == 3 ? 362 : 511;
            size_t samples = chunk / sizeof(int16_t);
            const int16_t *input = (const int16_t *)cursor;
            for (size_t i = 0; i < samples; ++i) {
                int value = ((int)input[i] * scale) / 256;
                if (value > INT16_MAX) value = INT16_MAX;
                if (value < INT16_MIN) value = INT16_MIN;
                boosted[i] = (int16_t)value;
            }
            write_data = (const uint8_t *)boosted;
        }
        size_t written = 0;
        esp_err_t result = i2s_channel_write(
            i2s_tx, write_data, chunk, &written, pdMS_TO_TICKS(2000));
        if (result != ESP_OK || written == 0) return false;
        cursor += written;
        size -= written;
    }
    return true;
}

static void set_playing(const play_request_t *request, uint8_t kind, uint16_t tone)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.playing = request != NULL;
    state.paused = false;
    state.source_button = request ? request->button : 0;
    state.kind = request ? kind : AUDIO_KIND_NONE;
    state.tone_hz = request ? tone : 0;
    current_source = request ? request->source : PLAY_SOURCE_LIBRARY;
    playback_stop_requested = false;
    snprintf(state.current, sizeof(state.current), "%s", request ? request->value : "");
    xSemaphoreGive(state_mutex);
}

typedef enum {
    PLAY_CONTINUE,
    PLAY_STOP,
    PLAY_REPLACE,
} play_action_t;

static play_action_t playback_control(play_request_t *replacement)
{
    while (true) {
        if (xQueueReceive(play_queue, replacement, 0) == pdTRUE) return PLAY_REPLACE;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool stop = playback_stop_requested;
        bool paused = state.paused;
        xSemaphoreGive(state_mutex);
        if (stop) return PLAY_STOP;
        if (!paused) return PLAY_CONTINUE;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static bool play_tone(const play_request_t *request, unsigned frequency,
                      play_request_t *replacement)
{
    static const int16_t sine[32] = {
        0, 5853, 11480, 16666, 21213, 24944, 27716, 29423,
        30000, 29423, 27716, 24944, 21213, 16666, 11480, 5853,
        0, -5853, -11480, -16666, -21213, -24944, -27716, -29423,
        -30000, -29423, -27716, -24944, -21213, -16666, -11480, -5853,
    };
    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    if (!open_codec(TONE_SAMPLE_RATE)) {
        xSemaphoreGive(audio_mutex);
        return false;
    }
    set_playing(request, AUDIO_KIND_TONE, frequency);
    uint32_t phase = 0;
    uint32_t increment = (uint32_t)(((uint64_t)frequency << 32) / TONE_SAMPLE_RATE);
    int16_t samples[512 * 2];
    unsigned remaining = TONE_SAMPLE_RATE * TONE_DURATION_MS / 1000;
    while (remaining) {
        unsigned count = remaining > 512 ? 512 : remaining;
        for (unsigned i = 0; i < count; ++i) {
            int16_t value = sine[phase >> 27];
            samples[i * 2] = value;
            samples[i * 2 + 1] = value;
            phase += increment;
        }
        if (!write_audio(samples, count * 4)) {
            set_error("I2S tone write timed out");
            break;
        }
        remaining -= count;
        play_action_t action = playback_control(replacement);
        if (action != PLAY_CONTINUE) {
            esp_codec_dev_close(codec);
            xSemaphoreGive(audio_mutex);
            return action == PLAY_REPLACE;
        }
    }
    esp_codec_dev_close(codec);
    xSemaphoreGive(audio_mutex);
    return false;
}

static bool play_wav(const play_request_t *request, play_request_t *replacement)
{
    char path[sizeof(RECORDING_DIR) + 70];
    if (request->source == PLAY_SOURCE_RECORDING) {
        recording_path(path, sizeof(path), request->value);
    } else {
        file_path(path, sizeof(path), request->value);
    }
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    FILE *file = fopen(path, "rb");
    wav_info_t wav;
    if (!file || !parse_wav(file, &wav)) {
        if (file) fclose(file);
        xSemaphoreGive(storage_mutex);
        set_error("Selected file is not supported 16-bit PCM WAV");
        return false;
    }
    uint8_t *input = heap_caps_malloc(2048, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int16_t *stereo = heap_caps_malloc(4096, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!input || !stereo) {
        free(input);
        free(stereo);
        fclose(file);
        xSemaphoreGive(storage_mutex);
        set_error("Not enough PSRAM for WAV playback");
        return false;
    }
    xSemaphoreGive(storage_mutex);
    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    if (!open_codec(wav.rate)) {
        free(input);
        free(stereo);
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        fclose(file);
        xSemaphoreGive(storage_mutex);
        xSemaphoreGive(audio_mutex);
        set_error("Could not start ES8311 playback");
        return false;
    }
    set_playing(request, request->source == PLAY_SOURCE_RECORDING
        ? AUDIO_KIND_RECORDING : AUDIO_KIND_FILE, 0);
    uint32_t remaining = wav.data_size;
    while (remaining) {
        size_t wanted = remaining > 2048 ? 2048 : remaining;
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        size_t received = fread(input, 1, wanted, file);
        xSemaphoreGive(storage_mutex);
        if (!received) break;
        void *output = input;
        size_t output_size = received;
        if (wav.channels == 1) {
            size_t count = received / 2;
            const int16_t *mono = (const int16_t *)input;
            for (size_t i = 0; i < count; ++i) {
                stereo[i * 2] = mono[i];
                stereo[i * 2 + 1] = mono[i];
            }
            output = stereo;
            output_size = count * 4;
        }
        if (!write_audio(output, output_size)) {
            set_error("I2S WAV write timed out");
            break;
        }
        remaining -= received;
        vTaskDelay(1);
        play_action_t action = playback_control(replacement);
        if (action != PLAY_CONTINUE) {
            free(input);
            free(stereo);
            xSemaphoreTake(storage_mutex, portMAX_DELAY);
            fclose(file);
            xSemaphoreGive(storage_mutex);
            esp_codec_dev_close(codec);
            xSemaphoreGive(audio_mutex);
            return action == PLAY_REPLACE;
        }
    }
    free(input);
    free(stereo);
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    fclose(file);
    xSemaphoreGive(storage_mutex);
    esp_codec_dev_close(codec);
    xSemaphoreGive(audio_mutex);
    return false;
}

static void playback_task(void *argument)
{
    (void)argument;
    play_request_t request;
    while (true) {
        xQueueReceive(play_queue, &request, portMAX_DELAY);
        bool replacement = true;
        while (replacement) {
            replacement = false;
            play_request_t next;
            if (strncmp(request.value, "tone:", 5) == 0) {
                unsigned frequency = (unsigned)atoi(request.value + 5);
                replacement = play_tone(&request, frequency, &next);
            } else {
                replacement = play_wav(&request, &next);
            }
            if (replacement) request = next;
        }
        set_playing(NULL, AUDIO_KIND_NONE, 0);
    }
}

static bool queue_value(uint8_t button, const char *value)
{
    if (!play_queue || !valid_assignment(value)) return false;
    play_request_t request = {.button = button, .source = PLAY_SOURCE_LIBRARY};
    snprintf(request.value, sizeof(request.value), "%s", value);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    recording_stop_requested = true;
    playback_stop_requested = false;
    state.paused = false;
    xSemaphoreGive(state_mutex);
    if (xQueueSend(play_queue, &request, 0) == pdTRUE) return true;
    play_request_t discarded;
    (void)xQueueReceive(play_queue, &discarded, 0);
    return xQueueSend(play_queue, &request, 0) == pdTRUE;
}

static bool queue_recording_playback(const char *name)
{
    if (!play_queue || !valid_filename(name)) return false;
    play_request_t request = {.source = PLAY_SOURCE_RECORDING};
    snprintf(request.value, sizeof(request.value), "%s", name);
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    recording_stop_requested = true;
    playback_stop_requested = false;
    state.paused = false;
    xSemaphoreGive(state_mutex);
    if (xQueueSend(play_queue, &request, 0) == pdTRUE) return true;
    play_request_t discarded;
    (void)xQueueReceive(play_queue, &discarded, 0);
    return xQueueSend(play_queue, &request, 0) == pdTRUE;
}

static void stop_playback(void)
{
    play_request_t discarded;
    while (xQueueReceive(play_queue, &discarded, 0) == pdTRUE) {}
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    playback_stop_requested = true;
    state.paused = false;
    xSemaphoreGive(state_mutex);
}

static void set_pause(bool paused)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.playing) state.paused = paused;
    xSemaphoreGive(state_mutex);
}

static void set_recording_state(const char *name, bool active, bool pending,
                                uint64_t bytes, uint32_t elapsed_ms)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.recording = active;
    state.recording_pending = pending;
    state.recording_bytes = bytes;
    state.recording_ms = elapsed_ms;
    snprintf(state.recording_name, sizeof(state.recording_name), "%s", name ? name : "");
    xSemaphoreGive(state_mutex);
}

static void recording_task(void *argument)
{
    (void)argument;
    record_request_t request;
    uint8_t *buffer = heap_caps_malloc(4096, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buffer) {
        set_error("Not enough PSRAM for microphone buffer");
        vTaskDelete(NULL);
    }
    while (true) {
        xQueueReceive(record_queue, &request, portMAX_DELAY);
        stop_playback();
        xSemaphoreTake(audio_mutex, portMAX_DELAY);

        char final_path[sizeof(RECORDING_DIR) + 70];
        char temporary[sizeof(RECORDING_DIR) + 76];
        recording_path(final_path, sizeof(final_path), request.name);
        snprintf(temporary, sizeof(temporary), "%s.part", final_path);
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        FILE *file = fopen(temporary, "wb+");
        bool okay = file && write_wav_header(
            file, RECORD_SAMPLE_RATE, RECORD_CHANNELS, 0);
        xSemaphoreGive(storage_mutex);
        if (!okay || !open_codec(RECORD_SAMPLE_RATE)) {
            if (file) {
                xSemaphoreTake(storage_mutex, portMAX_DELAY);
                fclose(file);
                unlink(temporary);
                xSemaphoreGive(storage_mutex);
            }
            set_recording_state(NULL, false, false, 0, 0);
            set_error("Could not start microphone recording");
            xSemaphoreGive(audio_mutex);
            continue;
        }

        int64_t started = esp_timer_get_time();
        uint64_t bytes = 0;
        set_recording_state(request.name, true, false, 0, 0);
        ESP_LOGI(TAG, "Recording %s", request.name);

        const uint64_t maximum = (uint64_t)RECORD_SAMPLE_RATE * RECORD_CHANNELS *
            2 * RECORD_MAX_SECONDS;
        while (bytes < maximum) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            bool stop = recording_stop_requested;
            xSemaphoreGive(state_mutex);
            if (stop) break;
            int result = esp_codec_dev_read(codec, buffer, 4096);
            if (result != ESP_CODEC_DEV_OK) {
                okay = false;
                set_error("ES8311 microphone read failed");
                break;
            }
            xSemaphoreTake(storage_mutex, portMAX_DELAY);
            size_t written = fwrite(buffer, 1, 4096, file);
            xSemaphoreGive(storage_mutex);
            if (written != 4096) {
                okay = false;
                set_error("Recording write failed or storage is full");
                break;
            }
            bytes += written;
            uint32_t elapsed = (uint32_t)((esp_timer_get_time() - started) / 1000);
            set_recording_state(request.name, true, false, bytes, elapsed);
        }

        esp_codec_dev_close(codec);
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        if (bytes > 0 && bytes <= UINT32_MAX) {
            okay = okay && write_wav_header(file, RECORD_SAMPLE_RATE,
                                             RECORD_CHANNELS, (uint32_t)bytes);
        } else {
            okay = false;
        }
        if (file) {
            fflush(file);
            fsync(fileno(file));
            fclose(file);
        }
        if (okay) {
            struct stat existing;
            okay = stat(final_path, &existing) != 0 &&
                    rename(temporary, final_path) == 0;
        }
        if (!okay) unlink(temporary);
        xSemaphoreGive(storage_mutex);

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (okay) {
            snprintf(state.last_recording, sizeof(state.last_recording), "%s", request.name);
            state.error[0] = 0;
        }
        state.recording = false;
        state.recording_pending = false;
        state.recording_name[0] = 0;
        xSemaphoreGive(state_mutex);
        refresh_storage_space();
        ESP_LOGI(TAG, "Recording %s: %llu bytes", okay ? "saved" : "failed",
                 (unsigned long long)bytes);
        xSemaphoreGive(audio_mutex);
    }
}

static bool choose_recording_name(const char *requested, char *output, size_t output_size)
{
    snprintf(output, output_size, "%s", requested);
    char path[sizeof(RECORDING_DIR) + 70];
    recording_path(path, sizeof(path), output);
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    struct stat info;
    bool available = stat(path, &info) != 0;
    xSemaphoreGive(storage_mutex);
    if (available) return true;

    size_t stem_length = strlen(requested) - 4;
    size_t digit_start = stem_length;
    while (digit_start > 0 && requested[digit_start - 1] >= '0' &&
           requested[digit_start - 1] <= '9') {
        --digit_start;
    }
    unsigned number = 2;
    int width = 2;
    bool had_number = digit_start < stem_length;
    if (had_number) {
        number = (unsigned)strtoul(requested + digit_start, NULL, 10) + 1;
        width = (int)(stem_length - digit_start);
    }
    size_t prefix_length = had_number ? digit_start : stem_length;
    for (unsigned attempt = 0; attempt < 10000; ++attempt, ++number) {
        if (had_number) {
            snprintf(output, output_size, "%.*s%0*u.wav", (int)prefix_length,
                     requested, width, number);
        } else {
            int usable = (int)output_size - 12;
            if ((int)prefix_length < usable) usable = (int)prefix_length;
            snprintf(output, output_size, "%.*s_%02u.wav", usable, requested, number);
        }
        if (!valid_filename(output)) continue;
        recording_path(path, sizeof(path), output);
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        available = stat(path, &info) != 0;
        xSemaphoreGive(storage_mutex);
        if (available) return true;
    }
    return false;
}

static bool start_recording(const char *name)
{
    if (!record_queue || !valid_filename(name)) return false;
    speaker_status_t snapshot;
    speaker_get_status(&snapshot);
    if (snapshot.recording || snapshot.recording_pending) return false;
    record_request_t request;
    if (!choose_recording_name(name, request.name, sizeof(request.name))) return false;
    stop_playback();
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    recording_stop_requested = false;
    state.recording_pending = true;
    snprintf(state.recording_name, sizeof(state.recording_name), "%s", request.name);
    xSemaphoreGive(state_mutex);
    if (xQueueSend(record_queue, &request, 0) == pdTRUE) return true;
    set_recording_state(NULL, false, false, 0, 0);
    return false;
}

static void stop_recording(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    recording_stop_requested = true;
    xSemaphoreGive(state_mutex);
}

static void load_assignments(void)
{
    snprintf(state.assignment[0], sizeof(state.assignment[0]), "tone:150");
    snprintf(state.assignment[1], sizeof(state.assignment[1]), "tone:600");
    nvs_handle_t nvs;
    state.mic_gain_db = mic_gain_db;
    state.speaker_volume = speaker_volume;
    state.speaker_boost_db = speaker_boost_db;
    if (nvs_open("speaker", NVS_READONLY, &nvs) != ESP_OK) return;
    for (int index = 0; index < 2; ++index) {
        char key[8];
        snprintf(key, sizeof(key), "button%d", index + 1);
        size_t size = sizeof(state.assignment[index]);
        char value[64];
        if (nvs_get_str(nvs, key, value, &size) == ESP_OK && valid_assignment(value)) {
            snprintf(state.assignment[index], sizeof(state.assignment[index]), "%s", value);
        }
    }
    uint8_t saved_gain = 0;
    if (nvs_get_u8(nvs, "mic_gain", &saved_gain) == ESP_OK &&
        saved_gain <= MAX_MIC_GAIN_DB && saved_gain % 6 == 0) {
        mic_gain_db = saved_gain;
        state.mic_gain_db = saved_gain;
    }
    uint8_t saved_volume = 0;
    if (nvs_get_u8(nvs, "volume", &saved_volume) == ESP_OK && saved_volume <= 100) {
        speaker_volume = saved_volume;
        state.speaker_volume = saved_volume;
    }
    uint8_t saved_boost = 0;
    if (nvs_get_u8(nvs, "boost", &saved_boost) == ESP_OK &&
        (saved_boost == 0 || saved_boost == 3 || saved_boost == 6)) {
        speaker_boost_db = saved_boost;
        state.speaker_boost_db = saved_boost;
    }
    nvs_close(nvs);
}

static bool save_mic_gain(unsigned gain)
{
    if (gain > MAX_MIC_GAIN_DB || gain % 6 != 0) return false;
    nvs_handle_t nvs;
    if (nvs_open("speaker", NVS_READWRITE, &nvs) != ESP_OK) return false;
    esp_err_t result = nvs_set_u8(nvs, "mic_gain", (uint8_t)gain);
    if (result == ESP_OK) result = nvs_commit(nvs);
    nvs_close(nvs);
    if (result != ESP_OK) return false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    mic_gain_db = (uint8_t)gain;
    state.mic_gain_db = (uint8_t)gain;
    xSemaphoreGive(state_mutex);
    return true;
}

static bool save_speaker_level(unsigned volume, unsigned boost)
{
    if (volume > 100 || boost > MAX_SPEAKER_BOOST_DB || boost % 3 != 0) return false;
    nvs_handle_t nvs;
    if (nvs_open("speaker", NVS_READWRITE, &nvs) != ESP_OK) return false;
    esp_err_t result = nvs_set_u8(nvs, "volume", (uint8_t)volume);
    if (result == ESP_OK) result = nvs_set_u8(nvs, "boost", (uint8_t)boost);
    if (result == ESP_OK) result = nvs_commit(nvs);
    nvs_close(nvs);
    if (result != ESP_OK) return false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    speaker_volume = (uint8_t)volume;
    speaker_boost_db = (uint8_t)boost;
    state.speaker_volume = (uint8_t)volume;
    state.speaker_boost_db = (uint8_t)boost;
    xSemaphoreGive(state_mutex);
    return true;
}

static bool save_assignment(int button, const char *value)
{
    if (button < 1 || button > 2 || !valid_assignment(value)) return false;
    if (valid_filename(value)) {
        char path[sizeof(SOUND_DIR) + 70];
        struct stat file_stat;
        file_path(path, sizeof(path), value);
        if (stat(path, &file_stat) != 0) return false;
    }
    nvs_handle_t nvs;
    if (nvs_open("speaker", NVS_READWRITE, &nvs) != ESP_OK) return false;
    char key[8];
    snprintf(key, sizeof(key), "button%d", button);
    esp_err_t result = nvs_set_str(nvs, key, value);
    if (result == ESP_OK) result = nvs_commit(nvs);
    nvs_close(nvs);
    if (result != ESP_OK) return false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.assignment[button - 1], sizeof(state.assignment[button - 1]), "%s", value);
    xSemaphoreGive(state_mutex);
    return true;
}

static void json_escape(char *output, size_t output_size, const char *input)
{
    size_t used = 0;
    for (size_t i = 0; input && input[i] && used + 2 < output_size; ++i) {
        char c = input[i];
        if (c == '"' || c == '\\') output[used++] = '\\';
        output[used++] = (c == '\r' || c == '\n') ? ' ' : c;
    }
    output[used] = 0;
}

static void url_decode(char *value)
{
    char *read = value;
    char *write = value;
    while (*read) {
        if (*read == '%' && read[1] && read[2]) {
            char hex[3] = {read[1], read[2], 0};
            char *end = NULL;
            long decoded = strtol(hex, &end, 16);
            if (end && *end == 0) {
                *write++ = (char)decoded;
                read += 3;
                continue;
            }
        }
        *write++ = *read == '+' ? ' ' : *read;
        ++read;
    }
    *write = 0;
}

static bool query_value(httpd_req_t *request, const char *key,
                        char *output, size_t output_size)
{
    char query[192];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, key, output, output_size) != ESP_OK) {
        return false;
    }
    url_decode(output);
    return true;
}

static int append_file_list(char *body, size_t capacity, int used, const char *directory_path)
{
    DIR *directory = opendir(directory_path);
    struct dirent *entry;
    bool first = true;
    while (directory && (entry = readdir(directory)) != NULL && used < (int)capacity - 320) {
        if (!valid_filename(entry->d_name)) continue;
        char path[320];
        snprintf(path, sizeof(path), "%s/%s", directory_path, entry->d_name);
        struct stat info;
        wav_info_t wav = {0};
        FILE *file = fopen(path, "rb");
        bool valid = file && parse_wav(file, &wav);
        if (file) fclose(file);
        if (!valid || stat(path, &info) != 0) continue;
        char escaped[130];
        json_escape(escaped, sizeof(escaped), entry->d_name);
        uint64_t duration_ms = (uint64_t)wav.data_size * 1000 /
            ((uint64_t)wav.rate * wav.channels * 2);
        used += snprintf(body + used, capacity - used,
            "%s{\"name\":\"%s\",\"bytes\":%llu,\"rate\":%u,"
            "\"channels\":%u,\"duration_ms\":%llu}",
            first ? "" : ",", escaped, (unsigned long long)info.st_size,
            (unsigned)wav.rate, (unsigned)wav.channels,
            (unsigned long long)duration_ms);
        first = false;
    }
    if (directory) closedir(directory);
    return used;
}

static esp_err_t send_status(httpd_req_t *request)
{
    speaker_status_t snapshot;
    speaker_get_status(&snapshot);
    const size_t capacity = 32768;
    char *body = calloc(1, capacity);
    if (!body) return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "memory");
    char current[130], assignment1[130], assignment2[130], error[260];
    json_escape(current, sizeof(current), snapshot.current);
    json_escape(assignment1, sizeof(assignment1), snapshot.assignment[0]);
    json_escape(assignment2, sizeof(assignment2), snapshot.assignment[1]);
    json_escape(error, sizeof(error), snapshot.error);
    int used = snprintf(body, capacity,
        "{\"ready\":%s,\"mounted\":%s,\"storage\":\"%s\",\"total\":%llu,\"free\":%llu,"
        "\"playing\":%s,\"paused\":%s,\"button\":%u,\"kind\":%u,\"tone_hz\":%u,\"current\":\"%s\","
        "\"volume\":%u,\"boost_db\":%u,"
        "\"assignments\":[\"%s\",\"%s\"],\"error\":\"%s\",\"files\":[",
        snapshot.ready ? "true" : "false", snapshot.storage_mounted ? "true" : "false",
        snapshot.storage_is_sd ? "microSD" : "internal", (unsigned long long)snapshot.total_bytes,
        (unsigned long long)snapshot.free_bytes, snapshot.playing ? "true" : "false",
        snapshot.paused ? "true" : "false",
        snapshot.source_button, snapshot.kind, snapshot.tone_hz, current,
        snapshot.speaker_volume, snapshot.speaker_boost_db,
        assignment1, assignment2, error);
    if (snapshot.storage_mounted) {
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        used = append_file_list(body, capacity, used, SOUND_DIR);
        xSemaphoreGive(storage_mutex);
    }
    used += snprintf(body + used, capacity - used, "],\"recordings\":[");
    if (snapshot.storage_mounted) {
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        used = append_file_list(body, capacity, used, RECORDING_DIR);
        xSemaphoreGive(storage_mutex);
    }
    snprintf(body + used, capacity - used, "]}");
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_sendstr(request, body);
    free(body);
    return result;
}

static const char speaker_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Rover Audio Library</title><style>*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#17202a;font-family:Arial,sans-serif}"
".panel{background:white;border:1px solid #cbd3da;border-radius:8px;padding:12px;margin-bottom:12px}h2{font-size:17px;margin:0 0 10px}.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.stat{border-left:4px solid #16816a;background:#f3f5f7;padding:9px;min-width:0}.stat small,.meta{display:block;color:#596673;font-size:12px}.stat strong{display:block;font-size:17px;overflow-wrap:anywhere}"
"button,input,select{min-height:44px;border:1px solid #aeb8c1;border-radius:6px;font-size:15px;padding:8px}button{font-weight:700;background:#e7ecf0}.primary{background:#1769aa;color:white;border-color:#1769aa}.danger{background:#b42318;color:white;border-color:#b42318}.control{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.row{display:grid;grid-template-columns:1fr auto;gap:8px;margin-top:8px}"
"label{display:grid;gap:5px;font-size:13px;font-weight:700}input,select{width:100%;background:white}.level{display:grid;grid-template-columns:minmax(0,1fr) 130px auto;gap:8px;align-items:end}.file{display:grid;grid-template-columns:minmax(0,1fr) auto auto;gap:8px;align-items:center;border-top:1px solid #dde2e7;padding:9px 0}.name{font-weight:700;overflow-wrap:anywhere}.log{font:12px monospace;white-space:pre-wrap;word-break:break-word;color:#4c5965}progress{width:100%;height:16px}.empty{color:#596673}"
"@media(max-width:540px){.stats,.grid,.level{grid-template-columns:1fr}.file{grid-template-columns:1fr 72px 72px}.file .info{grid-column:1/-1}.control{grid-template-columns:1fr 1fr}.control .danger{grid-column:1/-1}}"
"</style></head><body><header><div class=top><h1>Rover Audio Library</h1><div class=nav><a href='/'>Motors</a><a href='/mobile'>Drive</a><a href='/steppers'>Steppers</a><a href='/speaker'>Speaker</a><a href='/mic'>Mic</a><a href='/wifi'>Wi-Fi</a></div></div></header><main>"
"<section class=panel><div class=stats><div class=stat><small>Now playing</small><strong id=audio>Loading</strong></div><div class=stat><small>Storage</small><strong id=store>-</strong></div><div class=stat><small>Free</small><strong id=free>-</strong></div></div><div class=control><button id=pause onclick=pauseAudio()>Pause</button><button onclick=resumeAudio()>Resume</button><button class=danger onclick=stopAudio()>Stop</button></div></section>"
"<section class=panel><h2>Output level</h2><div class=level><label>Codec volume <span id=volumeOut>100%</span><input id=volume type=range min=0 max=100 value=100 oninput='levelDirty=true;showVolume()'></label><label>Digital boost<select id=boost onchange='levelDirty=true'><option value=0>Off</option><option value=3>+3 dB</option><option value=6>+6 dB</option></select></label><button onclick=saveLevel()>Save level</button></div><p class=meta>Boost can make quiet files louder but clips peaks. The codec and amplifier are already at maximum hardware output at 100%.</p></section>"
"<section class=panel><h2>Controller button assignments</h2><div class=grid><label>GP10 sound<select id=b1 onchange='assignmentDirty[0]=true'></select></label><label>GP11 sound<select id=b2 onchange='assignmentDirty[1]=true'></select></label><button onclick=assign(1)>Save GP10</button><button onclick=assign(2)>Save GP11</button><button class=primary onclick=play('tone:150')>Test 150 Hz</button><button class=primary onclick=play('tone:600')>Test 600 Hz</button></div></section>"
"<section class=panel><h2>Sound library</h2><input id=upload type=file accept='.wav,audio/wav'><div class=row><button class=primary onclick=upload()>Upload WAV</button><button onclick=refresh()>Refresh</button></div><progress id=progress value=0 max=100></progress><div id=files></div></section>"
"<section class=panel><h2>Microphone recordings</h2><div id=recordings></div></section>"
"<section class='panel log' id=log>Loading audio status...</section></main><script>let state=null;const $=x=>document.getElementById(x);const esc=x=>String(x).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]));function size(n){if(n<1024)return n+' B';if(n<1048576)return(n/1024).toFixed(1)+' KiB';if(n<1073741824)return(n/1048576).toFixed(1)+' MiB';return(n/1073741824).toFixed(2)+' GiB'}function duration(ms){let s=Math.round(ms/1000),m=Math.floor(s/60);return m+':'+String(s%60).padStart(2,'0')}"
"function fileRow(x,recording){return `<div class=file><div class=info><div class=name>${esc(x.name)}</div><span class=meta>${duration(x.duration_ms)} | ${x.rate/1000} kHz | ${x.channels===1?'mono':'stereo'} | ${size(x.bytes)}</span></div><button onclick='${recording?'playRecording':'play'}(${JSON.stringify(x.name)})'>Play</button><button class=danger onclick='${recording?'deleteRecording':'del'}(${JSON.stringify(x.name)})'>Delete</button></div>`}function options(selected){let a=['tone:150','tone:600'].concat(state.files.map(x=>x.name));return a.map(x=>`<option ${x===selected?'selected':''}>${esc(x)}</option>`).join('')}function render(s){state=s;let label=s.ready?'Ready':'Unavailable';if(s.playing)label=(s.kind===1?s.tone_hz+' Hz':s.current)+(s.paused?' paused':' playing');$('audio').textContent=label;$('store').textContent=s.mounted?s.storage:'not mounted';$('free').textContent=size(s.free)+' / '+size(s.total);if(!levelDirty){$('volume').value=s.volume;$('boost').value=s.boost_db;showVolume()}if(!assignmentDirty[0])$('b1').innerHTML=options(s.assignments[0]);if(!assignmentDirty[1])$('b2').innerHTML=options(s.assignments[1]);$('files').innerHTML=s.files.length?s.files.map(x=>fileRow(x,false)).join(''):'<p class=empty>No uploaded WAV files stored.</p>';$('recordings').innerHTML=s.recordings.length?s.recordings.map(x=>fileRow(x,true)).join(''):'<p class=empty>No microphone recordings stored.</p>';$('log').textContent='Volume: '+s.volume+'% | boost: +'+s.boost_db+' dB\\nGP10: '+s.assignments[0]+'\\nGP11: '+s.assignments[1]+'\\n'+(s.error?'error: '+s.error:'Accepted: 16-bit PCM WAV, mono or stereo, 8-48 kHz')}"
"let levelDirty=false,assignmentDirty=[false,false];function request(path){return fetch(path,{cache:'no-store'}).then(r=>{if(!r.ok)return r.text().then(t=>Promise.reject(t));return r.json()})}function get(path){return request(path).then(render).catch(e=>$('log').textContent=e)}function refresh(){get('/api/speaker/status')}function showVolume(){$('volumeOut').textContent=$('volume').value+'%'}function saveLevel(){let path='/api/speaker/level?volume='+$('volume').value+'&boost='+$('boost').value;request(path).then(s=>{levelDirty=false;render(s)}).catch(e=>$('log').textContent=e)}function play(v){get('/api/speaker/play?value='+encodeURIComponent(v))}function playRecording(v){get('/api/speaker/play?source=recording&value='+encodeURIComponent(v))}function pauseAudio(){get('/api/speaker/pause')}function resumeAudio(){get('/api/speaker/resume')}function stopAudio(){get('/api/speaker/stop')}function assign(n){let value=$('b'+n).value;request('/api/speaker/assign?button='+n+'&value='+encodeURIComponent(value)).then(s=>{assignmentDirty[n-1]=false;render(s)}).catch(e=>$('log').textContent=e)}function del(v){if(confirm('Delete '+v+'?'))get('/api/speaker/delete?name='+encodeURIComponent(v))}function deleteRecording(v){if(confirm('Delete recording '+v+'?'))get('/api/mic/delete?name='+encodeURIComponent(v))}function upload(){let f=$('upload').files[0];if(!f)return;let x=new XMLHttpRequest();x.open('POST','/api/speaker/upload?name='+encodeURIComponent(f.name));x.upload.onprogress=e=>{if(e.lengthComputable)$('progress').value=e.loaded*100/e.total};x.onload=()=>{try{render(JSON.parse(x.responseText));$('progress').value=0}catch(e){$('log').textContent=x.responseText}};x.send(f)}setInterval(refresh,1000);refresh()</script></body></html>";

static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, speaker_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    return send_status(request);
}

static esp_err_t play_handler(httpd_req_t *request)
{
    char value[64], source[16] = {0};
    if (!query_value(request, "value", value, sizeof(value)) || !valid_assignment(value)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid sound");
    }
    (void)query_value(request, "source", source, sizeof(source));
    bool queued = false;
    if (strcmp(source, "recording") == 0 && valid_filename(value)) {
        char path[sizeof(RECORDING_DIR) + 70];
        struct stat info;
        recording_path(path, sizeof(path), value);
        queued = stat(path, &info) == 0 && queue_recording_playback(value);
    } else if (!source[0]) {
        queued = queue_value(0, value);
    }
    if (!queued) {
        set_error("Speaker playback queue is full");
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "sound not found or playback queue full");
    }
    return send_status(request);
}

static esp_err_t level_handler(httpd_req_t *request)
{
    char volume[8], boost[8];
    if (!query_value(request, "volume", volume, sizeof(volume)) ||
        !query_value(request, "boost", boost, sizeof(boost)) ||
        !save_speaker_level((unsigned)atoi(volume), (unsigned)atoi(boost))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "volume must be 0-100; boost must be 0, 3, or 6 dB");
    }
    return send_status(request);
}

static esp_err_t pause_handler(httpd_req_t *request)
{
    set_pause(true);
    return send_status(request);
}

static esp_err_t resume_handler(httpd_req_t *request)
{
    set_pause(false);
    return send_status(request);
}

static esp_err_t stop_handler(httpd_req_t *request)
{
    stop_playback();
    return send_status(request);
}

static esp_err_t assign_handler(httpd_req_t *request)
{
    char button_text[8], value[64];
    if (!query_value(request, "button", button_text, sizeof(button_text)) ||
        !query_value(request, "value", value, sizeof(value)) ||
        !save_assignment(atoi(button_text), value)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid assignment");
    }
    return send_status(request);
}

static esp_err_t delete_handler(httpd_req_t *request)
{
    char name[64], path[sizeof(SOUND_DIR) + 70];
    if (!query_value(request, "name", name, sizeof(name)) || !valid_filename(name)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid filename");
    }
    speaker_status_t snapshot;
    speaker_get_status(&snapshot);
    if (strcmp(snapshot.assignment[0], name) == 0 || strcmp(snapshot.assignment[1], name) == 0) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "file is assigned to a button");
    }
    if (snapshot.playing && current_source == PLAY_SOURCE_LIBRARY &&
        strcmp(snapshot.current, name) == 0) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "stop file before deleting");
    }
    file_path(path, sizeof(path), name);
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    int result = unlink(path);
    xSemaphoreGive(storage_mutex);
    if (result != 0) return httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "file not found");
    refresh_storage_space();
    return send_status(request);
}

static esp_err_t upload_handler(httpd_req_t *request)
{
    char name[64], path[sizeof(SOUND_DIR) + 70], temporary[sizeof(SOUND_DIR) + 76];
    if (!state.storage_mounted || request->content_len <= 0 ||
        request->content_len > MAX_UPLOAD_BYTES ||
        !query_value(request, "name", name, sizeof(name)) || !valid_filename(name)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid upload");
    }
    file_path(path, sizeof(path), name);
    snprintf(temporary, sizeof(temporary), "%s.part", path);
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    FILE *file = fopen(temporary, "wb");
    if (!file) {
        xSemaphoreGive(storage_mutex);
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed");
    }
    xSemaphoreGive(storage_mutex);
    char buffer[2048];
    int remaining = request->content_len;
    bool okay = true;
    while (remaining > 0) {
        int received = httpd_req_recv(request, buffer,
            remaining > (int)sizeof(buffer) ? sizeof(buffer) : remaining);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (received <= 0) {
            okay = false;
            break;
        }
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        size_t written = fwrite(buffer, 1, received, file);
        xSemaphoreGive(storage_mutex);
        if (written != (size_t)received) {
            okay = false;
            break;
        }
        remaining -= received;
    }
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    fclose(file);
    wav_info_t wav;
    file = okay ? fopen(temporary, "rb") : NULL;
    okay = file && parse_wav(file, &wav);
    if (file) fclose(file);
    if (okay) {
        unlink(path);
        okay = rename(temporary, path) == 0;
    }
    if (!okay) unlink(temporary);
    xSemaphoreGive(storage_mutex);
    if (!okay) return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
        "only 16-bit PCM WAV mono/stereo 8-48 kHz is supported");
    refresh_storage_space();
    return send_status(request);
}

static esp_err_t send_mic_status(httpd_req_t *request)
{
    speaker_status_t snapshot;
    speaker_get_status(&snapshot);
    char *body = calloc(1, 16384);
    if (!body) return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "memory");
    char current[130], last[130], error[260];
    json_escape(current, sizeof(current), snapshot.recording_name);
    json_escape(last, sizeof(last), snapshot.last_recording);
    json_escape(error, sizeof(error), snapshot.error);
    int used = snprintf(body, 16384,
        "{\"ready\":%s,\"mounted\":%s,\"storage\":\"%s\",\"total\":%llu,\"free\":%llu,"
        "\"recording\":%s,\"pending\":%s,\"name\":\"%s\",\"last\":\"%s\","
        "\"bytes\":%llu,\"elapsed_ms\":%u,\"rate\":%u,\"channels\":%u,\"gain_db\":%u,"
        "\"playing\":%s,\"paused\":%s,\"current\":\"%s\",\"error\":\"%s\",\"files\":[",
        snapshot.ready ? "true" : "false", snapshot.storage_mounted ? "true" : "false",
        snapshot.storage_is_sd ? "microSD" : "internal",
        (unsigned long long)snapshot.total_bytes, (unsigned long long)snapshot.free_bytes,
        snapshot.recording ? "true" : "false", snapshot.recording_pending ? "true" : "false",
        current, last, (unsigned long long)snapshot.recording_bytes,
        (unsigned)snapshot.recording_ms,
        RECORD_SAMPLE_RATE, RECORD_CHANNELS, snapshot.mic_gain_db,
        snapshot.playing ? "true" : "false",
        snapshot.paused ? "true" : "false", snapshot.current, error);
    if (snapshot.storage_mounted) {
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        used = append_file_list(body, 16384, used, RECORDING_DIR);
        xSemaphoreGive(storage_mutex);
    }
    snprintf(body + used, 16384 - used, "]}");
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_sendstr(request, body);
    free(body);
    return result;
}

static const char mic_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Rover Microphone</title><style>*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#17202a;font-family:Arial,sans-serif}header{background:#17212b;color:white;border-bottom:3px solid #e2a400;padding:11px 14px}.top,main{max-width:820px;margin:auto}h1{font-size:19px;margin:0 0 5px}.nav a{color:white;margin-right:13px;font-size:13px}main{padding:12px}.panel{background:white;border:1px solid #cbd3da;border-radius:8px;padding:12px;margin-bottom:12px}h2{font-size:17px;margin:0 0 10px}.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.stat{border-left:4px solid #16816a;background:#f3f5f7;padding:9px;min-width:0}.stat small,.meta{display:block;color:#596673;font-size:12px}.stat strong{display:block;font-size:17px;overflow-wrap:anywhere}button,input,select{min-height:44px;border:1px solid #aeb8c1;border-radius:6px;font-size:15px;padding:8px}button{font-weight:700;background:#e7ecf0}.primary{background:#1769aa;color:white;border-color:#1769aa}.danger{background:#b42318;color:white;border-color:#b42318}.record{display:grid;grid-template-columns:minmax(0,1fr) auto auto;gap:8px}.gain{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:8px}.file{display:grid;grid-template-columns:minmax(0,1fr) auto auto auto;gap:8px;align-items:center;border-top:1px solid #dde2e7;padding:9px 0}.name{font-weight:700;overflow-wrap:anywhere}.log{font:12px monospace;white-space:pre-wrap;word-break:break-word;color:#4c5965}.empty{color:#596673}a.download{display:grid;place-items:center;min-height:44px;border:1px solid #aeb8c1;border-radius:6px;padding:8px;color:#17202a;text-decoration:none;font-weight:700;background:#e7ecf0}@media(max-width:560px){.stats{grid-template-columns:1fr}.record{grid-template-columns:1fr 1fr}.record input{grid-column:1/-1}.file{grid-template-columns:1fr 1fr 1fr}.file .info{grid-column:1/-1}}</style></head>"
"<body><header><div class=top><h1>Rover Microphone</h1><div class=nav><a href='/'>Motors</a><a href='/mobile'>Drive</a><a href='/steppers'>Steppers</a><a href='/speaker'>Speaker</a><a href='/mic'>Mic</a><a href='/wifi'>Wi-Fi</a></div></div></header><main>"
"<section class=panel><div class=stats><div class=stat><small>Microphone</small><strong id=status>Loading</strong></div><div class=stat><small>Recorded</small><strong id=elapsed>0:00</strong></div><div class=stat><small>Storage free</small><strong id=free>-</strong></div></div></section>"
"<section class=panel><h2>New recording</h2><div class=record><input id=name value='recording_01.wav' maxlength=63><button class=primary onclick=start()>Record</button><button class=danger onclick=stop()>Stop</button></div><p class=meta>16 kHz, 16-bit stereo WAV. Maximum 10 minutes per recording.</p></section>"
"<section class=panel><h2>Microphone sensitivity</h2><div class=gain><select id=gain onchange=setGain(this.value)><option value=0>0 dB</option><option value=6>6 dB</option><option value=12>12 dB</option><option value=18>18 dB</option><option value=24>24 dB</option><option value=30>30 dB</option><option value=36>36 dB</option><option value=42>42 dB</option></select><span id=gainSaved class=meta>Saved</span></div><p class=meta>Higher gain hears quieter sounds but also increases case noise and clipping. Applied to the next recording.</p></section>"
"<section class=panel><h2>Recordings</h2><div id=files></div></section><section class='panel log' id=log>Loading microphone status...</section></main>"
"<script>let state=null,lastSeen='',gainSaving=false;const $=x=>document.getElementById(x);const esc=x=>String(x).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]));function size(n){if(n<1024)return n+' B';if(n<1048576)return(n/1024).toFixed(1)+' KiB';if(n<1073741824)return(n/1048576).toFixed(1)+' MiB';return(n/1073741824).toFixed(2)+' GiB'}function duration(ms){let s=Math.round(ms/1000),m=Math.floor(s/60);return m+':'+String(s%60).padStart(2,'0')}function nextName(n){let m=n.match(/^(.*?)(\\d+)(\\.wav)$/i);if(m)return m[1]+String(Number(m[2])+1).padStart(m[2].length,'0')+m[3];return n.replace(/\\.wav$/i,'_02.wav')}function render(s){state=s;if(s.last&&s.last!==lastSeen){lastSeen=s.last;$('name').value=nextName(s.last)}if(!gainSaving&&document.activeElement!==$('gain'))$('gain').value=String(s.gain_db);$('gainSaved').textContent=gainSaving?'Saving...':'Saved at '+s.gain_db+' dB';$('status').textContent=s.recording?'Recording '+s.name:(s.pending?'Starting '+s.name:(s.playing?'Playing '+s.current:'Ready'));$('elapsed').textContent=duration(s.elapsed_ms)+' | '+size(s.bytes);$('free').textContent=size(s.free)+' / '+size(s.total);$('files').innerHTML=s.files.length?s.files.map(x=>`<div class=file><div class=info><div class=name>${esc(x.name)}</div><span class=meta>${duration(x.duration_ms)} | ${x.rate/1000} kHz | ${x.channels===1?'mono':'stereo'} | ${size(x.bytes)}</span></div><button onclick='play(${JSON.stringify(x.name)})'>Play</button><a class=download href='/api/mic/download?name=${encodeURIComponent(x.name)}'>Save</a><button class=danger onclick='del(${JSON.stringify(x.name)})'>Delete</button></div>`).join(''):'<p class=empty>No microphone recordings saved.</p>';$('log').textContent=(s.last?'Last saved: '+s.last+'\\n':'')+'Gain: '+s.gain_db+' dB\\n'+(s.error?'error: '+s.error:'Onboard ES8311 microphone ready')}function get(path){return fetch(path,{cache:'no-store'}).then(r=>{if(!r.ok)return r.text().then(t=>Promise.reject(t));return r.json()}).then(render).catch(e=>$('log').textContent=e)}function start(){get('/api/mic/start?name='+encodeURIComponent($('name').value))}function stop(){get('/api/mic/stop')}function setGain(v){gainSaving=true;$('gainSaved').textContent='Saving...';fetch('/api/mic/gain?db='+encodeURIComponent(v),{cache:'no-store'}).then(r=>{if(!r.ok)return r.text().then(t=>Promise.reject(t));return r.json()}).then(s=>{gainSaving=false;render(s)}).catch(e=>{gainSaving=false;$('gainSaved').textContent='Save failed';$('log').textContent=e})}function play(v){get('/api/mic/play?name='+encodeURIComponent(v))}function del(v){if(confirm('Delete '+v+'?'))get('/api/mic/delete?name='+encodeURIComponent(v))}function refresh(){get('/api/mic/status')}setInterval(refresh,500);refresh()</script></body></html>";

static esp_err_t mic_page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, mic_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t mic_status_handler(httpd_req_t *request)
{
    return send_mic_status(request);
}

static esp_err_t mic_start_handler(httpd_req_t *request)
{
    char name[64];
    if (!query_value(request, "name", name, sizeof(name)) || !valid_filename(name) ||
        !start_recording(name)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "invalid name or microphone busy");
    }
    return send_mic_status(request);
}

static esp_err_t mic_stop_handler(httpd_req_t *request)
{
    stop_recording();
    return send_mic_status(request);
}

static esp_err_t mic_gain_handler(httpd_req_t *request)
{
    char value[8];
    if (!query_value(request, "db", value, sizeof(value)) ||
        !save_mic_gain((unsigned)atoi(value))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "gain must be 0, 6, 12, 18, 24, 30, 36, or 42 dB");
    }
    return send_mic_status(request);
}

static esp_err_t mic_play_handler(httpd_req_t *request)
{
    char name[64], path[sizeof(RECORDING_DIR) + 70];
    struct stat info;
    if (!query_value(request, "name", name, sizeof(name)) || !valid_filename(name)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid filename");
    }
    recording_path(path, sizeof(path), name);
    if (stat(path, &info) != 0 || !queue_recording_playback(name)) {
        return httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "recording not found");
    }
    return send_mic_status(request);
}

static esp_err_t mic_delete_handler(httpd_req_t *request)
{
    char name[64], path[sizeof(RECORDING_DIR) + 70];
    if (!query_value(request, "name", name, sizeof(name)) || !valid_filename(name)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid filename");
    }
    speaker_status_t snapshot;
    speaker_get_status(&snapshot);
    if ((snapshot.recording && strcmp(snapshot.recording_name, name) == 0) ||
        (snapshot.playing && current_source == PLAY_SOURCE_RECORDING &&
         strcmp(snapshot.current, name) == 0)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "stop recording or playback before deleting");
    }
    recording_path(path, sizeof(path), name);
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    int result = unlink(path);
    xSemaphoreGive(storage_mutex);
    if (result != 0) return httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "not found");
    refresh_storage_space();
    return send_mic_status(request);
}

static esp_err_t mic_download_handler(httpd_req_t *request)
{
    char name[64], path[sizeof(RECORDING_DIR) + 70], disposition[100];
    if (!query_value(request, "name", name, sizeof(name)) || !valid_filename(name)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid filename");
    }
    recording_path(path, sizeof(path), name);
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    FILE *file = fopen(path, "rb");
    xSemaphoreGive(storage_mutex);
    if (!file) return httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "not found");
    snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", name);
    httpd_resp_set_type(request, "audio/wav");
    httpd_resp_set_hdr(request, "Content-Disposition", disposition);
    uint8_t buffer[2048];
    esp_err_t result = ESP_OK;
    while (true) {
        xSemaphoreTake(storage_mutex, portMAX_DELAY);
        size_t count = fread(buffer, 1, sizeof(buffer), file);
        xSemaphoreGive(storage_mutex);
        if (!count) break;
        result = httpd_resp_send_chunk(request, (const char *)buffer, count);
        if (result != ESP_OK) break;
    }
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    fclose(file);
    xSemaphoreGive(storage_mutex);
    if (result == ESP_OK) result = httpd_resp_send_chunk(request, NULL, 0);
    return result;
}

esp_err_t speaker_start(void)
{
    state_mutex = xSemaphoreCreateMutex();
    storage_mutex = xSemaphoreCreateMutex();
    audio_mutex = xSemaphoreCreateMutex();
    play_queue = xQueueCreate(4, sizeof(play_request_t));
    record_queue = xQueueCreate(1, sizeof(record_request_t));
    if (!state_mutex || !storage_mutex || !audio_mutex || !play_queue || !record_queue) {
        return ESP_ERR_NO_MEM;
    }
    load_assignments();

    esp_err_t storage_result = mount_storage();
    if (storage_result == ESP_OK) {
        if (mkdir(SOUND_DIR, 0775) != 0 && errno != EEXIST) {
            set_error("Could not create sounds directory");
        }
        if (mkdir(RECORDING_DIR, 0775) != 0 && errno != EEXIST) {
            set_error("Could not create recordings directory");
        }
        refresh_storage_space();
    } else {
        set_error("No microSD or internal sound storage available");
    }
    esp_err_t codec_result = initialize_codec();
    if (codec_result != ESP_OK) {
        char message[128];
        snprintf(message, sizeof(message), "ES8311 initialization failed: %s",
                 esp_err_to_name(codec_result));
        set_error(message);
        return codec_result;
    }
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.ready = true;
    xSemaphoreGive(state_mutex);
    if (xTaskCreate(playback_task, "speaker", 6144, NULL, 3, NULL) != pdPASS ||
        xTaskCreate(recording_task, "microphone", 6144, NULL, 3, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void speaker_controller_buttons(uint16_t button_mask)
{
    uint16_t pressed = button_mask & ~previous_buttons;
    previous_buttons = button_mask;
    if (pressed & (1U << 10)) {
        speaker_status_t snapshot;
        speaker_get_status(&snapshot);
        queue_value(1, snapshot.assignment[0]);
    }
    if (pressed & (1U << 11)) {
        speaker_status_t snapshot;
        speaker_get_status(&snapshot);
        queue_value(2, snapshot.assignment[1]);
    }
}

void speaker_get_status(speaker_status_t *status)
{
    if (!status || !state_mutex) return;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    *status = state;
    xSemaphoreGive(state_mutex);
}

esp_err_t speaker_register_routes(httpd_handle_t server)
{
    const httpd_uri_t routes[] = {
        {.uri = "/speaker", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/speaker/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/speaker/play", .method = HTTP_GET, .handler = play_handler},
        {.uri = "/api/speaker/level", .method = HTTP_GET, .handler = level_handler},
        {.uri = "/api/speaker/pause", .method = HTTP_GET, .handler = pause_handler},
        {.uri = "/api/speaker/resume", .method = HTTP_GET, .handler = resume_handler},
        {.uri = "/api/speaker/stop", .method = HTTP_GET, .handler = stop_handler},
        {.uri = "/api/speaker/assign", .method = HTTP_GET, .handler = assign_handler},
        {.uri = "/api/speaker/delete", .method = HTTP_GET, .handler = delete_handler},
        {.uri = "/api/speaker/upload", .method = HTTP_POST, .handler = upload_handler},
        {.uri = "/mic", .method = HTTP_GET, .handler = mic_page_handler},
        {.uri = "/api/mic/status", .method = HTTP_GET, .handler = mic_status_handler},
        {.uri = "/api/mic/start", .method = HTTP_GET, .handler = mic_start_handler},
        {.uri = "/api/mic/stop", .method = HTTP_GET, .handler = mic_stop_handler},
        {.uri = "/api/mic/gain", .method = HTTP_GET, .handler = mic_gain_handler},
        {.uri = "/api/mic/play", .method = HTTP_GET, .handler = mic_play_handler},
        {.uri = "/api/mic/delete", .method = HTTP_GET, .handler = mic_delete_handler},
        {.uri = "/api/mic/download", .method = HTTP_GET, .handler = mic_download_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &routes[i]), TAG,
                            "register speaker route");
    }
    return ESP_OK;
}
