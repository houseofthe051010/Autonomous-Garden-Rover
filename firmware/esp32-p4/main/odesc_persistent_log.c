#include "odesc_persistent_log.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "esp_crc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "speaker.h"

#define LOG_DIRECTORY "/storage/diagnostics"
#define LOG_PATH LOG_DIRECTORY "/odesc-p4-blackbox.bin"
#define LOG_BYTES (256U * 1024U)
#define LOG_RECORD_BYTES 128U
#define LOG_CAPACITY (LOG_BYTES / LOG_RECORD_BYTES)
#define LOG_QUEUE_LENGTH 32U
#define LOG_MAGIC 0x3142444fU
#define VALID_EPOCH_SECONDS 1704067200LL
#define REPETITIVE_EVENT_INTERVAL_MS 60000U
#define RETAINED_BOOT_SESSIONS 5U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t sequence;
    uint64_t uptime_ms;
    int64_t epoch_ms;
    uint32_t reset_reason;
    char message[96];
    uint32_t crc32;
} persistent_record_t;

_Static_assert(sizeof(persistent_record_t) == LOG_RECORD_BYTES,
               "ODESC persistent record must fill one slot");

static const char *TAG = "odesc_sd_log";
static QueueHandle_t event_queue;

static uint32_t record_crc(persistent_record_t *record)
{
    uint32_t saved = record->crc32;
    record->crc32 = 0;
    uint32_t crc = esp_crc32_le(UINT32_MAX, (const uint8_t *)record,
                                sizeof(*record));
    record->crc32 = saved;
    return crc;
}

static bool valid_record(persistent_record_t *record)
{
    return record->magic == LOG_MAGIC &&
           record->message[sizeof(record->message) - 1] == '\0' &&
           record->crc32 == record_crc(record);
}

static bool persistent_event(const char *message)
{
    static const char *const prefixes[] = {
        "BOOT", "LINK", "FAULT", "DEADMAN", "HANDHELD", "START",
        "STOP", "CLEAR", "LIMITS", "UART open failed",
        "RX timeout attempt 2", "VELOCITY write failed", "ODESCLOG",
    };
    if (!message) return false;
    for (size_t i = 0; i < sizeof(prefixes) / sizeof(prefixes[0]); ++i) {
        if (strncmp(message, prefixes[i], strlen(prefixes[i])) == 0) return true;
    }
    return false;
}

static int repetitive_event_kind(const char *message)
{
    if (strncmp(message, "RX timeout attempt 2", 20) == 0) return 0;
    if (strncmp(message, "LINK probe failed", 17) == 0) return 1;
    return -1;
}

static int64_t epoch_ms(void)
{
    struct timeval now = {0};
    gettimeofday(&now, NULL);
    if (now.tv_sec < VALID_EPOCH_SECONDS) return 0;
    return (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000;
}

static bool prepare_file(FILE **file_out, uint32_t *next_sequence)
{
    mkdir(LOG_DIRECTORY, 0775);
    FILE *file = fopen(LOG_PATH, "rb+");
    if (!file) {
        file = fopen(LOG_PATH, "wb+");
        if (!file || fseek(file, LOG_BYTES - 1, SEEK_SET) != 0 ||
            fputc(0, file) == EOF || fflush(file) != 0 ||
            fsync(fileno(file)) != 0) {
            if (file) fclose(file);
            return false;
        }
    }

    persistent_record_t record;
    bool found = false;
    uint32_t newest = 0;
    rewind(file);
    for (uint32_t slot = 0; slot < LOG_CAPACITY; ++slot) {
        if (fread(&record, 1, sizeof(record), file) != sizeof(record)) break;
        if (valid_record(&record) && (!found || (int32_t)(record.sequence - newest) > 0)) {
            newest = record.sequence;
            found = true;
        }
    }
    clearerr(file);
    *next_sequence = found ? newest + 1 : 0;
    *file_out = file;
    return true;
}

static bool write_record(FILE *file, persistent_record_t *record)
{
    long offset = (long)(record->sequence % LOG_CAPACITY) * LOG_RECORD_BYTES;
    record->crc32 = record_crc(record);
    return fseek(file, offset, SEEK_SET) == 0 &&
           fwrite(record, 1, sizeof(*record), file) == sizeof(*record) &&
           fflush(file) == 0 && fsync(fileno(file)) == 0;
}

static void logger_task(void *argument)
{
    (void)argument;
    FILE *file = NULL;
    uint32_t sequence = 0;
    char last_message[96] = {0};
    uint64_t last_write_ms = 0;
    uint64_t repetitive_write_ms[2] = {0};

    while (!file) {
        if (speaker_storage_available(true) && speaker_storage_lock(2000)) {
            bool ready = prepare_file(&file, &sequence);
            speaker_storage_unlock();
            if (!ready) ESP_LOGW(TAG, "Could not open %s", LOG_PATH);
        }
        if (!file) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Persistent ODESC event ring ready at %s (%u records)",
             LOG_PATH, (unsigned)LOG_CAPACITY);

    persistent_record_t queued;
    while (true) {
        if (xQueueReceive(event_queue, &queued, portMAX_DELAY) != pdTRUE) continue;
        if (strcmp(last_message, queued.message) == 0 &&
            queued.uptime_ms - last_write_ms < 10000U) {
            continue;
        }
        int repetitive_kind = repetitive_event_kind(queued.message);
        if (repetitive_kind >= 0 && repetitive_write_ms[repetitive_kind] != 0 &&
            queued.uptime_ms - repetitive_write_ms[repetitive_kind] <
                REPETITIVE_EVENT_INTERVAL_MS) {
            continue;
        }
        queued.sequence = sequence++;
        if (!speaker_storage_lock(2000)) continue;
        bool okay = write_record(file, &queued);
        speaker_storage_unlock();
        if (!okay) {
            ESP_LOGE(TAG, "Persistent ODESC event write failed");
            continue;
        }
        snprintf(last_message, sizeof(last_message), "%s", queued.message);
        last_write_ms = queued.uptime_ms;
        if (repetitive_kind >= 0) {
            repetitive_write_ms[repetitive_kind] = queued.uptime_ms;
        }
    }
}

esp_err_t odesc_persistent_log_start(void)
{
    event_queue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(persistent_record_t));
    if (!event_queue) return ESP_ERR_NO_MEM;
    return xTaskCreate(logger_task, "odesc_sd_log", 4096, NULL, 2, NULL) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

void odesc_persistent_log_event(int64_t uptime_ms, const char *message)
{
    if (!event_queue || !persistent_event(message)) return;
    persistent_record_t record = {
        .magic = LOG_MAGIC,
        .uptime_ms = uptime_ms < 0 ? 0 : (uint64_t)uptime_ms,
        .epoch_ms = epoch_ms(),
        .reset_reason = (uint32_t)esp_reset_reason(),
    };
    snprintf(record.message, sizeof(record.message), "%s", message);
    xQueueSend(event_queue, &record, 0);
}

esp_err_t odesc_persistent_log_handler(httpd_req_t *request)
{
    if (!speaker_storage_available(true) || !speaker_storage_lock(3000)) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_sendstr(request, "microSD log unavailable");
    }
    FILE *file = fopen(LOG_PATH, "rb");
    if (!file) {
        speaker_storage_unlock();
        return httpd_resp_send_err(request, HTTPD_404_NOT_FOUND,
                                   "no persistent ODESC log yet");
    }

    persistent_record_t *records = heap_caps_malloc(
        LOG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!records) {
        fclose(file);
        speaker_storage_unlock();
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "log snapshot allocation failed");
    }
    size_t slots = fread(records, sizeof(*records), LOG_CAPACITY, file);
    fclose(file);
    speaker_storage_unlock();

    bool found = false;
    uint32_t newest = 0;
    for (size_t i = 0; i < slots; ++i) {
        if (valid_record(&records[i]) &&
            (!found || (int32_t)(records[i].sequence - newest) > 0)) {
            newest = records[i].sequence;
            found = true;
        }
    }

    httpd_resp_set_type(request, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    char line[220];
    if (found) {
        uint32_t oldest = newest >= LOG_CAPACITY - 1 ? newest - LOG_CAPACITY + 1 : 0;
        uint32_t first_sequence = oldest;
        unsigned boot_count = 0;
        for (uint32_t sequence = newest;; --sequence) {
            persistent_record_t *record = &records[sequence % LOG_CAPACITY];
            if (valid_record(record) && record->sequence == sequence &&
                strncmp(record->message, "BOOT", 4) == 0 &&
                ++boot_count == RETAINED_BOOT_SESSIONS) {
                first_sequence = sequence;
                break;
            }
            if (sequence == oldest) break;
        }
        for (uint32_t sequence = first_sequence; sequence <= newest; ++sequence) {
            persistent_record_t *record = &records[sequence % LOG_CAPACITY];
            if (!valid_record(record) || record->sequence != sequence) continue;
            int length = snprintf(line, sizeof(line),
                                  "[%10u boot+%8llu.%03llu reset=%u epoch_ms=%lld] %s\n",
                                  (unsigned)record->sequence,
                                  (unsigned long long)(record->uptime_ms / 1000),
                                  (unsigned long long)(record->uptime_ms % 1000),
                                  (unsigned)record->reset_reason,
                                  (long long)record->epoch_ms, record->message);
            if (length > 0 && httpd_resp_send_chunk(request, line, length) != ESP_OK) {
                free(records);
                return ESP_FAIL;
            }
            if (sequence == UINT32_MAX) break;
        }
    }
    free(records);
    return httpd_resp_send_chunk(request, NULL, 0);
}
