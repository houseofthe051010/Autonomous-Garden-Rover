#include "battery_history.h"

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
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
#include "esp_netif_sntp.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "odesc_link.h"
#include "speaker.h"

#define BATTERY_DIR "/storage/battery"
#define JOURNAL_BYTES (512U * 1024U)
#define SECTOR_BYTES 512U
#define HEADER_SECTORS 2U
#define RECORD_BYTES 64U
#define SAMPLE_INTERVAL_MS 5000U
#define SNTP_RETRY_MS 15000U
#define MAX_API_SAMPLES 1600U
#define MAX_API_SESSIONS 64U
#define VALID_EPOCH_SECONDS 1704067200LL
#define BATTERY_NVS_NAMESPACE "battery_log"
#define BATTERY_NVS_ACTIVE_KEY "active"

#define RECORD_FLAG_CURRENT_VALID 0x01U
#define RECORD_FLAG_TIME_VALID 0x02U
#define RECORD_FLAG_SLOPE_VALID 0x04U
#define RECORD_FLAG_TELEMETRY_GAP 0x08U

#define BATTERY_SERIES_CELLS 10
#define BATTERY_PARALLEL_CELLS 4
#define BATTERY_CELL_CAPACITY_AH 2.6f
#define BATTERY_PACK_CAPACITY_AH \
    (BATTERY_PARALLEL_CELLS * BATTERY_CELL_CAPACITY_AH)

#define HEADER_MAGIC 0x31485442U
#define RECORD_MAGIC 0x31525442U
#define JOURNAL_VERSION 3U
#define COMPACT_JOURNAL_VERSION 2U
#define LEGACY_JOURNAL_VERSION 1U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t record_bytes;
    uint32_t capacity;
    uint32_t session_id;
    uint64_t boot_nonce;
    uint32_t crc32;
} journal_header_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t sequence;
    uint32_t flags;
    uint64_t uptime_ms;
    int64_t epoch_ms;
    float voltage_v;
    float current_a;
    float power_w;
    float energy_wh;
    float soc_percent;
    float slope_current_a;
    uint32_t crc32;
} journal_record_t;

_Static_assert(sizeof(journal_record_t) <= RECORD_BYTES,
               "battery journal record exceeds fixed slot");

typedef struct {
    uint64_t uptime_ms;
    float soc;
} slope_point_t;

typedef struct {
    char name[32];
    uint32_t session_id;
    uint32_t samples;
    uint32_t valid_samples;
    uint32_t gap_samples;
    int64_t boot_epoch_ms;
    int64_t last_epoch_ms;
    uint64_t last_uptime_ms;
    float last_voltage_v;
    uint8_t reset_reason;
    bool current;
} session_summary_t;

typedef struct {
    bool running;
    bool storage_ready;
    bool time_synced;
    uint32_t session_id;
    uint32_t samples;
    uint32_t capacity;
    uint64_t boot_nonce;
    uint64_t last_uptime_ms;
    int64_t last_epoch_ms;
    float voltage_v;
    float current_a;
    float power_w;
    float energy_wh;
    float soc_percent;
    float slope_current_a;
    bool slope_valid;
    char filename[32];
    char error[128];
} battery_state_t;

static const char *TAG = "battery_history";
static SemaphoreHandle_t state_mutex;
static battery_state_t state;
static FILE *journal;
static slope_point_t slope_points[80];
static unsigned slope_count;
static unsigned slope_head;
static uint64_t session_uptime_base_ms;

static bool valid_journal_name(const char *name);

static uint64_t uptime_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
}

static uint64_t session_uptime_ms(void)
{
    return session_uptime_base_ms + uptime_ms();
}

static int64_t epoch_ms(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    return now.tv_sec >= VALID_EPOCH_SECONDS ?
           (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000 : 0;
}

static uint32_t object_crc(void *object, size_t size, size_t crc_offset)
{
    uint32_t saved = 0;
    memcpy(&saved, (uint8_t *)object + crc_offset, sizeof(saved));
    memset((uint8_t *)object + crc_offset, 0, sizeof(saved));
    uint32_t result = esp_crc32_le(UINT32_MAX, object, size);
    memcpy((uint8_t *)object + crc_offset, &saved, sizeof(saved));
    return result;
}

static bool valid_header(journal_header_t *header)
{
    bool current = (header->version == JOURNAL_VERSION ||
                    header->version == COMPACT_JOURNAL_VERSION) &&
                   header->record_bytes == RECORD_BYTES;
    bool legacy = header->version == LEGACY_JOURNAL_VERSION &&
                  header->record_bytes == SECTOR_BYTES;
    if (header->magic != HEADER_MAGIC || (!current && !legacy) ||
        !header->capacity) return false;
    return header->crc32 == object_crc(header, sizeof(*header),
                                       offsetof(journal_header_t, crc32));
}

static bool valid_record(journal_record_t *record, uint32_t sequence)
{
    if (record->magic != RECORD_MAGIC || record->sequence != sequence ||
        !isfinite(record->voltage_v) || record->voltage_v < 0.0f ||
        record->voltage_v > 100.0f) return false;
    return record->crc32 == object_crc(record, sizeof(*record),
                                       offsetof(journal_record_t, crc32));
}

static float voltage_soc(float voltage)
{
    static const float curve[][2] = {
        {3.00f, 0.0f}, {3.30f, 3.0f}, {3.50f, 10.0f},
        {3.60f, 20.0f}, {3.70f, 35.0f}, {3.80f, 55.0f},
        {3.90f, 70.0f}, {4.00f, 82.0f}, {4.10f, 92.0f},
        {4.20f, 100.0f},
    };
    float cell = voltage / BATTERY_SERIES_CELLS;
    if (cell <= curve[0][0]) return 0.0f;
    if (cell >= curve[9][0]) return 100.0f;
    for (unsigned i = 1; i < 10; ++i) {
        if (cell <= curve[i][0]) {
            float ratio = (cell - curve[i - 1][0]) /
                          (curve[i][0] - curve[i - 1][0]);
            return curve[i - 1][1] + ratio *
                   (curve[i][1] - curve[i - 1][1]);
        }
    }
    return 0.0f;
}

static bool slope_estimate(uint64_t now, float soc, float *current)
{
    slope_points[slope_head] = (slope_point_t){.uptime_ms = now, .soc = soc};
    slope_head = (slope_head + 1) % (sizeof(slope_points) / sizeof(slope_points[0]));
    if (slope_count < sizeof(slope_points) / sizeof(slope_points[0])) slope_count++;

    for (unsigned age = slope_count; age > 0; --age) {
        unsigned index = (slope_head +
            (sizeof(slope_points) / sizeof(slope_points[0])) - age) %
            (sizeof(slope_points) / sizeof(slope_points[0]));
        slope_point_t previous = slope_points[index];
        uint64_t elapsed = now - previous.uptime_ms;
        if (elapsed < 300000) continue;
        float estimate = (previous.soc - soc) * BATTERY_PACK_CAPACITY_AH /
                         100.0f / (elapsed / 3600000.0f);
        if (estimate < -20.0f || estimate > 100.0f || !isfinite(estimate)) {
            return false;
        }
        *current = estimate;
        return true;
    }
    return false;
}

static long record_offset(const journal_header_t *header, uint32_t sequence)
{
    return (long)(HEADER_SECTORS * SECTOR_BYTES) +
           (long)sequence * header->record_bytes;
}

static bool read_header(FILE *file, journal_header_t *header)
{
    uint8_t sector[SECTOR_BYTES];
    for (unsigned copy = 0; copy < HEADER_SECTORS; ++copy) {
        if (fseek(file, copy * SECTOR_BYTES, SEEK_SET) != 0 ||
            fread(sector, 1, sizeof(sector), file) != sizeof(sector)) continue;
        memcpy(header, sector, sizeof(*header));
        if (valid_header(header)) return true;
    }
    return false;
}

static bool read_record(FILE *file, const journal_header_t *header,
                        uint32_t sequence, journal_record_t *record)
{
    uint8_t sector[SECTOR_BYTES];
    if (header->record_bytes > sizeof(sector) ||
        fseek(file, record_offset(header, sequence), SEEK_SET) != 0 ||
        fread(sector, 1, header->record_bytes, file) != header->record_bytes) {
        return false;
    }
    memcpy(record, sector, sizeof(*record));
    return valid_record(record, sequence);
}

static uint32_t record_count(FILE *file, const journal_header_t *header)
{
    uint32_t low = 0;
    uint32_t high = header->capacity;
    journal_record_t record;
    while (low < high) {
        uint32_t middle = low + (high - low) / 2;
        if (read_record(file, header, middle, &record)) low = middle + 1;
        else high = middle;
    }
    return low;
}

static bool write_sector(long offset, const void *object, size_t object_size)
{
    uint8_t sector[SECTOR_BYTES] = {0};
    memcpy(sector, object, object_size);
    return fseek(journal, offset, SEEK_SET) == 0 &&
           fwrite(sector, 1, sizeof(sector), journal) == sizeof(sector) &&
           fflush(journal) == 0 && fsync(fileno(journal)) == 0;
}

static bool write_record(const journal_header_t *header,
                         const journal_record_t *record)
{
    uint8_t slot[RECORD_BYTES] = {0};
    memcpy(slot, record, sizeof(*record));
    return fseek(journal, record_offset(header, record->sequence), SEEK_SET) == 0 &&
           fwrite(slot, 1, sizeof(slot), journal) == sizeof(slot) &&
           fflush(journal) == 0 && fsync(fileno(journal)) == 0;
}

static void set_error(const char *message)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snprintf(state.error, sizeof(state.error), "%s", message ? message : "");
    xSemaphoreGive(state_mutex);
    if (message && message[0]) ESP_LOGE(TAG, "%s", message);
}

static bool save_active_journal(const char *filename)
{
    nvs_handle_t handle;
    if (nvs_open(BATTERY_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    esp_err_t result = nvs_set_str(handle, BATTERY_NVS_ACTIVE_KEY, filename);
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    return result == ESP_OK;
}

static bool load_active_journal(char *filename, size_t size)
{
    nvs_handle_t handle;
    if (nvs_open(BATTERY_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    esp_err_t result = nvs_get_str(handle, BATTERY_NVS_ACTIVE_KEY,
                                   filename, &size);
    nvs_close(handle);
    return result == ESP_OK && valid_journal_name(filename);
}

static bool reset_keeps_power_session(esp_reset_reason_t reason)
{
    return reason != ESP_RST_POWERON && reason != ESP_RST_BROWNOUT &&
           reason != ESP_RST_DEEPSLEEP;
}

static bool find_latest_journal(char *filename, size_t size)
{
    DIR *directory = opendir(BATTERY_DIR);
    if (!directory) return false;
    int64_t newest_epoch = 0;
    time_t newest_mtime = 0;
    char newest[32] = {0};
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        if (!valid_journal_name(entry->d_name)) continue;
        char path[96];
        snprintf(path, sizeof(path), BATTERY_DIR "/%.16s", entry->d_name);
        FILE *file = fopen(path, "rb");
        journal_header_t header;
        if (!file || !read_header(file, &header)) {
            if (file) fclose(file);
            continue;
        }
        uint32_t samples = record_count(file, &header);
        journal_record_t last = {0};
        bool readable = !samples || read_record(file, &header, samples - 1, &last);
        fclose(file);
        if (!readable || samples >= header.capacity) continue;
        struct stat info = {0};
        (void)stat(path, &info);
        if (!newest[0] || last.epoch_ms > newest_epoch ||
            (last.epoch_ms == newest_epoch && info.st_mtime > newest_mtime)) {
            snprintf(newest, sizeof(newest), "%.16s", entry->d_name);
            newest_epoch = last.epoch_ms;
            newest_mtime = info.st_mtime;
        }
    }
    closedir(directory);
    if (!newest[0]) return false;
    snprintf(filename, size, "%s", newest);
    return true;
}

static bool resume_journal(void)
{
    char filename[32] = {0};
    if (!speaker_storage_available(true) || !speaker_storage_lock(5000)) {
        return false;
    }
    if (!load_active_journal(filename, sizeof(filename)) &&
        !find_latest_journal(filename, sizeof(filename))) {
        speaker_storage_unlock();
        return false;
    }
    bool okay = false;
    do {
        char path[96];
        snprintf(path, sizeof(path), BATTERY_DIR "/%s", filename);
        journal = fopen(path, "rb+");
        journal_header_t header;
        if (!journal || !read_header(journal, &header) ||
            header.record_bytes != RECORD_BYTES) break;
        uint32_t samples = record_count(journal, &header);
        if (samples >= header.capacity) break;

        journal_record_t last = {0};
        if (samples && !read_record(journal, &header, samples - 1, &last)) break;
        uint64_t raw_uptime = uptime_ms();
        uint64_t continuation = samples ? last.uptime_ms + SAMPLE_INTERVAL_MS : 0;
        int64_t wall = epoch_ms();
        if (samples && wall && last.epoch_ms && wall > last.epoch_ms) {
            uint64_t elapsed_wall = (uint64_t)(wall - last.epoch_ms);
            if (elapsed_wall < 7U * 24U * 60U * 60U * 1000U) {
                continuation = last.uptime_ms + elapsed_wall;
            }
        }
        session_uptime_base_ms = continuation > raw_uptime ?
                                 continuation - raw_uptime : continuation;

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.storage_ready = true;
        state.session_id = header.session_id;
        state.boot_nonce = header.boot_nonce;
        state.capacity = header.capacity;
        state.samples = samples;
        state.last_uptime_ms = samples ? last.uptime_ms : 0;
        state.last_epoch_ms = samples ? last.epoch_ms : 0;
        state.time_synced = samples && last.epoch_ms != 0;
        state.voltage_v = samples ? last.voltage_v : 0.0f;
        state.current_a = samples ? last.current_a : 0.0f;
        state.power_w = samples ? last.power_w : 0.0f;
        state.energy_wh = samples ? last.energy_wh : 0.0f;
        state.soc_percent = samples ? last.soc_percent : 0.0f;
        snprintf(state.filename, sizeof(state.filename), "%s", filename);
        state.error[0] = 0;
        xSemaphoreGive(state_mutex);
        if (!save_active_journal(filename)) {
            ESP_LOGW(TAG, "Could not preserve resumed journal name in NVS");
        }
        okay = true;
        ESP_LOGI(TAG, "Resumed physical-power session %s at sample %lu",
                 filename, (unsigned long)samples);
    } while (false);
    if (!okay && journal) {
        fclose(journal);
        journal = NULL;
    }
    speaker_storage_unlock();
    return okay;
}

static bool create_journal(void)
{
    if (!speaker_storage_available(true) || !speaker_storage_lock(5000)) return false;
    bool okay = false;
    do {
        if (mkdir(BATTERY_DIR, 0775) != 0 && errno != EEXIST) break;
        uint32_t session_id = 0;
        uint64_t nonce = ((uint64_t)esp_reset_reason() << 56) |
                         (((uint64_t)esp_random() << 32 | esp_random()) &
                          UINT64_C(0x00FFFFFFFFFFFFFF));
        char filename[32];
        char path[96];
        struct stat existing;
        for (unsigned attempt = 0; attempt < 16; ++attempt) {
            session_id = esp_random();
            snprintf(filename, sizeof(filename), "BAT_%08lX.jrn",
                     (unsigned long)session_id);
            snprintf(path, sizeof(path), BATTERY_DIR "/%s", filename);
            if (stat(path, &existing) != 0) break;
            session_id = 0;
        }
        if (!session_id) break;
        journal = fopen(path, "wb+");
        if (!journal) break;
        if (fseek(journal, JOURNAL_BYTES - 1, SEEK_SET) != 0 || fputc(0, journal) == EOF ||
            fflush(journal) != 0 || fsync(fileno(journal)) != 0) break;

        journal_header_t header = {
            .magic = HEADER_MAGIC,
            .version = JOURNAL_VERSION,
            .record_bytes = RECORD_BYTES,
            .capacity = (JOURNAL_BYTES - HEADER_SECTORS * SECTOR_BYTES) /
                        RECORD_BYTES,
            .session_id = session_id,
            .boot_nonce = nonce,
        };
        header.crc32 = object_crc(&header, sizeof(header),
                                  offsetof(journal_header_t, crc32));
        if (!write_sector(0, &header, sizeof(header)) ||
            !write_sector(SECTOR_BYTES, &header, sizeof(header))) break;

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.storage_ready = true;
        state.session_id = session_id;
        state.boot_nonce = nonce;
        state.capacity = header.capacity;
        snprintf(state.filename, sizeof(state.filename), "%s", filename);
        state.error[0] = 0;
        xSemaphoreGive(state_mutex);
        session_uptime_base_ms = 0;
        if (!save_active_journal(filename)) {
            ESP_LOGW(TAG, "Could not save active battery journal in NVS");
        }
        okay = true;
    } while (false);
    if (!okay && journal) {
        fclose(journal);
        journal = NULL;
    }
    speaker_storage_unlock();
    return okay;
}

static bool append_sample(const odesc_power_snapshot_t *power,
                          bool telemetry_available)
{
    battery_state_t snapshot;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshot = state;
    xSemaphoreGive(state_mutex);
    if (!journal || snapshot.samples >= snapshot.capacity ||
        !speaker_storage_lock(2000)) return false;

    uint64_t now = session_uptime_ms();
    int64_t wall = epoch_ms();
    float voltage = isfinite(power->voltage_v) ? power->voltage_v : 0.0f;
    float soc = telemetry_available ? voltage_soc(voltage) : snapshot.soc_percent;
    float slope = 0.0f;
    bool have_slope = telemetry_available && slope_estimate(now, soc, &slope);
    float energy = snapshot.energy_wh;
    if (snapshot.last_uptime_ms && telemetry_available && power->current_valid &&
        power->power_w > 0.0f && now - snapshot.last_uptime_ms <=
        SAMPLE_INTERVAL_MS * 2U) {
        energy += power->power_w * (now - snapshot.last_uptime_ms) / 3600000.0f;
    }
    journal_record_t record = {
        .magic = RECORD_MAGIC,
        .sequence = snapshot.samples,
        .flags = (telemetry_available && power->current_valid ?
                  RECORD_FLAG_CURRENT_VALID : 0U) |
                 (wall ? RECORD_FLAG_TIME_VALID : 0U) |
                 (have_slope ? RECORD_FLAG_SLOPE_VALID : 0U) |
                 (!telemetry_available ? RECORD_FLAG_TELEMETRY_GAP : 0U),
        .uptime_ms = now,
        .epoch_ms = wall,
        .voltage_v = telemetry_available ? voltage : snapshot.voltage_v,
        .current_a = telemetry_available ? power->current_a : 0.0f,
        .power_w = telemetry_available ? power->power_w : 0.0f,
        .energy_wh = energy,
        .soc_percent = soc,
        .slope_current_a = slope,
    };
    record.crc32 = object_crc(&record, sizeof(record),
                              offsetof(journal_record_t, crc32));
    journal_header_t header = {
        .record_bytes = RECORD_BYTES,
    };
    bool okay = write_record(&header, &record);
    speaker_storage_unlock();
    if (!okay) return false;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.samples++;
    state.last_uptime_ms = now;
    state.last_epoch_ms = wall;
    state.time_synced = wall != 0;
    if (telemetry_available) state.voltage_v = record.voltage_v;
    state.current_a = telemetry_available ? record.current_a : 0.0f;
    state.power_w = telemetry_available ? record.power_w : 0.0f;
    state.energy_wh = record.energy_wh;
    state.soc_percent = record.soc_percent;
    state.slope_current_a = record.slope_current_a;
    state.slope_valid = have_slope;
    state.error[0] = 0;
    xSemaphoreGive(state_mutex);
    return true;
}

static void logger_task(void *argument)
{
    (void)argument;
    while (!speaker_storage_available(true)) vTaskDelay(pdMS_TO_TICKS(500));
    esp_reset_reason_t reset_reason = esp_reset_reason();
    bool opened = reset_keeps_power_session(reset_reason) && resume_journal();
    if (!opened && !create_journal()) {
        set_error("Could not create preallocated battery journal on microSD");
        vTaskDelete(NULL);
    }
    int64_t next = 0;
    int64_t next_sntp_retry = 0;
    while (true) {
        int64_t now = esp_timer_get_time() / 1000;
        if (!epoch_ms() && now >= next_sntp_retry) {
            esp_err_t result = esp_netif_sntp_start();
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "SNTP retry failed: %s", esp_err_to_name(result));
            }
            next_sntp_retry = now + SNTP_RETRY_MS;
        }
        if (now >= next) {
            next = now + SAMPLE_INTERVAL_MS;
            odesc_power_snapshot_t power = {0};
            bool linked = odesc_link_get_power(&power);
            bool available = linked && power.voltage_valid &&
                             !power.voltage_clipped && power.voltage_age_ms >= 0 &&
                             power.voltage_age_ms <= 3000;
            if (!append_sample(&power, available)) {
                set_error("Battery journal write/sync failed");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static bool valid_journal_name(const char *name)
{
    if (!name || strlen(name) != 16 || strncmp(name, "BAT_", 4) != 0 ||
        strcmp(name + 12, ".jrn") != 0) return false;
    for (unsigned i = 4; i < 12; ++i) {
        char c = name[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return false;
    }
    return true;
}

static int compare_sessions(const void *left, const void *right)
{
    const session_summary_t *a = left;
    const session_summary_t *b = right;
    if (a->current != b->current) return a->current ? -1 : 1;
    if (a->boot_epoch_ms != b->boot_epoch_ms) return a->boot_epoch_ms < b->boot_epoch_ms ? 1 : -1;
    return a->session_id < b->session_id ? 1 : -1;
}

static unsigned list_sessions(session_summary_t *sessions, unsigned maximum)
{
    DIR *directory = opendir(BATTERY_DIR);
    if (!directory) return 0;
    battery_state_t current;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    current = state;
    xSemaphoreGive(state_mutex);
    unsigned count = 0;
    struct dirent *entry;
    while (count < maximum && (entry = readdir(directory)) != NULL) {
        if (!valid_journal_name(entry->d_name)) continue;
        char path[96];
        snprintf(path, sizeof(path), BATTERY_DIR "/%.16s", entry->d_name);
        FILE *file = fopen(path, "rb");
        journal_header_t header;
        if (!file || !read_header(file, &header)) {
            if (file) fclose(file);
            continue;
        }
        uint32_t samples = record_count(file, &header);
        session_summary_t *summary = &sessions[count++];
        memset(summary, 0, sizeof(*summary));
        memcpy(summary->name, entry->d_name, 16);
        summary->name[16] = 0;
        summary->session_id = header.session_id;
        summary->samples = samples;
        summary->reset_reason = header.version >= JOURNAL_VERSION ?
                                (uint8_t)(header.boot_nonce >> 56) : 0;
        journal_record_t record;
        for (uint32_t sequence = 0; sequence < samples; ++sequence) {
            if (!read_record(file, &header, sequence, &record)) break;
            summary->last_uptime_ms = record.uptime_ms;
            if (record.epoch_ms) {
                summary->last_epoch_ms = record.epoch_ms;
                summary->boot_epoch_ms = record.epoch_ms - record.uptime_ms;
            }
            if (record.flags & RECORD_FLAG_TELEMETRY_GAP) {
                summary->gap_samples++;
            } else {
                summary->valid_samples++;
                summary->last_voltage_v = record.voltage_v;
            }
        }
        fclose(file);
        summary->current = strcmp(entry->d_name, current.filename) == 0;
    }
    closedir(directory);
    qsort(sessions, count, sizeof(*sessions), compare_sessions);
    return count;
}

static bool query_value(httpd_req_t *request, const char *key,
                        char *output, size_t output_size)
{
    char query[96];
    return httpd_req_get_url_query_str(request, query, sizeof(query)) == ESP_OK &&
           httpd_query_key_value(query, key, output, output_size) == ESP_OK;
}

static esp_err_t send_chunkf(httpd_req_t *request, const char *format, ...)
{
    char chunk[512];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(chunk, sizeof(chunk), format, arguments);
    va_end(arguments);
    return httpd_resp_send_chunk(request, chunk, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_handler(httpd_req_t *request)
{
    session_summary_t sessions[MAX_API_SESSIONS];
    journal_record_t *samples = NULL;
    uint32_t selected_count = 0;
    uint32_t total_count = 0;
    char selected[32] = {0};
    battery_state_t current;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    current = state;
    xSemaphoreGive(state_mutex);
    if (!speaker_storage_available(true) || !speaker_storage_lock(5000)) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_sendstr(request, "microSD unavailable");
    }
    unsigned session_count = list_sessions(sessions, MAX_API_SESSIONS);
    if (!query_value(request, "session", selected, sizeof(selected)) ||
        !valid_journal_name(selected)) {
        snprintf(selected, sizeof(selected), "%s", current.filename);
    }
    char path[96];
    snprintf(path, sizeof(path), BATTERY_DIR "/%s", selected);
    FILE *file = fopen(path, "rb");
    journal_header_t header;
    if (file && read_header(file, &header)) {
        total_count = record_count(file, &header);
        uint32_t stride = total_count > MAX_API_SAMPLES ?
                          (total_count + MAX_API_SAMPLES - 1) / MAX_API_SAMPLES : 1;
        selected_count = total_count ? (total_count + stride - 1) / stride : 0;
        samples = heap_caps_calloc(selected_count, sizeof(*samples),
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (samples) {
            uint32_t output = 0;
            for (uint32_t sequence = 0; sequence < total_count && output < selected_count;
                 sequence += stride) {
                if (read_record(file, &header, sequence, &samples[output])) output++;
            }
            selected_count = output;
            if (selected_count && samples[selected_count - 1].sequence != total_count - 1 &&
                read_record(file, &header, total_count - 1,
                            &samples[selected_count - 1])) {
                /* Preserve the newest sample when downsampling. */
            }
        }
    }
    if (file) fclose(file);
    speaker_storage_unlock();
    if (selected_count && !samples) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "memory");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    send_chunkf(request,
        "{\"ok\":true,\"selected\":\"%s\",\"uptime_ms\":%llu,"
        "\"pack\":{\"series\":%d,\"parallel\":%d,\"capacity_ah\":%.1f},"
        "\"journal\":{\"sector_bytes\":%u,\"record_bytes\":%u,"
        "\"preallocated_bytes\":%u,"
        "\"sync_each_sample\":true},\"sessions\":[",
        selected, (unsigned long long)session_uptime_ms(), BATTERY_SERIES_CELLS,
        BATTERY_PARALLEL_CELLS, BATTERY_PACK_CAPACITY_AH,
        SECTOR_BYTES, RECORD_BYTES, JOURNAL_BYTES);
    for (unsigned i = 0; i < session_count; ++i) {
        session_summary_t *item = &sessions[i];
        send_chunkf(request,
            "%s{\"name\":\"%s\",\"id\":%lu,\"samples\":%lu,"
            "\"valid_samples\":%lu,\"gap_samples\":%lu,"
            "\"boot_epoch_ms\":%lld,\"last_epoch_ms\":%lld,"
            "\"last_uptime_ms\":%llu,\"last_voltage_v\":%.4f,"
            "\"reset_reason\":%u,\"current\":%s}",
            i ? "," : "", item->name, (unsigned long)item->session_id,
            (unsigned long)item->samples, (unsigned long)item->valid_samples,
            (unsigned long)item->gap_samples, (long long)item->boot_epoch_ms,
            (long long)item->last_epoch_ms, (unsigned long long)item->last_uptime_ms,
            item->last_voltage_v, item->reset_reason,
            item->current ? "true" : "false");
    }
    send_chunkf(request, "],\"sample_count\":%lu,\"samples\":[",
                (unsigned long)total_count);
    for (uint32_t i = 0; i < selected_count; ++i) {
        journal_record_t *sample = &samples[i];
        char voltage[32], soc[32], current[32], power[32], slope[32];
        bool gap = (sample->flags & RECORD_FLAG_TELEMETRY_GAP) != 0;
        if (!gap) {
            snprintf(voltage, sizeof(voltage), "%.4f", sample->voltage_v);
            snprintf(soc, sizeof(soc), "%.2f", sample->soc_percent);
        } else {
            snprintf(voltage, sizeof(voltage), "null");
            snprintf(soc, sizeof(soc), "null");
        }
        if (!gap && (sample->flags & RECORD_FLAG_CURRENT_VALID)) {
            snprintf(current, sizeof(current), "%.4f", sample->current_a);
            snprintf(power, sizeof(power), "%.3f", sample->power_w);
        } else {
            snprintf(current, sizeof(current), "null");
            snprintf(power, sizeof(power), "null");
        }
        if (sample->flags & RECORD_FLAG_SLOPE_VALID)
            snprintf(slope, sizeof(slope), "%.3f", sample->slope_current_a);
        else snprintf(slope, sizeof(slope), "null");
        send_chunkf(request,
            "%s{\"seq\":%lu,\"uptime_ms\":%llu,\"epoch_ms\":%lld,"
            "\"telemetry_valid\":%s,\"voltage_v\":%s,"
            "\"current_a\":%s,\"power_w\":%s,"
            "\"energy_wh\":%.5f,\"soc\":%s,\"slope_a\":%s}",
            i ? "," : "", (unsigned long)sample->sequence,
            (unsigned long long)sample->uptime_ms, (long long)sample->epoch_ms,
            gap ? "false" : "true", voltage, current, power,
            sample->energy_wh, soc, slope);
    }
    free(samples);
    send_chunkf(request, "],\"error\":\"%s\"}", current.error);
    return httpd_resp_send_chunk(request, NULL, 0);
}

static const char page_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>Rover Battery</title><style>*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#17212a;font-family:Arial,sans-serif}header{position:sticky;top:0;z-index:3;background:#17212b;color:white;border-bottom:3px solid #e2a400;padding:10px 12px}.head,main{max-width:900px;margin:auto}h1{font-size:19px;margin:0 0 4px}nav a{color:white;margin-right:13px;font-size:13px}main{padding:12px}.panel{background:white;border:1px solid #cbd3da;border-radius:8px;padding:12px;margin-bottom:12px}.metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px}.metric{border-left:4px solid #16816a;background:#f3f6f7;padding:9px;min-width:0}.metric small{display:block;color:#596673}.metric strong{font-size:18px;overflow-wrap:anywhere}select{width:100%;min-height:44px;padding:8px;font-size:15px;border:1px solid #adb8c2;border-radius:6px;background:white}canvas{display:block;width:100%;height:250px;border:1px solid #cbd3da;border-radius:6px;background:#f8fafb;touch-action:none}.title{display:flex;justify-content:space-between;font-size:12px;font-weight:700;margin-bottom:6px}.note{font-size:12px;line-height:1.45;color:#596673}.bad{color:#b42318}@media(max-width:650px){.metrics{grid-template-columns:1fr 1fr}canvas{height:210px}}@media(max-width:390px){.metrics{grid-template-columns:1fr}}</style></head><body>"
"<header><div class=head><h1>Battery and Power History</h1><nav><a href='/'>Motors</a><a href='/mobile'>Drive</a><a href='/steppers'>Steppers</a><a href='/odrive'>ODrive</a></nav></div></header><main>"
"<section class=panel><select id=session onchange=load(this.value)></select><p id=sessionInfo class=note>Loading journal...</p></section>"
"<section class='panel metrics'><div class=metric><small>Bus voltage</small><strong id=v>--</strong></div><div class=metric><small>Voltage SOC</small><strong id=soc>--</strong></div><div class=metric><small>ODESC current</small><strong id=a>--</strong></div><div class=metric><small>Bus power</small><strong id=w>--</strong></div><div class=metric><small>Session energy</small><strong id=wh>--</strong></div><div class=metric><small>Local fit current estimate</small><strong id=fitA>--</strong></div><div class=metric><small>Voltage trend</small><strong id=fitV>--</strong></div><div class=metric><small>Fit quality / points</small><strong id=fitQ>--</strong></div><div class=metric><small>Journal 5-minute estimate</small><strong id=slope>--</strong></div><div class=metric><small>Session elapsed</small><strong id=up>--</strong></div><div class=metric><small>Sample time</small><strong id=time>--</strong></div></section>"
"<section class=panel><div class=title><span>Pack voltage / estimated SOC</span><span id=range>--</span></div><canvas id=vc></canvas></section><section class=panel><div class=title><span>ODESC current / power</span><span>touch chart to inspect</span></div><canvas id=pc></canvas></section>"
"<section class=panel><p class=note>Each physical power session uses a 512 KiB preallocated journal. Software, OTA, panic, and watchdog resets resume that journal; power-on and brownout resets create a new one. Every five seconds a 64-byte CRC record is flushed and synced. Missing ODESC telemetry is recorded as an explicit chart gap instead of silently shortening the session. After abrupt power loss, an invalid final SD sector is ignored; up to the newest 40 seconds can be lost. One journal holds about 11.4 hours.</p><p class=note>Tap either chart to inspect the nearest durable sample. The blue local-fit overlay uses least-squares regression over up to five minutes before and after that point. Its voltage trend and SOC-derived current estimate are rough 10S4P estimates affected by load changes, sag, recovery, temperature, chemistry, and charging. Fit quality is R-squared; a low value means the estimate is not useful. ODESC current is measured controller-bus telemetry and remains the authoritative reading, but excludes loads wired outside its current path.</p><p id=err class='note bad'></p></section></main>"
"<script>addEventListener('DOMContentLoaded',()=>{let selectedSeq=null,fit=null;function linear(points,key){let n=points.length,mx=points.reduce((s,p)=>s+p.x,0)/n,my=points.reduce((s,p)=>s+p[key],0)/n,sxx=0,sxy=0,syy=0;points.forEach(p=>{let dx=p.x-mx,dy=p[key]-my;sxx+=dx*dx;sxy+=dx*dy;syy+=dy*dy});if(n<5||sxx<=0)return null;let b=sxy/sxx;return{a:my-b*mx,b,r2:syy>0?sxy*sxy/(sxx*syy):0}}function localFit(i){let center=D.samples[i].uptime_ms,points=D.samples.filter(o=>o.telemetry_valid&&Number.isFinite(o.voltage_v)&&Number.isFinite(o.soc)&&Math.abs(o.uptime_ms-center)<=300000).map(o=>({t:o.uptime_ms,x:(o.uptime_ms-center)/3600000,v:o.voltage_v,s:o.soc})),v=linear(points,'v'),s=linear(points,'s');if(!v||!s)return null;return{center,min:points[0].t,max:points[points.length-1].t,points:points.length,va:v.a,vb:v.b,vr2:v.r2,sb:s.b,sr2:s.r2,current:-s.b*D.pack.capacity_ah/100}}"
"chart=function(id,series){let q=ctx(id),x=q.x,w=q.w,h=q.h,p={l:42,r:18,t:20,b:24},a=D.samples||[];x.clearRect(0,0,w,h);x.strokeStyle='#dce2e6';for(let j=0;j<5;j++){let y=p.t+j*(h-p.t-p.b)/4;x.beginPath();x.moveTo(p.l,y);x.lineTo(w-p.r,y);x.stroke()}if(!a.length){x.fillText('No samples',w/2,h/2);return}let ranges=series.map(z=>{let v=a.map(z.get).filter(Number.isFinite);if(!v.length)return{lo:0,hi:1};let lo=z.lo??Math.min(...v),hi=z.hi??Math.max(...v);if(hi-lo<.01){lo-=1;hi+=1}return{lo,hi}}),t0=a[0].uptime_ms,t1=a[a.length-1].uptime_ms,ptx=t=>p.l+(t-t0)*(w-p.l-p.r)/Math.max(1,t1-t0),px=j=>ptx(a[j].uptime_ms);series.forEach((z,k)=>{let R=ranges[k],py=v=>p.t+(R.hi-v)*(h-p.t-p.b)/(R.hi-R.lo);x.strokeStyle=z.c;x.lineWidth=2;x.beginPath();let started=false;a.forEach((o,j)=>{let v=z.get(o);if(!Number.isFinite(v)){started=false;return}started?x.lineTo(px(j),py(v)):x.moveTo(px(j),py(v));started=true});x.stroke();x.fillStyle=z.c;x.fillText(z.n,p.l+k*120,13)});if(id==='vc'&&fit){let R=ranges[0],py=v=>p.t+(R.hi-v)*(h-p.t-p.b)/(R.hi-R.lo),fv=t=>fit.va+fit.vb*(t-fit.center)/3600000;x.save();x.strokeStyle='#30a8ff';x.lineWidth=2;x.setLineDash([6,4]);x.beginPath();x.moveTo(ptx(fit.min),py(fv(fit.min)));x.lineTo(ptx(fit.max),py(fv(fit.max)));x.stroke();x.restore()}if(I!==null){x.strokeStyle='#17212a';x.beginPath();x.moveTo(px(I),p.t);x.lineTo(px(I),h-p.b);x.stroke()}};"
"inspect=function(i){if(!D.samples.length)return;I=Math.max(0,Math.min(D.samples.length-1,i));let o=D.samples[I];selectedSeq=o.seq;fit=localFit(I);$('v').textContent=val(o.voltage_v,2,' V');$('soc').textContent=val(o.soc,1,'%');$('a').textContent=val(o.current_a,2,' A');$('w').textContent=val(o.power_w,1,' W');$('wh').textContent=val(o.energy_wh,3,' Wh');$('slope').textContent=val(o.slope_a,2,' A');$('fitA').textContent=fit?val(fit.current,2,' A'):'--';$('fitV').textContent=fit?val(fit.vb,3,' V/h'):'--';$('fitQ').textContent=fit?fit.sr2.toFixed(2)+' / '+fit.points:'--';$('up').textContent=dur(o.uptime_ms);$('time').textContent=o.epoch_ms?new Date(o.epoch_ms).toLocaleString():'internet time unavailable';draw()};let originalRender=render;render=function(d){let keep=selectedSeq;originalRender(d);if(keep!==null){let i=D.samples.findIndex(o=>o.seq===keep);if(i>=0)inspect(i)}}})</script>"
"<script>const $=x=>document.getElementById(x);let D=null,I=null;const resets=['legacy/unknown','power on','external reset','software/OTA','panic','interrupt watchdog','task watchdog','watchdog','deep sleep','brownout','SDIO','USB','JTAG','eFuse','power glitch','CPU lockup'];function dur(ms){let s=Math.round(ms/1000),h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m '+s%60+'s'}function val(x,n,u){return x===null||x===undefined?'--':Number(x).toFixed(n)+u}function ctx(id){let c=$(id),r=devicePixelRatio||1,w=Math.max(280,c.clientWidth),h=Math.max(180,c.clientHeight);c.width=w*r;c.height=h*r;let x=c.getContext('2d');x.setTransform(r,0,0,r,0,0);return{x,w,h}}function chart(id,series){let q=ctx(id),x=q.x,w=q.w,h=q.h,p={l:42,r:18,t:20,b:24},a=D.samples||[];x.clearRect(0,0,w,h);x.strokeStyle='#dce2e6';for(let j=0;j<5;j++){let y=p.t+j*(h-p.t-p.b)/4;x.beginPath();x.moveTo(p.l,y);x.lineTo(w-p.r,y);x.stroke()}if(!a.length){x.fillText('No samples',w/2,h/2);return}let ranges=series.map(z=>{let v=a.map(z.get).filter(Number.isFinite);if(!v.length)return{lo:0,hi:1};let lo=z.lo??Math.min(...v),hi=z.hi??Math.max(...v);if(hi-lo<.01){lo-=1;hi+=1}return{lo,hi}}),t0=a[0].uptime_ms,t1=a[a.length-1].uptime_ms,px=j=>p.l+(a[j].uptime_ms-t0)*(w-p.l-p.r)/Math.max(1,t1-t0);series.forEach((z,k)=>{let R=ranges[k],py=v=>p.t+(R.hi-v)*(h-p.t-p.b)/(R.hi-R.lo);x.strokeStyle=z.c;x.lineWidth=2;x.beginPath();let started=false;a.forEach((o,j)=>{let v=z.get(o);if(!Number.isFinite(v)){started=false;return}started?x.lineTo(px(j),py(v)):x.moveTo(px(j),py(v));started=true});x.stroke();x.fillStyle=z.c;x.fillText(z.n,p.l+k*120,13)});if(I!==null){x.strokeStyle='#17212a';x.beginPath();x.moveTo(px(I),p.t);x.lineTo(px(I),h-p.b);x.stroke()}}function draw(){chart('vc',[{n:'voltage V',c:'#1268a8',get:o=>o.voltage_v},{n:'SOC %',c:'#24734c',lo:0,hi:100,get:o=>o.soc}]);chart('pc',[{n:'current A',c:'#a96800',get:o=>o.current_a},{n:'power W',c:'#bd241c',get:o=>o.power_w}])}function inspect(i){if(!D.samples.length)return;I=Math.max(0,Math.min(D.samples.length-1,i));let o=D.samples[I];$('v').textContent=val(o.voltage_v,2,' V');$('soc').textContent=val(o.soc,1,'%');$('a').textContent=val(o.current_a,2,' A');$('w').textContent=val(o.power_w,1,' W');$('wh').textContent=val(o.energy_wh,3,' Wh');$('slope').textContent=val(o.slope_a,2,' A');$('up').textContent=dur(o.uptime_ms);$('time').textContent=o.epoch_ms?new Date(o.epoch_ms).toLocaleString():'internet time unavailable';draw()}function render(d){D=d;let selected=d.selected;$('session').innerHTML=d.sessions.map(s=>`<option value='${s.name}' ${s.name===selected?'selected':''}>${s.boot_epoch_ms?new Date(s.boot_epoch_ms).toLocaleString():'boot '+s.id} | ${dur(s.last_uptime_ms)} | ${resets[s.reset_reason]||'reset '+s.reset_reason}${s.current?' (current)':''}</option>`).join('');let s=d.sessions.find(x=>x.name===selected),valid=s?(s.valid_samples??s.samples):0,gaps=s?(s.gap_samples??0):0;$('sessionInfo').textContent=s?dur(s.last_uptime_ms)+' session | '+valid+' measured | '+gaps+' missing telemetry | '+s.samples+' durable records':'No session';$('err').textContent=d.error||'';if(d.samples.length){let v=d.samples.map(x=>x.voltage_v).filter(Number.isFinite);$('range').textContent=v.length?Math.min(...v).toFixed(2)+' to '+Math.max(...v).toFixed(2)+' V':'telemetry unavailable';inspect(d.samples.length-1)}else{I=null;draw()}}function load(s=''){fetch('/api/battery'+(s?'?session='+encodeURIComponent(s):''),{cache:'no-store'}).then(r=>r.json()).then(render).catch(e=>$('err').textContent=e)}['vc','pc'].forEach(id=>$(id).addEventListener('pointerdown',e=>{let r=e.target.getBoundingClientRect(),q=Math.max(0,Math.min(1,(e.clientX-r.left-42)/(r.width-60))),target=D.samples[0].uptime_ms+q*(D.samples[D.samples.length-1].uptime_ms-D.samples[0].uptime_ms),best=0;D.samples.forEach((o,i)=>{if(Math.abs(o.uptime_ms-target)<Math.abs(D.samples[best].uptime_ms-target))best=i});inspect(best)}));addEventListener('resize',draw);setInterval(()=>{if($('session').value&&D.sessions.find(s=>s.name===$('session').value)?.current)load($('session').value)},5000);load()</script></body></html>";

static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    return httpd_resp_send(request, page_html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t battery_history_start(void)
{
    state_mutex = xSemaphoreCreateMutex();
    if (!state_mutex) return ESP_ERR_NO_MEM;
    state.running = true;
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.start = false;
    config.renew_servers_after_new_IP = true;
    esp_err_t time_result = esp_netif_sntp_init(&config);
    if (time_result != ESP_OK && time_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SNTP initialization failed: %s", esp_err_to_name(time_result));
    }
    return xTaskCreate(logger_task, "battery_log", 6144, NULL, 2, NULL) == pdPASS ?
           ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t battery_history_register_routes(httpd_handle_t server)
{
    const httpd_uri_t routes[] = {
        {.uri = "/battery", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/batter", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/battery", .method = HTTP_GET, .handler = api_handler},
    };
    for (unsigned i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        esp_err_t result = httpd_register_uri_handler(server, &routes[i]);
        if (result != ESP_OK) return result;
    }
    return ESP_OK;
}
