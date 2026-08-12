#include "mower_history.h"

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
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "odesc_link.h"
#include "speaker.h"

#define MOWER_DIR "/storage/mower"
#define JOURNAL_BYTES (8U * 1024U * 1024U)
#define SECTOR_BYTES 512U
#define HEADER_SECTORS 2U
#define RECORD_BYTES 128U
#define SAMPLE_INTERVAL_MS 1000U
#define MAX_API_SESSIONS 64U
#define VALID_EPOCH_SECONDS 1704067200LL

#define HEADER_MAGIC 0x3148574dU
#define RECORD_MAGIC 0x3152574dU
#define JOURNAL_VERSION 1U

#define FLAG_LINK_VALID 0x0001U
#define FLAG_VOLTAGE_VALID 0x0002U
#define FLAG_CURRENT_VALID 0x0004U
#define FLAG_MOTION_VALID 0x0008U
#define FLAG_ACTIVE 0x0010U
#define FLAG_TIME_VALID 0x0020U
#define FLAG_FAULTED 0x0040U
#define FLAG_TELEMETRY_GAP 0x0080U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t record_bytes;
    uint32_t capacity;
    uint32_t session_id;
    uint64_t boot_nonce;
    uint32_t reset_reason;
    uint32_t crc32;
} journal_header_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t sequence;
    uint32_t flags;
    uint64_t uptime_ms;
    int64_t epoch_ms;
    uint64_t mower_active_ms;
    int32_t axis_state;
    uint32_t system_error;
    uint32_t axis_error;
    uint32_t motor_error;
    uint32_t controller_error;
    uint32_t estimator_error;
    float voltage_v;
    float bus_current_a;
    float bus_power_w;
    float energy_wh;
    float command_turns_s;
    float estimated_rpm;
    float iq_measured_a;
    float iq_setpoint_a;
    float id_measured_a;
    float id_setpoint_a;
    float phase_current_magnitude_a;
    float motor_voltage_v;
    float motor_power_w;
    float fet_temperature_c;
    uint32_t crc32;
} journal_record_t;

_Static_assert(sizeof(journal_record_t) <= RECORD_BYTES,
               "mower journal record exceeds fixed slot");

typedef struct {
    char name[20];
    uint32_t session_id;
    uint32_t samples;
    uint32_t reset_reason;
    int64_t boot_epoch_ms;
    int64_t last_epoch_ms;
    uint64_t elapsed_ms;
    uint64_t active_ms;
    float energy_wh;
    bool current;
} session_item_t;

typedef struct {
    uint32_t samples;
    uint32_t linked_samples;
    uint32_t motion_samples;
    uint32_t active_samples;
    uint32_t fault_samples;
    uint32_t gap_samples;
    uint64_t elapsed_ms;
    uint64_t active_ms;
    int64_t boot_epoch_ms;
    int64_t last_epoch_ms;
    float energy_wh;
    float voltage_min_v;
    float voltage_max_v;
    float average_voltage_v;
    float average_bus_current_a;
    float peak_bus_current_a;
    float average_bus_power_w;
    float peak_bus_power_w;
    float average_iq_a;
    float peak_iq_a;
    float average_id_a;
    float peak_id_a;
    float average_phase_current_a;
    float peak_phase_current_a;
    float average_rpm;
    float peak_rpm;
    float average_motor_power_w;
    float peak_motor_power_w;
    float average_motor_voltage_v;
    float peak_motor_voltage_v;
    float average_fet_temperature_c;
    float peak_fet_temperature_c;
    uint32_t final_system_error;
    uint32_t final_axis_error;
    uint32_t final_motor_error;
    uint32_t final_controller_error;
    uint32_t final_estimator_error;
} session_stats_t;

typedef struct {
    bool storage_ready;
    uint32_t session_id;
    uint32_t samples;
    uint32_t capacity;
    uint64_t active_ms;
    float energy_wh;
    bool previous_active;
    bool previous_power_valid;
    float previous_power_w;
    uint64_t previous_uptime_ms;
    char filename[20];
    char error[128];
} mower_state_t;

static const char *TAG = "mower_history";
static SemaphoreHandle_t state_mutex;
static mower_state_t state;
static FILE *journal;

static uint64_t uptime_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
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
    return header->magic == HEADER_MAGIC &&
           header->version == JOURNAL_VERSION &&
           header->record_bytes == RECORD_BYTES && header->capacity &&
           header->crc32 == object_crc(header, sizeof(*header),
                                       offsetof(journal_header_t, crc32));
}

static bool valid_record(journal_record_t *record, uint32_t sequence)
{
    return record->magic == RECORD_MAGIC && record->sequence == sequence &&
           record->crc32 == object_crc(record, sizeof(*record),
                                       offsetof(journal_record_t, crc32));
}

static bool valid_journal_name(const char *name)
{
    if (!name || strlen(name) != 16 || strncmp(name, "MOW_", 4) != 0 ||
        strcmp(name + 12, ".jrn") != 0) return false;
    for (unsigned i = 4; i < 12; ++i) {
        char c = name[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return false;
    }
    return true;
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
    uint8_t slot[RECORD_BYTES];
    if (header->record_bytes != RECORD_BYTES ||
        fseek(file, record_offset(header, sequence), SEEK_SET) != 0 ||
        fread(slot, 1, sizeof(slot), file) != sizeof(slot)) return false;
    memcpy(record, slot, sizeof(*record));
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

static bool write_record(const journal_record_t *record)
{
    uint8_t slot[RECORD_BYTES] = {0};
    memcpy(slot, record, sizeof(*record));
    journal_header_t header = {.record_bytes = RECORD_BYTES};
    return fseek(journal, record_offset(&header, record->sequence), SEEK_SET) == 0 &&
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

static bool create_journal(void)
{
    if (!speaker_storage_available(true) || !speaker_storage_lock(10000)) return false;
    bool okay = false;
    do {
        if (mkdir(MOWER_DIR, 0775) != 0 && errno != EEXIST) break;
        uint32_t session_id = 0;
        char filename[20];
        char path[96];
        struct stat existing;
        for (unsigned attempt = 0; attempt < 32; ++attempt) {
            session_id = esp_random();
            snprintf(filename, sizeof(filename), "MOW_%08lX.jrn",
                     (unsigned long)session_id);
            snprintf(path, sizeof(path), MOWER_DIR "/%s", filename);
            if (stat(path, &existing) != 0) break;
            session_id = 0;
        }
        if (!session_id) break;
        journal = fopen(path, "wb+");
        if (!journal) break;
        if (fseek(journal, JOURNAL_BYTES - 1, SEEK_SET) != 0 ||
            fputc(0, journal) == EOF || fflush(journal) != 0 ||
            fsync(fileno(journal)) != 0) break;

        journal_header_t header = {
            .magic = HEADER_MAGIC,
            .version = JOURNAL_VERSION,
            .record_bytes = RECORD_BYTES,
            .capacity = (JOURNAL_BYTES - HEADER_SECTORS * SECTOR_BYTES) /
                        RECORD_BYTES,
            .session_id = session_id,
            .boot_nonce = ((uint64_t)esp_random() << 32) | esp_random(),
            .reset_reason = (uint32_t)esp_reset_reason(),
        };
        header.crc32 = object_crc(&header, sizeof(header),
                                  offsetof(journal_header_t, crc32));
        if (!write_sector(0, &header, sizeof(header)) ||
            !write_sector(SECTOR_BYTES, &header, sizeof(header))) break;

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.storage_ready = true;
        state.session_id = session_id;
        state.capacity = header.capacity;
        snprintf(state.filename, sizeof(state.filename), "%s", filename);
        state.error[0] = 0;
        xSemaphoreGive(state_mutex);
        okay = true;
        ESP_LOGI(TAG, "Mower journal %s ready (%lu samples, %.1f hours)",
                 filename, (unsigned long)header.capacity,
                 header.capacity * SAMPLE_INTERVAL_MS / 3600000.0);
    } while (false);
    if (!okay && journal) {
        fclose(journal);
        journal = NULL;
    }
    speaker_storage_unlock();
    return okay;
}

static bool append_sample(const odesc_mower_snapshot_t *telemetry)
{
    mower_state_t previous;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    previous = state;
    xSemaphoreGive(state_mutex);
    if (!journal || previous.samples >= previous.capacity ||
        !speaker_storage_lock(2500)) return false;

    uint64_t now = uptime_ms();
    int64_t wall = epoch_ms();
    bool motion_fresh = telemetry->motion_valid && telemetry->telemetry_age_ms >= 0 &&
                        telemetry->telemetry_age_ms <=
                        (telemetry->active ? 1800 : 6500);
    bool current_valid = telemetry->current_valid && motion_fresh;
    bool active = telemetry->active && motion_fresh;
    uint64_t delta = previous.previous_uptime_ms ?
                     now - previous.previous_uptime_ms : 0;
    if (delta > SAMPLE_INTERVAL_MS * 3U) delta = 0;
    uint64_t active_ms = previous.active_ms;
    if (active && previous.previous_active) active_ms += delta;

    float energy_wh = previous.energy_wh;
    if (active && previous.previous_active && current_valid &&
        previous.previous_power_valid && delta) {
        float average_power = (telemetry->bus_power_w +
                               previous.previous_power_w) * 0.5f;
        if (average_power > 0.0f && isfinite(average_power)) {
            energy_wh += average_power * delta / 3600000.0f;
        }
    }
    uint32_t flags = (telemetry->connected ? FLAG_LINK_VALID : 0U) |
                     (telemetry->voltage_valid ? FLAG_VOLTAGE_VALID : 0U) |
                     (current_valid ? FLAG_CURRENT_VALID : 0U) |
                     (motion_fresh ? FLAG_MOTION_VALID : 0U) |
                     (active ? FLAG_ACTIVE : 0U) |
                     (wall ? FLAG_TIME_VALID : 0U) |
                     (telemetry->faulted ? FLAG_FAULTED : 0U) |
                     (!telemetry->connected || !motion_fresh ?
                      FLAG_TELEMETRY_GAP : 0U);
    journal_record_t record = {
        .magic = RECORD_MAGIC,
        .sequence = previous.samples,
        .flags = flags,
        .uptime_ms = now,
        .epoch_ms = wall,
        .mower_active_ms = active_ms,
        .axis_state = telemetry->axis_state,
        .system_error = telemetry->system_error,
        .axis_error = telemetry->axis_error,
        .motor_error = telemetry->motor_error,
        .controller_error = telemetry->controller_error,
        .estimator_error = telemetry->estimator_error,
        .voltage_v = telemetry->voltage_v,
        .bus_current_a = telemetry->bus_current_a,
        .bus_power_w = telemetry->bus_power_w,
        .energy_wh = energy_wh,
        .command_turns_s = telemetry->command_turns_s,
        .estimated_rpm = telemetry->estimated_rpm,
        .iq_measured_a = telemetry->iq_measured_a,
        .iq_setpoint_a = telemetry->iq_setpoint_a,
        .id_measured_a = telemetry->id_measured_a,
        .id_setpoint_a = telemetry->id_setpoint_a,
        .phase_current_magnitude_a = telemetry->phase_current_magnitude_a,
        .motor_voltage_v = telemetry->motor_voltage_v,
        .motor_power_w = telemetry->motor_power_w,
        .fet_temperature_c = telemetry->fet_temperature_c,
    };
    record.crc32 = object_crc(&record, sizeof(record),
                              offsetof(journal_record_t, crc32));
    bool okay = write_record(&record);
    speaker_storage_unlock();
    if (!okay) return false;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.samples++;
    state.active_ms = active_ms;
    state.energy_wh = energy_wh;
    state.previous_active = active;
    state.previous_power_valid = current_valid;
    state.previous_power_w = telemetry->bus_power_w;
    state.previous_uptime_ms = now;
    state.error[0] = 0;
    xSemaphoreGive(state_mutex);
    return true;
}

static void logger_task(void *argument)
{
    (void)argument;
    while (!speaker_storage_available(true)) vTaskDelay(pdMS_TO_TICKS(500));
    if (!create_journal()) {
        set_error("Could not create preallocated mower journal on microSD");
        vTaskDelete(NULL);
    }
    int64_t next = 0;
    while (true) {
        int64_t now = esp_timer_get_time() / 1000;
        if (now >= next) {
            next = now + SAMPLE_INTERVAL_MS;
            odesc_mower_snapshot_t telemetry = {0};
            (void)odesc_link_get_mower_telemetry(&telemetry);
            if (!append_sample(&telemetry)) {
                set_error("Mower journal write/sync failed");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static int compare_sessions(const void *left, const void *right)
{
    const session_item_t *a = left;
    const session_item_t *b = right;
    if (a->current != b->current) return a->current ? -1 : 1;
    if (a->boot_epoch_ms != b->boot_epoch_ms) {
        return a->boot_epoch_ms < b->boot_epoch_ms ? 1 : -1;
    }
    return a->session_id < b->session_id ? 1 : -1;
}

static unsigned list_sessions(session_item_t *items, unsigned maximum)
{
    DIR *directory = opendir(MOWER_DIR);
    if (!directory) return 0;
    mower_state_t current;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    current = state;
    xSemaphoreGive(state_mutex);
    unsigned count = 0;
    struct dirent *entry;
    while (count < maximum && (entry = readdir(directory)) != NULL) {
        if (!valid_journal_name(entry->d_name)) continue;
        char path[96];
        snprintf(path, sizeof(path), MOWER_DIR "/%.16s", entry->d_name);
        FILE *file = fopen(path, "rb");
        journal_header_t header;
        if (!file || !read_header(file, &header)) {
            if (file) fclose(file);
            continue;
        }
        uint32_t samples = record_count(file, &header);
        journal_record_t last = {0};
        bool have_last = samples && read_record(file, &header, samples - 1, &last);
        fclose(file);
        session_item_t *item = &items[count++];
        memset(item, 0, sizeof(*item));
        snprintf(item->name, sizeof(item->name), "%.16s", entry->d_name);
        item->session_id = header.session_id;
        item->reset_reason = header.reset_reason;
        item->samples = samples;
        item->current = strcmp(item->name, current.filename) == 0;
        if (have_last) {
            item->elapsed_ms = last.uptime_ms;
            item->active_ms = last.mower_active_ms;
            item->energy_wh = last.energy_wh;
            item->last_epoch_ms = last.epoch_ms;
            if (last.epoch_ms) item->boot_epoch_ms = last.epoch_ms - last.uptime_ms;
        }
    }
    closedir(directory);
    qsort(items, count, sizeof(*items), compare_sessions);
    return count;
}

static void update_abs_peak(float value, float *peak)
{
    value = fabsf(value);
    if (isfinite(value) && value > *peak) *peak = value;
}

static bool calculate_stats(FILE *file, const journal_header_t *header,
                            uint32_t count, session_stats_t *stats)
{
    memset(stats, 0, sizeof(*stats));
    stats->voltage_min_v = INFINITY;
    double voltage_sum = 0.0;
    double current_sum = 0.0;
    double iq_sum = 0.0;
    double id_sum = 0.0;
    double phase_sum = 0.0;
    double rpm_sum = 0.0;
    double motor_power_sum = 0.0;
    double motor_voltage_sum = 0.0;
    double fet_sum = 0.0;
    uint32_t voltage_samples = 0;
    uint32_t current_samples = 0;
    uint32_t iq_samples = 0;
    uint32_t id_samples = 0;
    uint32_t phase_samples = 0;
    uint32_t rpm_samples = 0;
    uint32_t motor_power_samples = 0;
    uint32_t motor_voltage_samples = 0;
    uint32_t fet_samples = 0;
    journal_record_t record;
    for (uint32_t sequence = 0; sequence < count; ++sequence) {
        if (!read_record(file, header, sequence, &record)) return false;
        stats->samples++;
        stats->elapsed_ms = record.uptime_ms;
        stats->active_ms = record.mower_active_ms;
        stats->energy_wh = record.energy_wh;
        stats->last_epoch_ms = record.epoch_ms;
        if (record.epoch_ms) stats->boot_epoch_ms = record.epoch_ms - record.uptime_ms;
        if (record.flags & FLAG_LINK_VALID) stats->linked_samples++;
        if (record.flags & FLAG_MOTION_VALID) stats->motion_samples++;
        if (record.flags & FLAG_ACTIVE) stats->active_samples++;
        if (record.flags & FLAG_FAULTED) stats->fault_samples++;
        if (record.flags & FLAG_TELEMETRY_GAP) stats->gap_samples++;
        if (record.flags & FLAG_VOLTAGE_VALID) {
            voltage_sum += record.voltage_v;
            voltage_samples++;
            if (record.voltage_v < stats->voltage_min_v) {
                stats->voltage_min_v = record.voltage_v;
            }
            if (record.voltage_v > stats->voltage_max_v) {
                stats->voltage_max_v = record.voltage_v;
            }
        }
        if ((record.flags & (FLAG_ACTIVE | FLAG_CURRENT_VALID)) ==
            (FLAG_ACTIVE | FLAG_CURRENT_VALID)) {
            float draw_current = fmaxf(record.bus_current_a, 0.0f);
            float draw_power = fmaxf(record.bus_power_w, 0.0f);
            current_sum += draw_current;
            current_samples++;
            if (draw_current > stats->peak_bus_current_a) {
                stats->peak_bus_current_a = draw_current;
            }
            if (draw_power > stats->peak_bus_power_w) {
                stats->peak_bus_power_w = draw_power;
            }
        }
        if ((record.flags & (FLAG_ACTIVE | FLAG_MOTION_VALID)) ==
            (FLAG_ACTIVE | FLAG_MOTION_VALID)) {
            iq_sum += fabsf(record.iq_measured_a);
            iq_samples++;
            update_abs_peak(record.iq_measured_a, &stats->peak_iq_a);
            if (isfinite(record.id_measured_a)) {
                id_sum += fabsf(record.id_measured_a);
                id_samples++;
            }
            update_abs_peak(record.id_measured_a, &stats->peak_id_a);
            if (isfinite(record.phase_current_magnitude_a)) {
                phase_sum += fabsf(record.phase_current_magnitude_a);
                phase_samples++;
            }
            update_abs_peak(record.phase_current_magnitude_a,
                            &stats->peak_phase_current_a);
            if (isfinite(record.estimated_rpm)) {
                rpm_sum += fabsf(record.estimated_rpm);
                rpm_samples++;
            }
            update_abs_peak(record.estimated_rpm, &stats->peak_rpm);
            if (isfinite(record.motor_power_w)) {
                motor_power_sum += fabsf(record.motor_power_w);
                motor_power_samples++;
            }
            update_abs_peak(record.motor_power_w, &stats->peak_motor_power_w);
            if (isfinite(record.motor_voltage_v)) {
                motor_voltage_sum += fabsf(record.motor_voltage_v);
                motor_voltage_samples++;
            }
            update_abs_peak(record.motor_voltage_v, &stats->peak_motor_voltage_v);
            if (isfinite(record.fet_temperature_c) &&
                record.fet_temperature_c > stats->peak_fet_temperature_c) {
                stats->peak_fet_temperature_c = record.fet_temperature_c;
            }
            if (isfinite(record.fet_temperature_c)) {
                fet_sum += record.fet_temperature_c;
                fet_samples++;
            }
        }
        stats->final_system_error = record.system_error;
        stats->final_axis_error = record.axis_error;
        stats->final_motor_error = record.motor_error;
        stats->final_controller_error = record.controller_error;
        stats->final_estimator_error = record.estimator_error;
    }
    if (!isfinite(stats->voltage_min_v)) stats->voltage_min_v = 0.0f;
    stats->average_voltage_v = voltage_samples ?
        (float)(voltage_sum / voltage_samples) : 0.0f;
    stats->average_bus_current_a = current_samples ?
        (float)(current_sum / current_samples) : 0.0f;
    stats->average_iq_a = iq_samples ? (float)(iq_sum / iq_samples) : 0.0f;
    stats->average_id_a = id_samples ? (float)(id_sum / id_samples) : 0.0f;
    stats->average_phase_current_a = phase_samples ?
        (float)(phase_sum / phase_samples) : 0.0f;
    stats->average_rpm = rpm_samples ? (float)(rpm_sum / rpm_samples) : 0.0f;
    stats->average_motor_power_w = motor_power_samples ?
        (float)(motor_power_sum / motor_power_samples) : 0.0f;
    stats->average_motor_voltage_v = motor_voltage_samples ?
        (float)(motor_voltage_sum / motor_voltage_samples) : 0.0f;
    stats->average_fet_temperature_c = fet_samples ?
        (float)(fet_sum / fet_samples) : 0.0f;
    stats->average_bus_power_w = stats->active_ms ?
        stats->energy_wh * 3600000.0f / stats->active_ms : 0.0f;
    return true;
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
    char chunk[640];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(chunk, sizeof(chunk), format, arguments);
    va_end(arguments);
    return httpd_resp_send_chunk(request, chunk, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_handler(httpd_req_t *request)
{
    mower_state_t current;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    current = state;
    xSemaphoreGive(state_mutex);
    if (!speaker_storage_available(true) || !speaker_storage_lock(8000)) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_sendstr(request, "microSD mower journal unavailable");
    }
    session_item_t sessions[MAX_API_SESSIONS];
    unsigned session_count = list_sessions(sessions, MAX_API_SESSIONS);
    char selected[20] = {0};
    if (!query_value(request, "session", selected, sizeof(selected)) ||
        !valid_journal_name(selected)) {
        snprintf(selected, sizeof(selected), "%s", current.filename);
    }
    char path[96];
    snprintf(path, sizeof(path), MOWER_DIR "/%s", selected);
    FILE *file = fopen(path, "rb");
    journal_header_t header = {0};
    session_stats_t stats = {0};
    bool okay = file && read_header(file, &header);
    uint32_t count = okay ? record_count(file, &header) : 0;
    if (okay) okay = calculate_stats(file, &header, count, &stats);
    if (file) fclose(file);
    speaker_storage_unlock();
    if (!okay) {
        return httpd_resp_send_err(request, HTTPD_404_NOT_FOUND,
                                   "mower session unavailable");
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    send_chunkf(request,
        "{\"ok\":true,\"current\":\"%s\",\"selected\":\"%s\","
        "\"journal\":{\"bytes\":%u,\"record_bytes\":%u,"
        "\"interval_ms\":%u,\"capacity\":%lu,\"sync_each_sample\":true},"
        "\"sessions\":[",
        current.filename, selected, JOURNAL_BYTES, RECORD_BYTES,
        SAMPLE_INTERVAL_MS, (unsigned long)header.capacity);
    for (unsigned i = 0; i < session_count; ++i) {
        session_item_t *item = &sessions[i];
        send_chunkf(request,
            "%s{\"name\":\"%s\",\"id\":%lu,\"samples\":%lu,"
            "\"reset_reason\":%lu,\"boot_epoch_ms\":%lld,"
            "\"last_epoch_ms\":%lld,\"elapsed_ms\":%llu,"
            "\"active_ms\":%llu,\"energy_wh\":%.5f,\"current\":%s}",
            i ? "," : "", item->name, (unsigned long)item->session_id,
            (unsigned long)item->samples, (unsigned long)item->reset_reason,
            (long long)item->boot_epoch_ms, (long long)item->last_epoch_ms,
            (unsigned long long)item->elapsed_ms,
            (unsigned long long)item->active_ms, item->energy_wh,
            item->current ? "true" : "false");
    }
    send_chunkf(request,
        "],\"stats\":{\"samples\":%lu,\"linked_samples\":%lu,"
        "\"motion_samples\":%lu,\"active_samples\":%lu,"
        "\"fault_samples\":%lu,\"gap_samples\":%lu,"
        "\"elapsed_ms\":%llu,\"active_ms\":%llu,"
        "\"boot_epoch_ms\":%lld,\"last_epoch_ms\":%lld,"
        "\"energy_wh\":%.6f,\"voltage_min_v\":%.4f,"
        "\"voltage_max_v\":%.4f,\"average_voltage_v\":%.4f,",
        (unsigned long)stats.samples, (unsigned long)stats.linked_samples,
        (unsigned long)stats.motion_samples, (unsigned long)stats.active_samples,
        (unsigned long)stats.fault_samples, (unsigned long)stats.gap_samples,
        (unsigned long long)stats.elapsed_ms,
        (unsigned long long)stats.active_ms,
        (long long)stats.boot_epoch_ms, (long long)stats.last_epoch_ms,
        stats.energy_wh, stats.voltage_min_v, stats.voltage_max_v,
        stats.average_voltage_v);
    send_chunkf(request,
        "\"average_bus_current_a\":%.4f,\"peak_bus_current_a\":%.4f,"
        "\"average_bus_power_w\":%.3f,\"peak_bus_power_w\":%.3f,"
        "\"average_iq_a\":%.4f,\"peak_iq_a\":%.4f,",
        stats.average_bus_current_a, stats.peak_bus_current_a,
        stats.average_bus_power_w, stats.peak_bus_power_w,
        stats.average_iq_a, stats.peak_iq_a);
    send_chunkf(request,
        "\"average_id_a\":%.4f,\"peak_id_a\":%.4f,"
        "\"average_phase_current_a\":%.4f,\"peak_phase_current_a\":%.4f,"
        "\"average_rpm\":%.2f,\"peak_rpm\":%.2f,",
        stats.average_id_a, stats.peak_id_a,
        stats.average_phase_current_a, stats.peak_phase_current_a,
        stats.average_rpm, stats.peak_rpm);
    send_chunkf(request,
        "\"average_motor_power_w\":%.3f,\"peak_motor_power_w\":%.3f,"
        "\"average_motor_voltage_v\":%.4f,\"peak_motor_voltage_v\":%.4f,"
        "\"average_fet_temperature_c\":%.3f,"
        "\"peak_fet_temperature_c\":%.3f,",
        stats.average_motor_power_w, stats.peak_motor_power_w,
        stats.average_motor_voltage_v, stats.peak_motor_voltage_v,
        stats.average_fet_temperature_c, stats.peak_fet_temperature_c);
    send_chunkf(request,
        "\"final_system_error\":%lu,\"final_axis_error\":%lu,"
        "\"final_motor_error\":%lu,\"final_controller_error\":%lu,"
        "\"final_estimator_error\":%lu},\"error\":\"%.110s\"}",
        (unsigned long)stats.final_system_error,
        (unsigned long)stats.final_axis_error,
        (unsigned long)stats.final_motor_error,
        (unsigned long)stats.final_controller_error,
        (unsigned long)stats.final_estimator_error, current.error);
    return httpd_resp_send_chunk(request, NULL, 0);
}

static void csv_number(char *output, size_t size, bool valid, float value)
{
    if (valid && isfinite(value)) snprintf(output, size, "%.6g", value);
    else output[0] = 0;
}

static esp_err_t csv_handler(httpd_req_t *request)
{
    char selected[20] = {0};
    if (!query_value(request, "session", selected, sizeof(selected)) ||
        !valid_journal_name(selected)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                   "valid mower session is required");
    }
    if (!speaker_storage_available(true) || !speaker_storage_lock(8000)) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        return httpd_resp_sendstr(request, "microSD unavailable");
    }
    char path[96];
    snprintf(path, sizeof(path), MOWER_DIR "/%s", selected);
    FILE *file = fopen(path, "rb");
    journal_header_t header;
    bool okay = file && read_header(file, &header);
    uint32_t count = okay ? record_count(file, &header) : 0;
    if (!okay) {
        if (file) fclose(file);
        speaker_storage_unlock();
        return httpd_resp_send_err(request, HTTPD_404_NOT_FOUND,
                                   "mower session unavailable");
    }
    char disposition[96];
    snprintf(disposition, sizeof(disposition),
             "attachment; filename=\"%s.csv\"", selected);
    httpd_resp_set_type(request, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(request, "Content-Disposition", disposition);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_send_chunk(request,
        "sequence,uptime_ms,epoch_ms,iso_utc,mower_active_ms,linked,voltage_valid,current_valid,motion_valid,active,faulted,axis_state,system_error,axis_error,motor_error,controller_error,estimator_error,vbus_voltage_v,bus_current_a,bus_power_w,energy_wh,command_turns_s,estimated_rpm,iq_measured_a,iq_setpoint_a,id_measured_a,id_setpoint_a,phase_current_magnitude_a,motor_voltage_v,motor_power_w,fet_temperature_c\r\n",
        HTTPD_RESP_USE_STRLEN);
    journal_record_t record;
    char line[640];
    for (uint32_t sequence = 0; sequence < count; ++sequence) {
        if (!read_record(file, &header, sequence, &record)) break;
        char iso[32] = "";
        if (record.epoch_ms) {
            time_t seconds = (time_t)(record.epoch_ms / 1000);
            struct tm utc;
            if (gmtime_r(&seconds, &utc)) {
                strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &utc);
            }
        }
        bool voltage_valid = (record.flags & FLAG_VOLTAGE_VALID) != 0;
        bool current_valid = (record.flags & FLAG_CURRENT_VALID) != 0;
        bool motion_valid = (record.flags & FLAG_MOTION_VALID) != 0;
        char voltage[24], bus_current[24], bus_power[24], command[24], rpm[24];
        char iqm[24], iqs[24], idm[24], ids[24], magnitude[24];
        char motor_voltage[24], motor_power[24], fet[24];
        csv_number(voltage, sizeof(voltage), voltage_valid, record.voltage_v);
        csv_number(bus_current, sizeof(bus_current), current_valid,
                   record.bus_current_a);
        csv_number(bus_power, sizeof(bus_power), current_valid,
                   record.bus_power_w);
        csv_number(command, sizeof(command), motion_valid, record.command_turns_s);
        csv_number(rpm, sizeof(rpm), motion_valid, record.estimated_rpm);
        csv_number(iqm, sizeof(iqm), motion_valid, record.iq_measured_a);
        csv_number(iqs, sizeof(iqs), motion_valid, record.iq_setpoint_a);
        csv_number(idm, sizeof(idm), motion_valid, record.id_measured_a);
        csv_number(ids, sizeof(ids), motion_valid, record.id_setpoint_a);
        csv_number(magnitude, sizeof(magnitude), motion_valid,
                   record.phase_current_magnitude_a);
        csv_number(motor_voltage, sizeof(motor_voltage), motion_valid,
                   record.motor_voltage_v);
        csv_number(motor_power, sizeof(motor_power), motion_valid,
                   record.motor_power_w);
        csv_number(fet, sizeof(fet), motion_valid, record.fet_temperature_c);
        snprintf(line, sizeof(line),
            "%lu,%llu,%lld,%s,%llu,%u,%u,%u,%u,%u,%u,%ld,%lu,%lu,%lu,%lu,%lu,%s,%s,%s,%.6g,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n",
            (unsigned long)record.sequence,
            (unsigned long long)record.uptime_ms, (long long)record.epoch_ms,
            iso, (unsigned long long)record.mower_active_ms,
            !!(record.flags & FLAG_LINK_VALID), voltage_valid, current_valid,
            motion_valid, !!(record.flags & FLAG_ACTIVE),
            !!(record.flags & FLAG_FAULTED), (long)record.axis_state,
            (unsigned long)record.system_error, (unsigned long)record.axis_error,
            (unsigned long)record.motor_error,
            (unsigned long)record.controller_error,
            (unsigned long)record.estimator_error, voltage, bus_current,
            bus_power, record.energy_wh, command, rpm, iqm, iqs, idm, ids,
            magnitude, motor_voltage, motor_power, fet);
        if (httpd_resp_send_chunk(request, line, HTTPD_RESP_USE_STRLEN) != ESP_OK) {
            okay = false;
            break;
        }
    }
    fclose(file);
    speaker_storage_unlock();
    if (!okay) return ESP_FAIL;
    return httpd_resp_send_chunk(request, NULL, 0);
}

static const char page_html[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'><title>Mower Logs</title><style>"
"*{box-sizing:border-box;letter-spacing:0}body{margin:0;background:#eef1f4;color:#17212a;font-family:Arial,sans-serif}header{background:#17212b;color:white;border-bottom:3px solid #e2a400;padding:10px 12px}.head,main{max-width:900px;margin:auto}h1{font-size:19px;margin:0 0 5px}nav a{color:white;margin-right:13px;font-size:13px}main{padding:12px}.panel{background:white;border:1px solid #cbd3da;border-radius:8px;padding:12px;margin-bottom:12px}.metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px}.metric{border-left:4px solid #16816a;background:#f3f6f7;padding:9px;min-width:0}.metric small{display:block;color:#596673}.metric strong{font-size:18px;overflow-wrap:anywhere}select,button{width:100%;min-height:44px;padding:8px;font-size:15px;border:1px solid #adb8c2;border-radius:6px;background:white}button{font-weight:700}.primary{background:#176daf;color:white;border-color:#176daf}.actions{display:grid;grid-template-columns:2fr 1fr;gap:8px;margin-top:8px}.note{font-size:12px;line-height:1.45;color:#596673}.bad{color:#b42318}@media(max-width:650px){.metrics{grid-template-columns:1fr 1fr}}@media(max-width:390px){.metrics,.actions{grid-template-columns:1fr}}</style></head><body>"
"<header><div class=head><h1>Mower Session Logs</h1><nav><a href='/'>Motors</a><a href='/mobile'>Drive</a><a href='/steppers'>Steppers</a><a href='/odrive'>ODrive</a><a href='/battery'>Battery</a></nav></div></header><main>"
"<section class=panel><select id=session onchange=load(this.value)></select><div class=actions><button class=primary onclick=downloadCsv()>Download selected CSV</button><button onclick=load(session.value)>Refresh</button></div><p id=info class=note>Loading mower journal...</p></section>"
"<section class='panel metrics'><div class=metric><small>Boot/session elapsed</small><strong id=elapsed>--</strong></div><div class=metric><small>Mower active time</small><strong id=active>--</strong></div><div class=metric><small>Energy consumed</small><strong id=energy>--</strong></div><div class=metric><small>Average bus power</small><strong id=avgPower>--</strong></div><div class=metric><small>Peak bus power</small><strong id=peakPower>--</strong></div><div class=metric><small>Average / peak bus current</small><strong id=busCurrent>--</strong></div><div class=metric><small>Average / peak |Iq|</small><strong id=iq>--</strong></div><div class=metric><small>Average / peak |Id|</small><strong id=id>--</strong></div><div class=metric><small>Average / peak phase current</small><strong id=phase>--</strong></div><div class=metric><small>Average / peak speed</small><strong id=rpm>--</strong></div><div class=metric><small>Average / peak motor power</small><strong id=motorPower>--</strong></div><div class=metric><small>Average / peak motor voltage</small><strong id=motorVoltage>--</strong></div><div class=metric><small>Average / peak FET temperature</small><strong id=fet>--</strong></div><div class=metric><small>VBUS average / range</small><strong id=voltage>--</strong></div><div class=metric><small>Samples / gaps / faults</small><strong id=quality>--</strong></div></section>"
"<section class=panel><p class=note>One CRC-protected 8 MiB journal is created per P4 boot and records once per second for about 18 hours. Every record is flushed and synced. A sudden power cut can invalidate only the record being written; CSV is generated on download from the valid prefix. The existing ODrive page continues to show only its rolling 60-second chart.</p><p class=note>CSV fields include VBUS, bus current and power, cumulative Wh, command and estimated speed, measured/setpoint Iq and Id, phase-current magnitude, motor voltage and electrical power, FET temperature, activity state, and every ODESC error word. Peaks are peaks of one-second UART samples; sub-millisecond hardware trips may only appear as error codes.</p><p class=note>Download completed sessions after mowing when possible. Downloading a long current session temporarily owns the shared microSD interface.</p><p id=err class='note bad'></p></section></main>"
"<script>const e=x=>document.getElementById(x);let data=null;function dur(ms){let s=Math.round(ms/1000),h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m '+s%60+'s'}function n(v,d,u){return Number(v||0).toFixed(d)+u}function pair(a,b,d,u){return n(a,d,' / ')+n(b,d,u)}function render(d){data=d;e('session').innerHTML=d.sessions.map(s=>`<option value='${s.name}' ${s.name===d.selected?'selected':''}>${s.boot_epoch_ms?new Date(s.boot_epoch_ms).toLocaleString():'boot '+s.id} | active ${dur(s.active_ms)}${s.current?' (current)':''}</option>`).join('');let s=d.stats;e('elapsed').textContent=dur(s.elapsed_ms);e('active').textContent=dur(s.active_ms);e('energy').textContent=n(s.energy_wh,3,' Wh');e('avgPower').textContent=n(s.average_bus_power_w,1,' W');e('peakPower').textContent=n(s.peak_bus_power_w,1,' W');e('busCurrent').textContent=pair(s.average_bus_current_a,s.peak_bus_current_a,2,' A');e('iq').textContent=pair(s.average_iq_a,s.peak_iq_a,2,' A');e('id').textContent=pair(s.average_id_a,s.peak_id_a,2,' A');e('phase').textContent=pair(s.average_phase_current_a,s.peak_phase_current_a,2,' A');e('rpm').textContent=pair(s.average_rpm,s.peak_rpm,0,' RPM');e('motorPower').textContent=pair(s.average_motor_power_w,s.peak_motor_power_w,1,' W');e('motorVoltage').textContent=pair(s.average_motor_voltage_v,s.peak_motor_voltage_v,2,' V');e('fet').textContent=pair(s.average_fet_temperature_c,s.peak_fet_temperature_c,1,' C');e('voltage').textContent=n(s.average_voltage_v,2,' V | ')+n(s.voltage_min_v,2,'-')+n(s.voltage_max_v,2,' V');e('quality').textContent=s.samples+' / '+s.gap_samples+' / '+s.fault_samples;e('info').textContent=s.active_samples+' active samples | '+s.motion_samples+' motion samples | final errors '+s.final_system_error+'/'+s.final_axis_error+'/'+s.final_motor_error+'/'+s.final_controller_error+'/'+s.final_estimator_error;e('err').textContent=d.error||''}function load(s=''){fetch('/api/mower-logs'+(s?'?session='+encodeURIComponent(s):''),{cache:'no-store'}).then(r=>{if(!r.ok)throw Error(r.statusText);return r.json()}).then(render).catch(x=>e('err').textContent=x)}function downloadCsv(){location.href='/api/mower-logs/csv?session='+encodeURIComponent(e('session').value)}load()</script></body></html>";

static esp_err_t page_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, page_html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t mower_history_start(void)
{
    state_mutex = xSemaphoreCreateMutex();
    if (!state_mutex) return ESP_ERR_NO_MEM;
    return xTaskCreate(logger_task, "mower_log", 7168, NULL, 2, NULL) == pdPASS ?
           ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t mower_history_register_routes(httpd_handle_t server)
{
    const httpd_uri_t routes[] = {
        {.uri = "/mower-logs", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/mower-logs", .method = HTTP_GET, .handler = api_handler},
        {.uri = "/api/mower-logs/csv", .method = HTTP_GET, .handler = csv_handler},
    };
    for (unsigned i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        esp_err_t result = httpd_register_uri_handler(server, &routes[i]);
        if (result != ESP_OK) return result;
    }
    return ESP_OK;
}
