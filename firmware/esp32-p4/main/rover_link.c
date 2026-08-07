#include "rover_link.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "network_ota.h"
#include "psa/crypto.h"
#include "rover_control_key.h"
#include "speaker.h"

#define ROVER_PORT 4210
#define PACKET_SIZE 48
#define AUTH_SIZE 16
#define CONTROLLER_ID 1
#define LINK_TIMEOUT_MS 400
#define HANDHELD_MIN_RAMP_PERCENT 200
#define FLAG_STOP 0x01
#define FLAG_EXIT 0x02
#define CREDENTIAL_RESPONSE_SIZE 136
#define CREDENTIAL_HEADER_SIZE 24
#define CREDENTIAL_PLAINTEXT_SIZE 96

static const char *TAG = "rover_link";
static const uint8_t control_key[32] = {ROVER_CONTROL_KEY_BYTES};
static psa_key_id_t hmac_key_id;
static psa_key_id_t aes_key_id;
static SemaphoreHandle_t link_mutex;
static rover_link_status_t link_status = {.age_ms = -1};
static rover_link_drive_fn drive_fn;
static int64_t last_packet_ms;
static bool sequence_valid;

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static int16_t read_i16(const uint8_t *data)
{
    return (int16_t)read_u16(data);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = value;
    data[1] = value >> 8;
    data[2] = value >> 16;
    data[3] = value >> 24;
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = value;
    data[1] = value >> 8;
}

static bool packet_hmac(const uint8_t *packet, uint8_t output[32])
{
    const psa_algorithm_t algorithm = PSA_ALG_HMAC(PSA_ALG_SHA_256);
    if (hmac_key_id == 0) {
        if (psa_crypto_init() != PSA_SUCCESS) return false;
        psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
        psa_set_key_usage_flags(&attributes,
            PSA_KEY_USAGE_SIGN_MESSAGE | PSA_KEY_USAGE_VERIFY_MESSAGE);
        psa_set_key_algorithm(&attributes, algorithm);
        psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
        psa_set_key_bits(&attributes, sizeof(control_key) * 8);
        psa_status_t imported = psa_import_key(&attributes, control_key,
            sizeof(control_key), &hmac_key_id);
        psa_reset_key_attributes(&attributes);
        if (imported != PSA_SUCCESS) return false;
    }
    size_t output_size = 0;
    return psa_mac_compute(hmac_key_id, algorithm, packet,
        PACKET_SIZE - AUTH_SIZE, output, 32, &output_size) == PSA_SUCCESS &&
        output_size == 32;
}

static bool constant_time_equal(const uint8_t *a, const uint8_t *b, size_t size)
{
    uint8_t difference = 0;
    for (size_t i = 0; i < size; ++i) difference |= a[i] ^ b[i];
    return difference == 0;
}

static bool credential_request_valid(const uint8_t packet[PACKET_SIZE],
                                     uint32_t *sequence)
{
    uint8_t hmac[32];
    if (memcmp(packet, "RVCQ", 4) != 0 || packet[4] != 1 ||
        packet[5] != CONTROLLER_ID || !packet_hmac(packet, hmac) ||
        !constant_time_equal(hmac, packet + PACKET_SIZE - AUTH_SIZE, AUTH_SIZE)) {
        return false;
    }
    *sequence = read_u32(packet + 8);
    return true;
}

static bool credential_aes_key(void)
{
    if (aes_key_id != 0) return true;
    if (psa_crypto_init() != PSA_SUCCESS) return false;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_GCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, sizeof(control_key) * 8);
    psa_status_t imported = psa_import_key(&attributes, control_key,
        sizeof(control_key), &aes_key_id);
    psa_reset_key_attributes(&attributes);
    return imported == PSA_SUCCESS;
}

static bool send_credentials(int socket_fd, const struct sockaddr_in *peer,
                             socklen_t peer_len, uint32_t sequence)
{
    char ssid[33] = {0};
    char password[65] = {0};
    if (!network_ota_get_saved_credentials(ssid, sizeof(ssid),
                                            password, sizeof(password)) ||
        !credential_aes_key()) {
        memset(password, 0, sizeof(password));
        return false;
    }
    const size_t ssid_length = strnlen(ssid, 32);
    const size_t password_length = strnlen(password, 64);
    uint8_t plaintext[CREDENTIAL_PLAINTEXT_SIZE] = {0};
    uint8_t response[CREDENTIAL_RESPONSE_SIZE] = {0};
    memcpy(plaintext, ssid, ssid_length);
    memcpy(plaintext + 32, password, password_length);
    memcpy(response, "RVC1", 4);
    response[4] = 1;
    response[5] = CONTROLLER_ID;
    response[6] = (uint8_t)ssid_length;
    response[7] = (uint8_t)password_length;
    write_u32(response + 8, sequence);
    esp_fill_random(response + 12, 12);
    size_t ciphertext_length = 0;
    psa_status_t status = psa_aead_encrypt(aes_key_id, PSA_ALG_GCM,
        response + 12, 12, response, CREDENTIAL_HEADER_SIZE,
        plaintext, sizeof(plaintext), response + CREDENTIAL_HEADER_SIZE,
        CREDENTIAL_RESPONSE_SIZE - CREDENTIAL_HEADER_SIZE, &ciphertext_length);
    memset(password, 0, sizeof(password));
    memset(plaintext, 0, sizeof(plaintext));
    if (status != PSA_SUCCESS ||
        ciphertext_length != CREDENTIAL_RESPONSE_SIZE - CREDENTIAL_HEADER_SIZE) {
        return false;
    }
    bool sent = sendto(socket_fd, response, sizeof(response), 0,
        (const struct sockaddr *)peer, peer_len) == sizeof(response);
    memset(response, 0, sizeof(response));
    if (sent) ESP_LOGI(TAG, "Encrypted router credentials sent to authenticated controller");
    return sent;
}

static int axis_value(uint16_t raw)
{
    int value = (int)raw - 256;
    if (value > -22 && value < 22) return 0;
    value += value > 0 ? -22 : 22;
    value = value * 4095 / 234;
    if (value > 4095) return 4095;
    if (value < -4095) return -4095;
    return value;
}

static void mix_tank(uint16_t x_raw, uint16_t y_raw, int *left, int *right)
{
    int x = axis_value(x_raw);
    int y = -axis_value(y_raw);
    int l = y + x;
    int r = y - x;
    int maximum = abs(l) > abs(r) ? abs(l) : abs(r);
    if (maximum > 4095) {
        l = l * 4095 / maximum;
        r = r * 4095 / maximum;
    }
    *left = l;
    *right = r;
}

typedef enum {
    PACKET_VALID = 0,
    PACKET_BAD_AUTH,
    PACKET_BAD_RANGE,
} packet_validation_t;

static packet_validation_t validate_packet(const uint8_t packet[PACKET_SIZE],
    uint32_t *sequence)
{
    uint8_t hmac[32];
    if (memcmp(packet, "RVR2", 4) != 0 || packet[4] != 2 ||
        packet[5] != CONTROLLER_ID || !packet_hmac(packet, hmac) ||
        !constant_time_equal(hmac, packet + PACKET_SIZE - AUTH_SIZE, AUTH_SIZE)) {
        return PACKET_BAD_AUTH;
    }
    for (int offset = 12; offset <= 18; offset += 2) {
        if (read_u16(packet + offset) > 512) return PACKET_BAD_RANGE;
    }
    *sequence = read_u32(packet + 8);
    return PACKET_VALID;
}

static bool send_ack(int socket_fd, const struct sockaddr_in *peer,
                     socklen_t peer_len, uint32_t sequence)
{
    uint8_t packet[PACKET_SIZE] = {0};
    uint8_t hmac[32];
    memcpy(packet, "RVA2", 4);
    packet[4] = 2;
    packet[5] = CONTROLLER_ID;
    write_u32(packet + 8, sequence);
    speaker_status_t audio = {0};
    speaker_get_status(&audio);
    packet[12] = audio.playing ? audio.source_button : 0;
    packet[13] = audio.playing ? audio.kind : 0;
    write_u16(packet + 14, audio.playing ? audio.tone_hz : 0);
    if (packet_hmac(packet, hmac)) {
        memcpy(packet + PACKET_SIZE - AUTH_SIZE, hmac, AUTH_SIZE);
        return sendto(socket_fd, packet, sizeof(packet), 0,
                      (const struct sockaddr *)peer, peer_len) == sizeof(packet);
    }
    return false;
}

static void rover_link_task(void *argument)
{
    (void)argument;
    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (socket_fd < 0) {
        ESP_LOGE(TAG, "socket failed: errno %d", errno);
        vTaskDelete(NULL);
    }
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 100000};
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(ROVER_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(socket_fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
        ESP_LOGE(TAG, "bind failed: errno %d", errno);
        close(socket_fd);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Authenticated controller UDP ready on port %d", ROVER_PORT);

    while (true) {
        uint8_t packet[CREDENTIAL_RESPONSE_SIZE];
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int received = recvfrom(socket_fd, packet, sizeof(packet), 0,
                                (struct sockaddr *)&peer, &peer_len);
        int64_t now = esp_timer_get_time() / 1000;
        if (received > 0) {
            xSemaphoreTake(link_mutex, portMAX_DELAY);
            link_status.datagrams++;
            if (received != PACKET_SIZE) link_status.wrong_size_packets++;
            xSemaphoreGive(link_mutex);
        }
        if (received == PACKET_SIZE && memcmp(packet, "RVCQ", 4) == 0) {
            uint32_t request_sequence = 0;
            if (!credential_request_valid(packet, &request_sequence) ||
                !send_credentials(socket_fd, &peer, peer_len, request_sequence)) {
                xSemaphoreTake(link_mutex, portMAX_DELAY);
                link_status.rejected_packets++;
                link_status.auth_rejects++;
                xSemaphoreGive(link_mutex);
            }
            continue;
        }
        if (received == PACKET_SIZE) {
            uint32_t sequence = 0;
            packet_validation_t validation = validate_packet(packet, &sequence);
            bool valid = validation == PACKET_VALID;
            xSemaphoreTake(link_mutex, portMAX_DELAY);
            bool fresh = !sequence_valid ||
                         (int32_t)(sequence - link_status.sequence) > 0 ||
                         now - last_packet_ms > LINK_TIMEOUT_MS;
            xSemaphoreGive(link_mutex);
            if (valid && fresh) {
                uint8_t flags = packet[6];
                uint16_t ramp = read_u16(packet + 22);
                if (ramp < 10 || ramp > 300) ramp = 80;
                if (ramp < HANDHELD_MIN_RAMP_PERCENT) {
                    ramp = HANDHELD_MIN_RAMP_PERCENT;
                }
                int left = 0;
                int right = 0;
                int16_t stepper_rpm[3] = {
                    read_i16(packet + 24), read_i16(packet + 26),
                    read_i16(packet + 28),
                };
                for (int i = 0; i < 3; ++i) {
                    if (stepper_rpm[i] < -300 || stepper_rpm[i] > 300) valid = false;
                }
                if (!valid) {
                    xSemaphoreTake(link_mutex, portMAX_DELAY);
                    link_status.rejected_packets++;
                    link_status.range_rejects++;
                    xSemaphoreGive(link_mutex);
                    continue;
                }
                mix_tank(read_u16(packet + 12), read_u16(packet + 14), &left, &right);
                if (flags & (FLAG_STOP | FLAG_EXIT)) {
                    left = right = 0;
                    memset(stepper_rpm, 0, sizeof(stepper_rpm));
                }

                xSemaphoreTake(link_mutex, portMAX_DELAY);
                link_status.active = (flags & FLAG_EXIT) == 0;
                link_status.age_ms = 0;
                link_status.sequence = sequence;
                link_status.right_x = read_u16(packet + 16);
                link_status.right_y = read_u16(packet + 18);
                link_status.button_mask = read_u16(packet + 20);
                memcpy(link_status.stepper_rpm, stepper_rpm, sizeof(stepper_rpm));
                link_status.valid_packets++;
                last_packet_ms = now;
                sequence_valid = true;
                xSemaphoreGive(link_mutex);

                speaker_controller_buttons(read_u16(packet + 20));

                drive_fn(left, right, ramp, (flags & (FLAG_STOP | FLAG_EXIT)) != 0,
                         stepper_rpm[0], stepper_rpm[1], stepper_rpm[2]);
                network_ota_set_controller_active((flags & FLAG_EXIT) == 0);
                const bool ack_sent = send_ack(socket_fd, &peer, peer_len, sequence);
                xSemaphoreTake(link_mutex, portMAX_DELAY);
                if (ack_sent) link_status.acks_sent++;
                else link_status.ack_failures++;
                xSemaphoreGive(link_mutex);
            } else if (!valid) {
                xSemaphoreTake(link_mutex, portMAX_DELAY);
                link_status.rejected_packets++;
                if (validation == PACKET_BAD_AUTH) link_status.auth_rejects++;
                else link_status.range_rejects++;
                xSemaphoreGive(link_mutex);
            } else {
                xSemaphoreTake(link_mutex, portMAX_DELAY);
                link_status.stale_packets++;
                xSemaphoreGive(link_mutex);
            }
        }

        xSemaphoreTake(link_mutex, portMAX_DELAY);
        bool timed_out = link_status.active && now - last_packet_ms > LINK_TIMEOUT_MS;
        if (last_packet_ms) link_status.age_ms = now - last_packet_ms;
        if (timed_out) link_status.active = false;
        xSemaphoreGive(link_mutex);
        if (timed_out) {
            ESP_LOGW(TAG, "Handheld controller timed out; stopping drive");
            drive_fn(0, 0, 80, true, 0, 0, 0);
            network_ota_set_controller_active(false);
            speaker_controller_buttons(0);
            sequence_valid = false;
        }
    }
}

esp_err_t rover_link_start(rover_link_drive_fn callback)
{
    if (!callback) return ESP_ERR_INVALID_ARG;
    drive_fn = callback;
    link_mutex = xSemaphoreCreateMutex();
    if (!link_mutex) return ESP_ERR_NO_MEM;
    return xTaskCreate(rover_link_task, "rover_udp", 6144, NULL, 9, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

void rover_link_get_status(rover_link_status_t *status)
{
    if (!status || !link_mutex) return;
    xSemaphoreTake(link_mutex, portMAX_DELAY);
    *status = link_status;
    xSemaphoreGive(link_mutex);
}
