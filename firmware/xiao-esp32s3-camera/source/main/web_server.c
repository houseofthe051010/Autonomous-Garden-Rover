#include "web_server.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_service.h"
#include "board_config.h"
#include "camera_service.h"
#include "network_service.h"
#include "esp_app_desc.h"
#include "esp_camera.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "img_converters.h"

#define STREAM_BOUNDARY "123456789000000000000987654321"
#define STREAM_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY

extern const unsigned char viewer_html_start[] asm("_binary_viewer_html_start");
extern const unsigned char configs_html_start[] asm("_binary_configs_html_start");
extern const unsigned char wifi_html_start[] asm("_binary_wifi_html_start");
extern const unsigned char update_html_start[] asm("_binary_update_html_start");

static const char *TAG = "web";
static httpd_handle_t s_main_server;
static httpd_handle_t s_video_server;
static httpd_handle_t s_audio_server;
static portMUX_TYPE s_audio_client_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_ota_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_audio_client_active;
static bool s_ota_active;

static void set_ota_active(bool active)
{
    portENTER_CRITICAL(&s_ota_lock);
    s_ota_active = active;
    portEXIT_CRITICAL(&s_ota_lock);
}

static bool ota_is_active(void)
{
    portENTER_CRITICAL(&s_ota_lock);
    bool active = s_ota_active;
    portEXIT_CRITICAL(&s_ota_lock);
    return active;
}

static esp_err_t send_html(httpd_req_t *req, const unsigned char *html)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_json(httpd_req_t *req, const char *json, ssize_t length)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, length);
}

static esp_err_t send_error(httpd_req_t *req, const char *status, const char *message)
{
    char body[192];
    snprintf(body, sizeof(body), "{\"error\":\"%s\"}", message);
    httpd_resp_set_status(req, status);
    return send_json(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/stream");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t viewer_handler(httpd_req_t *req) { return send_html(req, viewer_html_start); }
static esp_err_t configs_handler(httpd_req_t *req) { return send_html(req, configs_html_start); }
static esp_err_t wifi_handler(httpd_req_t *req) { return send_html(req, wifi_html_start); }
static esp_err_t update_handler(httpd_req_t *req) { return send_html(req, update_html_start); }

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t result = httpd_resp_set_type(req, STREAM_TYPE);
    if (result != ESP_OK) return result;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    camera_service_stream_started();

    while (!ota_is_active()) {
        int64_t started_us = esp_timer_get_time();
        camera_fb_t *frame = esp_camera_fb_get();
        if (!frame) {
            result = ESP_FAIL;
            break;
        }
        uint8_t *jpeg = frame->buf;
        size_t jpeg_len = frame->len;
        bool converted = false;
        if (frame->format != PIXFORMAT_JPEG) {
            converted = frame2jpg(frame, 80, &jpeg, &jpeg_len);
            if (!converted) {
                esp_camera_fb_return(frame);
                result = ESP_FAIL;
                break;
            }
        }

        char header[128];
        int header_len = snprintf(header, sizeof(header),
                                  "\r\n--" STREAM_BOUNDARY "\r\n"
                                  "Content-Type: image/jpeg\r\n"
                                  "Content-Length: %u\r\n\r\n",
                                  (unsigned)jpeg_len);
        if (header_len <= 0 ||
            httpd_resp_send_chunk(req, header, header_len) != ESP_OK ||
            httpd_resp_send_chunk(req, (const char *)jpeg, jpeg_len) != ESP_OK) {
            result = ESP_FAIL;
        }
        if (converted) free(jpeg);
        esp_camera_fb_return(frame);
        if (result != ESP_OK) break;

        int fps = camera_service_get_fps();
        if (fps > 0) {
            int64_t remaining_us = (1000000LL / fps) - (esp_timer_get_time() - started_us);
            if (remaining_us >= 1000) vTaskDelay(pdMS_TO_TICKS(remaining_us / 1000));
        }
    }
    camera_service_stream_stopped();
    return result;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) return httpd_resp_send_500(req);
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t result = httpd_resp_send(req, (const char *)frame->buf, frame->len);
    esp_camera_fb_return(frame);
    return result;
}

static bool query_pair(httpd_req_t *req, char *variable, size_t variable_size,
                       char *value, size_t value_size)
{
    size_t length = httpd_req_get_url_query_len(req) + 1;
    if (length <= 1 || length > 256) return false;
    char query[256];
    return httpd_req_get_url_query_str(req, query, length) == ESP_OK &&
           httpd_query_key_value(query, "var", variable, variable_size) == ESP_OK &&
           httpd_query_key_value(query, "val", value, value_size) == ESP_OK;
}

static esp_err_t camera_control_handler(httpd_req_t *req)
{
    char variable[32], value[32];
    if (!query_pair(req, variable, sizeof(variable), value, sizeof(value))) {
        return send_error(req, "400 Bad Request", "missing camera control");
    }
    int val = atoi(value);
    sensor_t *sensor = camera_service_sensor();
    int result = -1;
    if (!strcmp(variable, "framesize") && sensor->pixformat == PIXFORMAT_JPEG) {
        framesize_t size;
        if (camera_service_parse_framesize(value, &size)) result = sensor->set_framesize(sensor, size);
    } else if (!strcmp(variable, "quality")) result = sensor->set_quality(sensor, val);
    else if (!strcmp(variable, "contrast")) result = sensor->set_contrast(sensor, val);
    else if (!strcmp(variable, "brightness")) result = sensor->set_brightness(sensor, val);
    else if (!strcmp(variable, "saturation")) result = sensor->set_saturation(sensor, val);
    else if (!strcmp(variable, "sharpness")) result = sensor->set_sharpness(sensor, val);
    else if (!strcmp(variable, "gainceiling")) result = sensor->set_gainceiling(sensor, (gainceiling_t)val);
    else if (!strcmp(variable, "colorbar")) result = sensor->set_colorbar(sensor, val);
    else if (!strcmp(variable, "awb")) result = sensor->set_whitebal(sensor, val);
    else if (!strcmp(variable, "agc")) result = sensor->set_gain_ctrl(sensor, val);
    else if (!strcmp(variable, "aec")) result = sensor->set_exposure_ctrl(sensor, val);
    else if (!strcmp(variable, "hmirror")) result = sensor->set_hmirror(sensor, val);
    else if (!strcmp(variable, "vflip")) result = sensor->set_vflip(sensor, val);
    else if (!strcmp(variable, "awb_gain")) result = sensor->set_awb_gain(sensor, val);
    else if (!strcmp(variable, "agc_gain")) result = sensor->set_agc_gain(sensor, val);
    else if (!strcmp(variable, "aec_value")) result = sensor->set_aec_value(sensor, val);
    else if (!strcmp(variable, "aec2")) result = sensor->set_aec2(sensor, val);
    else if (!strcmp(variable, "dcw")) result = sensor->set_dcw(sensor, val);
    else if (!strcmp(variable, "bpc")) result = sensor->set_bpc(sensor, val);
    else if (!strcmp(variable, "wpc")) result = sensor->set_wpc(sensor, val);
    else if (!strcmp(variable, "raw_gma")) result = sensor->set_raw_gma(sensor, val);
    else if (!strcmp(variable, "lenc")) result = sensor->set_lenc(sensor, val);
    else if (!strcmp(variable, "special_effect")) result = sensor->set_special_effect(sensor, val);
    else if (!strcmp(variable, "wb_mode")) result = sensor->set_wb_mode(sensor, val);
    else if (!strcmp(variable, "ae_level")) result = sensor->set_ae_level(sensor, val);
    else if (!strcmp(variable, "stream_fps")) {
        camera_service_set_fps(val);
        result = 0;
    } else if (!strcmp(variable, "led_intensity")) {
        camera_service_set_led(val);
        result = 0;
    }
    if (result < 0) return send_error(req, "400 Bad Request", "camera control rejected");
    return httpd_resp_send(req, NULL, 0);
}

static void json_escape(char *output, size_t capacity, const char *input)
{
    size_t used = 0;
    if (!capacity) return;
    for (; input && *input && used + 2 < capacity; ++input) {
        unsigned char c = (unsigned char)*input;
        if (c == '\\' || c == '"') {
            if (used + 3 >= capacity) break;
            output[used++] = '\\';
            output[used++] = c;
        } else if (c >= 0x20) {
            output[used++] = c;
        }
    }
    output[used] = '\0';
}

static int camera_json(char *json, size_t capacity)
{
    sensor_t *s = camera_service_sensor();
    return snprintf(json, capacity,
        "{\"framesize\":\"%s\",\"quality\":%u,\"brightness\":%d,\"contrast\":%d,"
        "\"saturation\":%d,\"sharpness\":%d,\"special_effect\":%u,\"wb_mode\":%u,"
        "\"awb\":%u,\"awb_gain\":%u,\"aec\":%u,\"aec2\":%u,\"ae_level\":%d,"
        "\"aec_value\":%u,\"agc\":%u,\"agc_gain\":%u,\"gainceiling\":%u,"
        "\"bpc\":%u,\"wpc\":%u,\"raw_gma\":%u,\"lenc\":%u,\"hmirror\":%u,"
        "\"vflip\":%u,\"dcw\":%u,\"colorbar\":%u,\"stream_fps\":%d,\"led_intensity\":%d}",
        camera_service_framesize_name(s->status.framesize), s->status.quality,
        s->status.brightness, s->status.contrast, s->status.saturation, s->status.sharpness,
        s->status.special_effect, s->status.wb_mode, s->status.awb, s->status.awb_gain,
        s->status.aec, s->status.aec2, s->status.ae_level, s->status.aec_value,
        s->status.agc, s->status.agc_gain, s->status.gainceiling, s->status.bpc,
        s->status.wpc, s->status.raw_gma, s->status.lenc, s->status.hmirror,
        s->status.vflip, s->status.dcw, s->status.colorbar, camera_service_get_fps(),
        camera_service_get_led());
}

static esp_err_t camera_status_handler(httpd_req_t *req)
{
    char json[1024];
    int length = camera_json(json, sizeof(json));
    if (length < 0 || (size_t)length >= sizeof(json)) return httpd_resp_send_500(req);
    return send_json(req, json, length);
}

#include "system_metrics.h"
static esp_err_t status_handler(httpd_req_t *req)
{
    char camera[1024], ssid[96], json[2048];
    char metrics[256];
    system_metrics_json(metrics,sizeof(metrics));
    int camera_length = camera_json(camera, sizeof(camera));
    if (camera_length < 0 || (size_t)camera_length >= sizeof(camera)) {
        return httpd_resp_send_500(req);
    }
    audio_status_t audio;
    network_status_t network;
    audio_service_get_status(&audio);
    network_service_get_status(&network);
    json_escape(ssid, sizeof(ssid), network.sta_ssid);
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *ota = esp_ota_get_next_update_partition(NULL);
    int length = snprintf(json, sizeof(json),
        "{\"system\":%s,\"camera\":%s,\"audio\":{\"ready\":%s,\"listening\":%s,"
        "\"gain_db\":%.1f,\"rms_dbfs\":%.1f,\"peak_dbfs\":%.1f,"
        "\"clipped_samples\":%u,\"dropped_blocks\":%u,\"sample_rate\":%d},"
        "\"network\":{\"sta_configured\":%s,\"sta_connected\":%s,"
        "\"sta_ssid\":\"%s\",\"sta_ip\":\"%s\",\"sta_rssi\":%d,"
        "\"ap_ip\":\"%s\",\"ap_clients\":%u,\"ap_client_rssi\":%d,\"channel\":%u},"
        "\"firmware\":{\"version\":\"%s\",\"date\":\"%s\",\"time\":\"%s\","
        "\"ota_capacity\":%u},\"memory\":{\"heap_free\":%u,\"psram_free\":%u}}",
        metrics, camera, audio.ready ? "true" : "false", audio.listener_active ? "true" : "false",
        audio.gain_db, audio.rms_dbfs, audio.peak_dbfs, (unsigned)audio.clipped_samples,
        (unsigned)audio.dropped_blocks, MIC_SAMPLE_RATE, network.sta_configured ? "true" : "false",
        network.sta_connected ? "true" : "false", ssid, network.sta_ip,
        network.sta_rssi, network.ap_ip, network.ap_clients, network.ap_client_rssi,
        network.channel, app->version, app->date, app->time, ota ? (unsigned)ota->size : 0,
        (unsigned)esp_get_free_heap_size(),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    if (length < 0 || (size_t)length >= sizeof(json)) return httpd_resp_send_500(req);
    return send_json(req, json, length);
}

static esp_err_t audio_config_handler(httpd_req_t *req)
{
    size_t length = httpd_req_get_url_query_len(req) + 1;
    char query[96], gain[24];
    if (length <= 1 || length > sizeof(query) ||
        httpd_req_get_url_query_str(req, query, length) != ESP_OK ||
        httpd_query_key_value(query, "gain_db", gain, sizeof(gain)) != ESP_OK) {
        return send_error(req, "400 Bad Request", "missing gain_db");
    }
    char *end = NULL;
    float value = strtof(gain, &end);
    if (!end || *end != '\0') return send_error(req, "400 Bad Request", "invalid gain_db");
    esp_err_t err = audio_service_set_gain(value);
    if (err != ESP_OK) return send_error(req, "500 Internal Server Error", esp_err_to_name(err));
    return send_json(req, "{\"saved\":true}", HTTPD_RESP_USE_STRLEN);
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)tolower((unsigned char)c);
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static void url_decode(char *value)
{
    char *read = value;
    char *write = value;
    while (*read) {
        if (*read == '+') {
            *write++ = ' ';
            ++read;
        } else if (*read == '%' && read[1] && read[2]) {
            int high = hex_value(read[1]);
            int low = hex_value(read[2]);
            if (high >= 0 && low >= 0) {
                *write++ = (char)((high << 4) | low);
                read += 3;
            } else {
                *write++ = *read++;
            }
        } else {
            *write++ = *read++;
        }
    }
    *write = '\0';
}

static esp_err_t read_body(httpd_req_t *req, char *body, size_t capacity)
{
    if (req->content_len <= 0 || (size_t)req->content_len >= capacity) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t received = 0;
    while (received < (size_t)req->content_len) {
        int count = httpd_req_recv(req, body + received, req->content_len - received);
        if (count == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (count <= 0) return ESP_FAIL;
        received += count;
    }
    body[received] = '\0';
    return ESP_OK;
}

static esp_err_t wifi_scan_handler(httpd_req_t *req)
{
    char *json = malloc(4096);
    if (!json) return httpd_resp_send_500(req);
    size_t length = 0;
    esp_err_t err = network_service_scan_json(json, 4096, &length);
    esp_err_t result = err == ESP_OK ? send_json(req, json, length) :
        send_error(req, "503 Service Unavailable", esp_err_to_name(err));
    free(json);
    return result;
}

static esp_err_t wifi_connect_handler(httpd_req_t *req)
{
    char body[384], ssid[100], password[192];
    if (read_body(req, body, sizeof(body)) != ESP_OK ||
        httpd_query_key_value(body, "ssid", ssid, sizeof(ssid)) != ESP_OK ||
        httpd_query_key_value(body, "password", password, sizeof(password)) != ESP_OK) {
        return send_error(req, "400 Bad Request", "invalid Wi-Fi form");
    }
    url_decode(ssid);
    url_decode(password);
    esp_err_t err = network_service_connect(ssid, password);
    if (err != ESP_OK) return send_error(req, "400 Bad Request", esp_err_to_name(err));
    return send_json(req, "{\"connecting\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t wifi_forget_handler(httpd_req_t *req)
{
    esp_err_t err = network_service_forget();
    if (err != ESP_OK) return send_error(req, "500 Internal Server Error", esp_err_to_name(err));
    return send_json(req, "{\"forgotten\":true}", HTTPD_RESP_USE_STRLEN);
}

static void restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
}

static esp_err_t ota_handler(httpd_req_t *req)
{
    char confirmation[16];
    if (httpd_req_get_hdr_value_str(req, "X-OTA-Confirm", confirmation,
                                    sizeof(confirmation)) != ESP_OK ||
        strcmp(confirmation, "XIAO-S3")) {
        return send_error(req, "403 Forbidden", "OTA confirmation missing");
    }
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target || req->content_len <= 0 || (size_t)req->content_len > target->size) {
        return send_error(req, "413 Payload Too Large", "image does not fit OTA slot");
    }

    portENTER_CRITICAL(&s_ota_lock);
    bool busy = s_ota_active;
    if (!busy) s_ota_active = true;
    portEXIT_CRITICAL(&s_ota_lock);
    if (busy) return send_error(req, "409 Conflict", "OTA already active");

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, req->content_len, &handle);
    if (err != ESP_OK) {
        set_ota_active(false);
        return send_error(req, "500 Internal Server Error", esp_err_to_name(err));
    }
    uint8_t *buffer = malloc(4096);
    if (!buffer) {
        esp_ota_abort(handle);
        set_ota_active(false);
        return httpd_resp_send_500(req);
    }
    int remaining = req->content_len;
    while (remaining > 0) {
        int requested = remaining > 4096 ? 4096 : remaining;
        int count = httpd_req_recv(req, (char *)buffer, requested);
        if (count == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (count <= 0) {
            err = ESP_FAIL;
            break;
        }
        err = esp_ota_write(handle, buffer, count);
        if (err != ESP_OK) break;
        remaining -= count;
    }
    free(buffer);
    if (err != ESP_OK || remaining != 0) {
        esp_ota_abort(handle);
        set_ota_active(false);
        return send_error(req, "500 Internal Server Error", "OTA transfer failed");
    }
    err = esp_ota_end(handle);
    if (err == ESP_OK) err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        set_ota_active(false);
        return send_error(req, "400 Bad Request", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "OTA image installed in %s; restarting", target->label);
    esp_err_t result = send_json(req, "{\"installed\":true,\"restarting\":true}",
                                 HTTPD_RESP_USE_STRLEN);
    xTaskCreate(restart_task, "ota_restart", 2048, NULL, 3, NULL);
    return result;
}

static esp_err_t audio_stream_handler(httpd_req_t *req)
{
    portENTER_CRITICAL(&s_audio_client_lock);
    bool busy = s_audio_client_active;
    if (!busy) s_audio_client_active = true;
    portEXIT_CRITICAL(&s_audio_client_lock);
    if (busy) return send_error(req, "409 Conflict", "audio listener already active");

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache");
    httpd_resp_set_hdr(req, "X-Audio-Format", "s16le;rate=16000;channels=1");
    audio_service_set_listener(true);
    esp_err_t result = ESP_OK;
    while (!ota_is_active()) {
        audio_block_t block;
        if (!audio_service_take(&block, 1500)) continue;
        result = httpd_resp_send_chunk(req, (const char *)block.samples,
                                       block.count * sizeof(block.samples[0]));
        if (result != ESP_OK) break;
    }
    audio_service_set_listener(false);
    portENTER_CRITICAL(&s_audio_client_lock);
    s_audio_client_active = false;
    portEXIT_CRITICAL(&s_audio_client_lock);
    return result;
}

static httpd_uri_t route(const char *uri, httpd_method_t method,
                         esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t result = {.uri = uri, .method = method, .handler = handler};
    return result;
}

static esp_err_t register_main_routes(void)
{
    httpd_uri_t routes[] = {
        route("/", HTTP_GET, root_handler),
        route("/stream", HTTP_GET, viewer_handler),
        route("/configs", HTTP_GET, configs_handler),
        route("/wifi", HTTP_GET, wifi_handler),
        route("/update", HTTP_GET, update_handler),
        route("/capture", HTTP_GET, capture_handler),
        route("/status", HTTP_GET, camera_status_handler),
        route("/control", HTTP_GET, camera_control_handler),
        route("/api/status", HTTP_GET, status_handler),
        route("/api/audio", HTTP_GET, audio_config_handler),
        route("/api/wifi/scan", HTTP_GET, wifi_scan_handler),
        route("/api/wifi/connect", HTTP_POST, wifi_connect_handler),
        route("/api/wifi/forget", HTTP_POST, wifi_forget_handler),
        route("/api/ota", HTTP_POST, ota_handler),
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_main_server, &routes[i]), TAG,
                            "route registration failed: %s", routes[i].uri);
    }
    return ESP_OK;
}

esp_err_t web_server_start(void)
{
    httpd_config_t main_config = HTTPD_DEFAULT_CONFIG();
    main_config.max_uri_handlers = 18;
    // Three servers consume six listener/control sockets. Five control clients
    // plus two video and two audio sockets stay below the 16-socket LWIP cap.
    main_config.max_open_sockets = 5;
    main_config.lru_purge_enable = true;
    main_config.stack_size = 8192;
    main_config.recv_wait_timeout = 10;
    main_config.send_wait_timeout = 10;
    ESP_RETURN_ON_ERROR(httpd_start(&s_main_server, &main_config), TAG, "main server failed");
    ESP_RETURN_ON_ERROR(register_main_routes(), TAG, "main routes failed");

    httpd_config_t video_config = HTTPD_DEFAULT_CONFIG();
    video_config.server_port = 81;
    video_config.ctrl_port = 32769;
    video_config.max_uri_handlers = 2;
    video_config.max_open_sockets = 2;
    video_config.lru_purge_enable = true;
    video_config.stack_size = 6144;
    httpd_uri_t video = route("/video", HTTP_GET, stream_handler);
    ESP_RETURN_ON_ERROR(httpd_start(&s_video_server, &video_config), TAG, "video server failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_video_server, &video), TAG,
                        "video route failed");

    httpd_config_t audio_config = HTTPD_DEFAULT_CONFIG();
    audio_config.server_port = 82;
    audio_config.ctrl_port = 32770;
    audio_config.max_uri_handlers = 2;
    audio_config.max_open_sockets = 2;
    audio_config.lru_purge_enable = true;
    audio_config.stack_size = 5120;
    audio_config.send_wait_timeout = 5;
    httpd_uri_t audio = route("/audio.pcm", HTTP_GET, audio_stream_handler);
    ESP_RETURN_ON_ERROR(httpd_start(&s_audio_server, &audio_config), TAG, "audio server failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_audio_server, &audio), TAG,
                        "audio route failed");
    ESP_LOGI(TAG, "HTTP controls :80, MJPEG :81, PCM audio :82");
    return ESP_OK;
}
