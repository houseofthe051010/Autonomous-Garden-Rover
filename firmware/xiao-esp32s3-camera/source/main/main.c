#include "audio_service.h"
#include "camera_service.h"
#include "network_service.h"
#include "web_server.h"
#include "system_metrics.h"

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"

static const char *TAG = "xiao-camera";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "XIAO ESP32S3 Sense camera starting");
    ESP_ERROR_CHECK(camera_service_init());
    ESP_ERROR_CHECK(audio_service_init());
    ESP_ERROR_CHECK(network_service_init());
    ESP_ERROR_CHECK(web_server_start());
    system_metrics_start();

    // The application has passed its camera, microphone, network, and server
    // startup checks. Keep the previous slot available until this point.
    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
        ESP_LOGI(TAG, "OTA image validated");
    }

    ESP_LOGI(TAG, "AP viewer: http://192.168.4.1/stream");
}
