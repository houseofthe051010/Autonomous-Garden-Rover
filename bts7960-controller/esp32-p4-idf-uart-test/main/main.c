#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define STM32_UART UART_NUM_1
#define STM32_BAUD 115200
#define GPIO_A 21
#define GPIO_B 22
#define RX_PROBE_MS 2400
#define PONG_TIMEOUT_MS 1200
#define LINK_TIMEOUT_MS 3500
#define PING_INTERVAL_MS 1000
#define LINE_CAPACITY 192

static const char *TAG = "stm32_uart";
static bool uart_installed;
static char line_buffer[LINE_CAPACITY];
static size_t line_length;
static uint32_t rx_byte_count;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static bool is_link_line(const char *line)
{
    return strncmp(line, "HB ", 3) == 0 ||
           strncmp(line, "READY ", 6) == 0 ||
           strcmp(line, "OK PONG") == 0;
}

static bool read_line(char *output, size_t output_size, uint32_t timeout_ms)
{
    const int64_t deadline = now_ms() + timeout_ms;
    uint8_t bytes[32];

    while (now_ms() < deadline) {
        const int remaining = (int)(deadline - now_ms());
        const TickType_t wait_ticks = pdMS_TO_TICKS(remaining > 20 ? 20 : remaining);
        const int count = uart_read_bytes(STM32_UART, bytes, sizeof(bytes), wait_ticks);
        rx_byte_count += (uint32_t)count;

        for (int i = 0; i < count; ++i) {
            const uint8_t byte = bytes[i];
            if (byte == '\n') {
                if (line_length == 0) {
                    continue;
                }
                line_buffer[line_length] = '\0';
                snprintf(output, output_size, "%s", line_buffer);
                line_length = 0;
                return true;
            }
            if (byte == '\r') {
                continue;
            }
            if (line_length < sizeof(line_buffer) - 1) {
                line_buffer[line_length++] = (char)byte;
            } else {
                line_length = 0;
                ESP_LOGW(TAG, "Discarded an overlong UART line");
            }
        }
    }
    return false;
}

static void close_link_uart(void)
{
    if (uart_installed) {
        uart_driver_delete(STM32_UART);
        uart_installed = false;
    }
    gpio_reset_pin(GPIO_A);
    gpio_reset_pin(GPIO_B);
    line_length = 0;
}

static esp_err_t open_receive_probe(int rx_gpio)
{
    close_link_uart();

    const uart_config_t config = {
        .baud_rate = STM32_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(
        uart_driver_install(STM32_UART, 1024, 0, 0, NULL, 0),
        TAG, "uart_driver_install failed");
    uart_installed = true;

    ESP_RETURN_ON_ERROR(uart_param_config(STM32_UART, &config), TAG,
                        "uart_param_config failed");
    ESP_RETURN_ON_ERROR(
        uart_set_pin(STM32_UART, UART_PIN_NO_CHANGE, rx_gpio,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
        TAG, "uart_set_pin RX probe failed");

    gpio_set_pull_mode(rx_gpio, GPIO_PULLUP_ONLY);
    uart_flush_input(STM32_UART);
    return ESP_OK;
}

static bool probe_rx(int rx_gpio)
{
    char line[LINE_CAPACITY];

    ESP_LOGI(TAG, "Listening on GPIO%d for STM32 heartbeat (TX remains disabled)",
             rx_gpio);
    if (open_receive_probe(rx_gpio) != ESP_OK) {
        return false;
    }
    const uint32_t byte_count_at_start = rx_byte_count;

    const int64_t deadline = now_ms() + RX_PROBE_MS;
    while (now_ms() < deadline) {
        if (!read_line(line, sizeof(line), 100)) {
            continue;
        }
        ESP_LOGI(TAG, "RX GPIO%d: %s", rx_gpio, line);
        if (strncmp(line, "HB ", 3) == 0 || strncmp(line, "READY ", 6) == 0) {
            return true;
        }
    }
    ESP_LOGI(TAG, "GPIO%d probe received %lu UART bytes", rx_gpio,
             (unsigned long)(rx_byte_count - byte_count_at_start));
    return false;
}

static void send_command(const char *command)
{
    uart_write_bytes(STM32_UART, command, strlen(command));
    uart_write_bytes(STM32_UART, "\n", 1);
    uart_wait_tx_done(STM32_UART, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "TX: %s", command);
}

static bool verify_bidirectional_link(int tx_gpio, int rx_gpio)
{
    char line[LINE_CAPACITY];

    ESP_ERROR_CHECK(
        uart_set_pin(STM32_UART, tx_gpio, rx_gpio,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "STM32 wiring detected: ESP32-P4 TX GPIO%d, RX GPIO%d",
             tx_gpio, rx_gpio);

    // Establish a known safe motor state before doing any other test.
    send_command("MSTOP ALL");
    send_command("PING");

    const int64_t deadline = now_ms() + PONG_TIMEOUT_MS;
    while (now_ms() < deadline) {
        if (!read_line(line, sizeof(line), 100)) {
            continue;
        }
        ESP_LOGI(TAG, "STM32: %s", line);
        if (strcmp(line, "OK PONG") == 0) {
            ESP_LOGI(TAG, "UART ROUND TRIP VERIFIED: OK PONG");
            return true;
        }
    }

    ESP_LOGE(TAG,
             "Heartbeat was received on GPIO%d, but STM32 did not answer PING "
             "sent from GPIO%d",
             rx_gpio, tx_gpio);
    return false;
}

static bool find_link(int *tx_gpio, int *rx_gpio)
{
    const int candidates[] = {GPIO_A, GPIO_B};

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        const int candidate_rx = candidates[i];
        const int candidate_tx = candidate_rx == GPIO_A ? GPIO_B : GPIO_A;
        if (!probe_rx(candidate_rx)) {
            continue;
        }

        *rx_gpio = candidate_rx;
        *tx_gpio = candidate_tx;
        if (verify_bidirectional_link(*tx_gpio, *rx_gpio)) {
            return true;
        }

        // RX orientation is known. Retry TX once before restarting detection.
        vTaskDelay(pdMS_TO_TICKS(250));
        send_command("PING");
        char line[LINE_CAPACITY];
        const int64_t deadline = now_ms() + PONG_TIMEOUT_MS;
        while (now_ms() < deadline) {
            if (read_line(line, sizeof(line), 100)) {
                ESP_LOGI(TAG, "STM32: %s", line);
                if (strcmp(line, "OK PONG") == 0) {
                    ESP_LOGI(TAG, "UART ROUND TRIP VERIFIED: OK PONG");
                    return true;
                }
            }
        }
        break;
    }

    close_link_uart();
    return false;
}

static void monitor_link(void)
{
    char line[LINE_CAPACITY];
    int tx_gpio = -1;
    int rx_gpio = -1;

    while (true) {
        while (!find_link(&tx_gpio, &rx_gpio)) {
            ESP_LOGW(TAG,
                     "STM32 not found on GPIO21/22. Check common GND and crossed "
                     "TX/RX wiring; retrying in 1 second");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        int64_t last_rx = now_ms();
        int64_t next_ping = now_ms() + PING_INTERVAL_MS;

        while (now_ms() - last_rx <= LINK_TIMEOUT_MS) {
            if (read_line(line, sizeof(line), 50)) {
                ESP_LOGI(TAG, "STM32: %s", line);
                if (is_link_line(line)) {
                    last_rx = now_ms();
                }
            }
            if (now_ms() >= next_ping) {
                send_command("PING");
                next_ping = now_ms() + PING_INTERVAL_MS;
            }
        }

        ESP_LOGE(TAG, "STM32 UART timed out; returning GPIO21/22 to inputs");
        close_link_uart();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 STM32 UART diagnostic");
    ESP_LOGI(TAG, "Testing GPIO21 and GPIO22 at 115200 baud, 8N1");
    ESP_LOGI(TAG, "This firmware never sends a motor movement command");
    monitor_link();
}
