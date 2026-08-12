#pragma once

#include <stdint.h>

#include "usart.h"

struct UartBlackboxRecord {
    uint32_t magic;
    uint32_t sequence;
    uint32_t uptime_ms;
    uint32_t reason;
    uint32_t hal_error;
    uint32_t uart_status;
    uint32_t rx_state;
    uint32_t dma_remaining;
    uint32_t rx_bytes;
    uint32_t dma_restarts;
    uint32_t axis_error;
    uint32_t motor_error;
    uint32_t estimator_error;
    uint32_t axis_state;
    float velocity_turns_s;
    uint32_t crc32;
};

void uart_blackbox_init(UART_HandleTypeDef *uart);
void uart_blackbox_note_rx(uint32_t count);
void uart_blackbox_note_dma_restart(UART_HandleTypeDef *uart);
void uart_blackbox_poll(UART_HandleTypeDef *uart);
uint32_t uart_blackbox_record_count(void);
uint32_t uart_blackbox_rx_bytes(void);
uint32_t uart_blackbox_dma_restarts(void);
uint32_t uart_blackbox_error_events(void);
uint32_t uart_blackbox_silence_events(void);
bool uart_blackbox_read(uint32_t index, UartBlackboxRecord *record);
