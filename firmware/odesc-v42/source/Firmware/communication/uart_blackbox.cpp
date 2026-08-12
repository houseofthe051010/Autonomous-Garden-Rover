#include "uart_blackbox.hpp"

#include <string.h>

#include "odrive_main.h"
#include "stm32f4xx_hal_flash.h"

// STM32F405 sector 9. The linker script reserves this entire 128 KiB sector.
static constexpr uintptr_t LOG_BASE = 0x080A0000UL;
static constexpr size_t LOG_BYTES = 128U * 1024U;
static constexpr uint32_t LOG_MAGIC = 0x3142554fUL; // OUB1
static constexpr uint32_t ERASED_WORD = 0xffffffffUL;
static constexpr uint32_t UART_SILENCE_MS = 1500U;

static_assert(sizeof(UartBlackboxRecord) == 64,
              "UART blackbox records must be exactly 64 bytes");
static constexpr uint32_t LOG_CAPACITY = LOG_BYTES / sizeof(UartBlackboxRecord);

enum : uint32_t {
    REASON_BOOT = 1,
    REASON_UART_ERROR = 2,
    REASON_ACTIVE_SILENCE = 3,
    REASON_DMA_RESTART = 4,
};

static uint32_t next_sequence;
static uint32_t write_slot;
static uint32_t rx_bytes;
static uint32_t dma_restarts;
static uint32_t error_events;
static uint32_t silence_events;
static uint32_t last_rx_tick;
static uint32_t last_error_code;
static bool silence_latched;
static bool storage_usable;
static bool pending;
static UartBlackboxRecord pending_record;

static uint32_t crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xffffffffUL;
    while (length--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320UL & (0U - (crc & 1U)));
        }
    }
    return crc;
}

static bool valid_record(const UartBlackboxRecord& record) {
    UartBlackboxRecord copy = record;
    uint32_t expected = copy.crc32;
    copy.magic = 0;
    copy.crc32 = 0;
    return record.magic == LOG_MAGIC && expected == crc32(
        reinterpret_cast<const uint8_t *>(&copy), sizeof(copy));
}

static bool axis_is_safe_for_flash(void) {
    // This ODESC clone has one physical power stage (M0). Its inherited
    // two-axis object model must not prevent diagnostic commits forever.
    return !axes[0].motor_.is_armed_ &&
           axes[0].current_state_ == Axis::AXIS_STATE_IDLE;
}

static void capture(UART_HandleTypeDef *uart, uint32_t reason) {
    if (pending || !uart) return;
    pending_record = {};
    pending_record.sequence = next_sequence;
    pending_record.uptime_ms = HAL_GetTick();
    pending_record.reason = reason;
    pending_record.hal_error = uart->ErrorCode;
    pending_record.uart_status = uart->Instance->SR;
    pending_record.rx_state = uart->RxState;
    pending_record.dma_remaining = uart->hdmarx ? uart->hdmarx->Instance->NDTR : 0;
    pending_record.rx_bytes = rx_bytes;
    pending_record.dma_restarts = dma_restarts;
    pending_record.axis_error = axes[0].error_;
    pending_record.motor_error = axes[0].motor_.error_;
    pending_record.estimator_error = axes[0].sensorless_estimator_.error_;
    pending_record.axis_state = axes[0].current_state_;
    pending_record.velocity_turns_s =
        axes[0].sensorless_estimator_.vel_estimate_.any().value_or(0.0f);
    pending = true;
}

static bool append_pending(void) {
    if (!pending || !storage_usable || write_slot >= LOG_CAPACITY ||
        !axis_is_safe_for_flash()) return false;

    UartBlackboxRecord record = pending_record;
    record.magic = 0;
    record.crc32 = 0;
    record.crc32 = crc32(reinterpret_cast<const uint8_t *>(&record), sizeof(record));
    uintptr_t address = LOG_BASE + write_slot * sizeof(record);
    const uint32_t *words = reinterpret_cast<const uint32_t *>(&record);

    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    bool okay = true;
    // The validity magic is programmed last, so a power loss leaves an invalid slot.
    for (size_t i = 1; i < sizeof(record) / sizeof(uint32_t); ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4,
                              words[i]) != HAL_OK) {
            okay = false;
            break;
        }
    }
    if (okay && HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address,
                                  LOG_MAGIC) != HAL_OK) okay = false;
    HAL_FLASH_Lock();
    if (!okay) {
        storage_usable = false;
        return false;
    }
    pending = false;
    ++write_slot;
    ++next_sequence;
    return true;
}

void uart_blackbox_init(UART_HandleTypeDef *uart) {
    write_slot = 0;
    while (write_slot < LOG_CAPACITY) {
        const auto& record = reinterpret_cast<const UartBlackboxRecord *>(LOG_BASE)[write_slot];
        if (!valid_record(record)) break;
        next_sequence = record.sequence + 1;
        ++write_slot;
    }
    const uint32_t *next = reinterpret_cast<const uint32_t *>(
        LOG_BASE + write_slot * sizeof(UartBlackboxRecord));
    storage_usable = write_slot < LOG_CAPACITY && *next == ERASED_WORD;
    last_rx_tick = HAL_GetTick();
    capture(uart, REASON_BOOT);
}

void uart_blackbox_note_rx(uint32_t count) {
    rx_bytes += count;
    last_rx_tick = HAL_GetTick();
    silence_latched = false;
}

void uart_blackbox_note_dma_restart(UART_HandleTypeDef *uart) {
    ++dma_restarts;
    capture(uart, REASON_DMA_RESTART);
}

void uart_blackbox_poll(UART_HandleTypeDef *uart) {
    if (!uart) return;
    if (uart->ErrorCode && uart->ErrorCode != last_error_code) {
        last_error_code = uart->ErrorCode;
        ++error_events;
        capture(uart, REASON_UART_ERROR);
    } else if (!uart->ErrorCode) {
        last_error_code = 0;
    }
    bool armed = axes[0].motor_.is_armed_;
    if (armed && !silence_latched && HAL_GetTick() - last_rx_tick >= UART_SILENCE_MS) {
        silence_latched = true;
        ++silence_events;
        capture(uart, REASON_ACTIVE_SILENCE);
    }
    if (!armed) silence_latched = false;
    append_pending();
}

uint32_t uart_blackbox_record_count(void) { return write_slot; }
uint32_t uart_blackbox_rx_bytes(void) { return rx_bytes; }
uint32_t uart_blackbox_dma_restarts(void) { return dma_restarts; }
uint32_t uart_blackbox_error_events(void) { return error_events; }
uint32_t uart_blackbox_silence_events(void) { return silence_events; }

bool uart_blackbox_read(uint32_t index, UartBlackboxRecord *record) {
    if (!record || index >= write_slot) return false;
    *record = reinterpret_cast<const UartBlackboxRecord *>(LOG_BASE)[index];
    return valid_record(*record);
}
