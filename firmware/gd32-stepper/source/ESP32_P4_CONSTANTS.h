#pragma once

// Shared constants for an ESP32-P4 host implementation. This file intentionally
// contains no UART-driver dependency so it can be used from ESP-IDF or Arduino.

#define GD32_STEPPER_BAUD_RATE             115200
#define GD32_STEPPER_UPDATE_HZ             60
#define GD32_STEPPER_DEADMAN_MS            250
#define GD32_STEPPER_MAX_CONTINUOUS_RPM    600.0f
#define GD32_STEPPER_MICROSTEPS_PER_REV    3200

#define GD32_CMD_VELOCITY                  "M970"
#define GD32_CMD_COUNTED_MOVE              "M971"
#define GD32_CMD_COUNTED_SPEED             "M972"
#define GD32_CMD_E_ARM                     "M973"
#define GD32_CMD_DIRECT_STATUS              "M974"
#define GD32_CMD_DIRECT_STOP                "M975"

#define GD32_REPLY_VELOCITY_ACK            "VEL_ACK"
#define GD32_REPLY_VELOCITY_ERROR          "VEL_ERR"
#define GD32_REPLY_VELOCITY_TIMEOUT        "VEL_TIMEOUT"
#define GD32_REPLY_COUNTED_ACK             "COUNT_ACK"
#define GD32_REPLY_COUNTED_DONE            "COUNT_DONE"
#define GD32_REPLY_COUNTED_ERROR           "COUNT_ERR"
#define GD32_REPLY_DIRECT_STOPPED          "DIRECT_STOPPED"
#define GD32_REPLY_HEARTBEAT                "HB"
#define GD32_REPLY_HEARTBEAT_OK             "HB_ACK_OK"
#define GD32_REPLY_SWITCHES                 "SW"
