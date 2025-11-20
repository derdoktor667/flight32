/**
 * @file terminal_config.h
 * @brief Configuration settings for the serial terminal and communication manager tasks.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include "../utils/system_constants.h"

// --- Serial Manager Task Configuration ---
constexpr uint32_t SERIAL_MANAGER_TASK_STACK_SIZE = 8192;
constexpr uint8_t SERIAL_MANAGER_TASK_PRIORITY = 1;
constexpr int8_t SERIAL_MANAGER_TASK_CORE = 0;
constexpr uint8_t SERIAL_MANAGER_TASK_DELAY_MS = 10;
constexpr uint16_t TERMINAL_INPUT_BUFFER_SIZE = 128;

// --- Terminal/Utility ---
constexpr uint16_t MAX_THROTTLE_VALUE = 2047;
constexpr uint8_t ASCII_BACKSPACE = 127;

constexpr uint8_t TASK_NAME_COLUMN_WIDTH = 16;
constexpr uint8_t TASK_STATE_COLUMN_WIDTH = 10;
constexpr uint8_t TASK_PRIO_COLUMN_WIDTH = 6;
constexpr uint8_t TASK_CPU_COLUMN_WIDTH = 8;
constexpr uint8_t TASK_LOOP_COLUMN_WIDTH = 10;
constexpr uint8_t TASK_AVG_LOOP_COLUMN_WIDTH = 10;
constexpr uint8_t TASK_CORE_COLUMN_WIDTH = 5;
constexpr uint8_t TASK_STACK_HWM_COLUMN_WIDTH = 17;
constexpr uint8_t TERMINAL_DISPLAY_KEY_BUFFER = 2;

// New constants for terminal output formatting and channel indexing
constexpr uint8_t TERMINAL_MIN_CHANNEL_INDEX = 1;
constexpr uint8_t TERMINAL_MAX_CHANNEL_INDEX = 14;
constexpr uint8_t TERMINAL_COLUMN_BUFFER_WIDTH = 2;
constexpr uint8_t TERMINAL_COLUMN_MAX_WIDTH = 12;
constexpr uint8_t TERMINAL_RX_DATA_DISPLAY_CHANNELS = 10;
constexpr uint8_t TERMINAL_RX_SINGLE_DESC_WIDTH = 25; // "RX Flight Mode (CH14)" is 22 chars, plus buffer
constexpr uint8_t SETTING_NAME_DISPLAY_WIDTH = 20;
constexpr uint16_t TASK_STATUS_OUTPUT_BUFFER_SIZE = 256;
