/**
 * @file terminal_config.h
 * @brief Configuration settings for the serial terminal and communication manager tasks.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- Serial Manager Task Configuration ---
constexpr uint32_t SERIAL_MANAGER_TASK_STACK_SIZE = 8192;
constexpr uint8_t SERIAL_MANAGER_TASK_PRIORITY = 1;
constexpr int8_t SERIAL_MANAGER_TASK_CORE = 0;
constexpr uint8_t SERIAL_MANAGER_TASK_DELAY_MS = 10;
constexpr uint16_t TERMINAL_INPUT_BUFFER_SIZE = 128;

// --- Terminal/Utility ---
constexpr uint32_t ONE_SECOND_MICROSECONDS = 1000000;
constexpr uint16_t MAX_THROTTLE_VALUE = 2047;
constexpr uint8_t ASCII_BACKSPACE = 127;
constexpr int8_t INVALID_SETTING_VALUE = -1;
constexpr uint8_t UINT8_MAX_VALUE = 255;
constexpr uint8_t BYTE_BUFFER_SIZE = 15;
constexpr uint16_t BYTES_IN_KB = 1024;
constexpr uint32_t BYTES_IN_MB = (1024 * 1024);

// --- Terminal Formatting Constants ---
constexpr uint8_t TASK_NAME_COLUMN_WIDTH = 16;
constexpr uint8_t TASK_STATE_COLUMN_WIDTH = 10;
constexpr uint8_t TASK_PRIO_COLUMN_WIDTH = 6;
constexpr uint8_t TASK_CPU_COLUMN_WIDTH = 8;
constexpr uint8_t TASK_LOOP_COLUMN_WIDTH = 10;
constexpr uint8_t TASK_AVG_LOOP_COLUMN_WIDTH = 10;
constexpr uint8_t TASK_MAX_LOOP_COLUMN_WIDTH = 10;
constexpr uint8_t TASK_STACK_HWM_COLUMN_WIDTH = 17;
constexpr uint8_t TERMINAL_DISPLAY_KEY_BUFFER = 2;

// New constants for terminal output formatting and channel indexing
constexpr uint8_t TERMINAL_MIN_CHANNEL_INDEX = 1;
constexpr uint8_t TERMINAL_MAX_CHANNEL_INDEX = 14;
constexpr uint8_t TERMINAL_COLUMN_BUFFER_WIDTH = 2;
constexpr uint8_t TERMINAL_RX_DATA_DISPLAY_CHANNELS = 10;
constexpr uint8_t TERMINAL_RX_SINGLE_DESC_WIDTH = 25; // "RX Flight Mode (CH14)" is 22 chars, plus buffer
constexpr uint8_t SETTING_NAME_DISPLAY_WIDTH = 20;
constexpr uint16_t TASK_STATUS_OUTPUT_BUFFER_SIZE = 256;

// --- Time Constants ---
constexpr float MS_TO_SECONDS_FACTOR = 1000.0f;
constexpr float CPU_PERCENTAGE_FACTOR = 100.0f;