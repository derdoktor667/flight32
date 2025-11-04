#pragma once

#include <cstdint>

// --- Serial Configuration ---
static constexpr auto SERIAL_BAUD_RATE = 115200;

// --- Scheduler Configuration ---
static constexpr uint8_t MAX_TASKS = 10;

// --- Terminal Task Configuration ---
static constexpr uint32_t TERMINAL_TASK_STACK_SIZE = 4096;
static constexpr uint8_t TERMINAL_TASK_PRIORITY = 1;
static constexpr int8_t TERMINAL_TASK_CORE_ID = 1;
static constexpr uint16_t TERMINAL_INPUT_BUFFER_SIZE = 128;
static constexpr uint16_t TERMINAL_STATS_BUFFER_SIZE = 2048;
static constexpr uint8_t TERMINAL_TASK_DELAY_MS = 10;
