#pragma once

#include <cstdint>

// --- Com Task Configuration ---
constexpr uint32_t COM_TASK_STACK_SIZE = 4096;
constexpr uint8_t COM_TASK_PRIORITY = 2;
constexpr uint8_t COM_QUEUE_LENGTH = 50;
constexpr uint8_t COM_FLUSH_QUEUE_LENGTH = 1;
constexpr uint16_t COM_MESSAGE_MAX_LENGTH = 512;
constexpr uint32_t COM_TASK_STARTUP_DELAY_MS = 10;
