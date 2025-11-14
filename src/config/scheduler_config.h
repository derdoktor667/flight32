/**
 * @file scheduler_config.h
 * @brief Configuration settings for the FreeRTOS task scheduler.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- Scheduler Configuration ---
static constexpr uint8_t MAX_TASKS = 10;
static constexpr uint32_t SCHEDULER_TASK_STACK_SIZE = 4096;
static constexpr uint8_t SCHEDULER_TASK_PRIORITY = 2;
static constexpr int8_t SCHEDULER_TASK_CORE = 0;