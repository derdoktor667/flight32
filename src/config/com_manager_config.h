/**
 * @file com_manager_config.h
 * @brief Configuration settings for the communication manager task, including queue lengths and message sizes.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- Com Task Configuration ---
constexpr uint32_t COM_TASK_STACK_SIZE = 8192;
constexpr uint8_t COM_TASK_PRIORITY = 2;
constexpr int8_t COM_TASK_CORE = 0; // Assign to Core 0 (PRO_CPU)
constexpr uint8_t COM_TASK_DELAY_MS = 10;
constexpr uint8_t COM_QUEUE_LENGTH = 50;
constexpr uint8_t COM_FLUSH_QUEUE_LENGTH = 1;
constexpr uint16_t COM_MESSAGE_MAX_LENGTH = 512;
constexpr uint16_t MSP_RESPONSE_DELAY_US = 100;
constexpr uint8_t UNIQUE_ID_BUFFER_SIZE = 32;

// MSP FC Variant, Board Info
static constexpr const char *MSP_FC_VARIANT_NAME = "FL32";
static constexpr const char *MSP_BOARD_IDENTIFIER = "FL32";
static constexpr const char *MSP_BOARD_NAME = "flight32";
