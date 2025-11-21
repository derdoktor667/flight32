/**
 * @file serial_config.h
 * @brief Configuration settings for serial communication parameters.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- Serial Configuration ---
constexpr auto SERIAL_BAUD_RATE = 115200;

// --- MSP FC Variant ---

constexpr uint16_t MSP_HARDWARE_REVISION_VALUE = 0; // Example: 1.0 -> 100

constexpr uint32_t MSP_TIMEOUT_MS = 1000;
