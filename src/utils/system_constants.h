/**
 * @file system_constants.h
 * @brief Centralized header for generic system-wide utility constants.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint> // For uint8_t
#include <limits>  // For numeric_limits

// Max values
constexpr uint8_t UINT8_MAX_VALUE = std::numeric_limits<uint8_t>::max();

// Buffer sizes
constexpr uint8_t BYTE_BUFFER_SIZE = 128; // Generic byte buffer size, e.g., for serial communications

// Byte conversion constants
constexpr uint16_t BYTES_IN_KB = 1024;
constexpr uint32_t BYTES_IN_MB = BYTES_IN_KB * BYTES_IN_KB;

// Invalid setting value
constexpr int32_t INVALID_SETTING_VALUE = -99999; // Represents an invalid or uninitialized setting value
