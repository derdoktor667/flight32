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

// --- Firmware Version ---
static constexpr const char *FIRMWARE_VERSION = "0.4.0";

// --- Flight Controller Version Constants ---
static constexpr uint8_t FC_VERSION_MAJOR = 0;
static constexpr uint8_t FC_VERSION_MINOR = 4;
static constexpr uint8_t FC_VERSION_PATCH = 0;

// --- Task Names ---
constexpr const char *COM_TASK_NAME = "Com Manager";
constexpr const char *IMU_TASK_NAME = "IMU / Sensor";
constexpr const char *RX_TASK_NAME = "RX / Receiver";
constexpr const char *MOTOR_TASK_NAME = "Motor Controller";
constexpr const char *SERIAL_MANAGER_TASK_NAME = "Serial Manager";
constexpr const char *IDLE_TASK_NAME_0 = "IDLE0";
constexpr const char *IDLE_TASK_NAME_1 = "IDLE1";
constexpr const char *TIMER_SERVICE_TASK_NAME = "Tmr Svc";

// Conversion constants
constexpr float RADIANS_TO_DEGREES = 180.0f / M_PI;

// Time-related constants
constexpr float MS_PER_SECOND = 1000.0f;
constexpr uint64_t ONE_SECOND_MICROSECONDS = 1000000ULL;

constexpr float MS_TO_SECONDS_FACTOR = 0.001f;

// Angle-related constants
constexpr float HALF_PI_RADIANS = M_PI / 2.0f;

// Filter Calculation Constants
constexpr float BUTTERWORTH_Q_FACTOR = 0.7071f;
constexpr float BIQUAD_ALPHA_DENOMINATOR_FACTOR = 2.0f;

// CPU usage constants
constexpr float CPU_PERCENTAGE_FACTOR = 100.0f;
