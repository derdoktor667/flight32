/**
 * @file motor_config.h
 * @brief Configuration settings for motor control, including DShot protocol parameters and motor pin assignments.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>
#include <DShotRMT.h>
#include "../utils/system_constants.h"


// --- Motor Task Configuration ---
constexpr uint32_t MOTOR_TASK_STACK_SIZE = 4096;
constexpr uint8_t MOTOR_TASK_PRIORITY = 3;
constexpr int8_t MOTOR_TASK_CORE = 0;
constexpr uint8_t MOTOR_TASK_DELAY_MS = 10;
constexpr uint8_t NUM_MOTORS = 4;
constexpr uint8_t MOTOR_PIN_FR = 25;
constexpr uint8_t MOTOR_PIN_RL = 26;
constexpr uint8_t MOTOR_PIN_FL = 33;
constexpr uint8_t MOTOR_PIN_RR = 27;
constexpr dshot_mode_t DSHOT_PROTOCOL = DSHOT300;
constexpr int MOTOR_PINS_ARRAY[NUM_MOTORS] = {MOTOR_PIN_RR, MOTOR_PIN_FR, MOTOR_PIN_RL, MOTOR_PIN_FL};
constexpr uint16_t MOTOR_MIN_THROTTLE_RAW = 48;
constexpr uint16_t MOTOR_MAX_THROTTLE_RAW = 2047;
constexpr uint16_t THROTTLE_DEADZONE_THRESHOLD = 100; // Raw RC value below which motors are disarmed/stopped

enum class DshotProtocolIndex : uint8_t
{
    DSHOT150 = 0,
    DSHOT300 = 1,
    DSHOT600 = 2,
    DSHOT1200 = 3
};

constexpr uint8_t FIRST_MOTOR_INDEX = 0;

constexpr uint8_t MOTOR_INDEX_RR = 0; // Rear Right
constexpr uint8_t MOTOR_INDEX_FR = 1; // Front Right
constexpr uint8_t MOTOR_INDEX_RL = 2; // Rear Left
constexpr uint8_t MOTOR_INDEX_FL = 3; // Front Left

// --- Motor Protocol Settings ---
constexpr const char *NVS_KEY_MOTOR_PROTOCOL = "motor.proto";
constexpr uint8_t DEFAULT_MOTOR_PROTOCOL = static_cast<uint8_t>(DshotProtocolIndex::DSHOT300); // Corresponds to DSHOT300
extern const char *const DSHOT_PROTOCOL_STRINGS[];
extern const uint8_t NUM_DSHOT_PROTOCOLS;

// Normalized throttle values (0.0 to 1.0)
constexpr float MIN_NORMALIZED_THROTTLE = 0.0f;
constexpr float MAX_NORMALIZED_THROTTLE = 1.0f;

// Throttle percentage constants (0.0 to 100.0)
constexpr float MAX_THROTTLE_PERCENTAGE = CPU_PERCENTAGE_FACTOR;

// --- BLHeli_S 1-Wire Passthrough Constants ---
constexpr uint16_t sil_t_reset_us = 650;  // Reset time
constexpr uint16_t sil_t_delay_us = 10;   // Delay after reset
constexpr uint16_t sil_half_bit_delay_us = 14; // Half bit delay (critical timing)
constexpr uint16_t sil_t_start_us = 5;    // Start bit delay
constexpr uint16_t sil_t_stop_us = 10;    // Stop bit delay
constexpr uint16_t sil_t_byte_us = 200;   // Byte transmission delay