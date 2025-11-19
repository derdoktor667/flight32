/**
 * @file msp_protocol.h
 * @brief Defines constants and command IDs for the MultiWii Serial Protocol (MSP).
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- MSP Protocol Constants ---
static constexpr uint8_t MSP_PROTOCOL_VERSION = 0; // Changed to 0 for Betaflight compatibility
static constexpr uint8_t MSP_API_VERSION_MAJOR = 1; // From Betaflight example
static constexpr uint8_t MSP_API_VERSION_MINOR = 43; // From Betaflight example
static constexpr uint8_t MSP_CAPABILITY = 1;              // From Betaflight example
static constexpr char MSP_FC_IDENTIFIER = 'F';            // 'F' for Flight32
static constexpr uint8_t MSP_RESPONSE_OVERHEAD_BYTES = 2; // 1 byte for payload size, 1 byte for command ID
static constexpr uint8_t MSP_MIN_PAYLOAD_SIZE = 3;        // Minimum payload size for MSP commands (e.g., API_VERSION)

// --- MSP Constants ---
static constexpr uint8_t MSP_MAX_PAYLOAD_SIZE = 128;
static constexpr uint8_t MSP_PID_PAYLOAD_SIZE = 18;

// --- MSP Payload Sizes ---
static constexpr uint8_t MSP_API_VERSION_PAYLOAD_SIZE = 3;
static constexpr uint8_t MSP_FC_VARIANT_PAYLOAD_SIZE = 0;
static constexpr uint8_t MSP_FC_VERSION_PAYLOAD_SIZE = 3;
static constexpr uint8_t MSP_MEM_STATS_PAYLOAD_SIZE = 4;
static constexpr uint8_t MSP_RAW_IMU_PAYLOAD_SIZE = 18; // 3x Accel, 3x Gyro, 3x Mag (int16_t = 2 bytes each)
static constexpr uint8_t MSP_ATTITUDE_PAYLOAD_SIZE = 6; // 3x angles (int16_t = 2 bytes each)
static constexpr uint8_t MSP_RC_PAYLOAD_SIZE = 36;      // 18x RC channels (int16_t = 2 bytes each)
static constexpr uint8_t MSP_MOTOR_PAYLOAD_SIZE = 8;    // 4x motor outputs (int16_t = 2 bytes each)
static constexpr uint8_t MSP_MAX_MOTORS = 8; // Max number of motors expected by MSP_MOTOR command
static constexpr uint8_t MSP_BOXNAMES_PAYLOAD_SIZE = 0;
static constexpr uint8_t MSP_MODE_RANGES_PAYLOAD_SIZE = 16; // 4 mode ranges * 4 bytes each
static constexpr uint8_t MSP_MOTOR_CONFIG_PAYLOAD_SIZE = 10; // minthrottle (U16) + maxthrottle (U16) + mincommand (U16) + motorCount (U8) + motorPoleCount (U8) + useDshotTelemetry (U8) + featureIsEnabled(FEATURE_ESC_SENSOR) (U8)
static constexpr uint8_t MSP_FILTER_CONFIG_PAYLOAD_SIZE = 20; // 5 floats * 4 bytes each
static constexpr uint8_t MSP_BOX_PAYLOAD_SIZE = 4; // 4-byte bitmask for active modes
static constexpr uint8_t MSP_UID_PAYLOAD_SIZE = 12;           // 3x uint32_t for unique ID
static constexpr uint8_t MSP_STATUS_PAYLOAD_SIZE = 11;        // cycletime, i2c_errors, sensors, flightmode, profile

// --- Betaflight Permanent IDs for Modes ---
static constexpr uint8_t BF_PERMANENT_ID_ANGLE = 1;
static constexpr uint8_t BF_PERMANENT_ID_ARM = 0; // From Betaflight msp_box.c
static constexpr uint8_t BF_PERMANENT_ID_FAILSAFE = 27; // From Betaflight msp_box.c
static constexpr uint8_t BF_PERMANENT_ID_TELEMETRY = 20; // From Betaflight msp_box.c
static constexpr uint8_t BF_PERMANENT_ID_FLIPOVERAFTERCRASH = 35; // From Betaflight msp_box.c
static constexpr uint8_t BF_PERMANENT_ID_ACRO = 53; // Placeholder for Acro, as Betaflight doesn't have explicit Acro box

// --- MSP Flight Mode IDs ---
static constexpr uint8_t MSP_BOX_ACRO_ID = 0;
static constexpr uint8_t MSP_BOX_STABILIZED_ID = 1;

// --- MSP Scaling Factors ---
static constexpr float MSP_ATTITUDE_SCALE_FACTOR = 10.0f; // Roll/Pitch scaled by 10

// --- MSP Setting Payload Overheads ---
static constexpr uint8_t MSP_SETTING_KEY_VALUE_OVERHEAD_BYTES = 2; // 1 byte for key_len, 1 byte for value_len

// --- MSP Command IDs ---
static constexpr uint8_t MSP_API_VERSION = 1;
static constexpr uint8_t MSP_FC_VARIANT = 2;
static constexpr uint8_t MSP_FC_VERSION = 3;
static constexpr uint8_t MSP_BOARD_INFO = 4;
static constexpr uint8_t MSP_BUILD_INFO = 5;
static constexpr uint8_t MSP_REBOOT = 6;
static constexpr uint8_t MSP_MEM_STATS = 208;
static constexpr uint8_t MSP_GET_SETTING = 9;
static constexpr uint8_t MSP_SET_SETTING = 10;
static constexpr uint8_t MSP_PID = 102;
static constexpr uint8_t MSP_STATUS = 101;
static constexpr uint8_t MSP_RAW_IMU = 106;
static constexpr uint8_t MSP_MOTOR = 104;
static constexpr uint8_t MSP_RC = 105;
static constexpr uint8_t MSP_ATTITUDE = 100;
static constexpr uint8_t MSP_BOX = 117;
static constexpr uint8_t MSP_BOXNAMES = 116;
static constexpr uint8_t MSP_MODE_RANGES = 119;
static constexpr uint8_t MSP_MOTOR_CONFIG = 124;
static constexpr uint8_t MSP_UID = 160;
static constexpr uint8_t MSP_SENSOR_STATUS = 108; // Python defines SENSOR_STATUS as 108
static constexpr uint8_t MSP_SENSOR_STATUS_PAYLOAD_SIZE = 3; // Python's _decode_msp_sensor_status expects 3 bytes
static constexpr uint8_t MSP_EEPROM_WRITE = 200;
static constexpr uint8_t MSP_RESET_SETTINGS = 201;
static constexpr uint8_t MSP_SET_PID = 202;
static constexpr uint8_t MSP_GET_FILTER_CONFIG = 203;
static constexpr uint8_t MSP_SET_FILTER_CONFIG = 204;
static constexpr uint8_t MSP_SET_BOX = 205;
