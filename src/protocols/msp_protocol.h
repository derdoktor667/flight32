/**
 * @file msp_protocol.h
 * @brief Defines constants and command IDs for the MultiWii Serial Protocol (MSP).
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- MSP Protocol Constants ---
static constexpr uint8_t MSP_PROTOCOL_VERSION = 0;
static constexpr uint8_t MSP_API_VERSION_MAJOR = 1;
static constexpr uint8_t MSP_API_VERSION_MINOR = 0;

// --- MSP Constants ---
static constexpr uint8_t MSP_MAX_PAYLOAD_SIZE = 128;
static constexpr uint8_t MSP_PID_PAYLOAD_SIZE = 9;

// --- MSP Payload Sizes ---
static constexpr uint8_t MSP_API_VERSION_PAYLOAD_SIZE = 3;
static constexpr uint8_t MSP_FC_VARIANT_PAYLOAD_SIZE = 4;
static constexpr uint8_t MSP_FC_VERSION_PAYLOAD_SIZE = 3;
static constexpr uint8_t MSP_BOARD_INFO_PAYLOAD_SIZE = 8;
static constexpr uint8_t MSP_BUILD_INFO_PAYLOAD_SIZE = 19; // "Nov 12 202512:00:00" is 19 chars
static constexpr uint8_t MSP_MEM_STATS_PAYLOAD_SIZE = 4;
static constexpr uint8_t MSP_RAW_IMU_PAYLOAD_SIZE = 18; // 3x Accel, 3x Gyro, 3x Mag (int16_t = 2 bytes each)
static constexpr uint8_t MSP_ATTITUDE_PAYLOAD_SIZE = 6;  // 3x angles (int16_t = 2 bytes each)
static constexpr uint8_t MSP_RC_PAYLOAD_SIZE = 16;       // 8x RC channels (int16_t = 2 bytes each)
static constexpr uint8_t MSP_MOTOR_PAYLOAD_SIZE = 8;     // 4x motor outputs (int16_t = 2 bytes each)

// --- MSP Scaling Factors ---
static constexpr float MSP_ACCEL_SCALING_FACTOR = 512.0f; // 1G = 512 (approx)
static constexpr float MSP_GYRO_SCALING_FACTOR = 4.0f;    // 1 deg/s = 4 (approx)
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
static constexpr uint8_t MSP_STATUS = 7;
static constexpr uint8_t MSP_MEM_STATS = 8;
static constexpr uint8_t MSP_GET_SETTING = 9;
static constexpr uint8_t MSP_SET_SETTING = 10;
static constexpr uint8_t MSP_PID = 11;
static constexpr uint8_t MSP_RAW_IMU = 102;
static constexpr uint8_t MSP_MOTOR = 104;
static constexpr uint8_t MSP_RC = 105;
static constexpr uint8_t MSP_ATTITUDE = 108;
static constexpr uint8_t MSP_EEPROM_WRITE = 200;
static constexpr uint8_t MSP_RESET_SETTINGS = 201;
static constexpr uint8_t MSP_SET_PID = 202;