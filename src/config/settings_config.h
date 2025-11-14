/**
 * @file settings_config.h
 * @brief Configuration settings for the NVS-based settings manager.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- Settings Manager ---
constexpr const char *SETTINGS_NAMESPACE = "flight32";
constexpr uint16_t CURRENT_SCHEMA_VERSION = 2;
constexpr const char *KEY_SCHEMA_VERSION = "schema_ver";
constexpr uint16_t DEFAULT_SCHEMA_VERSION = 0;
constexpr uint8_t DEFAULT_GYRO_RANGE = 2; // Corresponds to 1000_DPS in GYRO_RANGE_STRINGS

// --- Settings Manager Keys ---
constexpr const char *KEY_SYSTEM_NAME = "system.name";
constexpr const char *DEFAULT_SYSTEM_NAME = "Flight32";
constexpr const char *KEY_MPU_GYRO_RANGE = "mpu.g_range";

constexpr const char *KEY_MPU_GYRO_OFF_X = "mpu.g_off.x";
constexpr const char *KEY_MPU_GYRO_OFF_Y = "mpu.g_off.y";
constexpr const char *KEY_MPU_GYRO_OFF_Z = "mpu.g_off.z";
constexpr const char *KEY_MPU_ACCEL_OFF_X = "mpu.a_off.x";
constexpr const char *KEY_MPU_ACCEL_OFF_Y = "mpu.a_off.y";
constexpr const char *KEY_MPU_ACCEL_OFF_Z = "mpu.a_off.z";