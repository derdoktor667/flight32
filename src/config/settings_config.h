/**
 * @file settings_config.h
 * @brief Configuration settings for the NVS-based settings manager.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- Settings Manager ---
constexpr const char *NVS_NAMESPACE = "f32_settings";
constexpr uint16_t NVS_CURRENT_SCHEMA_VERSION = 3;
constexpr const char *NVS_KEY_SCHEMA_VERSION = "schema_ver";
constexpr uint16_t NVS_DEFAULT_SCHEMA_VERSION = 0;
constexpr uint8_t IMU_DEFAULT_GYRO_RANGE = 2; // Corresponds to 1000_DPS in GYRO_RANGE_STRINGS

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

constexpr const char *NVS_KEY_MOTOR_MIN_THROTTLE = "motor.min_thr";
constexpr const char *NVS_KEY_MOTOR_MAX_THROTTLE = "motor.max_thr";
constexpr const char *NVS_KEY_MOTOR_MIN_COMMAND = "motor.min_cmd";

// --- Arming Settings ---
constexpr const char *NVS_KEY_ARM_AUTODISARM_DELAY = "arm.auto_disarm";
constexpr const char *NVS_KEY_ARM_KILLSWITCH_STATE = "arm.kill_switch";
