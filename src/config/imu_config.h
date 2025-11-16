/**
 * @file imu_config.h
 * @brief Configuration settings for the IMU sensor, including task parameters, I2C settings, and LPF bandwidth.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>
#include <cmath>

// --- IMU Task Configuration ---
constexpr uint32_t IMU_TASK_STACK_SIZE = 4096;
constexpr uint8_t IMU_TASK_PRIORITY = 5;
constexpr int8_t IMU_TASK_CORE = 1;
constexpr uint8_t IMU_TASK_DELAY_MS = 10;
constexpr uint8_t MPU6050_I2C_SDA = 21;
constexpr uint8_t MPU6050_I2C_SCL = 22;
constexpr uint32_t MPU6050_I2C_CLOCK_SPEED = 400000;
constexpr uint8_t MPU6050_I2C_ADDRESS = 0x70;     // usually 0x68
constexpr uint8_t MPU6050_FIFO_BUFFER_SIZE = 128; // Size of the MPU6050 FIFO buffer
constexpr bool IMU_DMP_ENABLED_DEFAULT = false;   // Default setting for enabling the Digital Motion Processor (DMP)

// --- IMU Protocol Configuration ---
enum class ImuType : uint8_t
{
    MPU6050 = 0,
    NONE = 255
};

constexpr const char *KEY_IMU_TYPE = "imu.type";
constexpr ImuType DEFAULT_IMU_TYPE = ImuType::MPU6050;

enum class ImuGyroRangeIndex : uint8_t
{
    GYRO_RANGE_250DPS = 0,
    GYRO_RANGE_500DPS = 1,
    GYRO_RANGE_1000DPS = 2,
    GYRO_RANGE_2000DPS = 3
};

enum class ImuAccelRangeIndex : uint8_t
{
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G = 1,
    ACCEL_RANGE_8G = 2,
    ACCEL_RANGE_16G = 3
};

enum class ImuLpfBandwidthIndex : uint8_t
{
    LPF_256HZ_N_0MS = 0,
    LPF_188HZ_N_2MS = 1,
    LPF_98HZ_N_3MS = 2,
    LPF_42HZ_N_5MS = 3,
    LPF_20HZ_N_10MS = 4,
    LPF_10HZ_N_13MS = 5,
    LPF_5HZ_N_18MS = 6
};

// --- IMU LPF Bandwidth Configuration ---
constexpr const char *KEY_IMU_LPF_BANDWIDTH = "imu.lpf_bw";
constexpr const char *KEY_IMU_DMP_ENABLED = "imu.dmp_enabled";
constexpr uint8_t DEFAULT_IMU_LPF_BANDWIDTH = static_cast<uint8_t>(ImuLpfBandwidthIndex::LPF_256HZ_N_0MS); // Corresponds to LPF_256HZ_N_0MS
constexpr const char *IMU_LPF_BANDWIDTH_STRINGS[] = {
    "LPF_256HZ_N_0MS", "LPF_188HZ_N_2MS", "LPF_98HZ_N_3MS",
    "LPF_42HZ_N_5MS", "LPF_20HZ_N_10MS", "LPF_10HZ_N_13MS", "LPF_5HZ_N_18MS"};
constexpr uint8_t NUM_IMU_LPF_BANDWIDTHS = sizeof(IMU_LPF_BANDWIDTH_STRINGS) / sizeof(IMU_LPF_BANDWIDTH_STRINGS[0]);

// --- Sensor ---
constexpr uint16_t SENSOR_POWER_UP_DELAY_MS = 100;
constexpr float MS_PER_SECOND = 1000.0f;
constexpr float HALF_PI_RADIANS = M_PI / 2.0f;