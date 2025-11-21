/**
 * @file imu_mpu6050.h
 * @brief Defines the wrapper for the MPU6050 sensor.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include "../imu_sensor.h"
#include "../../config/imu_config.h"

#include <MPU6500_WE.h>
#include "../../utils/MahonyAHRS.h"

class ImuMpu6050 : public ImuSensor
{
public:
    ImuMpu6050();
    ~ImuMpu6050();
    bool begin(uint32_t i2cClockSpeed = MPU6050_I2C_CLOCK_SPEED, bool useDMP = false, ImuGyroRangeIndex gyroRange = ImuGyroRangeIndex::GYRO_RANGE_250DPS, ImuAccelRangeIndex accelRange = ImuAccelRangeIndex::ACCEL_RANGE_2G, ImuLpfBandwidthIndex lpf = ImuLpfBandwidthIndex::LPF_256HZ_N_0MS) override;
    void calibrate() override;
    void read() override;

    ImuAxisData getGyroscopeOffset() override; // Removed const to match base class
    void setGyroscopeOffset(const ImuAxisData &offset) override;
    ImuAxisData getAccelerometerOffset() override; // Removed const to match base class
    void setAccelerometerOffset(const ImuAxisData &offset) override;
    ImuQuaternionData getQuaternion() const override;

    // Removed static LpfBandwidth getLpfBandwidthFromIndex(uint8_t index);
    uint16_t getI2CErrorCount() const override { return _i2c_error_count; }
    bool isSensorHealthy() const override { return _is_healthy; }

    // Static member and ISR for interrupt handling
    static volatile bool dmp_data_ready;
    static void dmp_isr();

private:
    MPU6500_WE _sensor; // Changed type from ESP32_MPU6050 to MPU6050 (from I2Cdevlib)
    ImuQuaternionData _quaternion;
    static uint16_t _i2c_error_count;
    bool _is_healthy = false;

    MahonyAHRS _mahony_filter; // Mahony filter instance
};
