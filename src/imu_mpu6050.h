/**
 * @file imu_mpu6050.h
 * @brief Defines the wrapper for the MPU6050 sensor.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include "imu_sensor.h"
#include "config.h"
#include <ESP32_MPU6050.h>

LpfBandwidth get_lpf_bandwidth_from_index(uint8_t index);

class ImuMpu6050 : public ImuSensor
{
public:
    ImuMpu6050();
    ~ImuMpu6050();
        bool begin(uint32_t i2cClockSpeed = MPU6050_I2C_CLOCK_SPEED, bool useDMP = false, GyroRange gyroRange = GYRO_RANGE_250DPS, AccelRange accelRange = ACCEL_RANGE_2G, LpfBandwidth lpf = LPF_256HZ_N_0MS) override;
    void calibrate() override;
    void read() override;

    ImuAxisData getGyroscopeOffset() const override;
    void setGyroscopeOffset(const ImuAxisData &offset) override;
    ImuAxisData getAccelerometerOffset() const override;
    void setAccelerometerOffset(const ImuAxisData &offset) override;
    ImuQuaternionData getQuaternion() const override;

private:
    bool _useDMP;
    ImuQuaternionData _quaternion;
    ESP32_MPU6050 _sensor;
};
