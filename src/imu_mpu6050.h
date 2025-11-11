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

class ImuMpu6050 : public ImuSensor
{
public:
    ImuMpu6050();
    bool begin() override;
    void calibrate() override;
    void read() override;

    ImuAxisData getGyroscopeOffset() const override;
    void setGyroscopeOffset(const ImuAxisData &offset) override;
    ImuAxisData getAccelerometerOffset() const override;
    void setAccelerometerOffset(const ImuAxisData &offset) override;

private:
    ESP32_MPU6050 _sensor;
};
