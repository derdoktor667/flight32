/**
 * @file imu_sensor.h
 * @brief Defines the abstract base class for IMU sensors.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "libraries/ESP32_MPU6050/src/ESP32_MPU6050.h"
#include "imu_config.h"

// A structure to hold IMU data
struct ImuData
{
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temp;
};

// A structure to hold 3-axis data
struct ImuAxisData
{
    float x, y, z;
};

struct ImuQuaternionData
{
    float w, x, y, z;
};
class ImuSensor
{
public:
    virtual ~ImuSensor() = default;

    // Initializes the sensor. Returns true if successful.
    virtual bool begin(uint32_t i2cClockSpeed = MPU6050_I2C_CLOCK_SPEED, bool useDMP = false, GyroRange gyroRange = GYRO_RANGE_250DPS, AccelRange accelRange = ACCEL_RANGE_2G, LpfBandwidth lpf = LPF_256HZ_N_0MS) = 0;

    // Calibrates the sensor.
    virtual void calibrate() = 0;

    // Reads the latest data from the sensor.
    virtual void read() = 0;

    // Gets the most recently read sensor data, protected by a mutex.
    ImuData getData() const
    {
        ImuData temp_data = {};
        if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE)
        {
            temp_data = _data;
            xSemaphoreGive(_data_mutex);
        }
        return temp_data;
    }

    virtual ImuAxisData getGyroscopeOffset() const = 0;
    virtual void setGyroscopeOffset(const ImuAxisData &offset) = 0;
    virtual ImuAxisData getAccelerometerOffset() const = 0;
    virtual void setAccelerometerOffset(const ImuAxisData &offset) = 0;
    virtual ImuQuaternionData getQuaternion() const { return {0, 0, 0, 0}; };

protected:
    ImuData _data = {};
    SemaphoreHandle_t _data_mutex;
};
