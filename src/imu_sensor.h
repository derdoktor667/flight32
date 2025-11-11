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

class ImuSensor
{
public:
    virtual ~ImuSensor() = default;

    // Initializes the sensor.
    // Returns true if initialization was successful, false otherwise.
    virtual bool begin() = 0;

    // Calibrates the sensor (e.g., gyro).
    virtual void calibrate() = 0;

    // Reads the latest data from the sensor.
    virtual void read() = 0;

    // Gets the most recently read sensor data.
    ImuData getData() const {
        ImuData temp_data = {};
        if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE) {
            temp_data = _data;
            xSemaphoreGive(_data_mutex);
        }
        return temp_data;
    }

    virtual ImuAxisData getGyroscopeOffset() const = 0;
    virtual void setGyroscopeOffset(const ImuAxisData &offset) = 0;
    virtual ImuAxisData getAccelerometerOffset() const = 0;
    virtual void setAccelerometerOffset(const ImuAxisData &offset) = 0;

protected:
    ImuData _data = {};
    SemaphoreHandle_t _data_mutex;
};
