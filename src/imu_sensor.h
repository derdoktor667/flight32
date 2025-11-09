/**
 * @file imu_sensor.h
 * @brief Defines the abstract base class for IMU sensors.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <cstdint>

// A structure to hold IMU data
struct ImuData
{
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temp;
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
    const ImuData &getData() const { return _data; }

protected:
    ImuData _data = {};
};
