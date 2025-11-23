/**
 * @file imu_sensor.h
 * @brief Defines the abstract base class for IMU sensors.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "../config/imu_config.h"

// A structure to hold IMU data
struct ImuData
{
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float angleX, angleY; // New: Roll and Pitch angles
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
    virtual bool begin(bool useDMP = false, ImuGyroRangeIndex gyroRange = ImuGyroRangeIndex::GYRO_RANGE_250DPS, ImuAccelRangeIndex accelRange = ImuAccelRangeIndex::ACCEL_RANGE_2G, ImuLpfBandwidthIndex lpf = ImuLpfBandwidthIndex::LPF_256HZ_N_0MS) = 0;

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

    virtual ImuAxisData getGyroscopeOffset() = 0; // Removed const
    virtual void setGyroscopeOffset(const ImuAxisData &offset) = 0;
    virtual ImuAxisData getAccelerometerOffset() = 0; // Removed const
    virtual void setAccelerometerOffset(const ImuAxisData &offset) = 0;
    virtual ImuQuaternionData getQuaternion() const { return {0, 0, 0, 0}; };
    virtual uint16_t getI2CErrorCount() const = 0;
    virtual bool isSensorHealthy() const = 0;

protected:
    ImuData _data = {};
    SemaphoreHandle_t _data_mutex;
};