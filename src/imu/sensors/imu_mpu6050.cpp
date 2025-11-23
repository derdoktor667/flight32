/**
 * @file imu_mpu6050.cpp
 * @brief Implements the wrapper for the MPU6050 sensor.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include <Arduino.h>
#include "imu_mpu6050.h"
#include "../../config/imu_config.h"
#include "../../com_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <Wire.h>

// Static members initialization
uint16_t ImuMpu6050::_i2c_error_count = 0;

ImuMpu6050::ImuMpu6050() : _sensor(&Wire, MPU6050_I2C_ADDRESS), _mahony_filter(MAHONY_SAMPLE_RATE_HZ) // Initialize Mahony filter with sample rate
{
    _data_mutex = xSemaphoreCreateMutex();
    if (_data_mutex == nullptr)
    {
        com_send_log(ComMessageType::LOG_ERROR, "Failed to create IMU data mutex");
    }
}

ImuMpu6050::~ImuMpu6050()
{
    if (_data_mutex != nullptr)
    {
        vSemaphoreDelete(_data_mutex);
    }
}

bool ImuMpu6050::begin(bool useDMP_unused, ImuGyroRangeIndex gyroRange, ImuAccelRangeIndex accelRange, ImuLpfBandwidthIndex lpf)
{

    Wire.begin(MPU6050_I2C_SDA, MPU6050_I2C_SCL);

    // Initialize the MPU6500
    if (!_sensor.init())
    {
        com_send_log(ComMessageType::LOG_ERROR, "Failed to initialize MPU6500_WE sensor.");
        _i2c_error_count++;
        _is_healthy = false;

        return false;
    }

    // Configure Gyroscope and Accelerometer ranges
    _sensor.setGyrRange(static_cast<MPU9250_gyroRange>(gyroRange));

    _sensor.setAccRange(static_cast<MPU9250_accRange>(accelRange));

    _sensor.setGyrDLPF(static_cast<MPU9250_dlpf>(lpf));
    _sensor.setAccDLPF(static_cast<MPU9250_dlpf>(lpf));

    // DMP initialization is not directly supported by MPU6500_WE/MPU9250_WE for quaternion output.
    // Raw sensor data will be used. No interrupt needed as Mahony filter is updated on task frequency.

    _is_healthy = true;

    return true;
}

void ImuMpu6050::calibrate()
{
    com_send_log(ComMessageType::LOG_INFO, "Calibrating MPU6500_WE...");
    _sensor.autoOffsets(); // Assuming autoOffset() performs both gyro and accel calibration
    com_send_log(ComMessageType::LOG_INFO, "MPU6500_WE Calibration complete.");
}

void ImuMpu6050::read()
{
    // Reading raw sensor data.
    xyzFloat accelData = _sensor.getGValues();  // getGValues for accelerometer
    xyzFloat gyroData = _sensor.getGyrValues(); // getGyrValues for gyroscope

    // Update Mahony filter with raw IMU data
    // MahonyAHRS.updateIMU(gx, gy, gz, ax, ay, az);
    _mahony_filter.updateIMU(gyroData.x, gyroData.y, gyroData.z, accelData.x, accelData.y, accelData.z);

    if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE)
    {
        _data.accelX = static_cast<int16_t>(accelData.x * ACCEL_G_TO_MG_FACTOR); // Convert G to mg
        _data.accelY = static_cast<int16_t>(accelData.y * ACCEL_G_TO_MG_FACTOR);
        _data.accelZ = static_cast<int16_t>(accelData.z * ACCEL_G_TO_MG_FACTOR);
        _data.gyroX = static_cast<int16_t>(gyroData.x * GYRO_DEGS_TO_MDEGS_FACTOR); // Convert deg/s to mdeg/s
        _data.gyroY = static_cast<int16_t>(gyroData.y * GYRO_DEGS_TO_MDEGS_FACTOR);
        _data.gyroZ = static_cast<int16_t>(gyroData.z * GYRO_DEGS_TO_MDEGS_FACTOR);

        // Retrieve quaternions from Mahony filter
        _quaternion.w = _mahony_filter.q4;
        _quaternion.x = _mahony_filter.q5;
        _quaternion.y = _mahony_filter.q6;
        _quaternion.z = _mahony_filter.q7;

        xSemaphoreGive(_data_mutex);
    }
}

ImuAxisData ImuMpu6050::getGyroscopeOffset()
{
    xyzFloat gyrOffset = _sensor.getGyrOffsets(); // Assuming getGyrOffsets returns xyzFloat
    return {(float)gyrOffset.x,
            (float)gyrOffset.y,
            (float)gyrOffset.z};
}

void ImuMpu6050::setGyroscopeOffset(const ImuAxisData &offset)
{
    xyzFloat gyrOffset = {offset.x, offset.y, offset.z}; // Create xyzFloat from ImuAxisData
    _sensor.setGyrOffsets(gyrOffset);                    // Assuming setGyrOffsets takes xyzFloat
}

ImuAxisData ImuMpu6050::getAccelerometerOffset()
{
    xyzFloat accOffset = _sensor.getAccOffsets(); // Assuming getAccOffsets returns xyzFloat
    return {(float)accOffset.x,
            (float)accOffset.y,
            (float)accOffset.z};
}

void ImuMpu6050::setAccelerometerOffset(const ImuAxisData &offset)
{
    xyzFloat accOffset = {offset.x, offset.y, offset.z}; // Create xyzFloat from ImuAxisData
    _sensor.setAccOffsets(accOffset);                    // Assuming setAccOffsets takes xyzFloat
}

ImuQuaternionData ImuMpu6050::getQuaternion() const
{
    ImuQuaternionData temp_quat = {0, 0, 0, 0};
    if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE)
    {
        temp_quat = _quaternion;
        xSemaphoreGive(_data_mutex);
    }
    return temp_quat;
}