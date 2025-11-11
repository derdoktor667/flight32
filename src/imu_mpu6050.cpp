/**
 * @file imu_mpu6050.cpp
 * @brief Implements the wrapper for the MPU6050 sensor.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "imu_mpu6050.h"
#include "com_manager.h"

ImuMpu6050::ImuMpu6050() : _sensor()
{
}

bool ImuMpu6050::begin()
{
    if (!_sensor.begin((GyroRange)DEFAULT_GYRO_RANGE, AccelRange::ACCEL_RANGE_2G, LpfBandwidth::LPF_188HZ_N_2MS, 0x68))
    {
        com_send_log(LOG_ERROR, "Failed to find MPU6050 chip");
        return false;
    }
    com_send_log(LOG_INFO, "MPU6050 Found!");
    return true;
}

void ImuMpu6050::calibrate()
{
    com_send_log(LOG_INFO, "Calibrating MPU6050...");
    _sensor.calibrate();
    com_send_log(LOG_INFO, "MPU6050 Calibration complete.");
}

void ImuMpu6050::read()
{
    _sensor.update();
    _data.accelX = _sensor.readings.accelerometer.x;
    _data.accelY = _sensor.readings.accelerometer.y;
    _data.accelZ = _sensor.readings.accelerometer.z;
    _data.gyroX = _sensor.readings.gyroscope.x;
    _data.gyroY = _sensor.readings.gyroscope.y;
    _data.gyroZ = _sensor.readings.gyroscope.z;
    _data.temp = _sensor.readings.temperature_celsius;
}

ImuAxisData ImuMpu6050::getGyroscopeOffset() const
{
    AxisData offset = _sensor.getGyroscopeOffset();
    return {offset.x, offset.y, offset.z};
}

void ImuMpu6050::setGyroscopeOffset(const ImuAxisData &offset)
{
    _sensor.setGyroscopeOffset({offset.x, offset.y, offset.z});
}

ImuAxisData ImuMpu6050::getAccelerometerOffset() const
{
    AxisData offset = _sensor.getAccelerometerOffset();
    return {offset.x, offset.y, offset.z};
}

void ImuMpu6050::setAccelerometerOffset(const ImuAxisData &offset)
{
    _sensor.setAccelerometerOffset({offset.x, offset.y, offset.z});
}
