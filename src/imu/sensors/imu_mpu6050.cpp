/**
 * @file imu_mpu6050.cpp
 * @brief Implements the wrapper for the MPU6050 sensor.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "imu_mpu6050.h"
#include "../../config/imu_config.h"
#include "../../com_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

uint16_t ImuMpu6050::_i2c_error_count = 0;

ImuMpu6050::ImuMpu6050() : _sensor()
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

bool ImuMpu6050::begin(uint32_t i2cClockSpeed, bool useDMP, ImuGyroRangeIndex gyroRange, ImuAccelRangeIndex accelRange, ImuLpfBandwidthIndex lpf)
{
    _useDMP = useDMP;
    if (!_sensor.begin(static_cast<GyroRange>(gyroRange), static_cast<AccelRange>(accelRange), static_cast<LpfBandwidth>(lpf), i2cClockSpeed, MPU6050_I2C_ADDRESS))
    {
        com_send_log(ComMessageType::LOG_ERROR, "Failed to find MPU6050 chip");
        _i2c_error_count++;
        _is_healthy = false;
        return false;
    }

    if (_useDMP)
    {
        com_send_log(ComMessageType::LOG_INFO, "Initializing MPU6050 DMP...");
        uint8_t dmpStatus = _sensor.dmpInitialize();
        if (dmpStatus != 0)
        {
            com_send_log(ComMessageType::LOG_ERROR, "Failed to initialize DMP. DMP Status: %d", dmpStatus);
            _i2c_error_count++;
            _is_healthy = false;
            return false;
        }
        com_send_log(ComMessageType::LOG_INFO, "MPU6050 DMP initialized.");
    }
    com_send_log(ComMessageType::LOG_INFO, "MPU6050 Found!");
    _is_healthy = true;
    return true;
}

void ImuMpu6050::calibrate()
{
    com_send_log(ComMessageType::LOG_INFO, "Calibrating MPU6050...");
    _sensor.calibrate();
    com_send_log(ComMessageType::LOG_INFO, "MPU6050 Calibration complete.");
}

void ImuMpu6050::read()
{
    if (_useDMP)
    {
        // Read DMP FIFO
        uint8_t fifoBuffer[MPU6050_FIFO_BUFFER_SIZE]; // MPU6050_DMP_PACKET_SIZE is 28, so 128 is enough for multiple packets
        Quaternion q;

        if (_sensor.dmpPacketAvailable())
        {
            if (_sensor.dmpGetCurrentFIFOPacket(fifoBuffer) == 0)
            {
                _sensor.dmpGetQuaternion(&q, fifoBuffer);

                if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE)
                {
                    _quaternion.w = q.w;
                    _quaternion.x = q.x;
                    _quaternion.y = q.y;
                    _quaternion.z = q.z;
                    // For now, we are not populating accel/gyro/temp from DMP output directly into _data
                    // If needed, dmpGetAccel and dmpGetGyro can be used here.
                    xSemaphoreGive(_data_mutex);
                }
            }
        }
    }
    else
    {
        _sensor.update();

        if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE)
        {
            _data.accelX = _sensor.readings.accelerometer.x;
            _data.accelY = _sensor.readings.accelerometer.y;
            _data.accelZ = _sensor.readings.accelerometer.z;
            _data.gyroX = _sensor.readings.gyroscope.x;
            _data.gyroY = _sensor.readings.gyroscope.y;
            _data.gyroZ = _sensor.readings.gyroscope.z;
            _data.temp = _sensor.readings.temperature_celsius;
            xSemaphoreGive(_data_mutex);
        }
    }
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
