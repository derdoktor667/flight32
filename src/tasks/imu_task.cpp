/**
 * @file imu_task.cpp
 * @brief Implements the ImuTask for reading and updating IMU sensor data.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "imu_task.h"
#include "../config/imu_config.h"
#include "../config/filter_config.h"
#include "../com_manager.h"
#include "../settings_manager.h"

ImuTask::ImuTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms, ImuSensor &imu_sensor, SettingsManager *settings_manager)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms), _imu_sensor(imu_sensor), _settings_manager(settings_manager) {}

void ImuTask::setup()
{
    float sample_freq = MS_PER_SECOND / IMU_TASK_DELAY_MS; // Assuming IMU_TASK_DELAY_MS is in ms

    // Get filter settings from SettingsManager
    float gyro_lpf_hz = _settings_manager->getFloat(NVS_KEY_GYRO_LPF_HZ);
    float notch1_hz = _settings_manager->getFloat(NVS_KEY_NOTCH1_HZ);
    float notch1_q = _settings_manager->getFloat(NVS_KEY_NOTCH1_Q);
    float notch2_hz = _settings_manager->getFloat(NVS_KEY_NOTCH2_HZ);
    float notch2_q = _settings_manager->getFloat(NVS_KEY_NOTCH2_Q);

    for (int i = 0; i < 3; ++i)
    {
        _gyro_lpf[i].configureLowpass(sample_freq, gyro_lpf_hz);
        _gyro_notch1[i].configureNotch(sample_freq, notch1_hz, notch1_q);
        _gyro_notch2[i].configureNotch(sample_freq, notch2_hz, notch2_q);
    }
}

void ImuTask::run()
{
    _imu_sensor.read(); // Read raw data from the sensor

    // Get raw IMU data
    ImuData raw_imu_data = _imu_sensor.getData();

    // Copy accelerometer and temperature data directly
    _filtered_imu_data.accelX = raw_imu_data.accelX;
    _filtered_imu_data.accelY = raw_imu_data.accelY;
    _filtered_imu_data.accelZ = raw_imu_data.accelZ;
    _filtered_imu_data.temp = raw_imu_data.temp;

    // Apply filters to gyro data
    _filtered_imu_data.gyroX = _gyro_notch2[0].apply(_gyro_notch1[0].apply(_gyro_lpf[0].apply(raw_imu_data.gyroX)));
    _filtered_imu_data.gyroY = _gyro_notch2[1].apply(_gyro_notch1[1].apply(_gyro_lpf[1].apply(raw_imu_data.gyroY)));
    _filtered_imu_data.gyroZ = _gyro_notch2[2].apply(_gyro_notch1[2].apply(_gyro_lpf[2].apply(raw_imu_data.gyroZ)));
}