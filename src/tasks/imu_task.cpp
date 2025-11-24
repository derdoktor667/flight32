/**
 * @file imu_task.cpp
 * @brief Implements the ImuTask for reading and updating IMU sensor data.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "imu_task.h"
#include "../config/imu_config.h"
#include "../config/filter_config.h"

#include "../settings_manager.h"
#include "../utils/system_constants.h"


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

    for (int i = 0; i < NUM_AXES; ++i)
    {
        _gyro_lpf[i].configureLowpass(sample_freq, gyro_lpf_hz);
        _gyro_notch1[i].configureNotch(sample_freq, notch1_hz, notch1_q);
        _gyro_notch2[i].configureNotch(sample_freq, notch2_hz, notch2_q);
    }
}

void ImuTask::run()
{
    _imu_sensor.read(); // This will now update _quaternion and _data in ImuMpu6050

    // Get raw IMU data. This assumes ImuMpu6050::read() has updated _data.
    // However, ImuMpu6050::read() only updates _quaternion from DMP FIFO currently.
    // So, if needed, _filtered_imu_data should get accel/gyro from a separate non-DMP reading
    // or from DMP's accel/gyro output if available (and if _data was updated there).
    
    // For now, let's assume _imu_sensor's _data member is correctly updated
    // or we'll need to fetch raw accel/gyro explicitly if required for filtering here.
    // The previous implementation used _imu_sensor.getData() for raw_imu_data.
    // We need to ensure that ImuMpu6050::read() populates _data as well.

    ImuData raw_imu_data = _imu_sensor.getData(); // Get data from the sensor (now _data member of ImuMpu6050)

    // Copy accelerometer and temperature data directly
    _filtered_imu_data.accelX = raw_imu_data.accelX;
    _filtered_imu_data.accelY = raw_imu_data.accelY;
    _filtered_imu_data.accelZ = raw_imu_data.accelZ;
    _filtered_imu_data.temp = raw_imu_data.temp;

    // Apply filters to gyro data
    float raw_gyros[NUM_AXES] = {raw_imu_data.gyroX, raw_imu_data.gyroY, raw_imu_data.gyroZ};
    float filtered_gyros[NUM_AXES];
    for (int i = 0; i < NUM_AXES; ++i)
    {
        filtered_gyros[i] = _gyro_notch2[i].apply(_gyro_notch1[i].apply(_gyro_lpf[i].apply(raw_gyros[i])));
    }
    _filtered_imu_data.gyroX = filtered_gyros[0];
    _filtered_imu_data.gyroY = filtered_gyros[1];
    _filtered_imu_data.gyroZ = filtered_gyros[2];
}