/**
 * @file imu_task.h
 * @brief Defines the ImuTask class for reading data from a generic IMU sensor.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include "../scheduler/task_base.h"
#include "../imu/imu_sensor.h"
#include "../config/imu_config.h"
#include "../settings_manager.h"
#include "../utils/filter.h"

class ImuTask : public TaskBase
{
public:
    ImuTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms, ImuSensor &imu_sensor, SettingsManager *settings_manager);
    void setup() override;
    void run() override;

    ImuSensor &getImuSensor() { return _imu_sensor; }
    ImuData getFilteredImuData() const { return _filtered_imu_data; }

private:
    ImuSensor &_imu_sensor;
    SettingsManager *_settings_manager;
    BiquadFilter _gyro_lpf[3];    // Low-pass filter for gyro (X, Y, Z)
    BiquadFilter _gyro_notch1[3]; // First notch filter for gyro (X, Y, Z)
    BiquadFilter _gyro_notch2[3]; // Second notch filter for gyro (X, Y, Z)

    ImuData _filtered_imu_data; // Stores filtered IMU data
};
