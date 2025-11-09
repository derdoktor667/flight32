/**
 * @file imu_task.h
 * @brief Defines the ImuTask class for reading data from a generic IMU sensor.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include "../scheduler/task_base.h"
#include "../imu_sensor.h"

class ImuTask : public TaskBase
{
public:
    ImuTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms, ImuSensor &imu_sensor);
    void setup() override;
    void run() override;

    ImuSensor &getImuSensor() { return _imu_sensor; }

private:
    ImuSensor &_imu_sensor;
};