/**
 * @file imu_task.cpp
 * @brief Implements the ImuTask for reading and updating IMU sensor data.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "imu_task.h"
#include "../com_manager.h"

ImuTask::ImuTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms, ImuSensor &imu_sensor)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms), _imu_sensor(imu_sensor) {}

void ImuTask::setup()
{
    // Nothing to do here, IMU is initialized in FlightController
}

void ImuTask::run()
{
    _imu_sensor.read();
}