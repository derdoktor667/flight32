/**
 * @file flight_controller.h
 * @brief Defines the FlightController class, managing all core components and tasks.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include "scheduler/scheduler.h"
#include "com_manager.h"
#include "tasks/imu_task.h"
#include "imu/imu_sensor.h"
#include "tasks/rx_task.h"
#include "protocols/rx_protocol.h"
#include "protocols/rx_ibus_protocol.h"
#include "tasks/serial_manager_task.h"
#include "tasks/motor_task.h"
#include "tasks/pid_task.h"

#include "settings_manager.h"

class FlightController
{
public:
    FlightController() = default;
    ~FlightController();
    void setup();

private:
    SettingsManager _settings_manager;
    Scheduler _scheduler;
    ImuSensor *_imu_sensor = nullptr;
    ImuTask *_imu_task = nullptr;
    RxTask *_rx_task = nullptr;
    SerialManagerTask *_serial_manager_task = nullptr;
    MotorTask *_motor_task = nullptr;
    PidTask *_pid_task = nullptr;
};