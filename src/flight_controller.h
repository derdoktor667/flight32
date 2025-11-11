/**
 * @file flight_controller.h
 * @brief Defines the FlightController class, managing all core components and tasks.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include "scheduler/scheduler.h"
#include "com_manager.h"
#include "tasks/imu_task.h"
#include "imu_sensor.h"
#include "tasks/rx_task.h"
#include "rx_protocol.h"
#include "rx_ibus_protocol.h"
#include "tasks/terminal_task.h"
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
    TerminalTask *_terminal_task = nullptr;
    MotorTask *_motor_task = nullptr;
    PidTask *_pid_task = nullptr;
};