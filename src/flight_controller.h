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
#include "tasks/mpu6050_task.h"
#include "tasks/ibus_task.h"
#include "tasks/terminal_task.h"
#include "tasks/motor_task.h"
#include "tasks/pid_task.h"

#include "settings_manager.h"

// --- Flight Controller Class ---
class FlightController
{
public:
    FlightController() = default;
    ~FlightController();
    void setup();

private:
    SettingsManager _settings_manager;
    Scheduler _scheduler;
    ESP32_MPU6050 _mpu6050_sensor;
    Mpu6050Task *_mpu6050_task = nullptr;
    IbusTask *_ibus_receiver_task = nullptr;
    TerminalTask *_terminal_task = nullptr;
    MotorTask *_motor_task = nullptr;
    PidTask *_pid_task = nullptr;
};