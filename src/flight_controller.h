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
#include "tasks/com_manager_task.h"

#include "settings_manager.h"
#include "utils/system_state.h"
#include <memory>

class FlightController
{
public:
    FlightController();

    void setup();

    SystemState getSystemState() const;

private:
    void setSystemState(SystemState new_state);

    SystemState _system_state;
    SettingsManager _settings_manager;
    Scheduler _scheduler;
    std::unique_ptr<ImuSensor> _imu_sensor;
    std::unique_ptr<ImuTask> _imu_task;
    std::unique_ptr<RxTask> _rx_task;
    std::unique_ptr<SerialManagerTask> _serial_manager_task;
    std::unique_ptr<MotorTask> _motor_task;
    std::unique_ptr<PidTask> _pid_task;
    std::unique_ptr<ComManagerTask> _com_manager_task;
};
