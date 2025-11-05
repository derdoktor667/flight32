#pragma once

#include "scheduler/scheduler.h"
#include "com_manager.h"
#include "tasks/mpu6050_task.h"
#include "tasks/ibus_task.h"
#include "tasks/terminal_task.h"
#include "tasks/motor_task.h"

// --- Flight Controller Class ---
class FlightController
{
public:
    FlightController();
    void setup();

private:
    Scheduler _scheduler;
    ESP32_MPU6050 _mpu6050_sensor;
    Mpu6050Task _mpu6050_task;
    IbusTask _ibus_receiver_task;
    TerminalTask _terminal_task;
    MotorTask _motor_task;
};