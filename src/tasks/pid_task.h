#pragma once

#include "../scheduler/task_base.h"
#include "../pid/pid_controller.h"
#include "mpu6050_task.h"
#include "ibus_task.h"
#include "motor_task.h"
#include "../settings_manager.h"
#include "../config.h"

class PidTask : public TaskBase
{
public:
    PidTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
            Mpu6050Task *mpu6050_task,
            IbusTask *ibus_task,
            MotorTask *motor_task,
            SettingsManager *settings_manager);

    void setup() override;
    void run() override;

    PidGains getGains(PidAxis axis);
    void setGains(PidAxis axis, PidGains gains);

private:
    Mpu6050Task *_mpu6050_task;
    IbusTask *_ibus_task;
    MotorTask *_motor_task;
    SettingsManager *_settings_manager;

    PIDController _pid_roll;
    PIDController _pid_pitch;
    PIDController _pid_yaw;

    // Helper to load gains from settings
    void _load_gains();
};
