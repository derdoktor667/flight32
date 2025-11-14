/**
 * @file pid_task.h
 * @brief Defines the PID calculation task.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include "../scheduler/task_base.h"
#include "../pid/pid_controller.h"
#include "imu_task.h"
#include "motor_task.h"
#include "../config/pid_config.h"

class RxTask;          // Forward declaration
class SettingsManager; // Forward declaration

class PidTask : public TaskBase
{
public:
    PidTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
            ImuTask *imu_task,
            RxTask *rx_task,
            MotorTask *motor_task,
            SettingsManager *settings_manager);

    void setup() override;
    void run() override;

    PidGains getGains(PidAxis axis);
    void setGains(PidAxis axis, PidGains gains);
    void resetToDefaults();

public:
    bool isArmed() const { return _isArmed; }

private:
    ImuTask *_imu_task;
    RxTask *_rx_task;
    MotorTask *_motor_task;
    SettingsManager *_settings_manager;
    bool _isArmed;

    PIDController _pid_roll;
    PIDController _pid_pitch;
    PIDController _pid_yaw;

    void _load_gains();
    void _set_and_save_gains(PidAxis axis, PidGains gains);
};
