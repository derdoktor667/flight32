/**
 * @file pid_task.cpp
 * @brief Implements the PID calculation task.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "pid_task.h"
#include "../com_manager.h"
#include "../rc_config.h"

PidTask::PidTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
                 Mpu6050Task *mpu6050_task,
                 RxTask *rx_task,
                 MotorTask *motor_task,
                 SettingsManager *settings_manager)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms),
      _mpu6050_task(mpu6050_task),
      _rx_task(rx_task),
      _motor_task(motor_task),
      _settings_manager(settings_manager),
      _pid_roll(0, 0, 0),
      _pid_pitch(0, 0, 0),
      _pid_yaw(0, 0, 0)
{
}

void PidTask::setup()
{
    _load_gains();
    com_send_log(LOG_INFO, "PidTask: Initialized.");
}

void PidTask::run()
{
    // Get the desired setpoints from the receiver
    // Raw IBUS values can vary (e.g., 988-2024). For internal calculations, we constrain them
    // to the standard 1000-2000 range to ensure stable flight control.
    // The terminal will still display the raw, unconstrained values for user calibration.
    int constrained_roll = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(SettingsManager::KEY_IBUS_CHANNEL_ROLL).toInt()), IBUS_CHANNEL_MIN_RAW, IBUS_CHANNEL_MAX_RAW);
    int constrained_pitch = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(SettingsManager::KEY_IBUS_CHANNEL_PITCH).toInt()), IBUS_CHANNEL_MIN_RAW, IBUS_CHANNEL_MAX_RAW);
    int constrained_yaw = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(SettingsManager::KEY_IBUS_CHANNEL_YAW).toInt()), IBUS_CHANNEL_MIN_RAW, IBUS_CHANNEL_MAX_RAW);
    int constrained_throttle = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(SettingsManager::KEY_IBUS_CHANNEL_THRO).toInt()), IBUS_CHANNEL_MIN_RAW, IBUS_CHANNEL_MAX_RAW);

    float desired_roll_rate = (constrained_roll - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
    float desired_pitch_rate = (constrained_pitch - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
    float desired_yaw_rate = (constrained_yaw - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
    float throttle = (constrained_throttle - RC_CHANNEL_MIN) / RC_CHANNEL_RANGE_THROTTLE;

    float actual_roll_rate = _mpu6050_task->getMpu6050Sensor().readings.gyroscope.x;
    float actual_pitch_rate = _mpu6050_task->getMpu6050Sensor().readings.gyroscope.y;
    float actual_yaw_rate = _mpu6050_task->getMpu6050Sensor().readings.gyroscope.z;

    float dt = getTaskDelayMs() / MS_TO_SECONDS_FACTOR;

    // Calculate PID outputs
    float roll_output = _pid_roll.update(desired_roll_rate, actual_roll_rate, dt);
    float pitch_output = _pid_pitch.update(desired_pitch_rate, actual_pitch_rate, dt);
    float yaw_output = _pid_yaw.update(desired_yaw_rate, actual_yaw_rate, dt);

    // Update the motor task with the new commands
    _motor_task->update(throttle, pitch_output, roll_output, yaw_output);
}

PidGains PidTask::getGains(PidAxis axis)
{
    switch (axis)
    {
    case PidAxis::ROLL:
        return _pid_roll.getGains();
    case PidAxis::PITCH:
        return _pid_pitch.getGains();
    case PidAxis::YAW:
        return _pid_yaw.getGains();
    default:
        return {0.0f, 0.0f, 0.0f}; // Should not happen
    }
}

void PidTask::setGains(PidAxis axis, PidGains gains)
{
    _set_and_save_gains(axis, gains);
}

void PidTask::_load_gains()
{
    _pid_roll.setGains(
        _settings_manager->getSettingValue(KEY_PID_ROLL_P).toFloat(),
        _settings_manager->getSettingValue(KEY_PID_ROLL_I).toFloat(),
        _settings_manager->getSettingValue(KEY_PID_ROLL_D).toFloat());

    _pid_pitch.setGains(
        _settings_manager->getSettingValue(KEY_PID_PITCH_P).toFloat(),
        _settings_manager->getSettingValue(KEY_PID_PITCH_I).toFloat(),
        _settings_manager->getSettingValue(KEY_PID_PITCH_D).toFloat());

    _pid_yaw.setGains(
        _settings_manager->getSettingValue(KEY_PID_YAW_P).toFloat(),
        _settings_manager->getSettingValue(KEY_PID_YAW_I).toFloat(),
        _settings_manager->getSettingValue(KEY_PID_YAW_D).toFloat());

    com_send_log(LOG_INFO, "PidTask: Loaded PID gains from settings.");
}

void PidTask::resetToDefaults()
{
    _set_and_save_gains(PidAxis::ROLL, {DEFAULT_PID_ROLL_P, DEFAULT_PID_ROLL_I, DEFAULT_PID_ROLL_D});
    _set_and_save_gains(PidAxis::PITCH, {DEFAULT_PID_PITCH_P, DEFAULT_PID_PITCH_I, DEFAULT_PID_PITCH_D});
    _set_and_save_gains(PidAxis::YAW, {DEFAULT_PID_YAW_P, DEFAULT_PID_YAW_I, DEFAULT_PID_YAW_D});

    _settings_manager->saveSettings();
    com_send_log(LOG_INFO, "PidTask: Reset PID gains to defaults and saved.");
}

void PidTask::_set_and_save_gains(PidAxis axis, PidGains gains)
{
    switch (axis)
    {
    case PidAxis::ROLL:
        _pid_roll.setGains(gains.p, gains.i, gains.d);
        _settings_manager->setSettingValue(KEY_PID_ROLL_P, String(gains.p));
        _settings_manager->setSettingValue(KEY_PID_ROLL_I, String(gains.i));
        _settings_manager->setSettingValue(KEY_PID_ROLL_D, String(gains.d));
        break;
    case PidAxis::PITCH:
        _pid_pitch.setGains(gains.p, gains.i, gains.d);
        _settings_manager->setSettingValue(KEY_PID_PITCH_P, String(gains.p));
        _settings_manager->setSettingValue(KEY_PID_PITCH_I, String(gains.i));
        _settings_manager->setSettingValue(KEY_PID_PITCH_D, String(gains.d));
        break;
    case PidAxis::YAW:
        _pid_yaw.setGains(gains.p, gains.i, gains.d);
        _settings_manager->setSettingValue(KEY_PID_YAW_P, String(gains.p));
        _settings_manager->setSettingValue(KEY_PID_YAW_I, String(gains.i));
        _settings_manager->setSettingValue(KEY_PID_YAW_D, String(gains.d));
        break;
    }
}