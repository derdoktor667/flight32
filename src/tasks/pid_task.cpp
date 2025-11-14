#include "pid_task.h"
#include "../imu/imu_sensor.h"
#include "../protocols/rx_protocol.h"
#include "../config/rx_config.h"
#include "../settings_manager.h"
#include "../config/pid_config.h"
#include "../config/terminal_config.h"
#include "../com_manager.h"
#include "motor_task.h"
#include "imu_task.h"
#include "rx_task.h"

#include <Arduino.h> // For constrain()
#include <stdint.h>  // For uint32_t

PidTask::PidTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
                 ImuTask *imu_task,
                 RxTask *rx_task,
                 MotorTask *motor_task,
                 SettingsManager *settings_manager)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms),
      _imu_task(imu_task),
      _rx_task(rx_task),
      _motor_task(motor_task),
      _settings_manager(settings_manager),
      _pid_roll(DEFAULT_PID_ROLL_P, DEFAULT_PID_ROLL_I, DEFAULT_PID_ROLL_D),
      _pid_pitch(DEFAULT_PID_PITCH_P, DEFAULT_PID_PITCH_I, DEFAULT_PID_PITCH_D),
      _pid_yaw(DEFAULT_PID_YAW_P, DEFAULT_PID_YAW_I, DEFAULT_PID_YAW_D),
      _isArmed(false)
{
}

void PidTask::setup()
{
    _load_gains();
}

void PidTask::run()
{
    int constrained_roll = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_ROLL).toInt()), RC_CHANNEL_MIN_RAW, RC_CHANNEL_MAX_RAW);
    int constrained_pitch = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_PITCH).toInt()), RC_CHANNEL_MIN_RAW, RC_CHANNEL_MAX_RAW);
    int constrained_yaw = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_YAW).toInt()), RC_CHANNEL_MIN_RAW, RC_CHANNEL_MAX_RAW);
    int constrained_throttle = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_THRO).toInt()), RC_CHANNEL_MIN_RAW, RC_CHANNEL_MAX_RAW);
    int arm_channel_value = _rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_ARM).toInt());

    _isArmed = (arm_channel_value > RC_CHANNEL_ARM_THRESHOLD);

    float desired_roll_rate = (constrained_roll - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
    float desired_pitch_rate = (constrained_pitch - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
    float desired_yaw_rate = (constrained_yaw - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
    float throttle = (constrained_throttle - RC_CHANNEL_MIN_RAW) / RC_CHANNEL_RANGE_THROTTLE;

    const ImuData &imu_data = _imu_task->getImuSensor().getData();
    float actual_roll_rate = imu_data.gyroX;
    float actual_pitch_rate = imu_data.gyroY;
    float actual_yaw_rate = imu_data.gyroZ;

    float dt = getTaskDelayMs() / MS_TO_SECONDS_FACTOR;

    float roll_output = _pid_roll.update(desired_roll_rate, actual_roll_rate, dt);
    float pitch_output = _pid_pitch.update(desired_pitch_rate, actual_pitch_rate, dt);
    float yaw_output = _pid_yaw.update(desired_yaw_rate, actual_yaw_rate, dt);

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
        return {0.0f, 0.0f, 0.0f};
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
    const char *p_key, *i_key, *d_key;

    switch (axis)
    {
    case PidAxis::ROLL:
        _pid_roll.setGains(gains.p, gains.i, gains.d);
        p_key = KEY_PID_ROLL_P;
        i_key = KEY_PID_ROLL_I;
        d_key = KEY_PID_ROLL_D;
        break;
    case PidAxis::PITCH:
        _pid_pitch.setGains(gains.p, gains.i, gains.d);
        p_key = KEY_PID_PITCH_P;
        i_key = KEY_PID_PITCH_I;
        d_key = KEY_PID_PITCH_D;
        break;
    case PidAxis::YAW:
        _pid_yaw.setGains(gains.p, gains.i, gains.d);
        p_key = KEY_PID_YAW_P;
        i_key = KEY_PID_YAW_I;
        d_key = KEY_PID_YAW_D;
        break;
    default:
        return;
    }

    _settings_manager->setSettingValue(p_key, String(gains.p));
    _settings_manager->setSettingValue(i_key, String(gains.i));
    _settings_manager->setSettingValue(d_key, String(gains.d));
}