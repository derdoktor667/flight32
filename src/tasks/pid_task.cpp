/**
 * @file pid_task.cpp
 * @brief Implements the PidTask for PID control loop execution.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

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

#include <Arduino.h>
#include <cstdint>
#include "../utils/math_constants.h"

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
      _pid_roll(PidConfig::DEFAULT_RATE_P, PidConfig::DEFAULT_RATE_I, PidConfig::DEFAULT_RATE_D),
      _pid_pitch(PidConfig::DEFAULT_RATE_P, PidConfig::DEFAULT_RATE_I, PidConfig::DEFAULT_RATE_D),
      _pid_yaw(PidConfig::DEFAULT_YAW_P, PidConfig::DEFAULT_YAW_I, PidConfig::DEFAULT_YAW_D),
      _pid_angle_roll(PidConfig::DEFAULT_ANGLE_P, PidConfig::DEFAULT_ANGLE_I, (float)PidConfig::ANGLE_D_GAIN_DISABLED),
      _pid_angle_pitch(PidConfig::DEFAULT_ANGLE_P, PidConfig::DEFAULT_ANGLE_I, (float)PidConfig::ANGLE_D_GAIN_DISABLED),
      _isArmed(false),
      _flight_mode(FlightMode::ACRO)
{
}

void PidTask::setup()
{
    _load_gains();
    com_send_log(ComMessageType::LOG_INFO, "PidTask: Initial flight mode set to ACRO.");
}

void PidTask::run()
{
    uint32_t start_time = micros();

    int constrained_roll = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_ROLL).toInt()), RC_CHANNEL_MIN_RAW, RC_CHANNEL_MAX_RAW);
    int constrained_pitch = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_PITCH).toInt()), RC_CHANNEL_MIN_RAW, RC_CHANNEL_MAX_RAW);
    int constrained_yaw = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_YAW).toInt()), RC_CHANNEL_MIN_RAW, RC_CHANNEL_MAX_RAW);
    int constrained_throttle = constrain(_rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_THRO).toInt()), RC_CHANNEL_MIN_RAW, RC_CHANNEL_MAX_RAW);
    int arm_channel_value = _rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_ARM).toInt());
    int fmode_channel_value = _rx_task->getChannel(_settings_manager->getSettingValue(KEY_RC_CHANNEL_FMODE).toInt());

    _isArmed = (arm_channel_value > RC_CHANNEL_ARM_THRESHOLD);

    // Determine flight mode based on fmode_channel_value
    if (fmode_channel_value > RC_CHANNEL_FMODE_STABILIZED_THRESHOLD)
    {
        _flight_mode = FlightMode::STABILIZED;
    }
    else
    {
        _flight_mode = FlightMode::ACRO;
    }

    float desired_roll_rate = 0.0f;
    float desired_pitch_rate = 0.0f;
    float desired_yaw_rate = (constrained_yaw - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
    float throttle = (constrained_throttle - RC_CHANNEL_MIN_RAW) / RC_CHANNEL_RANGE_THROTTLE;

    const ImuData &imu_data = _imu_task->getImuSensor().getData();
    float actual_roll_rate = imu_data.gyroX;
    float actual_pitch_rate = imu_data.gyroY;
    float actual_yaw_rate = imu_data.gyroZ;

    float dt = getTaskDelayMs() / MS_TO_SECONDS_FACTOR;

    switch (_flight_mode)
    {
    case FlightMode::ACRO:
    {
        desired_roll_rate = (constrained_roll - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
        desired_pitch_rate = (constrained_pitch - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC;
        break;
    }
    case FlightMode::STABILIZED:
    {
        // Outer loop: Angle PID for Roll and Pitch
        float desired_roll_angle = (constrained_roll - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC * PidConfig::MAX_ANGLE_DEGREES;
        float desired_pitch_angle = (constrained_pitch - RC_CHANNEL_CENTER) / RC_CHANNEL_RANGE_SYMMETRIC * PidConfig::MAX_ANGLE_DEGREES;

        float actual_roll_angle = imu_data.angleX;
        float actual_pitch_angle = imu_data.angleY;

        desired_roll_rate = _pid_angle_roll.update(desired_roll_angle, actual_roll_angle, dt);
        desired_pitch_rate = _pid_angle_pitch.update(desired_pitch_angle, actual_pitch_angle, dt);
        break;
    }
    }

    float roll_output = _pid_roll.update(desired_roll_rate, actual_roll_rate, dt);
    float pitch_output = _pid_pitch.update(desired_pitch_rate, actual_pitch_rate, dt);
    float yaw_output = _pid_yaw.update(desired_yaw_rate, actual_yaw_rate, dt);

    _motor_task->update(throttle, pitch_output, roll_output, yaw_output);

    _cycleTime = micros() - start_time;
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
    case PidAxis::ANGLE_ROLL:
        return _pid_angle_roll.getGains();
    case PidAxis::ANGLE_PITCH:
        return _pid_angle_pitch.getGains();
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

    float angle_roll_p = _settings_manager->getSettingValue(KEY_PID_ANG_R_P).toFloat();
    float angle_roll_i = _settings_manager->getSettingValue(KEY_PID_ANG_R_I).toFloat();
    _pid_angle_roll.setGains(
        angle_roll_p,
        angle_roll_i,
        (float)PidConfig::ANGLE_D_GAIN_DISABLED); // Angle PID does not use D gain

    float angle_pitch_p = _settings_manager->getSettingValue(KEY_PID_ANG_P_P).toFloat();
    float angle_pitch_i = _settings_manager->getSettingValue(KEY_PID_ANG_P_I).toFloat();
    _pid_angle_pitch.setGains(
        angle_pitch_p,
        angle_pitch_i,
        (float)PidConfig::ANGLE_D_GAIN_DISABLED); // Angle PID does not use D gain

    com_send_log(ComMessageType::LOG_INFO, "PidTask: Loaded PID gains from settings.");
}

void PidTask::resetToDefaults()
{
    _set_and_save_gains(PidAxis::ROLL, {PidConfig::DEFAULT_RATE_P, PidConfig::DEFAULT_RATE_I, PidConfig::DEFAULT_RATE_D});
    _set_and_save_gains(PidAxis::PITCH, {PidConfig::DEFAULT_RATE_P, PidConfig::DEFAULT_RATE_I, PidConfig::DEFAULT_RATE_D});
    _set_and_save_gains(PidAxis::YAW, {PidConfig::DEFAULT_YAW_P, PidConfig::DEFAULT_YAW_I, PidConfig::DEFAULT_YAW_D});
    _set_and_save_gains(PidAxis::ANGLE_ROLL, {PidConfig::DEFAULT_ANGLE_P, PidConfig::DEFAULT_ANGLE_I, PidConfig::ANGLE_D_GAIN_DISABLED});
    _set_and_save_gains(PidAxis::ANGLE_PITCH, {PidConfig::DEFAULT_ANGLE_P, PidConfig::DEFAULT_ANGLE_I, PidConfig::ANGLE_D_GAIN_DISABLED});

    _settings_manager->saveSettings();
    com_send_log(ComMessageType::LOG_INFO, "PidTask: Reset PID gains to defaults and saved.");
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
    case PidAxis::ANGLE_ROLL:
        _pid_angle_roll.setGains(gains.p, gains.i, (float)PidConfig::ANGLE_D_GAIN_DISABLED); // Angle PID does not use D gain
        p_key = KEY_PID_ANG_R_P;
        i_key = KEY_PID_ANG_R_I;
        d_key = nullptr; // No D gain for angle PID
        break;
    case PidAxis::ANGLE_PITCH:
        _pid_angle_pitch.setGains(gains.p, gains.i, (float)PidConfig::ANGLE_D_GAIN_DISABLED); // Angle PID does not use D gain
        p_key = KEY_PID_ANG_P_P;
        i_key = KEY_PID_ANG_P_I;
        d_key = nullptr; // No D gain for angle PID
        break;
    default:
        return;
    }

    _settings_manager->setSettingValue(p_key, String(gains.p));
    _settings_manager->setSettingValue(i_key, String(gains.i));
    if (d_key != nullptr)
    {
        _settings_manager->setSettingValue(d_key, String(gains.d));
    }
}
