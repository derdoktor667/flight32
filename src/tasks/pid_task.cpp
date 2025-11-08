#include "pid_task.h"
#include "../com_manager.h"

PidTask::PidTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
                 Mpu6050Task *mpu6050_task,
                 IbusTask *ibus_task,
                 MotorTask *motor_task,
                 SettingsManager *settings_manager)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms),
      _mpu6050_task(mpu6050_task),
      _ibus_task(ibus_task),
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
    // The IBUS values are typically in the range 1000-2000. We need to map them to a more usable range, e.g., -1.0 to 1.0 for roll/pitch/yaw rates.
    // For now, let's assume a simple mapping. This will need refinement.
    float desired_roll_rate = (_ibus_task->getChannel(1) - 1500.0f) / 500.0f;  // Channel 1 for Roll
    float desired_pitch_rate = (_ibus_task->getChannel(0) - 1500.0f) / 500.0f; // Channel 0 for Pitch
    float desired_yaw_rate = (_ibus_task->getChannel(3) - 1500.0f) / 500.0f;   // Channel 3 for Yaw
    float throttle = (_ibus_task->getChannel(2) - 1000.0f) / 1000.0f;          // Channel 2 for Throttle (0.0 to 1.0)

    float actual_roll_rate = _mpu6050_task->getMpu6050Sensor().readings.gyroscope.x;
    float actual_pitch_rate = _mpu6050_task->getMpu6050Sensor().readings.gyroscope.y;
    float actual_yaw_rate = _mpu6050_task->getMpu6050Sensor().readings.gyroscope.z;

    float dt = getTaskDelayMs() / 1000.0f;

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