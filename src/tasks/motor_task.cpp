/**
 * @file motor_task.cpp
 * @brief Implements motor control using DShotRMT for ESP32.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "motor_task.h"
#include "../com_manager.h"
#include "../settings_manager.h"

dshot_mode_t get_dshot_mode_from_index(uint8_t index)
{
    switch (index)
    {
    case 0:
        return DSHOT150;
    case 1:
        return DSHOT300;
    case 2:
        return DSHOT600;
    case 3:
        return DSHOT1200;
    default:
        return DSHOT300;
    }
}

MotorTask::MotorTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
                     const int *motor_pins, SettingsManager *settings_manager)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms),
      _motor_pins(motor_pins),
      _settings_manager(settings_manager),
      _throttle_command(0.0f),
      _pitch_command(0.0f),
      _roll_command(0.0f),
      _yaw_command(0.0f)
{
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _motor_throttles[i] = 0;
        _dshot_drivers[i] = nullptr;
    }
}

MotorTask::~MotorTask()
{
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (_dshot_drivers[i])
        {
            delete _dshot_drivers[i];
        }
    }
}

void MotorTask::setup()
{
    uint8_t protocol_index = _settings_manager->getSettingValue(KEY_MOTOR_PROTOCOL).toInt();
    dshot_mode_t dshot_protocol = get_dshot_mode_from_index(protocol_index);

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _dshot_drivers[i] = new DShotRMT(_motor_pins[i], dshot_protocol);
        _dshot_drivers[i]->begin();
    }

    com_send_log(LOG_INFO, "MotorTask: DShotRMT drivers initialized for %d motors on protocol %s.", NUM_MOTORS, _settings_manager->getSettingValueHumanReadable(KEY_MOTOR_PROTOCOL).c_str());
}

void MotorTask::run()
{
    if (!_dshot_drivers[0])
        return;

    if (_throttle_command < THROTTLE_DEADZONE_THRESHOLD)
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            _motor_throttles[i] = 0;
        }
    }
    else
    {
        float m_rr = _throttle_command - _pitch_command + _roll_command - _yaw_command; // Motor 1 (Rear Right)
        float m_fr = _throttle_command - _pitch_command - _roll_command + _yaw_command; // Motor 2 (Front Right)
        float m_rl = _throttle_command + _pitch_command + _roll_command + _yaw_command; // Motor 3 (Rear Left)
        float m_fl = _throttle_command + _pitch_command - _roll_command - _yaw_command; // Motor 4 (Front Left)

        const float min_throttle = 48.0f;
        const float max_throttle = 2047.0f;

        _motor_throttles[0] = constrain(m_rr * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[1] = constrain(m_fr * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[2] = constrain(m_rl * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[3] = constrain(m_fl * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
    }

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (_dshot_drivers[i])
        {
            _dshot_drivers[i]->sendThrottle(_motor_throttles[i]);
        }
    }
}

void MotorTask::update(float throttle, float pitch, float roll, float yaw)
{
    _throttle_command = throttle;
    _pitch_command = pitch;
    _roll_command = roll;
    _yaw_command = yaw;
}

void MotorTask::setThrottle(uint8_t motor_id, uint16_t throttle)
{
    if (motor_id < NUM_MOTORS && _dshot_drivers[motor_id]) {
        // It bypasses the mixer and sends a command directly to the motor.
        _dshot_drivers[motor_id]->sendThrottle(throttle);
        _motor_throttles[motor_id] = throttle; // Also update the state for consistency
    }
}