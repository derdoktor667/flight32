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
#include "../config.h" // For THROTTLE_DEADZONE_THRESHOLD, MOTOR_MIN_THROTTLE_RAW, MOTOR_MAX_THROTTLE_RAW

dshot_mode_t get_dshot_mode_from_index(uint8_t index)
{
    switch ((DshotProtocolIndex)index)
    {
    case DshotProtocolIndex::DSHOT150:
        return DSHOT150;
    case DshotProtocolIndex::DSHOT300:
        return DSHOT300;
    case DshotProtocolIndex::DSHOT600:
        return DSHOT600;
    case DshotProtocolIndex::DSHOT1200:
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
    if (!_dshot_drivers[FIRST_MOTOR_INDEX])
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
        float m_rr = _throttle_command - _pitch_command + _roll_command - _yaw_command;
        float m_fr = _throttle_command - _pitch_command - _roll_command + _yaw_command;
        float m_rl = _throttle_command + _pitch_command + _roll_command + _yaw_command;
        float m_fl = _throttle_command + _pitch_command - _roll_command - _yaw_command;

        const float min_throttle = MOTOR_MIN_THROTTLE_RAW;
        const float max_throttle = MOTOR_MAX_THROTTLE_RAW;

        _motor_throttles[(uint8_t)MotorIndex::RR] = constrain(m_rr * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[(uint8_t)MotorIndex::FR] = constrain(m_fr * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[(uint8_t)MotorIndex::RL] = constrain(m_rl * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[(uint8_t)MotorIndex::FL] = constrain(m_fl * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
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
        _dshot_drivers[motor_id]->sendThrottle(throttle);
        _motor_throttles[motor_id] = throttle;
    }
}