/**
 * @file motor_task.cpp
 * @brief Implements motor control using DShotRMT for ESP32.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

// Motot Mixing matrix:
// RR =  T - P + R - Y  (rear right)
// FR =  T - P - R + Y  (front right)
// RL =  T + P + R + Y  (rear left)
// FL =  T + P - R - Y  (front left)

#include "../config/motor_config.h"
#include "motor_task.h"
#include "../com_manager.h"
#include "../settings_manager.h"
#include "../config/motor_config.h"
#include <DShotRMT.h>

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
      _yaw_command(0.0f),
      _motorTestState(IDLE),
      _testMotorNum(0),
      _testThrottle(0.0f),
      _testDuration(0),
      _testStartTime(0)
{
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _motor_throttles[i] = 0;
        _dshot_drivers[i] = nullptr;
    }
}



void MotorTask::setup()
{
    uint8_t protocol_index = _settings_manager->getSettingValue(KEY_MOTOR_PROTOCOL).toInt();
    dshot_mode_t dshot_protocol = get_dshot_mode_from_index(protocol_index);

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _dshot_drivers[i] = std::make_unique<DShotRMT>(_motor_pins[i], dshot_protocol);
        _dshot_drivers[i]->begin();
    }

    com_send_log(ComMessageType::LOG_INFO, "MotorTask: DShotRMT drivers initialized for %d motors on protocol %s.", NUM_MOTORS, _settings_manager->getSettingValueHumanReadable(KEY_MOTOR_PROTOCOL).c_str());
}

void MotorTask::run()
{
    if (!_dshot_drivers[FIRST_MOTOR_INDEX].get())
        return;

    if (_motorTestState != IDLE)
    {
        // Handle motor test mode
        if (_motorTestState == SPINNING_TIMED && (millis() - _testStartTime >= _testDuration))
        {
            stopMotorTest(); // Time's up, stop the motor
            return;          // Exit run() after stopping
        }

        // Apply test throttle to the designated motor, others are off
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            if (_dshot_drivers[i])
            {
                if (i == _testMotorNum)
                {
                    // Convert normalized throttle (0-1) to raw DShot throttle (MOTOR_MIN_THROTTLE_RAW to MOTOR_MAX_THROTTLE_RAW)
                    uint16_t raw_throttle = (uint16_t)constrain(_testThrottle * (MOTOR_MAX_THROTTLE_RAW - MOTOR_MIN_THROTTLE_RAW) + MOTOR_MIN_THROTTLE_RAW, MOTOR_MIN_THROTTLE_RAW, MOTOR_MAX_THROTTLE_RAW);
                    _dshot_drivers[i].get()->sendThrottle(raw_throttle);
                }
                else
                {
                    _dshot_drivers[i]->sendThrottle(0); // Other motors off
                }
            }
        }
        return; // Exit run() as we are in test mode
    }

    // Normal PID control logic
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

        _motor_throttles[MOTOR_INDEX_RR] = constrain(m_rr * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[MOTOR_INDEX_FR] = constrain(m_fr * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[MOTOR_INDEX_RL] = constrain(m_rl * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[MOTOR_INDEX_FL] = constrain(m_fl * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
    }

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (_dshot_drivers[i])
        {
            _dshot_drivers[i].get()->sendThrottle(_motor_throttles[i]);
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
    if (motor_id < NUM_MOTORS && _dshot_drivers[motor_id])
    {
        _dshot_drivers[motor_id].get()->sendThrottle(throttle);
        _motor_throttles[motor_id] = throttle;
    }
}

void MotorTask::startMotorTest(uint8_t motorNum, float throttle, uint32_t duration_ms)
{
    if (motorNum >= NUM_MOTORS)
    {
        com_send_log(ComMessageType::LOG_ERROR, "MotorTest: Invalid motor number %d.", motorNum);
        return;
    }
    if (!_is_valid_throttle_percentage(throttle))
    {
        return;
    }

    stopMotorTest(); // Stop any ongoing test first

    _testMotorNum = motorNum;
    _testThrottle = throttle;
    _testDuration = duration_ms;
    _testStartTime = millis();
    _motorTestState = SPINNING_TIMED;
    com_send_log(ComMessageType::LOG_INFO, "MotorTest: Motor %d spinning at %.1f%% throttle for %lu ms.", motorNum + 1, throttle * CPU_PERCENTAGE_FACTOR, duration_ms);
}

void MotorTask::startContinuousMotorTest(uint8_t motorNum, float throttle)
{
    if (motorNum >= NUM_MOTORS)
    {
        com_send_log(ComMessageType::LOG_ERROR, "MotorTest: Invalid motor number %d.", motorNum);
        return;
    }
    if (!_is_valid_throttle_percentage(throttle))
    {
        return;
    }

    stopMotorTest(); // Stop any ongoing test first

    _testMotorNum = motorNum;
    _testThrottle = throttle;
    _motorTestState = SPINNING_CONTINUOUS;
    com_send_log(ComMessageType::LOG_INFO, "MotorTest: Motor %d spinning continuously at %.1f%% throttle. Send 'motor stop' to stop.", motorNum + 1, throttle * CPU_PERCENTAGE_FACTOR);
}

void MotorTask::stopMotorTest()
{
    if (_motorTestState != IDLE)
    {
        _motorTestState = IDLE;
        _testMotorNum = 0;
        _testThrottle = 0.0f;
        _testDuration = 0;
        _testStartTime = 0;
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            if (_dshot_drivers[i])
            {
                _dshot_drivers[i].get()->sendThrottle(0); // Stop all motors
            }
        }
        com_send_log(ComMessageType::LOG_INFO, "MotorTest: All motors stopped.");
    }
}

bool MotorTask::_is_valid_throttle_percentage(float throttle)
{
    if (throttle < MIN_NORMALIZED_THROTTLE || throttle > MAX_NORMALIZED_THROTTLE)
    {
        com_send_log(ComMessageType::LOG_ERROR, "MotorTest: Invalid throttle percentage %f. Must be between %.0f and %.0f.", throttle, MIN_NORMALIZED_THROTTLE, MAX_NORMALIZED_THROTTLE);
        return false;
    }
    return true;
}

uint16_t MotorTask::getMotorOutput(uint8_t motor_id) const
{
    if (motor_id < NUM_MOTORS)
    {
        return _motor_throttles[motor_id];
    }
    return 0; // Return 0 for invalid motor_id
}