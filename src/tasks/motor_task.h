/**
 * @file motor_task.h
 * @brief Defines the MotorTask class for DShot motor control.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include <DShotRMT.h>
#include "../config.h"
#include "../settings_manager.h"

class MotorTask : public TaskBase
{
public:
    MotorTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
              const int *motor_pins, SettingsManager *settings_manager);

    void setup() override;
    void run() override;

    void update(float throttle, float pitch, float roll, float yaw);
    void setThrottle(uint8_t motor_id, uint16_t throttle);

    ~MotorTask();

public:
    enum MotorTestState {
        IDLE,
        SPINNING_TIMED,
        SPINNING_CONTINUOUS
    };

    void startMotorTest(uint8_t motorNum, float throttle, uint32_t duration_ms);
    void startContinuousMotorTest(uint8_t motorNum, float throttle);
    void stopMotorTest();
    bool isInTestMode() const { return _motorTestState != IDLE; }

private:
    DShotRMT *_dshot_drivers[NUM_MOTORS];
    uint16_t _motor_throttles[NUM_MOTORS];
    const int *_motor_pins;
    SettingsManager *_settings_manager;

    float _throttle_command;
    float _pitch_command;
    float _roll_command;
    float _yaw_command;

    MotorTestState _motorTestState;
    uint8_t _testMotorNum;
    float _testThrottle;
    uint32_t _testDuration;
    uint32_t _testStartTime;
};