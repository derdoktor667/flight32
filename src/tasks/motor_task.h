/**
 * @file motor_task.h
 * @brief Defines the MotorTask class for DShot motor control.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <memory>
#include "../scheduler/task_base.h"
#include "DShotRMT.h"
#include "../config/motor_config.h"
#include <driver/gpio.h> // Required for gpio_num_t

class SettingsManager; // Forward declaration

class MotorTask : public TaskBase
{
public:
    MotorTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
              const int *motor_pins, SettingsManager *settings_manager);

    void setup() override;
    void run() override;

    void update(float throttle, float pitch, float roll, float yaw);
    void setThrottle(uint8_t motor_id, uint16_t throttle);
    uint16_t getMotorOutput(uint8_t motor_id) const;

public:
    enum MotorTestState
    {
        IDLE,
        SPINNING_TIMED,
        SPINNING_CONTINUOUS
    };

    void startMotorTest(uint8_t motorNum, float throttle, uint32_t duration_ms);
    void startContinuousMotorTest(uint8_t motorNum, float throttle);
    void stopMotorTest();
    bool isInTestMode() const { return _motorTestState != IDLE; }

    void enterPassthroughMode();
    void exitPassthroughMode();
    bool isInPassthroughMode() const { return _inPassthroughMode; }

    // Passthrough command handlers
    uint8_t passthroughRead(uint8_t motorId, uint16_t address, uint8_t* data, uint8_t length);
    uint8_t passthroughWrite(uint8_t motorId, uint16_t address, const uint8_t* data, uint8_t length);
    uint8_t passthroughErase(uint8_t motorId, uint16_t address);

private:
    std::unique_ptr<DShotRMT> _dshot_drivers[NUM_MOTORS];
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
    bool _inPassthroughMode = false;

    bool _is_valid_throttle_percentage(float throttle);

    // 1-Wire Passthrough related members
    gpio_num_t _passthrough_gpio = GPIO_NUM_NC;
    uint8_t _passthrough_motor_idx = 0;
    dshot_mode_t _original_dshot_mode = DSHOT_OFF; // To restore original DShot mode

    // 1-Wire Passthrough helper functions
    void _1wire_init(uint8_t motor_idx);
    void _1wire_deinit(uint8_t motor_idx);
    void _1wire_output_mode();
    void _1wire_input_mode();
    void _1wire_set_high();
    void _1wire_set_low();
    int _1wire_read_pin();
    void _1wire_half_bit_delay();
    void _1wire_byte_delay();
    uint8_t _1wire_read_byte();
    void _1wire_send_byte(uint8_t byte);
    uint8_t _1wire_send_byte_and_read_byte(uint8_t byte);

    // High-level 1-Wire transaction
    uint8_t _1wire_transaction(uint8_t cmd, uint16_t address, uint8_t* data, uint8_t length, bool is_write);
};