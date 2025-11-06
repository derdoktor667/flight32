#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include <DShotRMT.h>
#include "../config.h"



class MotorTask : public TaskBase
{
public:
    MotorTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
              const int *motor_pins, dshot_mode_t dshot_protocol);

    void setup() override;
    void run() override;

    void setThrottle(uint8_t motor_id, uint16_t throttle);

private:
    DShotRMT *_dshot_driver;
    uint16_t _motor_throttles[NUM_MOTORS];
    const int *_motor_pins;
    dshot_mode_t _dshot_protocol;
};