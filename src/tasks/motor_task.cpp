#include "motor_task.h"
#include "../com_manager.h"

MotorTask::MotorTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
                     const int *motor_pins, dshot_mode_t dshot_protocol)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms),
      _dshot_driver(new DShotRMT(motor_pins[0], dshot_protocol)), // Initialize with first pin and protocol
      _motor_pins(motor_pins),
      _dshot_protocol(dshot_protocol)
{
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _motor_throttles[i] = 0; // Initialize all throttles to 0
    }
}

void MotorTask::setup()
{
    // Initialize DShotRMT driver with pins and protocol
    _dshot_driver->begin(); // Call begin without RMT channels
    com_send_log(LOG_INFO, "MotorTask: DShotRMT driver initialized with %d motors on protocol %d.", NUM_MOTORS, _dshot_protocol);
}

void MotorTask::run()
{
    if (!_dshot_driver)
        return; // Ensure driver is initialized

    // Send current throttle values to motors
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _dshot_driver->sendThrottle(_motor_throttles[i]); // Use sendThrottle(uint16_t throttle)
    }
}

void MotorTask::setThrottle(uint8_t motor_id, uint16_t throttle)
{
    if (motor_id < NUM_MOTORS)
    {
        _motor_throttles[motor_id] = throttle;
    }
}