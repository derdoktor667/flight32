#include "motor_task.h"
#include "../com_manager.h"
#include "../settings_manager.h"

// Helper function to convert DShot protocol index to dshot_mode_t
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
        return DSHOT300; // Default to DSHOT300 if invalid index
    }
}

MotorTask::MotorTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
                     const int *motor_pins, SettingsManager *settings_manager)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms),
      _dshot_driver(nullptr), // Initialize to nullptr, will be set in setup()
      _motor_pins(motor_pins),
      _settings_manager(settings_manager)
{
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _motor_throttles[i] = 0; // Initialize all throttles to 0
    }
}

void MotorTask::setup()
{
    uint8_t protocol_index = _settings_manager->getSettingValue(KEY_MOTOR_PROTOCOL).toInt();
    dshot_mode_t dshot_protocol = get_dshot_mode_from_index(protocol_index);

    _dshot_driver = new DShotRMT(_motor_pins[0], dshot_protocol);
    _dshot_driver->begin(); // Call begin without RMT channels
    com_send_log(LOG_INFO, "MotorTask: DShotRMT driver initialized with %d motors on protocol %s.", NUM_MOTORS, _settings_manager->getSettingValueHumanReadable(KEY_MOTOR_PROTOCOL).c_str());
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