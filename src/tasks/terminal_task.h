#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include "../scheduler/scheduler.h" // Include for Scheduler class
#include <ESP32_MPU6050.h>
#include "ibus_task.h"
#include "motor_task.h"

#include "../settings_manager.h"

// Forward declaration of TerminalTask is needed for the function pointer type
class TerminalTask;

// Define a function pointer type for command handlers.
// It points to a member function of TerminalTask that takes a String reference and returns void.
using CommandHandler = void (TerminalTask::*)(String &args);

enum class CommandCategory {
    SYSTEM,
    MPU6050,
    IBUS,
    MOTOR,
    SETTINGS
};
struct Command {
    const char *name;
    CommandHandler handler;
    const char *help;
    CommandCategory category;
};

class TerminalTask : public TaskBase {
public:
    TerminalTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ESP32_MPU6050 *mpu6050_sensor, IbusTask *ibus_receiver_task, MotorTask *motor_task, SettingsManager *settings_manager);

    void setup() override;
    void run() override;

private:
    Scheduler *_scheduler;
    ESP32_MPU6050 *_mpu6050_sensor;
    IbusTask *_ibus_receiver_task;
    MotorTask *_motor_task;
    SettingsManager *_settings_manager;

    String _input_buffer;

    // Helper functions
    bool _check_mpu6050_sensor_available();
    bool _check_ibus_receiver_available();
    bool _check_motor_task_available();

    void _parse_command(String &command);
    void _show_prompt();

    // New handler declarations for categorized commands
    void _handle_mpu_data(String &args);
    void _handle_mpu_config(String &args);
    void _handle_mpu_calibrate(String &args);
    void _handle_ibus_data(String &args);
    void _handle_ibus_status(String &args);
    void _handle_motor_throttle(String &args);

    // Existing handler declarations
    void _handle_help(String &args);
    void _handle_status(String &args);
    void _handle_tasks(String &args);
    void _handle_mem(String &args);
    void _handle_reboot(String &args);
    void _handle_get_setting(String &args);
    void _handle_set_setting(String &args);
    void _handle_save_settings(String &args);
    void _handle_factory_reset(String &args);
    void _handle_list_settings(String &args);
    void _handle_dump_settings(String &args);

    // Command table definition
    static const Command _commands[];
    static const int _num_commands;

    char _byte_buffer[15]; // Buffer for human-readable byte output
    const char* _format_bytes(uint32_t bytes);
};