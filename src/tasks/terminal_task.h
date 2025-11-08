#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include "../scheduler/scheduler.h" // Include for Scheduler class
#include <ESP32_MPU6050.h>
#include "ibus_task.h"
#include "motor_task.h"
#include "pid_task.h"

#include "../settings_manager.h"

// Forward declaration of TerminalTask is needed for the function pointer type
class TerminalTask;

// Enum for command categories
enum class CommandCategory
{
    SYSTEM,
    MPU6050,
    IBUS,
    MOTOR,
    PID,
    SETTINGS,
    UNKNOWN // Added for handling invalid categories
};

// Struct to hold command information
struct Command
{
    const char *name;
    void (TerminalTask::*handler)(String &);
    const char *help;
    CommandCategory category;
};

class TerminalTask : public TaskBase
{
public:
    TerminalTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ESP32_MPU6050 *mpu6050_sensor, IbusTask *ibus_receiver_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager);

    void setup() override;
    void run() override;

    void _show_prompt();

private:
    Scheduler *_scheduler;
    ESP32_MPU6050 *_mpu6050_sensor;
    IbusTask *_ibus_receiver_task;
    MotorTask *_motor_task;
    PidTask *_pid_task;
    SettingsManager *_settings_manager;

    String _input_buffer;

    // Helper functions
    bool _check_mpu6050_sensor_available();
    bool _check_ibus_receiver_available();
    bool _check_motor_task_available();
    bool _check_pid_task_available();

    void _parse_command(String &command);

    // New helper declarations for help command
    const char *_get_category_string(CommandCategory category);
    CommandCategory _get_category_from_string(String &category_str);

    // New handler declarations for categorized commands
    void _handle_mpu_data(String &args);
    void _handle_mpu_config(String &args);
    void _handle_mpu_calibrate(String &args);
    void _handle_ibus_data(String &args);
    void _handle_ibus_status(String &args);
    void _handle_motor_throttle(String &args);
    void _handle_pid_get(String &args);
    void _handle_pid_set(String &args);

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
    const char *_format_bytes(uint32_t bytes);
};