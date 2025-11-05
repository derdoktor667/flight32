#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include "../scheduler/scheduler.h" // Include for Scheduler class
#include <ESP32_MPU6050.h>
#include "ibus_task.h"

// Forward declaration of TerminalTask is needed for the function pointer type
class TerminalTask;

// Define a function pointer type for command handlers.
// It points to a member function of TerminalTask that takes a String reference and returns void.
using CommandHandler = void (TerminalTask::*)(String &args);

// Struct to define the properties of a terminal command
struct Command
{
    const char *name;       // The command string (e.g., "help")
    CommandHandler handler; // Pointer to the handler function
    const char *help;       // The help text for the command
};

class TerminalTask : public TaskBase
{
public:
    TerminalTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ESP32_MPU6050 *mpu6050_sensor, IbusTask *ibus_receiver_task);
    void setup() override;
    void run() override;

private:
    String _input_buffer;
    Scheduler *_scheduler;          // Pointer to the scheduler instance
    ESP32_MPU6050 *_mpu6050_sensor; // Pointer to the MPU6050 instance
    IbusTask *_ibus_receiver_task;  // Pointer to the IbusTask instance
    void _parse_command(String &command);
    bool _check_mpu6050_sensor_available();
    bool _check_ibus_receiver_available();

    // Command handler functions
    void _handle_help(String &args);
    void _handle_status(String &args);
    void _handle_tasks(String &args);
    void _handle_mem(String &args);
    void _handle_reboot(String &args);
    void _handle_get_mpu_data(String &args);
    void _handle_get_mpu_config(String &args);
    void _handle_calibrate_mpu_sensor(String &args);
    void _handle_get_ibus_data(String &args);
    void _handle_get_ibus_status(String &args);

    // Command table definition
    static const Command _commands[];
    static const int _num_commands;
};
