/**
 * @file terminal_task.h
 * @brief Defines the TerminalTask class for handling command-line interface interactions.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include "../scheduler/scheduler.h"
#include "imu_task.h"
#include "rx_task.h"
#include "motor_task.h"
#include "pid_task.h"

#include "../settings_manager.h"
#include "../config.h"

// Forward declaration of TerminalTask is needed for the function pointer type
class TerminalTask;

enum class CommandCategory
{
    SYSTEM,
    IMU,
    RX,
    MOTOR,
    PID,
    SETTINGS,
    UNKNOWN
};

struct Command
{
    const char *name;
    void (TerminalTask::*handler)(String &);
    const char *help;
    CommandCategory category;
};

struct CategoryInfo {
    CommandCategory category;
    const char* prefix;
    const char* description;
};

struct ChannelMapping {
    const char* name;
    const char* key;
};

class TerminalTask : public TaskBase
{
public:
    TerminalTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager);

    void setup() override;
    void run() override;

    void _show_prompt();

private:
    Scheduler *_scheduler;
    ImuTask *_imu_task;
    RxTask *_rx_task;
    MotorTask *_motor_task;
    PidTask *_pid_task;
    SettingsManager *_settings_manager;

    String _input_buffer;
    bool _should_quit;

    bool _check_imu_task_available();
    void _handle_quit(String &args);
    bool _check_rx_task_available();
    bool _check_motor_task_available();
    bool _check_pid_task_available();

    void _parse_command(String &command);

    const char *_get_category_string(CommandCategory category);
    CommandCategory _get_category_from_string(String &category_str);
    CommandCategory _get_setting_category(const char *display_key);

    void _handle_imu_data(String &args);
    void _handle_imu_calibrate(String &args);
    void _handle_imu_lpf_bandwidth(String &args);
    void _handle_rx_data(String &args);
    void _handle_rx_status(String &args);
    void _handle_rx_protocol(String &args);
    void _handle_rx_value_single(String &args);
    void _handle_rx_value_all(String &args);
    void _handle_rx_channel_mapping(String &args);
    void _handle_motor_throttle(String &args);
    void _handle_motor_test(String &args);
    void _handle_motor_stop(String &args);
    void _handle_pid_get(String &args);
    void _handle_pid_set(String &args);
    void _handle_pid_reset_defaults(String &args);

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

    static const Command _commands[];
    static const int _num_commands;

    static const CategoryInfo _category_info[];
    static const int _num_categories;

    char _byte_buffer[BYTE_BUFFER_SIZE];
    const char *_format_bytes(uint32_t bytes);
    const char *_get_motor_name(uint8_t motor_id);
    int8_t _get_motor_id(String &motor_name);

    static const char *_get_task_state_string(eTaskState state);
};