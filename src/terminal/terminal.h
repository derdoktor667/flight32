/**
 * @file terminal.h
 * @brief Defines the Terminal class for handling CLI terminal commands and parsing.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <memory>

#include <Arduino.h>
#include "../settings_manager.h"
#include "../config/terminal_config.h"
#include "../com_manager.h"
#include "../scheduler/scheduler.h"
#include "../tasks/imu_task.h"
#include "../tasks/rx_task.h"
#include "../tasks/motor_task.h"
#include "../tasks/pid_task.h"

// Terminal Command Categories
enum class CommandCategory
{
    SYSTEM,
    IMU,
    RX,
    MOTOR,
    PID,
    FILTER,
    SETTINGS,
    RC_CHANNELS,
    UNKNOWN
};

// Forward declaration of Terminal class for Command struct
class Terminal;

// Command struct definition
struct Command
{
    const char *name;
    void (Terminal::*handler)(String &);
    const char *help;
    CommandCategory category;
};

struct CategoryInfo
{
    CommandCategory category;
    const char *prefix;
    const char *description;
};

struct ChannelMapping
{
    const char *name;
    const char *key;
};

class Terminal
{
public:
    Terminal(Scheduler *scheduler, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager);

    void handleInput(char incoming_char);
    void showPrompt();
    bool shouldQuit() const { return _should_quit; }

private:
    Scheduler *_scheduler;
    ImuTask *_imu_task;
    RxTask *_rx_task;
    MotorTask *_motor_task;
    PidTask *_pid_task;
    SettingsManager *_settings_manager;

    String _input_buffer;
    bool _should_quit = false;

    void _parse_command(String &command_line);

    // Helper functions
    const char *_get_category_string(CommandCategory category);
    CommandCategory _get_category_from_string(String &category_str);
    String _generate_separator(int column_width, int description_width);
    bool _category_has_settings(CommandCategory category); // New helper function // New helper function

public:
    static CommandCategory _get_setting_category(const char *display_key);
    const char *_get_motor_name(uint8_t motor_id);
    int8_t _get_motor_id(String &motor_name);
    static const char *_get_task_state_string(eTaskState state);

    // Command Handlers
    void _handle_help(String &args);
    void _handle_status(String &args);
    void _handle_tasks(String &args);
    void _handle_mem(String &args);
    void _handle_reboot(String &args);
    void _handle_quit(String &args);
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
    void _handle_filter_get_setting(String &args);
    void _handle_filter_set_setting(String &args);
    void _handle_filter_reset_defaults(String &args);
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
    static const ChannelMapping _channel_map[];
    static const int _num_channel_mappings;
};
