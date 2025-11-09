/**
 * @file terminal_task.cpp
 * @brief Implements the interactive command-line terminal for Flight32.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "terminal_task.h"
#include "../firmware.h"
#include "../config.h"
#include "../com_manager.h"
#include "pid_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define the command table
const Command TerminalTask::_commands[] = {
    {"help", &TerminalTask::_handle_help, "Shows this help message.", CommandCategory::SYSTEM},
    {"status", &TerminalTask::_handle_status, "Shows firmware information.", CommandCategory::SYSTEM},
    {"tasks", &TerminalTask::_handle_tasks, "Shows information about running tasks.", CommandCategory::SYSTEM},
    {"mem", &TerminalTask::_handle_mem, "Shows current memory usage.", CommandCategory::SYSTEM},
    {"reboot", &TerminalTask::_handle_reboot, "Reboots the ESP32.", CommandCategory::SYSTEM},

    {"get mpu.data", &TerminalTask::_handle_mpu_data, "Displays the latest MPU6050 readings.", CommandCategory::MPU6050},
    {"get mpu.config", &TerminalTask::_handle_mpu_config, "Shows the current MPU6050 settings.", CommandCategory::MPU6050},
    {"set mpu.calibrate", &TerminalTask::_handle_mpu_calibrate, "Calibrates the MPU6050.", CommandCategory::MPU6050},

    {"get ibus.data", &TerminalTask::_handle_ibus_data, "Shows the latest IBUS channel data.", CommandCategory::IBUS},
    {"get ibus.status", &TerminalTask::_handle_ibus_status, "Shows the IBUS connection status.", CommandCategory::IBUS},

    {"set motor.throttle", &TerminalTask::_handle_motor_throttle, "Sets the throttle for a specific motor (e.g., 'set motor.throttle 0 1000').", CommandCategory::MOTOR},

    {"get pid", &TerminalTask::_handle_pid_get, "Gets the current PID gains.", CommandCategory::PID},
    {"set pid", &TerminalTask::_handle_pid_set, "Sets a PID gain (e.g., 'set pid roll p 0.1').", CommandCategory::PID},
    {"reset pid", &TerminalTask::_handle_pid_reset_defaults, "Resets PID gains to default values.", CommandCategory::PID},

    {"get", &TerminalTask::_handle_get_setting, "Gets a setting value (e.g., 'get gyro.resolution').", CommandCategory::SETTINGS},
    {"set", &TerminalTask::_handle_set_setting, "Sets a setting value (e.g., 'set gyro.resolution = 250_DPS').", CommandCategory::SETTINGS},
    {"save", &TerminalTask::_handle_save_settings, "Saves all settings to persistent storage.", CommandCategory::SETTINGS},
    {"factory_reset", &TerminalTask::_handle_factory_reset, "Resets all settings to their default values.", CommandCategory::SETTINGS},
    {"settings", &TerminalTask::_handle_list_settings, "Lists all available settings.", CommandCategory::SETTINGS},
    {"dump", &TerminalTask::_handle_dump_settings, "Dumps all settings for backup.", CommandCategory::SETTINGS}};

// Calculate the number of commands
const int TerminalTask::_num_commands = sizeof(TerminalTask::_commands) / sizeof(Command);

TerminalTask::TerminalTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ESP32_MPU6050 *mpu6050_sensor, IbusTask *ibus_receiver_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms),
      _scheduler(scheduler),
      _mpu6050_sensor(mpu6050_sensor),
      _ibus_receiver_task(ibus_receiver_task),
      _motor_task(motor_task),
      _pid_task(pid_task),
      _settings_manager(settings_manager)
{
}

//
void TerminalTask::setup()
{
    _input_buffer.reserve(TERMINAL_INPUT_BUFFER_SIZE);
}

//
void TerminalTask::run()
{
    while (Serial.available() > 0)
    {
        char incoming_char = Serial.read();
        if (incoming_char == '\n' || incoming_char == '\r')
        {
            if (_input_buffer.length() > 0)
            {
                TerminalTask::_parse_command(_input_buffer);
                _input_buffer = "";
            }
            TerminalTask::_show_prompt();
        }
        else if (isPrintable(incoming_char))
        {
            _input_buffer += incoming_char;
        }
        else if (incoming_char == ASCII_BACKSPACE) // Backspace
        {
            if (_input_buffer.length() > 0)
            {
                _input_buffer.remove(_input_buffer.length() - 1);
            }
        }
    }
}

//
void TerminalTask::_handle_help(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");

    if (args.length() == 0)
    {
        // General help message
        com_send_log(TERMINAL_OUTPUT, "--- Flight32 Terminal Help ---");
        com_send_log(TERMINAL_OUTPUT, "");
        com_send_log(TERMINAL_OUTPUT, "Usage: help <category>");
        com_send_log(TERMINAL_OUTPUT, "");
        com_send_log(TERMINAL_OUTPUT, "Core Commands:");
        com_send_log(TERMINAL_OUTPUT, "  %-19s %s", "status", "Shows firmware information.");
        com_send_log(TERMINAL_OUTPUT, "  %-19s %s", "tasks", "Shows information about running tasks.");
        com_send_log(TERMINAL_OUTPUT, "  %-19s %s", "mem", "Shows current memory usage.");
        com_send_log(TERMINAL_OUTPUT, "  %-19s %s", "reboot", "Reboots the ESP32.");
        com_send_log(TERMINAL_OUTPUT, "  %-19s %s", "factory_reset", "Resets all settings to their default values.");
        com_send_log(TERMINAL_OUTPUT, "");
        com_send_log(TERMINAL_OUTPUT, "Available Command Categories:");

        // List unique categories in a formatted way
        CommandCategory displayed_categories[static_cast<int>(CommandCategory::SETTINGS) + 1];
        int num_displayed_categories = 0;

        for (int i = 0; i < _num_commands; ++i)
        {
            bool already_displayed = false;
            for (int j = 0; j < num_displayed_categories; ++j)
            {
                if (displayed_categories[j] == _commands[i].category)
                {
                    already_displayed = true;
                    break;
                }
            }

            if (!already_displayed)
            {
                displayed_categories[num_displayed_categories++] = _commands[i].category;
                const char *category_str = _get_category_string(_commands[i].category);
                com_send_log(TERMINAL_OUTPUT, "  - %s", category_str);
            }
        }
        com_send_log(TERMINAL_OUTPUT, "\n------------------------------");
    }
    else
    {
        // Category-specific help
        CommandCategory requested_category = _get_category_from_string(args);

        if (requested_category == CommandCategory::UNKNOWN)
        {
            com_send_log(LOG_ERROR, "Unknown help category: '%s'. Type 'help' for a list of categories.", args.c_str());
            return;
        }

        com_send_log(TERMINAL_OUTPUT, "--- %s Commands ---", _get_category_string(requested_category));
        com_send_log(TERMINAL_OUTPUT, "");
        com_send_log(TERMINAL_OUTPUT, "  Command             Description");
        com_send_log(TERMINAL_OUTPUT, "  --------------------------------------------------");
        for (int i = 0; i < _num_commands; ++i)
        {
            if (_commands[i].category == requested_category)
            {
                com_send_log(TERMINAL_OUTPUT, "  %-19s %s", _commands[i].name, _commands[i].help);
            }
        }
        com_send_log(TERMINAL_OUTPUT, "  --------------------------------------------------");
        com_send_log(TERMINAL_OUTPUT, "\n--------------------------");
    }
}

// Helper to convert CommandCategory enum to string
const char *TerminalTask::_get_category_string(CommandCategory category)
{
    switch (category)
    {
    case CommandCategory::SYSTEM:
        return "System";
    case CommandCategory::MPU6050:
        return "MPU6050 Sensor";
    case CommandCategory::IBUS:
        return "IBUS Receiver";
    case CommandCategory::MOTOR:
        return "Motor Control";
    case CommandCategory::PID:
        return "PID Controller";
    case CommandCategory::SETTINGS:
        return "Settings Management";
    default:
        return "Unknown";
    }
}

// Helper to convert string to CommandCategory enum
CommandCategory TerminalTask::_get_category_from_string(String &category_str)
{
    if (category_str.equalsIgnoreCase("system"))
        return CommandCategory::SYSTEM;
    if (category_str.equalsIgnoreCase("mpu6050") || category_str.equalsIgnoreCase("sensor"))
        return CommandCategory::MPU6050;
    if (category_str.equalsIgnoreCase("ibus") || category_str.equalsIgnoreCase("receiver"))
        return CommandCategory::IBUS;
    if (category_str.equalsIgnoreCase("motor") || category_str.equalsIgnoreCase("motors"))
        return CommandCategory::MOTOR;
    if (category_str.equalsIgnoreCase("pid") || category_str.equalsIgnoreCase("controller"))
        return CommandCategory::PID;
    if (category_str.equalsIgnoreCase("settings") || category_str.equalsIgnoreCase("config"))
        return CommandCategory::SETTINGS;
    return CommandCategory::UNKNOWN;
}

void TerminalTask::_handle_status(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Flight32 Firmware v%s", get_firmware_version());
}

// Helper to convert task state enum to a readable string
const char *TerminalTask::_get_task_state_string(eTaskState state)
{
    switch (state)
    {
    case eRunning:
        return "Running";
    case eReady:
        return "Ready";
    case eBlocked:
        return "Waiting";
    case eSuspended:
        return "Suspended";
    case eDeleted:
        return "Deleted";
    default:
        return "Invalid";
    }
}

void TerminalTask::_handle_tasks(String &args)
{
    if (!_scheduler)
    {
        com_send_log(LOG_ERROR, "Scheduler not available.");
        return;
    }

    UBaseType_t num_freertos_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *freertos_task_status_array = (TaskStatus_t *)pvPortMalloc(num_freertos_tasks * sizeof(TaskStatus_t));
    uint32_t total_run_time = 0;

    if (freertos_task_status_array == NULL)
    {
        com_send_log(LOG_ERROR, "Failed to allocate memory for FreeRTOS task list.");
        return;
    }

    // Get FreeRTOS task states and total run time
    num_freertos_tasks = uxTaskGetSystemState(freertos_task_status_array, num_freertos_tasks, &total_run_time);

    // Print header
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "% -16s %-10s %-6s %-8s %-10s %-10s %-10s %s",
                 "Task Name", "State", "Prio", "CPU %", "Loop (us)", "Avg (us)", "Max (us)", "Stack HWM (bytes)");
    com_send_log(TERMINAL_OUTPUT, "-------------------------------------------------------------------------------------------------------------------");

    // Iterate through tasks managed by our scheduler
    for (uint8_t i = 0; i < _scheduler->getTaskCount(); i++)
    {
        TaskBase *current_task = _scheduler->getTask(i);
        if (!current_task)
            continue;

        const char *name = current_task->getName();
        // Filter out irrelevant tasks (IDLE and Tmr Svc are handled by FreeRTOS directly, not our TaskBase)
        if (strcmp(name, IDLE_TASK_NAME_0) == 0 || strcmp(name, IDLE_TASK_NAME_1) == 0 || strcmp(name, TIMER_SERVICE_TASK_NAME) == 0)
        {
            continue;
        }

        // Find the corresponding FreeRTOS task status
        TaskStatus_t freertos_status = {};
        bool found_freertos_status = false;
        for (UBaseType_t j = 0; j < num_freertos_tasks; j++)
        {
            if (freertos_task_status_array[j].xHandle == current_task->getHandle())
            {
                freertos_status = freertos_task_status_array[j];
                found_freertos_status = true;
                break;
            }
        }

        float cpu_percentage = 0.0f;
        if (found_freertos_status && total_run_time > 0)
        {
            cpu_percentage = (float)freertos_status.ulRunTimeCounter / total_run_time * 100.0f;
        }

        char cpu_str[8];
        snprintf(cpu_str, sizeof(cpu_str), "%.2f", cpu_percentage);

        char output_buffer[256]; // Sufficiently large buffer for the output line
        snprintf(output_buffer, sizeof(output_buffer),
                 "% -*s %-*s %-*u %-*s %-*u %-*u %-*u %u",
                 TASK_NAME_COLUMN_WIDTH, name,
                 TASK_STATE_COLUMN_WIDTH, TerminalTask::_get_task_state_string(freertos_status.eCurrentState),
                 TASK_PRIO_COLUMN_WIDTH, freertos_status.uxBasePriority,
                 TASK_CPU_COLUMN_WIDTH, cpu_str,
                 TASK_LOOP_COLUMN_WIDTH, current_task->getLoopTime(),
                 TASK_AVG_LOOP_COLUMN_WIDTH, current_task->getAvgLoopTime(),
                 TASK_MAX_LOOP_COLUMN_WIDTH, current_task->getMaxLoopTime(),
                 freertos_status.usStackHighWaterMark);
        com_send_log(TERMINAL_OUTPUT, output_buffer);
    }
    com_send_log(TERMINAL_OUTPUT, "-------------------------------------------------------------------------------------------------------------------");

    // Free the allocated memory
    vPortFree(freertos_task_status_array);
}

void TerminalTask::_handle_mem(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Memory (Heap):");
    com_send_log(TERMINAL_OUTPUT, "  Total: %s", TerminalTask::_format_bytes(ESP.getHeapSize()));
    com_send_log(TERMINAL_OUTPUT, "  Free:  %s", TerminalTask::_format_bytes(ESP.getFreeHeap()));
    com_send_log(TERMINAL_OUTPUT, "  Min Free: %s", TerminalTask::_format_bytes(ESP.getMinFreeHeap()));
}

void TerminalTask::_handle_reboot(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Rebooting...");

    // ...short break before restarting
    delayMicroseconds(ONE_SECOND_MICROSECONDS);

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "");

    ESP.restart();
}

void TerminalTask::_handle_mpu_data(String &args)
{
    if (!TerminalTask::_check_mpu6050_sensor_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "Acc: x=%.2f, y=%.2f, z=%.2f | Gyro: x=%.2f, y=%.2f, z=%.2f | Temp: %.2f C",
                 _mpu6050_sensor->readings.accelerometer.x,
                 _mpu6050_sensor->readings.accelerometer.y,
                 _mpu6050_sensor->readings.accelerometer.z,
                 _mpu6050_sensor->readings.gyroscope.x,
                 _mpu6050_sensor->readings.gyroscope.y,
                 _mpu6050_sensor->readings.gyroscope.z,
                 _mpu6050_sensor->readings.temperature_celsius);
}

void TerminalTask::_handle_mpu_config(String &args)
{
    if (!TerminalTask::_check_mpu6050_sensor_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "MPU6050 Settings:");
    com_send_log(TERMINAL_OUTPUT, "  Gyro Range: %d DPS", _mpu6050_sensor->getGyroscopeRange());
    com_send_log(TERMINAL_OUTPUT, "  Accel Range: %d G", _mpu6050_sensor->getAccelerometerRange());
    com_send_log(TERMINAL_OUTPUT, "  LPF Bandwidth: %d Hz", _mpu6050_sensor->getLpfBandwidth());
}

void TerminalTask::_handle_mpu_calibrate(String &args)
{
    if (!TerminalTask::_check_mpu6050_sensor_available())
        return;

    com_send_log(LOG_INFO, "Calibrating MPU6050 sensor...");
    _mpu6050_sensor->calibrate();
    com_send_log(LOG_INFO, "MPU6050 sensor calibration complete.");
}

void TerminalTask::_handle_ibus_data(String &args)
{
    if (!TerminalTask::_check_ibus_receiver_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "IBUS Channels:");
    for (int i = 0; i < _ibus_receiver_task->getChannelCount(); ++i)
    {
        com_send_log(TERMINAL_OUTPUT, "  CH%d: %d", i + 1, _ibus_receiver_task->getChannel(i));
    }
}

void TerminalTask::_handle_ibus_status(String &args)
{
    if (!TerminalTask::_check_ibus_receiver_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "IBUS Status: Task Available");
}

void TerminalTask::_handle_motor_throttle(String &args)
{
    if (!TerminalTask::_check_motor_task_available())
        return;

    // Parse arguments: <motor_id> <throttle_value>
    int space_index = args.indexOf(' ');
    if (space_index == -1)
    {
        com_send_log(LOG_ERROR, "Usage: motor.throttle <motor_id> <throttle_value>");
        return;
    }

    String motor_id_str = args.substring(0, space_index);
    String throttle_str = args.substring(space_index + 1);

    uint8_t motor_id = motor_id_str.toInt();
    uint16_t throttle_value = throttle_str.toInt();

    if (motor_id >= NUM_MOTORS)
    {
        com_send_log(LOG_ERROR, "Invalid motor ID: %d. Must be between 0 and %d.", motor_id, NUM_MOTORS - 1);
        return;
    }

    if (throttle_value > MAX_THROTTLE_VALUE)
    {
        com_send_log(LOG_ERROR, "Invalid throttle value: %d. Must be between 0 and %d.", throttle_value, MAX_THROTTLE_VALUE);
        return;
    }

    _motor_task->setThrottle(motor_id, throttle_value);
    com_send_log(TERMINAL_OUTPUT, "Motor %d throttle set to %d.", motor_id, throttle_value);
}

void TerminalTask::_handle_get_setting(String &args)
{
    if (args.length() == 0)
    {
        com_send_log(LOG_ERROR, "Usage: get <key>");
        return;
    }

    const char *internal_key = _settings_manager->getInternalKeyFromDisplayKey(args.c_str());

    if (internal_key == nullptr)
    {
        com_send_log(LOG_ERROR, "Unknown setting: %s", args.c_str());
        return;
    }

    String value = _settings_manager->getSettingValueHumanReadable(internal_key);
    const char *description = _settings_manager->getSettingDescription(internal_key); // Use internal_key for description
    if (value.length() > 0)
    {
        com_send_log(TERMINAL_OUTPUT, "%s (%s): %s", args.c_str(), description, value.c_str());
    }
    else
    {
        // This case should ideally not be reached if internal_key is valid and setting exists
        com_send_log(LOG_ERROR, "Could not retrieve value for setting: %s", args.c_str());
    }
}

void TerminalTask::_handle_set_setting(String &args)
{
    int equals_index = args.indexOf('=');
    if (equals_index == -1)
    {
        com_send_log(LOG_ERROR, "Usage: set <key> = <value>");
        return;
    }

    String display_key = args.substring(0, equals_index);
    display_key.trim();
    String value_str = args.substring(equals_index + 1);
    value_str.trim();

    const char *internal_key = _settings_manager->getInternalKeyFromDisplayKey(display_key.c_str());

    if (internal_key == nullptr)
    {
        com_send_log(LOG_ERROR, "Unknown setting: %s", display_key.c_str());
        return;
    }

    if (_settings_manager->setSettingValue(internal_key, value_str))
    {
        com_send_log(LOG_INFO, "Set %s to %s", display_key.c_str(), _settings_manager->getSettingValueHumanReadable(internal_key).c_str());
    }
    else
    {
        const char *description = _settings_manager->getSettingDescription(internal_key);
        if (strcmp(description, "Unknown Setting") != 0)
        {
            com_send_log(LOG_ERROR, "Failed to set %s (%s) to %s. Invalid value or out of range.", display_key.c_str(), description, value_str.c_str());
        }
        else
        {
            com_send_log(LOG_ERROR, "Unknown setting: %s", display_key.c_str());
        }
    }
}

void TerminalTask::_handle_save_settings(String &args)
{
    _settings_manager->saveSettings();
    com_send_log(LOG_INFO, "Settings saved to NVS.");
}

void TerminalTask::_handle_factory_reset(String &args)
{
    if (args.equalsIgnoreCase("confirm"))
    {
        _settings_manager->factoryReset();
        com_send_log(LOG_INFO, "Settings reset to default values. Rebooting...");
        ESP.restart();
    }
    else
    {
        com_send_log(LOG_WARN, "Factory reset will erase all settings. To confirm, type 'factory_reset confirm'");
    }
}

void TerminalTask::_handle_list_settings(String &args)
{
    _settings_manager->listSettings();
}

void TerminalTask::_handle_dump_settings(String &args)
{
    _settings_manager->dumpSettings();
}

// --- Helper Functions ---

bool TerminalTask::_check_mpu6050_sensor_available()
{
    if (!_mpu6050_sensor)
    {
        com_send_log(LOG_ERROR, "MPU6050 sensor not available.");
        return false;
    }
    return true;
}

bool TerminalTask::_check_ibus_receiver_available()
{
    if (!_ibus_receiver_task)
    {
        com_send_log(LOG_ERROR, "IBUS receiver not available.");
        return false;
    }
    return true;
}

bool TerminalTask::_check_motor_task_available()
{
    if (!_motor_task)
    {
        com_send_log(LOG_ERROR, "Motor task not available.");
        return false;
    }
    return true;
}

bool TerminalTask::_check_pid_task_available()
{
    if (!_pid_task)
    {
        com_send_log(LOG_ERROR, "PID task not available.");
        return false;
    }
    return true;
}

void TerminalTask::_show_prompt()
{
    com_flush_output();
    String system_name = _settings_manager->getSettingValue(SettingsManager::KEY_SYSTEM_NAME);
    String prompt = "[" + system_name + " ~]$";
    com_send_prompt(prompt.c_str());
}

const char *TerminalTask::_format_bytes(uint32_t bytes)
{
    if (bytes < BYTES_IN_KB)
    {
        snprintf(_byte_buffer, sizeof(_byte_buffer), "%lu B", bytes);
    }
    else if (bytes < BYTES_IN_MB)
    {
        snprintf(_byte_buffer, sizeof(_byte_buffer), "%.1f KB", (float)bytes / BYTES_IN_KB);
    }
    else
    {
        snprintf(_byte_buffer, sizeof(_byte_buffer), "%.1f MB", (float)bytes / BYTES_IN_MB);
    }
    return _byte_buffer;
}

void TerminalTask::_parse_command(String &command_line)
{
    command_line.trim();
    if (command_line.length() == 0)
        return;

    // Iterate through commands to find the longest matching command name
    int best_match_index = -1;
    int longest_match_len = 0;

    for (int i = 0; i < _num_commands; i++)
    {
        const char *cmd_name = _commands[i].name;
        int cmd_name_len = strlen(cmd_name);

        // Check if command_line starts with cmd_name
        if (command_line.startsWith(cmd_name))
        {
            // Check if it's an exact match or followed by a space
            if (command_line.length() == cmd_name_len || command_line.charAt(cmd_name_len) == ' ')
            {
                if (cmd_name_len > longest_match_len)
                {
                    longest_match_len = cmd_name_len;
                    best_match_index = i;
                }
            }
        }
    }

    if (best_match_index != -1)
    {
        // Found a matching command
        const Command &matched_command = _commands[best_match_index];
        String args = "";
        if (command_line.length() > longest_match_len)
        {
            args = command_line.substring(longest_match_len + 1); // +1 to skip the space
            args.trim();
        }
        (this->*matched_command.handler)(args);
        return;
    }

    // No command found
    com_send_log(LOG_ERROR, "Unknown command: %s", command_line.c_str());
}

void TerminalTask::_handle_pid_get(String &args)
{
    if (!TerminalTask::_check_pid_task_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "PID Gains (scaled by 100):");
    com_send_log(TERMINAL_OUTPUT, "  Roll:  P=%d, I=%d, D=%d",
                 (int)(_pid_task->getGains(PidAxis::ROLL).p * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::ROLL).i * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::ROLL).d * 100.0f));
    com_send_log(TERMINAL_OUTPUT, "  Pitch: P=%d, I=%d, D=%d",
                 (int)(_pid_task->getGains(PidAxis::PITCH).p * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::PITCH).i * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::PITCH).d * 100.0f));
    com_send_log(TERMINAL_OUTPUT, "  Yaw:   P=%d, I=%d, D=%d",
                 (int)(_pid_task->getGains(PidAxis::YAW).p * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::YAW).i * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::YAW).d * 100.0f));
}

void TerminalTask::_handle_pid_set(String &args)
{
    if (!TerminalTask::_check_pid_task_available())
        return;

    // Parse arguments: <axis> <p|i|d> <value>
    int first_space = args.indexOf(' ');
    if (first_space == -1)
    {
        com_send_log(LOG_ERROR, "Usage: set pid <axis> <p|i|d> <value> (value is scaled by 100)");
        return;
    }
    int second_space = args.indexOf(' ', first_space + 1);
    if (second_space == -1)
    {
        com_send_log(LOG_ERROR, "Usage: set pid <axis> <p|i|d> <value> (value is scaled by 100)");
        return;
    }

    String axis_str = args.substring(0, first_space);
    String gain_str = args.substring(first_space + 1, second_space);
    String value_str = args.substring(second_space + 1);

    PidAxis axis;
    if (axis_str.equalsIgnoreCase("roll"))
    {
        axis = PidAxis::ROLL;
    }
    else if (axis_str.equalsIgnoreCase("pitch"))
    {
        axis = PidAxis::PITCH;
    }
    else if (axis_str.equalsIgnoreCase("yaw"))
    {
        axis = PidAxis::YAW;
    }
    else
    {
        com_send_log(LOG_ERROR, "Invalid axis: %s. Must be 'roll', 'pitch', or 'yaw'.", axis_str.c_str());
        return;
    }

    PidGains gains = _pid_task->getGains(axis);
    // Parse as integer and scale down to float
    int int_value = value_str.toInt();
    float value = (float)int_value / 100.0f;

    if (gain_str.equalsIgnoreCase("p"))
    {
        gains.p = value;
    }
    else if (gain_str.equalsIgnoreCase("i"))
    {
        gains.i = value;
    }
    else if (gain_str.equalsIgnoreCase("d"))
    {
        gains.d = value;
    }
    else
    {
        com_send_log(LOG_ERROR, "Invalid gain: %s. Must be 'p', 'i', or 'd'.", gain_str.c_str());
        return;
    }

    _pid_task->setGains(axis, gains);
    com_send_log(TERMINAL_OUTPUT, "Set %s %s to %d (%.2f actual).", axis_str.c_str(), gain_str.c_str(), int_value, value);
}

void TerminalTask::_handle_pid_reset_defaults(String &args)
{
    if (!TerminalTask::_check_pid_task_available())
        return;

    if (args.equalsIgnoreCase("confirm"))
    {
        _pid_task->resetToDefaults();
        com_send_log(LOG_INFO, "PID gains reset to default values.");
    }
    else
    {
        com_send_log(LOG_WARN, "Reset PID gains to default values. To confirm, type 'reset pid confirm'");
    }
}
