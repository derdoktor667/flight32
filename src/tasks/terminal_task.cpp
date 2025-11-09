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

    {"get imu.data", &TerminalTask::_handle_imu_data, "Displays the latest IMU readings.", CommandCategory::IMU},
    {"get imu.config", &TerminalTask::_handle_imu_config, "Shows the current IMU settings.", CommandCategory::IMU},
    {"set imu.calibrate", &TerminalTask::_handle_imu_calibrate, "Calibrates the IMU.", CommandCategory::IMU},

    {"get rx.data", &TerminalTask::_handle_rx_data, "Shows the latest RX channel data.", CommandCategory::RX},
    {"get rx.status", &TerminalTask::_handle_rx_status, "Shows the RX connection status.", CommandCategory::RX},
    {"get ppm.pin", &TerminalTask::_handle_get_setting, "Gets the PPM input pin.", CommandCategory::RX},
    {"set ppm.pin", &TerminalTask::_handle_set_setting, "Sets the PPM input pin (GPIO number).", CommandCategory::RX},
    {"set rx.protocol", &TerminalTask::_handle_rx_protocol, "Sets the RX protocol (e.g., 'set rx.protocol IBUS'). Available: IBUS, PPM.", CommandCategory::RX},
    {"get rx.value.all", &TerminalTask::_handle_rx_value_all, "Shows all mapped RX channel values.", CommandCategory::RX},
    {"get rx.value.roll", &TerminalTask::_handle_rx_value_single, "Shows the RX Roll channel value.", CommandCategory::RX},
    {"get rx.value.pitch", &TerminalTask::_handle_rx_value_single, "Shows the RX Pitch channel value.", CommandCategory::RX},
    {"get rx.value.throttle", &TerminalTask::_handle_rx_value_single, "Shows the RX Throttle channel value.", CommandCategory::RX},
    {"get rx.value.yaw", &TerminalTask::_handle_rx_value_single, "Shows the RX Yaw channel value.", CommandCategory::RX},
    {"get rx.value.arm", &TerminalTask::_handle_rx_value_single, "Shows the RX Arm channel value.", CommandCategory::RX},
    {"get rx.value.fmode", &TerminalTask::_handle_rx_value_single, "Shows the RX Flight Mode channel value.", CommandCategory::RX},
    {"get rx.value.aux1", &TerminalTask::_handle_rx_value_single, "Shows the RX Auxiliary 1 channel value.", CommandCategory::RX},
    {"get rx.value.aux2", &TerminalTask::_handle_rx_value_single, "Shows the RX Auxiliary 2 channel value.", CommandCategory::RX},
    {"get rx.value.aux3", &TerminalTask::_handle_rx_value_single, "Shows the RX Auxiliary 3 channel value.", CommandCategory::RX},
    {"get rx.value.aux4", &TerminalTask::_handle_rx_value_single, "Shows the RX Auxiliary 4 channel value.", CommandCategory::RX},

    {"set rx.channel.roll", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Roll channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.pitch", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Pitch channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.throttle", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Throttle channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.yaw", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Yaw channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.arm", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Arm channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.fmode", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Flight Mode channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux1", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Auxiliary 1 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux2", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Auxiliary 2 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux3", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Auxiliary 3 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux4", &TerminalTask::_handle_rx_channel_mapping, "Sets the RX Auxiliary 4 channel index (1-based).", CommandCategory::RX},

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

const CategoryInfo TerminalTask::_category_info[] = {
    {CommandCategory::SYSTEM, "system", "System commands and settings"},
    {CommandCategory::IMU, "imu", "IMU sensor commands"},
    {CommandCategory::RX, "rx", "Receiver commands"},
    {CommandCategory::MOTOR, "motor", "Motor control commands"},
    {CommandCategory::PID, "pid", "PID controller commands"},
    {CommandCategory::SETTINGS, "settings", "Settings management commands"},
};
const int TerminalTask::_num_categories = sizeof(TerminalTask::_category_info) / sizeof(CategoryInfo);

TerminalTask::TerminalTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms),
      _scheduler(scheduler),
      _imu_task(imu_task),
      _rx_task(rx_task),
      _motor_task(motor_task),
      _pid_task(pid_task),
      _settings_manager(settings_manager)
{
}

void TerminalTask::setup()
{
    _input_buffer.reserve(TERMINAL_INPUT_BUFFER_SIZE);
}

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

        // --- Core Commands ---
        com_send_log(TERMINAL_OUTPUT, "Core Commands:");

        // Calculate max length for core commands
        int max_core_cmd_len = 0;
        const char* core_commands[] = {"status", "tasks", "mem", "reboot", "factory_reset"};
        const char* core_commands_desc[] = {
            "Shows firmware information.",
            "Shows information about running tasks.",
            "Shows current memory usage.",
            "Reboots the ESP32.",
            "Resets all settings to their default values."
        };
        int num_core_commands = sizeof(core_commands) / sizeof(core_commands[0]);

        for (int i = 0; i < num_core_commands; ++i) {
            int len = strlen(core_commands[i]);
            if (len > max_core_cmd_len) {
                max_core_cmd_len = len;
            }
        }
        max_core_cmd_len += TERMINAL_COLUMN_BUFFER_WIDTH; // Add buffer

        // Print header for core commands
        com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_core_cmd_len, "Command", "Description");
        String core_separator = "  ";
        for (int i = 0; i < max_core_cmd_len; ++i) {
            core_separator += "-";
        }
        core_separator += "--------------------------------------------------";
        com_send_log(TERMINAL_OUTPUT, core_separator.c_str());

        for (int i = 0; i < num_core_commands; ++i)
        {
            com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_core_cmd_len, core_commands[i], core_commands_desc[i]);
        }
        com_send_log(TERMINAL_OUTPUT, core_separator.c_str());
        com_send_log(TERMINAL_OUTPUT, "");

        // --- Available Command Categories ---
        com_send_log(TERMINAL_OUTPUT, "Available Command Categories:");

        // Calculate max length for category prefixes
        int max_category_prefix_len = 0;
        for (int i = 0; i < _num_categories; ++i)
        {
            int len = strlen(_category_info[i].prefix);
            if (len > max_category_prefix_len)
            {
                max_category_prefix_len = len;
            }
        }
        max_category_prefix_len += TERMINAL_COLUMN_BUFFER_WIDTH; // Add buffer

        // Print header for categories
        com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_category_prefix_len, "Category", "Description");
        String category_separator = "  ";
        for (int i = 0; i < max_category_prefix_len; ++i) {
            category_separator += "-";
        }
        category_separator += "--------------------------------------------------";
        com_send_log(TERMINAL_OUTPUT, category_separator.c_str());

        for (int i = 0; i < _num_categories; ++i)
        {
            com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_category_prefix_len, _category_info[i].prefix, _category_info[i].description);
        }
        com_send_log(TERMINAL_OUTPUT, category_separator.c_str());
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

        // Calculate max command name length for this category
        int max_command_name_len = 0;
        for (int i = 0; i < _num_commands; ++i)
        {
            if (_commands[i].category == requested_category)
            {
                int len = strlen(_commands[i].name);
                if (len > max_command_name_len)
                {
                    max_command_name_len = len;
                }
            }
        }

        // Add a small buffer for spacing
        max_command_name_len += TERMINAL_COLUMN_BUFFER_WIDTH; 
        if (max_command_name_len < strlen("Command")) { // Ensure header is not cut off
            max_command_name_len = strlen("Command");
        }

        // Print header
        com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_command_name_len, "Command", "Description");
        // Print separator line
        String separator = "  ";
        for (int i = 0; i < max_command_name_len; ++i) {
            separator += "-";
        }
        separator += "--------------------------------------------------"; // Fixed length for description part
        com_send_log(TERMINAL_OUTPUT, separator.c_str());

        for (int i = 0; i < _num_commands; ++i)
        {
            if (_commands[i].category == requested_category)
            {
                com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_command_name_len, _commands[i].name, _commands[i].help);
            }
        }
        com_send_log(TERMINAL_OUTPUT, separator.c_str());
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
    case CommandCategory::IMU:
        return "IMU Sensor";
    case CommandCategory::RX:
        return "Receiver";
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
    if (category_str.equalsIgnoreCase("imu") || category_str.equalsIgnoreCase("sensor"))
        return CommandCategory::IMU;
    if (category_str.equalsIgnoreCase("rx") || category_str.equalsIgnoreCase("receiver"))
        return CommandCategory::RX;
    if (category_str.equalsIgnoreCase("motor") || category_str.equalsIgnoreCase("motors"))
        return CommandCategory::MOTOR;
    if (category_str.equalsIgnoreCase("pid") || category_str.equalsIgnoreCase("controller"))
        return CommandCategory::PID;
    if (category_str.equalsIgnoreCase("settings") || category_str.equalsIgnoreCase("config"))
        return CommandCategory::SETTINGS;
    return CommandCategory::UNKNOWN;
}

// Helper to determine the category of a setting based on its display_key
CommandCategory TerminalTask::_get_setting_category(const char *display_key)
{
    String key_str = String(display_key);
    if (key_str.startsWith("system."))
        return CommandCategory::SYSTEM;
    if (key_str.startsWith("gyro.") || key_str.startsWith("imu."))
        return CommandCategory::IMU;
    if (key_str.startsWith("rx."))
        return CommandCategory::RX;
    if (key_str.startsWith("motor."))
        return CommandCategory::MOTOR;
    if (key_str.startsWith("pid."))
        return CommandCategory::PID;
    // Settings that don't fit neatly into other categories, or general settings
    return CommandCategory::SETTINGS;
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

    // Calculate max length for labels
    int max_label_len = 0;
    const char* labels[] = {"Total:", "Free:", "Min Free:"};
    for (int i = 0; i < sizeof(labels) / sizeof(labels[0]); ++i) {
        int len = strlen(labels[i]);
        if (len > max_label_len) {
            max_label_len = len;
        }
    }
    max_label_len += TERMINAL_COLUMN_BUFFER_WIDTH; // Add a small buffer

    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Total:", TerminalTask::_format_bytes(ESP.getHeapSize()));
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Free:", TerminalTask::_format_bytes(ESP.getFreeHeap()));
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Min Free:", TerminalTask::_format_bytes(ESP.getMinFreeHeap()));
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

void TerminalTask::_handle_imu_data(String &args)
{
    if (!TerminalTask::_check_imu_task_available())
        return;

    const ImuData &data = _imu_task->getImuSensor().getData();
    com_send_log(TERMINAL_OUTPUT, "Acc: x=%.2f, y=%.2f, z=%.2f | Gyro: x=%.2f, y=%.2f, z=%.2f | Temp: %.2f C",
                 data.accelX, data.accelY, data.accelZ,
                 data.gyroX, data.gyroY, data.gyroZ,
                 data.temp);
}

void TerminalTask::_handle_imu_config(String &args)
{
    if (!TerminalTask::_check_imu_task_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "IMU Settings: (Note: Specific settings depend on the sensor type)");
    // This is now generic. We can't know the specific settings of the sensor.
    // We could add virtual functions to the ImuSensor interface to get this info.
    // For now, we just print a generic message.
}

void TerminalTask::_handle_imu_calibrate(String &args)
{
    if (!TerminalTask::_check_imu_task_available())
        return;

    com_send_log(LOG_INFO, "Calibrating IMU sensor...");
    _imu_task->getImuSensor().calibrate();
    com_send_log(LOG_INFO, "IMU sensor calibration complete.");
}

void TerminalTask::_handle_rx_data(String &args)
{
    if (!TerminalTask::_check_rx_task_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "RX Channels:");

    // Calculate max length for "CHX" (e.g., "CH10")
    int max_ch_len = 0;
    for (int i = 0; i < TERMINAL_RX_DATA_DISPLAY_CHANNELS; ++i)
    {
        String ch_str = "CH" + String(i + 1);
        if (ch_str.length() > max_ch_len) {
            max_ch_len = ch_str.length();
        }
    }
    max_ch_len += TERMINAL_COLUMN_BUFFER_WIDTH; // Add a small buffer

    // Print header
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_ch_len, "Channel", "Value");
    String separator = "  ";
    for (int i = 0; i < max_ch_len; ++i) {
        separator += "-";
    }
    separator += "--------------------"; // Fixed length for value part
    com_send_log(TERMINAL_OUTPUT, separator.c_str());

    // Print all channels
    for (int i = 0; i < TERMINAL_RX_DATA_DISPLAY_CHANNELS; ++i)
    {
        String ch_str = "CH" + String(i + 1);
        com_send_log(TERMINAL_OUTPUT, "  %-*s %d", max_ch_len, ch_str.c_str(), _rx_task->getChannel(i));
    }
    com_send_log(TERMINAL_OUTPUT, separator.c_str());
}

void TerminalTask::_handle_rx_status(String &args)
{
    if (!TerminalTask::_check_rx_task_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "RX Status: Task Available");
}

void TerminalTask::_handle_rx_protocol(String &args)
{
    if (args.length() == 0)
    {
        // Get current RC protocol
        String current_protocol = _settings_manager->getSettingValueHumanReadable(KEY_RC_PROTOCOL_TYPE);
        com_send_log(TERMINAL_OUTPUT, "Current RC Protocol: %s", current_protocol.c_str());
        com_send_log(TERMINAL_OUTPUT, "Available protocols: IBUS, CRSF, SBUS");
    }
    else
    {
        // Set RC protocol
        if (_settings_manager->setSettingValue(KEY_RC_PROTOCOL_TYPE, args))
        {
            com_send_log(LOG_INFO, "RC Protocol set to %s. Reboot to apply changes.", _settings_manager->getSettingValueHumanReadable(KEY_RC_PROTOCOL_TYPE).c_str());
        }
        else
        {
            com_send_log(LOG_ERROR, "Failed to set RC Protocol to %s. Invalid protocol or value.", args.c_str());
        }
    }
}

void TerminalTask::_handle_rx_value_single(String &args)
{
    if (!TerminalTask::_check_rx_task_available())
        return;

    // Extract the channel name from the command (e.g., "roll" from "get rx.value.roll")
    int last_dot_index = args.lastIndexOf('.');
    String channel_name = args.substring(last_dot_index + 1);

    const char *key = nullptr;
    if (channel_name.equalsIgnoreCase("roll"))
        key = SettingsManager::KEY_RC_CHANNEL_ROLL;
    else if (channel_name.equalsIgnoreCase("pitch"))
        key = SettingsManager::KEY_RC_CHANNEL_PITCH;
    else if (channel_name.equalsIgnoreCase("throttle"))
        key = SettingsManager::KEY_RC_CHANNEL_THRO;
    else if (channel_name.equalsIgnoreCase("yaw"))
        key = SettingsManager::KEY_RC_CHANNEL_YAW;
    else if (channel_name.equalsIgnoreCase("arm"))
        key = SettingsManager::KEY_RC_CHANNEL_ARM;
    else if (channel_name.equalsIgnoreCase("fmode"))
        key = SettingsManager::KEY_RC_CHANNEL_FMODE;
    else if (channel_name.equalsIgnoreCase("aux1"))
        key = SettingsManager::KEY_RC_CHANNEL_AUX1;
    else if (channel_name.equalsIgnoreCase("aux2"))
        key = SettingsManager::KEY_RC_CHANNEL_AUX2;
    else if (channel_name.equalsIgnoreCase("aux3"))
        key = SettingsManager::KEY_RC_CHANNEL_AUX3;
    else if (channel_name.equalsIgnoreCase("aux4"))
        key = SettingsManager::KEY_RC_CHANNEL_AUX4;
    else
    {
        com_send_log(LOG_ERROR, "Unknown RX channel: %s", channel_name.c_str());
        return;
    }

    int channel_index = _settings_manager->getSettingValue(key).toInt();
    int16_t value = _rx_task->getChannel(channel_index);
    
    // Determine the longest possible channel description for consistent formatting
    // "RX Flight Mode (CH14)" is likely the longest.
    const int fixed_desc_width = TERMINAL_RX_SINGLE_DESC_WIDTH;

    String desc_str = "RX " + channel_name + " (CH" + String(channel_index + 1) + ")";
    com_send_log(TERMINAL_OUTPUT, "  %-*s %d", fixed_desc_width, desc_str.c_str(), value);
}

void TerminalTask::_handle_rx_value_all(String &args)
{
    if (!TerminalTask::_check_rx_task_available())
        return;

    com_send_log(TERMINAL_OUTPUT, "All Mapped RX Channels:");

    // Define channel names and their corresponding setting keys
    struct ChannelInfo {
        const char* name;
        const char* key;
    };

    const ChannelInfo channel_map[] = {
        {"Roll", SettingsManager::KEY_RC_CHANNEL_ROLL},
        {"Pitch", SettingsManager::KEY_RC_CHANNEL_PITCH},
        {"Throttle", SettingsManager::KEY_RC_CHANNEL_THRO},
        {"Yaw", SettingsManager::KEY_RC_CHANNEL_YAW},
        {"Arm", SettingsManager::KEY_RC_CHANNEL_ARM},
        {"Flight Mode", SettingsManager::KEY_RC_CHANNEL_FMODE},
        {"Aux1", SettingsManager::KEY_RC_CHANNEL_AUX1},
        {"Aux2", SettingsManager::KEY_RC_CHANNEL_AUX2},
        {"Aux3", SettingsManager::KEY_RC_CHANNEL_AUX3},
        {"Aux4", SettingsManager::KEY_RC_CHANNEL_AUX4}
    };
    const int num_mapped_channels = sizeof(channel_map) / sizeof(channel_map[0]);

    // Calculate max length for the descriptive part (e.g., "Roll (CHX)")
    int max_desc_len = 0;
    for (int i = 0; i < num_mapped_channels; ++i) {
        int channel_index_1_based = _settings_manager->getSettingValue(channel_map[i].key).toInt() + 1;
        // Format: "Name (CHX)"
        String desc_str = String(channel_map[i].name) + " (CH" + String(channel_index_1_based) + ")";
        if (desc_str.length() > max_desc_len) {
            max_desc_len = desc_str.length();
        }
    }
    max_desc_len += TERMINAL_COLUMN_BUFFER_WIDTH; // Add a small buffer

    // Print header
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_desc_len, "Channel", "Value");
    String separator = "  ";
    for (int i = 0; i < max_desc_len; ++i) {
        separator += "-";
    }
    separator += "--------------------"; // Fixed length for value part
    com_send_log(TERMINAL_OUTPUT, separator.c_str());

    // Print all mapped channels
    for (int i = 0; i < num_mapped_channels; ++i) {
        int channel_index_0_based = _settings_manager->getSettingValue(channel_map[i].key).toInt();
        int channel_index_1_based = channel_index_0_based + 1;
        int16_t value = _rx_task->getChannel(channel_index_0_based);
        String desc_str = String(channel_map[i].name) + " (CH" + String(channel_index_1_based) + ")";
        com_send_log(TERMINAL_OUTPUT, "  %-*s %d", max_desc_len, desc_str.c_str(), value);
    }
    com_send_log(TERMINAL_OUTPUT, separator.c_str());
}

void TerminalTask::_handle_rx_channel_mapping(String &args)
{
    // Expected format: <channel_name> <channel_index_1_based>
    int space_index = args.indexOf(' ');
    if (space_index == -1)
    {
        com_send_log(LOG_ERROR, "Usage: set rx.channel.<name> <1-based_index>");
        return;
    }

    String channel_name_arg = args.substring(0, space_index);
    String index_str = args.substring(space_index + 1);
    int channel_index_1_based = index_str.toInt();

    if (channel_index_1_based < TERMINAL_MIN_CHANNEL_INDEX || channel_index_1_based > TERMINAL_MAX_CHANNEL_INDEX) // Assuming max 14 channels
    {
        com_send_log(LOG_ERROR, "Invalid channel index: %d. Must be between %d and %d.", channel_index_1_based, TERMINAL_MIN_CHANNEL_INDEX, TERMINAL_MAX_CHANNEL_INDEX);
        return;
    }

    // Convert 1-based to 0-based for internal use
    int channel_index_0_based = channel_index_1_based - 1;

    const char *key = nullptr;
    if (channel_name_arg.equalsIgnoreCase("roll"))
        key = SettingsManager::KEY_RC_CHANNEL_ROLL;
    else if (channel_name_arg.equalsIgnoreCase("pitch"))
        key = SettingsManager::KEY_RC_CHANNEL_PITCH;
    else if (channel_name_arg.equalsIgnoreCase("throttle"))
        key = SettingsManager::KEY_RC_CHANNEL_THRO;
    else if (channel_name_arg.equalsIgnoreCase("yaw"))
        key = SettingsManager::KEY_RC_CHANNEL_YAW;
    else if (channel_name_arg.equalsIgnoreCase("arm"))
        key = SettingsManager::KEY_RC_CHANNEL_ARM;
    else if (channel_name_arg.equalsIgnoreCase("fmode"))
        key = SettingsManager::KEY_RC_CHANNEL_FMODE;
    else if (channel_name_arg.equalsIgnoreCase("aux1"))
        key = SettingsManager::KEY_RC_CHANNEL_AUX1;
    else if (channel_name_arg.equalsIgnoreCase("aux2"))
        key = SettingsManager::KEY_RC_CHANNEL_AUX2;
    else if (channel_name_arg.equalsIgnoreCase("aux3"))
        key = SettingsManager::KEY_RC_CHANNEL_AUX3;
    else if (channel_name_arg.equalsIgnoreCase("aux4"))
        key = SettingsManager::KEY_RC_CHANNEL_AUX4;
    else
    {
        com_send_log(LOG_ERROR, "Unknown RX channel name for mapping: %s", channel_name_arg.c_str());
        return;
    }

    if (_settings_manager->setSettingValue(key, String(channel_index_0_based)))
    {
        com_send_log(LOG_INFO, "RX channel '%s' mapped to physical channel %d. Reboot to apply changes.", channel_name_arg.c_str(), channel_index_1_based);
    }
    else
    {
        com_send_log(LOG_ERROR, "Failed to map RX channel '%s' to physical channel %d.", channel_name_arg.c_str(), channel_index_1_based);
    }
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
        // If internal_key is valid but setting failed, it's due to invalid value or out of range.
        // The description is always available if internal_key is not nullptr.
        com_send_log(LOG_ERROR, "Failed to set %s (%s) to %s. Invalid value or out of range.", display_key.c_str(), _settings_manager->getSettingDescription(internal_key), value_str.c_str());
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
    com_send_log(TERMINAL_OUTPUT, "\n# Settings Dump");

    // Iterate through each category and print settings belonging to it
    for (int cat_idx = 0; cat_idx < _num_categories; ++cat_idx)
    {
        CommandCategory current_category = _category_info[cat_idx].category;
        const char *category_name = _get_category_string(current_category);

        // Skip UNKNOWN category
        if (current_category == CommandCategory::UNKNOWN)
            continue;

        // Calculate max display_key length for this category
        int max_display_key_len = 0;
        bool category_has_settings = false;
        for (int i = 0; i < _settings_manager->_num_settings; ++i)
        {
            const char *display_key = _settings_manager->_settings_metadata[i].display_key;
            if (_get_setting_category(display_key) == current_category)
            {
                int len = strlen(display_key);
                if (len > max_display_key_len)
                {
                    max_display_key_len = len;
                }
                category_has_settings = true;
            }
        }

        if (category_has_settings)
        {
            com_send_log(TERMINAL_OUTPUT, "\n--- %s Settings ---", category_name);
            max_display_key_len += TERMINAL_COLUMN_BUFFER_WIDTH; // Add a small buffer
            for (int i = 0; i < _settings_manager->_num_settings; ++i)
            {
                const char *display_key = _settings_manager->_settings_metadata[i].display_key;
                const char *internal_key = _settings_manager->_settings_metadata[i].key;
                if (_get_setting_category(display_key) == current_category)
                {
                    com_send_log(TERMINAL_OUTPUT, "set %-*s = %s", max_display_key_len, display_key, _settings_manager->getSettingValueHumanReadable(internal_key).c_str());
                }
            }
        }
    }
    com_send_log(TERMINAL_OUTPUT, "\n# End of Dump");
}

// --- Helper Functions ---

bool TerminalTask::_check_imu_task_available()
{
    if (!_imu_task)
    {
        com_send_log(LOG_ERROR, "IMU task not available.");
        return false;
    }
    return true;
}

bool TerminalTask::_check_rx_task_available()
{
    if (!_rx_task)
    {
        com_send_log(LOG_ERROR, "RX task not available.");
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
    com_send_log(TERMINAL_OUTPUT, "");
    String system_name = _settings_manager->getSettingValue(SettingsManager::KEY_SYSTEM_NAME);
    String prompt = "[" + system_name + " ~]$ ";
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

    // Calculate max length for labels
    int max_label_len = 0;
    const char* labels[] = {"Roll:", "Pitch:", "Yaw:"};
    for (int i = 0; i < sizeof(labels) / sizeof(labels[0]); ++i) {
        int len = strlen(labels[i]);
        if (len > max_label_len) {
            max_label_len = len;
        }
    }
    max_label_len += TERMINAL_COLUMN_BUFFER_WIDTH; // Add a small buffer

    // Print header
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Axis", "Gains (P, I, D)");
    String separator = "  ";
    for (int i = 0; i < max_label_len; ++i) {
        separator += "-";
    }
    separator += "-------------------------"; // Fixed length for "Gains (P, I, D)" part
    com_send_log(TERMINAL_OUTPUT, separator.c_str());

    com_send_log(TERMINAL_OUTPUT, "  %-*s P=%d, I=%d, D=%d", max_label_len, "Roll:",
                 (int)(_pid_task->getGains(PidAxis::ROLL).p * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::ROLL).i * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::ROLL).d * 100.0f));
    com_send_log(TERMINAL_OUTPUT, "  %-*s P=%d, I=%d, D=%d", max_label_len, "Pitch:",
                 (int)(_pid_task->getGains(PidAxis::PITCH).p * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::PITCH).i * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::PITCH).d * 100.0f));
    com_send_log(TERMINAL_OUTPUT, "  %-*s P=%d, I=%d, D=%d", max_label_len, "Yaw:",
                 (int)(_pid_task->getGains(PidAxis::YAW).p * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::YAW).i * 100.0f),
                 (int)(_pid_task->getGains(PidAxis::YAW).d * 100.0f));
    com_send_log(TERMINAL_OUTPUT, separator.c_str());
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
    com_send_log(TERMINAL_OUTPUT, "Set %s %s to %d.", axis_str.c_str(), gain_str.c_str(), int_value);
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
