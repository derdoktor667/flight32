/**
 * @file terminal.cpp
 * @brief Implements the Terminal class for handling CLI terminal commands and parsing.
 * @author Wastl Kraus
 * @date 2025-11-12
 * @license MIT
 */

#include "terminal.h"
#include "../firmware_info.h"

const ChannelMapping Terminal::_channel_map[] = {
    {"roll", SettingsManager::KEY_RC_CHANNEL_ROLL},
    {"pitch", SettingsManager::KEY_RC_CHANNEL_PITCH},
    {"throttle", SettingsManager::KEY_RC_CHANNEL_THRO},
    {"yaw", SettingsManager::KEY_RC_CHANNEL_YAW},
    {"arm", SettingsManager::KEY_RC_CHANNEL_ARM},
    {"fmode", SettingsManager::KEY_RC_CHANNEL_FMODE},
    {"aux1", SettingsManager::KEY_RC_CHANNEL_AUX1},
    {"aux2", SettingsManager::KEY_RC_CHANNEL_AUX2},
    {"aux3", SettingsManager::KEY_RC_CHANNEL_AUX3},
    {"aux4", SettingsManager::KEY_RC_CHANNEL_AUX4}};
constexpr int Terminal::_num_channel_mappings = sizeof(Terminal::_channel_map) / sizeof(ChannelMapping);

const Command Terminal::_commands[] = {
    {"help", &Terminal::_handle_help, "Shows this help message.", CommandCategory::SYSTEM},
    {"status", &Terminal::_handle_status, "Shows firmware information.", CommandCategory::SYSTEM},
    {"tasks", &Terminal::_handle_tasks, "Shows information about running tasks.", CommandCategory::SYSTEM},
    {"mem", &Terminal::_handle_mem, "Shows current memory usage.", CommandCategory::SYSTEM},
    {"reboot", &Terminal::_handle_reboot, "Reboots the ESP32.", CommandCategory::SYSTEM},
    {"quit", &Terminal::_handle_quit, "Exits the terminal session.", CommandCategory::SYSTEM},

    {"get imu.data", &Terminal::_handle_imu_data, "Displays the latest IMU readings.", CommandCategory::IMU},
    {"set imu.calibrate", &Terminal::_handle_imu_calibrate, "Calibrates the IMU.", CommandCategory::IMU},
    {"get imu.lpf_bandwidth", &Terminal::_handle_imu_lpf_bandwidth, "Gets the current IMU LPF bandwidth.", CommandCategory::IMU},
    {"set imu.lpf_bandwidth", &Terminal::_handle_imu_lpf_bandwidth, "Sets the IMU LPF bandwidth (e.g., 'set imu.lpf_bw LPF_42HZ_N_5MS').", CommandCategory::IMU},

    {"get rx.data", &Terminal::_handle_rx_data, "Shows the latest RX channel data.", CommandCategory::RX},
    {"get rx.status", &Terminal::_handle_rx_status, "Shows the RX connection status.", CommandCategory::RX},
    {"get ppm.pin", &Terminal::_handle_get_setting, "Gets the PPM input pin.", CommandCategory::RX},
    {"set ppm.pin", &Terminal::_handle_set_setting, "Sets the PPM input pin (GPIO number).", CommandCategory::RX},
    {"set rx.protocol", &Terminal::_handle_rx_protocol, "Sets the RX protocol (e.g., 'set rx.protocol IBUS'). Available: IBUS, PPM.", CommandCategory::RX},
    {"get rx.value.all", &Terminal::_handle_rx_value_all, "Shows all mapped RX channel values.", CommandCategory::RX},
    {"get rx.value.roll", &Terminal::_handle_rx_value_single, "Shows the RX Roll channel value.", CommandCategory::RX},
    {"get rx.value.pitch", &Terminal::_handle_rx_value_single, "Shows the RX Pitch channel value.", CommandCategory::RX},
    {"get rx.value.throttle", &Terminal::_handle_rx_value_single, "Shows the RX Throttle channel value.", CommandCategory::RX},
    {"get rx.value.yaw", &Terminal::_handle_rx_value_single, "Shows the RX Yaw channel value.", CommandCategory::RX},
    {"get rx.value.arm", &Terminal::_handle_rx_value_single, "Shows the RX Arm channel value.", CommandCategory::RX},
    {"get rx.value.fmode", &Terminal::_handle_rx_value_single, "Shows the RX Flight Mode channel value.", CommandCategory::RX},
    {"get rx.value.aux1", &Terminal::_handle_rx_value_single, "Shows the RX Auxiliary 1 channel value.", CommandCategory::RX},
    {"get rx.value.aux2", &Terminal::_handle_rx_value_single, "Shows the RX Auxiliary 2 channel value.", CommandCategory::RX},
    {"get rx.value.aux3", &Terminal::_handle_rx_value_single, "Shows the RX Auxiliary 3 channel value.", CommandCategory::RX},
    {"get rx.value.aux4", &Terminal::_handle_rx_value_single, "Shows the RX Auxiliary 4 channel value.", CommandCategory::RX},

    {"set rx.channel.roll", &Terminal::_handle_rx_channel_mapping, "Sets the RX Roll channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.pitch", &Terminal::_handle_rx_channel_mapping, "Sets the RX Pitch channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.throttle", &Terminal::_handle_rx_channel_mapping, "Sets the RX Throttle channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.yaw", &Terminal::_handle_rx_channel_mapping, "Sets the RX Yaw channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.arm", &Terminal::_handle_rx_channel_mapping, "Sets the RX Arm channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.fmode", &Terminal::_handle_rx_channel_mapping, "Sets the RX Flight Mode channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux1", &Terminal::_handle_rx_channel_mapping, "Sets the RX Auxiliary 1 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux2", &Terminal::_handle_rx_channel_mapping, "Sets the RX Auxiliary 2 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux3", &Terminal::_handle_rx_channel_mapping, "Sets the RX Auxiliary 3 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux4", &Terminal::_handle_rx_channel_mapping, "Sets the RX Auxiliary 4 channel index (1-based).", CommandCategory::RX},

    {"set motor.throttle", &Terminal::_handle_motor_throttle, "Sets the throttle for a specific motor (e.g., 'set motor.throttle FL 1000').", CommandCategory::MOTOR},
    {"motor test", &Terminal::_handle_motor_test, "Tests individual motors (e.g., 'motor test FL 20 5000' for motor FL at 20% for 5s).", CommandCategory::MOTOR},
    {"motor stop", &Terminal::_handle_motor_stop, "Stops all motors from testing.", CommandCategory::MOTOR},

    {"get pid", &Terminal::_handle_pid_get, "Gets the current PID gains.", CommandCategory::PID},
    {"set pid", &Terminal::_handle_pid_set, "Sets a PID gain (e.g., 'set pid roll p 0.1').", CommandCategory::PID},
    {"reset pid", &Terminal::_handle_pid_reset_defaults, "Resets PID gains to default values.", CommandCategory::PID},

    {"get", &Terminal::_handle_get_setting, "Gets a setting value (e.g., 'get gyro.resolution').", CommandCategory::SETTINGS},
    {"set", &Terminal::_handle_set_setting, "Sets a setting value (e.g., 'set gyro.resolution = 250_DPS').", CommandCategory::SETTINGS},
    {"save", &Terminal::_handle_save_settings, "Saves all settings to persistent storage.", CommandCategory::SETTINGS},
    {"factory_reset", &Terminal::_handle_factory_reset, "Resets all settings to their default values.", CommandCategory::SETTINGS},
    {"settings", &Terminal::_handle_list_settings, "Lists all available settings.", CommandCategory::SETTINGS},
    {"dump", &Terminal::_handle_dump_settings, "Dumps all settings for backup.", CommandCategory::SETTINGS}};
constexpr int Terminal::_num_commands = sizeof(Terminal::_commands) / sizeof(Command);

const CategoryInfo Terminal::_category_info[] = {
    {CommandCategory::SYSTEM, "system", "System commands and settings"},
    {CommandCategory::IMU, "imu", "IMU sensor commands"},
    {CommandCategory::RX, "rx", "Receiver commands"},
    {CommandCategory::MOTOR, "motor", "Motor control commands"},
    {CommandCategory::PID, "pid", "PID controller commands"},
    {CommandCategory::SETTINGS, "settings", "Settings management commands"},
    {CommandCategory::RC_CHANNELS, "rc.channels", "RC Channel Mapping Settings"},
};
constexpr int Terminal::_num_categories = sizeof(Terminal::_category_info) / sizeof(CategoryInfo);

Terminal::Terminal(Scheduler *scheduler, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager)
    : _scheduler(scheduler),
      _imu_task(imu_task),
      _rx_task(rx_task),
      _motor_task(motor_task),
      _pid_task(pid_task),
      _settings_manager(settings_manager)
{
    _input_buffer.reserve(TERMINAL_INPUT_BUFFER_SIZE);
}

void Terminal::handleInput(char incoming_char)
{
    if (incoming_char == '\n' || incoming_char == '\r')
    {
        if (_input_buffer.length() > 0)
        {
            _parse_command(_input_buffer);
            _input_buffer = "";
        }
        if (!_should_quit)
        { // Only show prompt if not quitting
            showPrompt();
        }
    }
    else if (isPrintable(incoming_char))
    {
        _input_buffer += incoming_char;
    }
    else if (incoming_char == ASCII_BACKSPACE)
    {
        if (_input_buffer.length() > 0)
        {
            _input_buffer.remove(_input_buffer.length() - 1);
        }
    }
}

void Terminal::showPrompt()
{
    com_flush_output();
    com_send_log(TERMINAL_OUTPUT, "");
    String system_name = _settings_manager->getSettingValue(SettingsManager::KEY_SYSTEM_NAME);
    String prompt = "[" + system_name + " ~]";
    com_send_prompt(prompt.c_str());
}

void Terminal::_parse_command(String &command_line)
{
    command_line.trim();
    if (command_line.length() == 0)
        return;

    int best_match_index = -1;
    int longest_match_len = 0;

    for (int i = 0; i < _num_commands; i++)
    {
        const char *cmd_name = _commands[i].name;
        int cmd_name_len = strlen(cmd_name);

        if (command_line.startsWith(cmd_name))
        {
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
        const Command &matched_command = _commands[best_match_index];
        String args = "";
        if (command_line.length() > longest_match_len)
        {
            args = command_line.substring(longest_match_len + 1);
            args.trim();
        }
        (this->*matched_command.handler)(args);
        return;
    }

    com_send_log(LOG_ERROR, "Unknown command: %s", command_line.c_str());
}

const char *Terminal::_get_category_string(CommandCategory category)
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
    case CommandCategory::RC_CHANNELS:
        return "RC Channel Mapping";
    default:
        return "Unknown";
    }
}

CommandCategory Terminal::_get_category_from_string(String &category_str)
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
    if (category_str.equalsIgnoreCase("rc.channels") || category_str.equalsIgnoreCase("channels"))
        return CommandCategory::RC_CHANNELS;
    return CommandCategory::UNKNOWN;
}

CommandCategory Terminal::_get_setting_category(const char *display_key)
{
    String key_str = String(display_key);
    CommandCategory assigned_category = CommandCategory::SETTINGS;

    if (key_str.startsWith("system."))
        assigned_category = CommandCategory::SYSTEM;
    else if (key_str.startsWith("gyro.") || key_str.startsWith("imu.") || key_str.startsWith("mpu."))
        assigned_category = CommandCategory::IMU;
    else if (key_str.startsWith("rx.") || key_str.startsWith("rc.protocol"))
        assigned_category = CommandCategory::RX;
    else if (key_str.startsWith("motor."))
        assigned_category = CommandCategory::MOTOR;
    else if (key_str.startsWith("pid."))
        assigned_category = CommandCategory::PID;
    else if (key_str.startsWith("rc.ch."))
        assigned_category = CommandCategory::RC_CHANNELS;

    return assigned_category;
}

const char *Terminal::_get_motor_name(uint8_t motor_id)
{
    switch ((MotorIndex)motor_id)
    {
    case MotorIndex::FL:
        return "FL";
    case MotorIndex::FR:
        return "FR";
    case MotorIndex::RL:
        return "RL";
    case MotorIndex::RR:
        return "RR";
    default:
        return "UNKNOWN";
    }
}

int8_t Terminal::_get_motor_id(String &motor_name)
{
    if (motor_name.equalsIgnoreCase("FL"))
        return (int8_t)MotorIndex::FL;
    if (motor_name.equalsIgnoreCase("FR"))
        return (int8_t)MotorIndex::FR;
    if (motor_name.equalsIgnoreCase("RL"))
        return (int8_t)MotorIndex::RL;
    if (motor_name.equalsIgnoreCase("RR"))
        return (int8_t)MotorIndex::RR;
    return -1; // Invalid motor name
}

const char *Terminal::_get_task_state_string(eTaskState state)
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

// Command Handlers
void Terminal::_handle_help(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");

    if (args.length() == 0)
    {
        com_send_log(TERMINAL_OUTPUT, "--- Flight32 Terminal Help ---");
        com_send_log(TERMINAL_OUTPUT, "");
        com_send_log(TERMINAL_OUTPUT, "Usage: help <category>");
        com_send_log(TERMINAL_OUTPUT, "");

        com_send_log(TERMINAL_OUTPUT, "Core Commands:");

        int max_core_cmd_len = 0;
        for (int i = 0; i < _num_commands; ++i)
        {
            if (_commands[i].category == CommandCategory::SYSTEM)
            {
                int len = strlen(_commands[i].name);
                if (len > max_core_cmd_len)
                {
                    max_core_cmd_len = len;
                }
            }
        }
        max_core_cmd_len += TERMINAL_COLUMN_BUFFER_WIDTH;

        com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_core_cmd_len, "Command", "Description");
        String core_separator = "  ";
        for (int i = 0; i < max_core_cmd_len; ++i)
        {
            core_separator += "-";
        }
        core_separator += "------------------------------";
        com_send_log(TERMINAL_OUTPUT, core_separator.c_str());

        for (int i = 0; i < _num_commands; ++i)
        {
            if (_commands[i].category == CommandCategory::SYSTEM)
            {
                com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_core_cmd_len, _commands[i].name, _commands[i].help);
            }
        }
        com_send_log(TERMINAL_OUTPUT, core_separator.c_str());
        com_send_log(TERMINAL_OUTPUT, "");

        com_send_log(TERMINAL_OUTPUT, "Available Command Categories:");

        int max_category_prefix_len = 0;
        for (int i = 0; i < _num_categories; ++i)
        {
            int len = strlen(_category_info[i].prefix);
            if (len > max_category_prefix_len)
            {
                max_category_prefix_len = len;
            }
        }
        max_category_prefix_len += TERMINAL_COLUMN_BUFFER_WIDTH;

        com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_category_prefix_len, "Category", "Description");
        String category_separator = "  ";
        for (int i = 0; i < max_category_prefix_len; ++i)
        {
            category_separator += "-";
        }
        category_separator += "------------------------------";
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
        CommandCategory requested_category = _get_category_from_string(args);

        if (requested_category == CommandCategory::UNKNOWN)
        {
            com_send_log(LOG_ERROR, "Unknown help category: '%s'. Type 'help' for a list of categories.", args.c_str());
            return;
        }

        com_send_log(TERMINAL_OUTPUT, "--- %s Commands ---", _get_category_string(requested_category));
        com_send_log(TERMINAL_OUTPUT, "");

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

        max_command_name_len += TERMINAL_COLUMN_BUFFER_WIDTH;
        if (max_command_name_len < strlen("Command"))
        {
            max_command_name_len = strlen("Command");
        }

        int max_description_len = strlen("Description"); // Start with header length
        for (int i = 0; i < _num_commands; ++i)
        {
            if (_commands[i].category == requested_category)
            {
                int len = strlen(_commands[i].help);
                if (len > max_description_len)
                {
                    max_description_len = len;
                }
            }
        }

        com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_command_name_len, "Command", "Description");
        String separator = "  ";
        for (int i = 0; i < max_command_name_len; ++i)
        {
            separator += "-";
        }
        separator += " "; // Space between command and description separator
        for (int i = 0; i < max_description_len; ++i)
        {
            separator += "-";
        }
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

void Terminal::_handle_status(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Flight32 Firmware v%s", FirmwareInfo::getFirmwareVersion());
}

void Terminal::_handle_tasks(String &args)
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

    num_freertos_tasks = uxTaskGetSystemState(freertos_task_status_array, num_freertos_tasks, &total_run_time);

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "% -16s %-10s %-6s %-8s %-10s %-10s %-10s %s",
                 "Task Name", "State", "Prio", "CPU %", "Loop (us)", "Avg (us)", "Max (us)", "Stack HWM (bytes)");
    com_send_log(TERMINAL_OUTPUT, "---------------------------------------------------------------------------------------------------");

    for (uint8_t i = 0; i < _scheduler->getTaskCount(); i++)
    {
        TaskBase *current_task = _scheduler->getTask(i);
        if (!current_task)
            continue;

        const char *name = current_task->getName();
        if (strcmp(name, IDLE_TASK_NAME_0) == 0 || strcmp(name, IDLE_TASK_NAME_1) == 0 || strcmp(name, TIMER_SERVICE_TASK_NAME) == 0)
        {
            continue;
        }

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

        char output_buffer[256];
        snprintf(output_buffer, sizeof(output_buffer),
                 "% -*s %-*s %-*u %-*s %-*u %-*u %-*u %u",
                 TASK_NAME_COLUMN_WIDTH, name,
                 TASK_STATE_COLUMN_WIDTH, Terminal::_get_task_state_string(freertos_status.eCurrentState),
                 TASK_PRIO_COLUMN_WIDTH, freertos_status.uxBasePriority,
                 TASK_CPU_COLUMN_WIDTH, cpu_str,
                 TASK_LOOP_COLUMN_WIDTH, current_task->getLoopTime(),
                 TASK_AVG_LOOP_COLUMN_WIDTH, current_task->getAvgLoopTime(),
                 TASK_MAX_LOOP_COLUMN_WIDTH, current_task->getMaxLoopTime(),
                 freertos_status.usStackHighWaterMark);
        com_send_log(TERMINAL_OUTPUT, output_buffer);
    }
    com_send_log(TERMINAL_OUTPUT, "---------------------------------------------------------------------------------------------------");

    vPortFree(freertos_task_status_array);
}

void Terminal::_handle_mem(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Memory (Heap):");

    int max_label_len = 0;
    const char *labels[] = {"Total:", "Free:", "Min Free:"};
    for (int i = 0; i < sizeof(labels) / sizeof(labels[0]); ++i)
    {
        int len = strlen(labels[i]);
        if (len > max_label_len)
        {
            max_label_len = len;
        }
    }
    max_label_len += TERMINAL_COLUMN_BUFFER_WIDTH;

    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Total:", com_format_bytes(ESP.getHeapSize()));
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Free:", com_format_bytes(ESP.getFreeHeap()));
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Min Free:", com_format_bytes(ESP.getMinFreeHeap()));
}

void Terminal::_handle_reboot(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Rebooting...");

    delayMicroseconds(ONE_SECOND_MICROSECONDS);

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "");

    ESP.restart();
}

void Terminal::_handle_quit(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "Saving settings and exiting...");
    if (_motor_task && _motor_task->isInTestMode())
    {
        _motor_task->stopMotorTest();
    }
    _settings_manager->saveSettings(); // Save settings before quitting
    com_send_log(TERMINAL_OUTPUT, "Goodbye!");
    _should_quit = true;
}

void Terminal::_handle_imu_data(String &args)
{
    if (!_imu_task)
    {
        com_send_log(LOG_ERROR, "IMU task not available.");
        return;
    }

    const ImuData &data = _imu_task->getImuSensor().getData();
    com_send_log(TERMINAL_OUTPUT, "Acc: x=%.2f, y=%.2f, z=%.2f | Gyro: x=%.2f, y=%.2f, z=%.2f | Temp: %.2f C",
                 data.accelX, data.accelY, data.accelZ,
                 data.gyroX, data.gyroY, data.gyroZ,
                 data.temp);
}

void Terminal::_handle_imu_calibrate(String &args)
{
    if (!_imu_task)
    {
        com_send_log(LOG_ERROR, "IMU task not available.");
        return;
    }

    com_send_log(LOG_INFO, "Calibrating IMU sensor...");
    _imu_task->getImuSensor().calibrate();
    com_send_log(LOG_INFO, "IMU sensor calibration complete.");
}

void Terminal::_handle_imu_lpf_bandwidth(String &args)
{
    if (!_imu_task)
    {
        com_send_log(LOG_ERROR, "IMU task not available.");
        return;
    }

    if (args.length() == 0)
    {
        // Get current LPF bandwidth
        String current_lpf_bandwidth_index_str = _settings_manager->getSettingValue(KEY_IMU_LPF_BANDWIDTH);
        String current_lpf_bandwidth_str = _settings_manager->getSettingValueHumanReadable(KEY_IMU_LPF_BANDWIDTH);
        com_send_log(TERMINAL_OUTPUT, "Current IMU LPF Bandwidth: %s (Index: %s)", current_lpf_bandwidth_str.c_str(), current_lpf_bandwidth_index_str.c_str());
        com_send_log(TERMINAL_OUTPUT, "Available options:");
        String options_str = _settings_manager->getSettingOptionsHumanReadable(KEY_IMU_LPF_BANDWIDTH);
        int start_index = 0;
        int end_index = options_str.indexOf(',');
        while (end_index != -1)
        {
            String option = options_str.substring(start_index, end_index);
            option.trim();
            com_send_log(TERMINAL_OUTPUT, "  - %s", option.c_str());
            start_index = end_index + 1;
            end_index = options_str.indexOf(',', start_index);
        }
        String last_option = options_str.substring(start_index);
        last_option.trim();
        com_send_log(TERMINAL_OUTPUT, "  - %s", last_option.c_str());
    }
    else
    {
        // Set LPF bandwidth
        if (_settings_manager->setSettingValue(KEY_IMU_LPF_BANDWIDTH, args))
        {
            com_send_log(LOG_INFO, "IMU LPF Bandwidth set to %s. Reboot to apply changes.", _settings_manager->getSettingValueHumanReadable(KEY_IMU_LPF_BANDWIDTH).c_str());
        }
        else
        {
            com_send_log(LOG_ERROR, "Failed to set IMU LPF Bandwidth to %s. Invalid option.", args.c_str());
        }
    }
}

void Terminal::_handle_rx_data(String &args)
{
    if (!_rx_task)
    {
        com_send_log(LOG_ERROR, "RX task not available.");
        return;
    }

    com_send_log(TERMINAL_OUTPUT, "RX Channels:");

    int max_ch_len = 0;
    for (int i = 0; i < TERMINAL_RX_DATA_DISPLAY_CHANNELS; ++i)
    {
        String ch_str = "CH" + String(i + RC_CHANNEL_INDEX_OFFSET);
        if (ch_str.length() > max_ch_len)
        {
            max_ch_len = ch_str.length();
        }
    }
    max_ch_len += TERMINAL_COLUMN_BUFFER_WIDTH;

    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_ch_len, "Channel", "Value");
    String separator = "  ";
    for (int i = 0; i < max_ch_len; ++i)
    {
        separator += "-";
    }
    separator += "--------------------";
    com_send_log(TERMINAL_OUTPUT, separator.c_str());

    for (int i = 0; i < TERMINAL_RX_DATA_DISPLAY_CHANNELS; ++i)
    {
        String ch_str = "CH" + String(i + RC_CHANNEL_INDEX_OFFSET);
        com_send_log(TERMINAL_OUTPUT, "  %-*s %d", max_ch_len, ch_str.c_str(), _rx_task->getChannel(i));
    }
    com_send_log(TERMINAL_OUTPUT, separator.c_str());
}

void Terminal::_handle_rx_status(String &args)
{
    if (!_rx_task)
    {
        com_send_log(LOG_ERROR, "RX task not available.");
        return;
    }

    com_send_log(TERMINAL_OUTPUT, "RX Status: Task Available");
}

void Terminal::_handle_rx_protocol(String &args)
{
    if (args.length() == 0)
    {
        String current_protocol = _settings_manager->getSettingValueHumanReadable(KEY_RC_PROTOCOL_TYPE);
        com_send_log(TERMINAL_OUTPUT, "Current RC Protocol: %s", current_protocol.c_str());
        com_send_log(TERMINAL_OUTPUT, "Available protocols: IBUS, CRSF, SBUS");
    }
    else
    {
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

void Terminal::_handle_rx_value_single(String &args)
{
    if (!_rx_task)
    {
        com_send_log(LOG_ERROR, "RX task not available.");
        return;
    }

    int last_dot_index = args.lastIndexOf('.');
    String channel_name = args.substring(last_dot_index + RC_CHANNEL_INDEX_OFFSET);

    const char *key = nullptr;
    for (const auto &mapping : _channel_map)
    {
        if (channel_name.equalsIgnoreCase(mapping.name))
        {
            key = mapping.key;
            break;
        }
    }

    if (key == nullptr)
    {
        com_send_log(LOG_ERROR, "Unknown RX channel: %s", channel_name.c_str());
        return;
    }

    int channel_index = _settings_manager->getSettingValue(key).toInt();
    int16_t value = _rx_task->getChannel(channel_index);

    const int fixed_desc_width = TERMINAL_RX_SINGLE_DESC_WIDTH;

    String desc_str = "RX " + channel_name + " (CH" + String(channel_index + RC_CHANNEL_INDEX_OFFSET) + ")";
    com_send_log(TERMINAL_OUTPUT, "  %-*s %d", fixed_desc_width, desc_str.c_str(), value);
}

void Terminal::_handle_rx_value_all(String &args)
{
    if (!_rx_task)
    {
        com_send_log(LOG_ERROR, "RX task not available.");
        return;
    }

    com_send_log(TERMINAL_OUTPUT, "All Mapped RX Channels:");

    int max_desc_len = 0;
    for (const auto &mapping : _channel_map)
    {
        int channel_index_1_based = _settings_manager->getSettingValue(mapping.key).toInt() + RC_CHANNEL_INDEX_OFFSET;
        String desc_str = String(mapping.name) + " (CH" + String(channel_index_1_based) + ")";
        if (desc_str.length() > max_desc_len)
        {
            max_desc_len = desc_str.length();
        }
    }
    max_desc_len += TERMINAL_COLUMN_BUFFER_WIDTH;

    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_desc_len, "Channel", "Value");
    String separator = "  ";
    for (int i = 0; i < max_desc_len; ++i)
    {
        separator += "-";
    }
    separator += "--------------------";
    com_send_log(TERMINAL_OUTPUT, separator.c_str());

    for (const auto &mapping : _channel_map)
    {
        int channel_index_0_based = _settings_manager->getSettingValue(mapping.key).toInt();
        int channel_index_1_based = channel_index_0_based + RC_CHANNEL_INDEX_OFFSET;
        int16_t value = _rx_task->getChannel(channel_index_0_based);
        String desc_str = String(mapping.name) + " (CH" + String(channel_index_1_based) + ")";
        com_send_log(TERMINAL_OUTPUT, "  %-*s %d", max_desc_len, desc_str.c_str(), value);
    }
    com_send_log(TERMINAL_OUTPUT, separator.c_str());
}

void Terminal::_handle_rx_channel_mapping(String &args)
{
    int space_index = args.indexOf(' ');
    if (space_index == -1)
    {
        com_send_log(LOG_ERROR, "Usage: set rx.channel.<name> <1-based_index>");
        return;
    }

    String channel_name_arg = args.substring(0, space_index);
    String index_str = args.substring(space_index + 1);
    int channel_index_1_based = index_str.toInt();

    if (channel_index_1_based < TERMINAL_MIN_CHANNEL_INDEX || channel_index_1_based > TERMINAL_MAX_CHANNEL_INDEX)
    {
        com_send_log(LOG_ERROR, "Invalid channel index: %d. Must be between %d and %d.", channel_index_1_based, TERMINAL_MIN_CHANNEL_INDEX, TERMINAL_MAX_CHANNEL_INDEX);
        return;
    }

    int channel_index_0_based = channel_index_1_based - RC_CHANNEL_INDEX_OFFSET;

    const char *key = nullptr;
    for (const auto &mapping : _channel_map)
    {
        if (channel_name_arg.equalsIgnoreCase(mapping.name))
        {
            key = mapping.key;
            break;
        }
    }

    if (key == nullptr)
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

void Terminal::_handle_motor_throttle(String &args)
{
    if (!_motor_task)
    {
        com_send_log(LOG_ERROR, "Motor task not available.");
        return;
    }

    int space_index = args.indexOf(' ');
    if (space_index == -1)
    {
        com_send_log(LOG_ERROR, "Usage: motor.throttle <motor_name> <throttle_value>");
        return;
    }

    String motor_name_str = args.substring(0, space_index);
    String throttle_str = args.substring(space_index + 1);

    int8_t motor_id = _get_motor_id(motor_name_str);
    uint16_t throttle_value = throttle_str.toInt();

    if (motor_id == -1)
    {
        com_send_log(LOG_ERROR, "Invalid motor name: %s. Must be FL, FR, RL, or RR.", motor_name_str.c_str());
        return;
    }

    if (throttle_value > MAX_THROTTLE_VALUE)
    {
        com_send_log(LOG_ERROR, "Invalid throttle value: %d. Must be between 0 and %d.", throttle_value, MAX_THROTTLE_VALUE);
        return;
    }

    _motor_task->setThrottle(motor_id, throttle_value);
    com_send_log(TERMINAL_OUTPUT, "Motor %s throttle set to %d.", _get_motor_name(motor_id), throttle_value);
}

void Terminal::_handle_motor_test(String &args)
{
    if (!_motor_task)
    {
        com_send_log(LOG_ERROR, "Motor task not available.");
        return;
    }
    if (_pid_task && _pid_task->isArmed())
    {
        com_send_log(LOG_ERROR, "Cannot run motor test while armed. Disarm first.");
        return;
    }

    int8_t motor_id = -1;
    float throttle_percentage = -1.0f;
    uint32_t duration_ms = 0; // 0 for continuous

    // Parse arguments: <motor_name> <throttle_percentage> [duration_ms]
    int first_space = args.indexOf(' ');
    if (first_space == -1)
    {
        com_send_log(LOG_ERROR, "Usage: motor test <motor_name> <throttle_percentage> [duration_ms]");
        return;
    }

    String motor_name_str = args.substring(0, first_space);
    motor_id = _get_motor_id(motor_name_str);
    String remaining_args = args.substring(first_space + 1);

    int second_space = remaining_args.indexOf(' ');
    if (second_space == -1)
    { // No duration_ms provided, continuous spin
        throttle_percentage = remaining_args.toFloat();
    }
    else
    { // Duration_ms provided
        throttle_percentage = remaining_args.substring(0, second_space).toFloat();
        duration_ms = remaining_args.substring(second_space + 1).toInt();
    }

    if (motor_id == -1)
    {
        com_send_log(LOG_ERROR, "Invalid motor name: %s. Must be FL, FR, RL, or RR.", motor_name_str.c_str());
        return;
    }
    if (throttle_percentage < 0.0f || throttle_percentage > 100.0f)
    {
        com_send_log(LOG_ERROR, "Invalid throttle percentage. Must be between 0 and 100.");
        return;
    }

    // Convert percentage to normalized 0-1 float
    float normalized_throttle = throttle_percentage / 100.0f;

    if (duration_ms > 0)
    {
        _motor_task->startMotorTest(motor_id, normalized_throttle, duration_ms);
    }
    else
    {
        _motor_task->startContinuousMotorTest(motor_id, normalized_throttle);
    }
}

void Terminal::_handle_motor_stop(String &args)
{
    if (!_motor_task)
    {
        com_send_log(LOG_ERROR, "Motor task not available.");
        return;
    }
    _motor_task->stopMotorTest();
}

void Terminal::_handle_get_setting(String &args)
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
    const char *description = _settings_manager->getSettingDescription(internal_key);
    if (value.length() > 0)
    {
        com_send_log(TERMINAL_OUTPUT, "%s (%s): %s", args.c_str(), description, value.c_str());
    }
    else
    {
        com_send_log(LOG_ERROR, "Could not retrieve value for setting: %s", args.c_str());
    }
}

void Terminal::_handle_set_setting(String &args)
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
        com_send_log(LOG_ERROR, "Failed to set %s (%s) to %s. Invalid value or out of range.", display_key.c_str(), _settings_manager->getSettingDescription(internal_key), value_str.c_str());
    }
}

void Terminal::_handle_save_settings(String &args)
{
    _settings_manager->saveSettings();
    com_send_log(LOG_INFO, "Settings saved to NVS.");
}

void Terminal::_handle_factory_reset(String &args)
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

void Terminal::_handle_list_settings(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "\n--- Available Settings ---");

    for (int cat_idx = 0; cat_idx < _num_categories; ++cat_idx)
    {
        CommandCategory current_category = _category_info[cat_idx].category;
        const char *category_name = _get_category_string(current_category);

        if (current_category == CommandCategory::UNKNOWN)
            continue;

        // Check if there are any settings for this category before printing header
        bool category_has_settings = false;
        for (int i = 0; i < _settings_manager->_num_settings; ++i)
        {
            if (Terminal::_get_setting_category(_settings_manager->_settings_metadata[i].display_key) == current_category)
            {
                category_has_settings = true;
                break;
            }
        }

        if (category_has_settings)
        {
            com_send_log(TERMINAL_OUTPUT, "\n--- %s Settings ---", category_name);
            _settings_manager->listSettings(current_category);
        }
    }
    com_send_log(TERMINAL_OUTPUT, "--------------------------");
}

void Terminal::_handle_dump_settings(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "\n# Settings Dump");

    for (int cat_idx = 0; cat_idx < _num_categories; ++cat_idx)
    {
        CommandCategory current_category = _category_info[cat_idx].category;
        const char *category_name = _get_category_string(current_category);

        if (current_category == CommandCategory::UNKNOWN)
            continue;

        // Check if there are any settings for this category before printing header
        bool category_has_settings = false;
        for (int i = 0; i < _settings_manager->_num_settings; ++i)
        {
            if (Terminal::_get_setting_category(_settings_manager->_settings_metadata[i].display_key) == current_category)
            {
                category_has_settings = true;
                break;
            }
        }

        if (category_has_settings)
        {
            com_send_log(TERMINAL_OUTPUT, "\n--- %s Settings ---", category_name);
            _settings_manager->dumpSettings(current_category);
        }
    }
    com_send_log(TERMINAL_OUTPUT, "\n# End of Dump");
}

void Terminal::_handle_pid_get(String &args)
{
    if (!_pid_task)
    {
        com_send_log(LOG_ERROR, "PID task not available.");
        return;
    }

    com_send_log(TERMINAL_OUTPUT, "PID Gains (scaled by %d):", (int)PID_SCALE_FACTOR);

    int max_label_len = 0;
    const char *labels[] = {"Roll:", "Pitch:", "Yaw:"};
    for (int i = 0; i < sizeof(labels) / sizeof(labels[0]); ++i)
    {
        int len = strlen(labels[i]);
        if (len > max_label_len)
        {
            max_label_len = len;
        }
    }
    max_label_len += TERMINAL_COLUMN_BUFFER_WIDTH;

    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Axis", "Gains (P, I, D)");
    String separator = "  ";
    for (int i = 0; i < max_label_len; ++i)
    {
        separator += "-";
    }
    separator += "-------------------------";
    com_send_log(TERMINAL_OUTPUT, separator.c_str());

    com_send_log(TERMINAL_OUTPUT, "  %-*s P=%d, I=%d, D=%d", max_label_len, "Roll:",
                 (int)(_pid_task->getGains(PidAxis::ROLL).p * PID_SCALE_FACTOR),
                 (int)(_pid_task->getGains(PidAxis::ROLL).i * PID_SCALE_FACTOR),
                 (int)(_pid_task->getGains(PidAxis::ROLL).d * PID_SCALE_FACTOR));
    com_send_log(TERMINAL_OUTPUT, "  %-*s P=%d, I=%d, D=%d", max_label_len, "Pitch:",
                 (int)(_pid_task->getGains(PidAxis::PITCH).p * PID_SCALE_FACTOR),
                 (int)(_pid_task->getGains(PidAxis::PITCH).i * PID_SCALE_FACTOR),
                 (int)(_pid_task->getGains(PidAxis::PITCH).d * PID_SCALE_FACTOR));
    com_send_log(TERMINAL_OUTPUT, "  %-*s P=%d, I=%d, D=%d", max_label_len, "Yaw:",
                 (int)(_pid_task->getGains(PidAxis::YAW).p * PID_SCALE_FACTOR),
                 (int)(_pid_task->getGains(PidAxis::YAW).i * PID_SCALE_FACTOR),
                 (int)(_pid_task->getGains(PidAxis::YAW).d * PID_SCALE_FACTOR));
    com_send_log(TERMINAL_OUTPUT, separator.c_str());
}

void Terminal::_handle_pid_set(String &args)
{
    if (!_pid_task)
    {
        com_send_log(LOG_ERROR, "PID task not available.");
        return;
    }

    int first_space = args.indexOf(' ');
    if (first_space == -1)
    {
        com_send_log(LOG_ERROR, "Usage: set pid <axis> <p|i|d> <value> (value is scaled by %d)", (int)PID_SCALE_FACTOR);
        return;
    }
    int second_space = args.indexOf(' ', first_space + 1);
    if (second_space == -1)
    {
        com_send_log(LOG_ERROR, "Usage: set pid <axis> <p|i|d> <value> (value is scaled by %d)", (int)PID_SCALE_FACTOR);
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
    int int_value = value_str.toInt();
    float value = (float)int_value / PID_SCALE_FACTOR;

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

void Terminal::_handle_pid_reset_defaults(String &args)
{
    if (!_pid_task)
    {
        com_send_log(LOG_ERROR, "PID task not available.");
        return;
    }

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
