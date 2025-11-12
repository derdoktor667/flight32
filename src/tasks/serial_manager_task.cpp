/**
 * @file serial_manager_task.cpp
 * @brief Implements the SerialManagerTask class for managing serial communication.
 * @author Your Name
 * @date 2025-11-12
 * @license MIT
 */

#include "serial_manager_task.h"
#include "../firmware.h"

// Re-define static arrays from old TerminalTask
const ChannelMapping SerialManagerTask::_terminal_channel_map[] = {
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
const int SerialManagerTask::_num_terminal_channel_mappings = sizeof(SerialManagerTask::_terminal_channel_map) / sizeof(ChannelMapping);

const Command SerialManagerTask::_terminal_commands[] = {
    {"help", &SerialManagerTask::_handle_help, "Shows this help message.", CommandCategory::SYSTEM},
    {"status", &SerialManagerTask::_handle_status, "Shows firmware information.", CommandCategory::SYSTEM},
    {"tasks", &SerialManagerTask::_handle_tasks, "Shows information about running tasks.", CommandCategory::SYSTEM},
    {"mem", &SerialManagerTask::_handle_mem, "Shows current memory usage.", CommandCategory::SYSTEM},
    {"reboot", &SerialManagerTask::_handle_reboot, "Reboots the ESP32.", CommandCategory::SYSTEM},
    {"quit", &SerialManagerTask::_handle_quit, "Exits the terminal session.", CommandCategory::SYSTEM},

    {"get imu.data", &SerialManagerTask::_handle_imu_data, "Displays the latest IMU readings.", CommandCategory::IMU},
    {"set imu.calibrate", &SerialManagerTask::_handle_imu_calibrate, "Calibrates the IMU.", CommandCategory::IMU},
    {"get imu.lpf_bandwidth", &SerialManagerTask::_handle_imu_lpf_bandwidth, "Gets the current IMU LPF bandwidth.", CommandCategory::IMU},
    {"set imu.lpf_bandwidth", &SerialManagerTask::_handle_imu_lpf_bandwidth, "Sets the IMU LPF bandwidth (e.g., 'set imu.lpf_bw LPF_42HZ_N_5MS').", CommandCategory::IMU},

    {"get rx.data", &SerialManagerTask::_handle_rx_data, "Shows the latest RX channel data.", CommandCategory::RX},
    {"get rx.status", &SerialManagerTask::_handle_rx_status, "Shows the RX connection status.", CommandCategory::RX},
    {"get ppm.pin", &SerialManagerTask::_handle_get_setting, "Gets the PPM input pin.", CommandCategory::RX},
    {"set ppm.pin", &SerialManagerTask::_handle_set_setting, "Sets the PPM input pin (GPIO number).", CommandCategory::RX},
    {"set rx.protocol", &SerialManagerTask::_handle_rx_protocol, "Sets the RX protocol (e.g., 'set rx.protocol IBUS'). Available: IBUS, PPM.", CommandCategory::RX},
    {"get rx.value.all", &SerialManagerTask::_handle_rx_value_all, "Shows all mapped RX channel values.", CommandCategory::RX},
    {"get rx.value.roll", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Roll channel value.", CommandCategory::RX},
    {"get rx.value.pitch", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Pitch channel value.", CommandCategory::RX},
    {"get rx.value.throttle", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Throttle channel value.", CommandCategory::RX},
    {"get rx.value.yaw", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Yaw channel value.", CommandCategory::RX},
    {"get rx.value.arm", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Arm channel value.", CommandCategory::RX},
    {"get rx.value.fmode", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Flight Mode channel value.", CommandCategory::RX},
    {"get rx.value.aux1", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Auxiliary 1 channel value.", CommandCategory::RX},
    {"get rx.value.aux2", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Auxiliary 2 channel value.", CommandCategory::RX},
    {"get rx.value.aux3", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Auxiliary 3 channel value.", CommandCategory::RX},
    {"get rx.value.aux4", &SerialManagerTask::_handle_rx_value_single, "Shows the RX Auxiliary 4 channel value.", CommandCategory::RX},

    {"set rx.channel.roll", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Roll channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.pitch", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Pitch channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.throttle", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Throttle channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.yaw", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Yaw channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.arm", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Arm channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.fmode", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Flight Mode channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux1", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Auxiliary 1 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux2", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Auxiliary 2 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux3", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Auxiliary 3 channel index (1-based).", CommandCategory::RX},
    {"set rx.channel.aux4", &SerialManagerTask::_handle_rx_channel_mapping, "Sets the RX Auxiliary 4 channel index (1-based).", CommandCategory::RX},

    {"set motor.throttle", &SerialManagerTask::_handle_motor_throttle, "Sets the throttle for a specific motor (e.g., 'set motor.throttle FL 1000').", CommandCategory::MOTOR},
    {"motor test", &SerialManagerTask::_handle_motor_test, "Tests individual motors (e.g., 'motor test FL 20 5000' for motor FL at 20% for 5s).", CommandCategory::MOTOR},
    {"motor stop", &SerialManagerTask::_handle_motor_stop, "Stops all motors from testing.", CommandCategory::MOTOR},

    {"get pid", &SerialManagerTask::_handle_pid_get, "Gets the current PID gains.", CommandCategory::PID},
    {"set pid", &SerialManagerTask::_handle_pid_set, "Sets a PID gain (e.g., 'set pid roll p 0.1').", CommandCategory::PID},
    {"reset pid", &SerialManagerTask::_handle_pid_reset_defaults, "Resets PID gains to default values.", CommandCategory::PID},

    {"get", &SerialManagerTask::_handle_get_setting, "Gets a setting value (e.g., 'get gyro.resolution').", CommandCategory::SETTINGS},
    {"set", &SerialManagerTask::_handle_set_setting, "Sets a setting value (e.g., 'set gyro.resolution = 250_DPS').", CommandCategory::SETTINGS},
    {"save", &SerialManagerTask::_handle_save_settings, "Saves all settings to persistent storage.", CommandCategory::SETTINGS},
    {"factory_reset", &SerialManagerTask::_handle_factory_reset, "Resets all settings to their default values.", CommandCategory::SETTINGS},
    {"settings", &SerialManagerTask::_handle_list_settings, "Lists all available settings.", CommandCategory::SETTINGS},
    {"dump", &SerialManagerTask::_handle_dump_settings, "Dumps all settings for backup.", CommandCategory::SETTINGS}};

const int SerialManagerTask::_num_terminal_commands = sizeof(SerialManagerTask::_terminal_commands) / sizeof(Command);

const CategoryInfo SerialManagerTask::_terminal_category_info[] = {
    {CommandCategory::SYSTEM, "system", "System commands and settings"},
    {CommandCategory::IMU, "imu", "IMU sensor commands"},
    {CommandCategory::RX, "rx", "Receiver commands"},
    {CommandCategory::MOTOR, "motor", "Motor control commands"},
    {CommandCategory::PID, "pid", "PID controller commands"},
    {CommandCategory::SETTINGS, "settings", "Settings management commands"},
};
const int SerialManagerTask::_num_terminal_categories = sizeof(SerialManagerTask::_terminal_category_info) / sizeof(CategoryInfo);

SerialManagerTask::SerialManagerTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms),
      _scheduler(scheduler),
      _imu_task(imu_task),
      _rx_task(rx_task),
      _motor_task(motor_task),
      _pid_task(pid_task),
      _settings_manager(settings_manager)
{
}

void SerialManagerTask::setup()
{
    _terminal_input_buffer.reserve(TERMINAL_INPUT_BUFFER_SIZE);
}

void SerialManagerTask::run()
{
    while (Serial.available() > 0)
    {
        uint8_t c = Serial.read();

        // Handshake logic: Check for MSP preamble
        if (_current_mode == SerialMode::TERMINAL) {
            // Look for '$', 'R', 'M', 'S', 'P' sequence
            static uint8_t handshake_state = 0;
            const char msp_handshake[] = "$RMSP";

            if (c == msp_handshake[handshake_state]) {
                handshake_state++;
                if (handshake_state == strlen(msp_handshake)) {
                    _current_mode = SerialMode::MSP;
                    _last_msp_activity_ms = millis();
                    com_send_log(LOG_INFO, "Switched to MSP mode.");
                    handshake_state = 0; // Reset handshake state
                    continue; // Don't process this char as terminal input
                }
            } else {
                handshake_state = 0; // Reset if sequence is broken
            }
        }

        if (_current_mode == SerialMode::TERMINAL)
        {
            _handle_terminal_input(c);
        }
        else // MSP Mode
        {
            _parse_msp_char(c);
            _last_msp_activity_ms = millis(); // Update activity on any MSP char
        }
    }

    // MSP Timeout logic
    if (_current_mode == SerialMode::MSP && (millis() - _last_msp_activity_ms > MSP_TIMEOUT_MS)) {
        _current_mode = SerialMode::TERMINAL;
        com_send_log(LOG_INFO, "MSP timeout. Switched back to Terminal mode.");
        showPrompt();
    }

    // If in terminal mode and should quit, signal scheduler
    if (_current_mode == SerialMode::TERMINAL && _terminal_should_quit) {
        // _scheduler->stop(); // Scheduler does not have a stop method. Implement system halt if needed.
    }
}

void SerialManagerTask::showPrompt()
{
    com_flush_output();
    com_send_log(TERMINAL_OUTPUT, "");
    String system_name = _settings_manager->getSettingValue(SettingsManager::KEY_SYSTEM_NAME);
    String prompt = "[" + system_name + " ~]$";
    com_send_prompt(prompt.c_str());
}

const char *SerialManagerTask::_format_bytes(uint32_t bytes)
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

// Terminal Mode Functions
void SerialManagerTask::_handle_terminal_input(char incoming_char)
{
    if (incoming_char == '\n' || incoming_char == '\r')
    {
        if (_terminal_input_buffer.length() > 0)
        {
            SerialManagerTask::_parse_terminal_command(_terminal_input_buffer);
            _terminal_input_buffer = "";
        }
        if (!_terminal_should_quit) { // Only show prompt if not quitting
            showPrompt();
        }
    }
    else if (isPrintable(incoming_char))
    {
        _terminal_input_buffer += incoming_char;
    }
    else if (incoming_char == ASCII_BACKSPACE)
    {
        if (_terminal_input_buffer.length() > 0)
        {
            _terminal_input_buffer.remove(_terminal_input_buffer.length() - 1);
        }
    }
}

void SerialManagerTask::_parse_terminal_command(String &command_line)
{
    command_line.trim();
    if (command_line.length() == 0)
        return;

    int best_match_index = -1;
    int longest_match_len = 0;

    for (int i = 0; i < _num_terminal_commands; i++)
    {
        const char *cmd_name = _terminal_commands[i].name;
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
        const Command &matched_command = _terminal_commands[best_match_index];
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

const char *SerialManagerTask::_get_category_string(CommandCategory category)
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

CommandCategory SerialManagerTask::_get_category_from_string(String &category_str)
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

CommandCategory SerialManagerTask::_get_setting_category(const char *display_key)
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
    return CommandCategory::SETTINGS;
}

const char* SerialManagerTask::_get_motor_name(uint8_t motor_id) {
    switch ((MotorIndex)motor_id) {
        case MotorIndex::FL: return "FL";
        case MotorIndex::FR: return "FR";
        case MotorIndex::RL: return "RL";
        case MotorIndex::RR: return "RR";
        default: return "UNKNOWN";
    }
}

int8_t SerialManagerTask::_get_motor_id(String &motor_name) {
    if (motor_name.equalsIgnoreCase("FL")) return (int8_t)MotorIndex::FL;
    if (motor_name.equalsIgnoreCase("FR")) return (int8_t)MotorIndex::FR;
    if (motor_name.equalsIgnoreCase("RL")) return (int8_t)MotorIndex::RL;
    if (motor_name.equalsIgnoreCase("RR")) return (int8_t)MotorIndex::RR;
    return -1; // Invalid motor name
}

const char *SerialManagerTask::_get_task_state_string(eTaskState state)
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

// Terminal Command Handlers
void SerialManagerTask::_handle_help(String &args)
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
        max_core_cmd_len += TERMINAL_COLUMN_BUFFER_WIDTH;

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

        com_send_log(TERMINAL_OUTPUT, "Available Command Categories:");

        int max_category_prefix_len = 0;
        for (int i = 0; i < _num_terminal_categories; ++i)
        {
            int len = strlen(_terminal_category_info[i].prefix);
            if (len > max_category_prefix_len)
            {
                max_category_prefix_len = len;
            }
        }
        max_category_prefix_len += TERMINAL_COLUMN_BUFFER_WIDTH;

        com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_category_prefix_len, "Category", "Description");
        String category_separator = "  ";
        for (int i = 0; i < max_category_prefix_len; ++i) {
            category_separator += "-";
        }
        category_separator += "--------------------------------------------------";
        com_send_log(TERMINAL_OUTPUT, category_separator.c_str());

        for (int i = 0; i < _num_terminal_categories; ++i)
        {
            com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_category_prefix_len, _terminal_category_info[i].prefix, _terminal_category_info[i].description);
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
        for (int i = 0; i < _num_terminal_commands; ++i)
        {
            if (_terminal_commands[i].category == requested_category)
            {
                int len = strlen(_terminal_commands[i].name);
                if (len > max_command_name_len)
                {
                    max_command_name_len = len;
                }
            }
        }

        max_command_name_len += TERMINAL_COLUMN_BUFFER_WIDTH; 
        if (max_command_name_len < strlen("Command")) { 
            max_command_name_len = strlen("Command");
        }

        int max_description_len = strlen("Description"); // Start with header length
        for (int i = 0; i < _num_terminal_commands; ++i)
        {
            if (_terminal_commands[i].category == requested_category)
            {
                int len = strlen(_terminal_commands[i].help);
                if (len > max_description_len)
                {
                    max_description_len = len;
                }
            }
        }

        com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_command_name_len, "Command", "Description");
        String separator = "  ";
        for (int i = 0; i < max_command_name_len; ++i) {
            separator += "-";
        }
        separator += " "; // Space between command and description separator
        for (int i = 0; i < max_description_len; ++i) {
            separator += "-";
        }
        com_send_log(TERMINAL_OUTPUT, separator.c_str());

        for (int i = 0; i < _num_terminal_commands; ++i)
        {
            if (_terminal_commands[i].category == requested_category)
            {
                com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_command_name_len, _terminal_commands[i].name, _terminal_commands[i].help);
            }
        }
        com_send_log(TERMINAL_OUTPUT, separator.c_str());
        com_send_log(TERMINAL_OUTPUT, "\n--------------------------");
    }
}

void SerialManagerTask::_handle_status(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Flight32 Firmware v%s", get_firmware_version());
}

void SerialManagerTask::_handle_tasks(String &args)
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
    com_send_log(TERMINAL_OUTPUT, "-------------------------------------------------------------------------------------------------------------------");

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
                 TASK_STATE_COLUMN_WIDTH, SerialManagerTask::_get_task_state_string(freertos_status.eCurrentState),
                 TASK_PRIO_COLUMN_WIDTH, freertos_status.uxBasePriority,
                 TASK_CPU_COLUMN_WIDTH, cpu_str,
                 TASK_LOOP_COLUMN_WIDTH, current_task->getLoopTime(),
                 TASK_AVG_LOOP_COLUMN_WIDTH, current_task->getAvgLoopTime(),
                 TASK_MAX_LOOP_COLUMN_WIDTH, current_task->getMaxLoopTime(),
                 freertos_status.usStackHighWaterMark);
        com_send_log(TERMINAL_OUTPUT, output_buffer);
    }
    com_send_log(TERMINAL_OUTPUT, "-------------------------------------------------------------------------------------------------------------------");

    vPortFree(freertos_task_status_array);
}

void SerialManagerTask::_handle_mem(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Memory (Heap):");

    int max_label_len = 0;
    const char* labels[] = {"Total:", "Free:", "Min Free:"};
    for (int i = 0; i < sizeof(labels) / sizeof(labels[0]); ++i) {
        int len = strlen(labels[i]);
        if (len > max_label_len) {
            max_label_len = len;
        }
    }
    max_label_len += TERMINAL_COLUMN_BUFFER_WIDTH;

    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Total:", SerialManagerTask::_format_bytes(ESP.getHeapSize()));
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Free:", SerialManagerTask::_format_bytes(ESP.getFreeHeap()));
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Min Free:", SerialManagerTask::_format_bytes(ESP.getMinFreeHeap()));
}

void SerialManagerTask::_handle_reboot(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Rebooting...");

    delayMicroseconds(ONE_SECOND_MICROSECONDS);

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "");

    ESP.restart();
}

void SerialManagerTask::_handle_quit(String &args) {
    com_send_log(TERMINAL_OUTPUT, "Saving settings and exiting...");
    if (_motor_task && _motor_task->isInTestMode()) {
        _motor_task->stopMotorTest();
    }
    _settings_manager->saveSettings(); // Save settings before quitting
    com_send_log(TERMINAL_OUTPUT, "Goodbye!");
    _terminal_should_quit = true;
}

void SerialManagerTask::_handle_imu_data(String &args)
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

void SerialManagerTask::_handle_imu_calibrate(String &args)
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

void SerialManagerTask::_handle_imu_lpf_bandwidth(String &args) {
    if (!SerialManagerTask::_check_imu_task_available()) return;

    if (args.length() == 0) {
        // Get current LPF bandwidth
        String current_lpf_bandwidth_index_str = _settings_manager->getSettingValue(KEY_IMU_LPF_BANDWIDTH);
        String current_lpf_bandwidth_str = _settings_manager->getSettingValueHumanReadable(KEY_IMU_LPF_BANDWIDTH);
        com_send_log(TERMINAL_OUTPUT, "Current IMU LPF Bandwidth: %s (Index: %s)", current_lpf_bandwidth_str.c_str(), current_lpf_bandwidth_index_str.c_str());
        com_send_log(TERMINAL_OUTPUT, "Available options:");
        String options_str = _settings_manager->getSettingOptionsHumanReadable(KEY_IMU_LPF_BANDWIDTH);
        int start_index = 0;
        int end_index = options_str.indexOf(',');
        while (end_index != -1) {
            String option = options_str.substring(start_index, end_index);
            option.trim();
            com_send_log(TERMINAL_OUTPUT, "  - %s", option.c_str());
            start_index = end_index + 1;
            end_index = options_str.indexOf(',', start_index);
        }
        String last_option = options_str.substring(start_index);
        last_option.trim();
        com_send_log(TERMINAL_OUTPUT, "  - %s", last_option.c_str());
    } else {
        // Set LPF bandwidth
        if (_settings_manager->setSettingValue(KEY_IMU_LPF_BANDWIDTH, args)) {
            com_send_log(LOG_INFO, "IMU LPF Bandwidth set to %s. Reboot to apply changes.", _settings_manager->getSettingValueHumanReadable(KEY_IMU_LPF_BANDWIDTH).c_str());
        } else {
            com_send_log(LOG_ERROR, "Failed to set IMU LPF Bandwidth to %s. Invalid option.", args.c_str());
        }
    }
}

void SerialManagerTask::_handle_rx_data(String &args)
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
        if (ch_str.length() > max_ch_len) {
            max_ch_len = ch_str.length();
        }
    }
    max_ch_len += TERMINAL_COLUMN_BUFFER_WIDTH;

    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_ch_len, "Channel", "Value");
    String separator = "  ";
    for (int i = 0; i < max_ch_len; ++i) {
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

void SerialManagerTask::_handle_rx_status(String &args)
{
    if (!_rx_task)
    {
        com_send_log(LOG_ERROR, "RX task not available.");
        return;
    }

    com_send_log(TERMINAL_OUTPUT, "RX Status: Task Available");
}

void SerialManagerTask::_handle_rx_protocol(String &args)
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

void SerialManagerTask::_handle_rx_value_single(String &args)
{
    if (!_rx_task)
    {
        com_send_log(LOG_ERROR, "RX task not available.");
        return;
    }

    int last_dot_index = args.lastIndexOf('.');
    String channel_name = args.substring(last_dot_index + RC_CHANNEL_INDEX_OFFSET);

    const char *key = nullptr;
    for (const auto &mapping : _terminal_channel_map)
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

void SerialManagerTask::_handle_rx_value_all(String &args)
{
    if (!_rx_task)
    {
        com_send_log(LOG_ERROR, "RX task not available.");
        return;
    }

    com_send_log(TERMINAL_OUTPUT, "All Mapped RX Channels:");

    int max_desc_len = 0;
    for (const auto &mapping : _terminal_channel_map)
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

    for (const auto &mapping : _terminal_channel_map)
    {
        int channel_index_0_based = _settings_manager->getSettingValue(mapping.key).toInt();
        int channel_index_1_based = channel_index_0_based + RC_CHANNEL_INDEX_OFFSET;
        int16_t value = _rx_task->getChannel(channel_index_0_based);
        String desc_str = String(mapping.name) + " (CH" + String(channel_index_1_based) + ")";
        com_send_log(TERMINAL_OUTPUT, "  %-*s %d", max_desc_len, desc_str.c_str(), value);
    }
    com_send_log(TERMINAL_OUTPUT, separator.c_str());
}

void SerialManagerTask::_handle_rx_channel_mapping(String &args)
{
    int space_index = args.indexOf(' ');
    if (space_index == -1)
    {
        com_send_log(LOG_ERROR, "Usage: set rx.channel.<name> <1-based_index>");
        return;
    }

    String channel_name_arg = args.substring(0, space_index);
    String index_str = args.substring(space_index + RC_CHANNEL_INDEX_OFFSET);
    int channel_index_1_based = index_str.toInt();

    if (channel_index_1_based < TERMINAL_MIN_CHANNEL_INDEX || channel_index_1_based > TERMINAL_MAX_CHANNEL_INDEX)
    {
        com_send_log(LOG_ERROR, "Invalid channel index: %d. Must be between %d and %d.", channel_index_1_based, TERMINAL_MIN_CHANNEL_INDEX, TERMINAL_MAX_CHANNEL_INDEX);
        return;
    }

    int channel_index_0_based = channel_index_1_based - RC_CHANNEL_INDEX_OFFSET;

    const char *key = nullptr;
    for (const auto &mapping : _terminal_channel_map)
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

void SerialManagerTask::_handle_motor_throttle(String &args)
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

void SerialManagerTask::_handle_motor_test(String &args) {
    if (!_motor_task) {
        com_send_log(LOG_ERROR, "Motor task not available.");
        return;
    }
    if (_pid_task && _pid_task->isArmed()) {
        com_send_log(LOG_ERROR, "Cannot run motor test while armed. Disarm first.");
        return;
    }

    int8_t motor_id = -1;
    float throttle_percentage = -1.0f;
    uint32_t duration_ms = 0; // 0 for continuous

    // Parse arguments: <motor_name> <throttle_percentage> [duration_ms]
    int first_space = args.indexOf(' ');
    if (first_space == -1) {
        com_send_log(LOG_ERROR, "Usage: motor test <motor_name> <throttle_percentage> [duration_ms]");
        return;
    }

    String motor_name_str = args.substring(0, first_space);
    motor_id = _get_motor_id(motor_name_str);
    String remaining_args = args.substring(first_space + 1);

    int second_space = remaining_args.indexOf(' ');
    if (second_space == -1) { // No duration_ms provided, continuous spin
        throttle_percentage = remaining_args.toFloat();
    } else { // Duration_ms provided
        throttle_percentage = remaining_args.substring(0, second_space).toFloat();
        duration_ms = remaining_args.substring(second_space + 1).toInt();
    }

    if (motor_id == -1) {
        com_send_log(LOG_ERROR, "Invalid motor name: %s. Must be FL, FR, RL, or RR.", motor_name_str.c_str());
        return;
    }
    if (throttle_percentage < 0.0f || throttle_percentage > 100.0f) {
        com_send_log(LOG_ERROR, "Invalid throttle percentage. Must be between 0 and 100.");
        return;
    }

    // Convert percentage to normalized 0-1 float
    float normalized_throttle = throttle_percentage / 100.0f;

    if (duration_ms > 0) {
        _motor_task->startMotorTest(motor_id, normalized_throttle, duration_ms);
    } else {
        _motor_task->startContinuousMotorTest(motor_id, normalized_throttle);
    }
}

void SerialManagerTask::_handle_motor_stop(String &args) {
    if (!_motor_task) {
        com_send_log(LOG_ERROR, "Motor task not available.");
        return;
    }
    _motor_task->stopMotorTest();
}

void SerialManagerTask::_handle_get_setting(String &args)
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

void SerialManagerTask::_handle_set_setting(String &args)
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

void SerialManagerTask::_handle_save_settings(String &args)
{
    _settings_manager->saveSettings();
    com_send_log(LOG_INFO, "Settings saved to NVS.");
}

void SerialManagerTask::_handle_factory_reset(String &args)
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

void SerialManagerTask::_handle_list_settings(String &args)
{
    _settings_manager->listSettings();
}

void SerialManagerTask::_handle_dump_settings(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "\n# Settings Dump");

    for (int cat_idx = 0; cat_idx < _num_terminal_categories; ++cat_idx)
    {
        CommandCategory current_category = _terminal_category_info[cat_idx].category;
        const char *category_name = _get_category_string(current_category);

        if (current_category == CommandCategory::UNKNOWN)
            continue;

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
            max_display_key_len += TERMINAL_COLUMN_BUFFER_WIDTH;
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

void SerialManagerTask::_handle_pid_get(String &args)
{
    if (!_pid_task)
    {
        com_send_log(LOG_ERROR, "PID task not available.");
        return;
    }

    com_send_log(TERMINAL_OUTPUT, "PID Gains (scaled by %d):", (int)PID_SCALE_FACTOR);

    int max_label_len = 0;
    const char* labels[] = {"Roll:", "Pitch:", "Yaw:"};
    for (int i = 0; i < sizeof(labels) / sizeof(labels[0]); ++i) {
        int len = strlen(labels[i]);
        if (len > max_label_len) {
            max_label_len = len;
        }
    }
    max_label_len += TERMINAL_COLUMN_BUFFER_WIDTH;

    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_label_len, "Axis", "Gains (P, I, D)");
    String separator = "  ";
    for (int i = 0; i < max_label_len; ++i) {
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

void SerialManagerTask::_handle_pid_set(String &args)
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

void SerialManagerTask::_handle_pid_reset_defaults(String &args)
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

// MSP Mode Functions
void SerialManagerTask::_parse_msp_char(uint8_t c)
{
    switch (_msp_state)
    {
    case MspState::IDLE:
        if (c == '$') {
            _msp_state = MspState::HEADER_START;
        }
        break;
    case MspState::HEADER_START:
        _msp_state = (c == 'M') ? MspState::HEADER_SIZE : MspState::IDLE;
        break;
    case MspState::HEADER_SIZE:
        _msp_payload_size = c;
        _msp_crc = c;
        _msp_payload_index = 0;
        _msp_state = MspState::HEADER_CMD;
        break;
    case MspState::HEADER_CMD:
        _msp_command_id = c;
        _msp_crc ^= c;
        _msp_state = (_msp_payload_size > 0) ? MspState::PAYLOAD : MspState::CRC;
        break;
    case MspState::PAYLOAD:
        _msp_payload_buffer[_msp_payload_index++] = c;
        _msp_crc ^= c;
        if (_msp_payload_index >= _msp_payload_size) {
            _msp_state = MspState::CRC;
        }
        break;
    case MspState::CRC:
        if (_msp_crc == c) {
            _process_msp_message();
        }
        _msp_state = MspState::IDLE;
        break;
    default:
        _msp_state = MspState::IDLE;
        break;
    }
}

void SerialManagerTask::_process_msp_message()
{
    switch (_msp_command_id)
    {
    case MSP_API_VERSION:
        _handle_msp_api_version();
        break;
    case MSP_FC_VARIANT:
        _handle_msp_fc_variant();
        break;
    case MSP_FC_VERSION:
        _handle_msp_fc_version();
        break;
    case MSP_BOARD_INFO:
        _handle_msp_board_info();
        break;
    case MSP_BUILD_INFO:
        _handle_msp_build_info();
        break;
    case MSP_REBOOT:
        _handle_msp_reboot();
        break;
    case MSP_EEPROM_WRITE:
        _handle_msp_eeprom_write();
        break;
    case MSP_RESET_SETTINGS:
        _handle_msp_reset_settings();
        break;
    case MSP_STATUS:
        _handle_msp_status();
        break;
    case MSP_MEM_STATS:
        _handle_msp_mem_stats();
        break;
    case MSP_GET_SETTING:
        _handle_msp_get_setting();
        break;
    case MSP_SET_SETTING:
        _handle_msp_set_setting();
        break;
    case MSP_PID:
        _handle_msp_pid_get();
        break;
    case MSP_SET_PID:
        _handle_msp_pid_set();
        break;
    // Add more command handlers here
    default:
        // Send empty response for unsupported command
        _send_msp_response(_msp_command_id, nullptr, 0);
        break;
    }
}

void SerialManagerTask::_send_msp_response(uint8_t cmd, uint8_t *payload, uint8_t size)
{
    Serial.write('$');
    Serial.write('M');
    Serial.write('>');
    uint8_t crc = 0;
    Serial.write(size);
    crc ^= size;
    Serial.write(cmd);
    crc ^= cmd;
    for (uint8_t i = 0; i < size; i++)
    {
        Serial.write(payload[i]);
        crc ^= payload[i];
    }
    Serial.write(crc);
}

// MSP Command Handlers
void SerialManagerTask::_handle_msp_api_version()
{
    uint8_t payload[3];
    payload[0] = MSP_PROTOCOL_VERSION;
    payload[1] = MSP_API_VERSION_MAJOR;
    payload[2] = MSP_API_VERSION_MINOR;
    _send_msp_response(MSP_API_VERSION, payload, 3);
}

void SerialManagerTask::_handle_msp_fc_variant()
{
    uint8_t payload[4] = {'F', 'L', '3', '2'};
    _send_msp_response(MSP_FC_VARIANT, payload, 4);
}

void SerialManagerTask::_handle_msp_fc_version()
{
    uint8_t payload[3] = {FC_VERSION_MAJOR, FC_VERSION_MINOR, FC_VERSION_PATCH};
    _send_msp_response(MSP_FC_VERSION, payload, 3);
}

void SerialManagerTask::_handle_msp_board_info()
{
    uint8_t payload[8] = {'F', 'L', '3', '2', 0, 0, 0, 0};
    _send_msp_response(MSP_BOARD_INFO, payload, 8);
}

void SerialManagerTask::_handle_msp_build_info()
{
    uint8_t payload[20] = "Nov 12 202512:00:00";
    _send_msp_response(MSP_BUILD_INFO, payload, 19);
}

void SerialManagerTask::_handle_msp_reboot() {
    _send_msp_response(MSP_REBOOT, nullptr, 0); // Acknowledge
    delayMicroseconds(ONE_SECOND_MICROSECONDS);
    ESP.restart();
}

void SerialManagerTask::_handle_msp_eeprom_write() {
    _settings_manager->saveSettings();
    _send_msp_response(MSP_EEPROM_WRITE, nullptr, 0); // Acknowledge
}

void SerialManagerTask::_handle_msp_reset_settings() {
    _settings_manager->factoryReset();
    _send_msp_response(MSP_RESET_SETTINGS, nullptr, 0); // Acknowledge
    delayMicroseconds(ONE_SECOND_MICROSECONDS);
    ESP.restart();
}

void SerialManagerTask::_handle_msp_status() {
    // For simplicity, just send firmware version for now
    // More detailed status would require a custom payload structure
    String version_str = get_firmware_version();
    uint8_t payload[version_str.length() + 1];
    memcpy(payload, version_str.c_str(), version_str.length());
    payload[version_str.length()] = 0; // Null terminator
    _send_msp_response(MSP_STATUS, payload, version_str.length() + 1);
}

void SerialManagerTask::_handle_msp_mem_stats() {
    // Send free heap size (4 bytes)
    uint32_t free_heap = ESP.getFreeHeap();
    uint8_t payload[4];
    payload[0] = (free_heap >> 0) & 0xFF;
    payload[1] = (free_heap >> 8) & 0xFF;
    payload[2] = (free_heap >> 16) & 0xFF;
    payload[3] = (free_heap >> 24) & 0xFF;
    _send_msp_response(MSP_MEM_STATS, payload, 4);
}

void SerialManagerTask::_handle_msp_get_setting() {
    if (_msp_payload_size == 0) {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: no key provided
        return;
    }

    // MSP payload for get setting: [key_length (1 byte)] [key_string (variable length)]
    uint8_t key_len = _msp_payload_buffer[0];
    if (key_len == 0 || key_len > (_msp_payload_size - 1)) {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: invalid key length
        return;
    }

    char key_cstr[key_len + 1];
    memcpy(key_cstr, &_msp_payload_buffer[1], key_len);
    key_cstr[key_len] = '\0';
    String display_key_str = String(key_cstr);

    const char *internal_key = _settings_manager->getInternalKeyFromDisplayKey(display_key_str.c_str());

    if (internal_key == nullptr) {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: unknown setting
        return;
    }

    String value_str = _settings_manager->getSettingValueHumanReadable(internal_key);
    
    // Response payload: [key_length (1 byte)] [key_string (variable)] [value_length (1 byte)] [value_string (variable)]
    // Max payload size is 128, so need to be careful with string lengths
    uint8_t response_key_len = display_key_str.length();
    uint8_t response_value_len = value_str.length();

    if (response_key_len + response_value_len + 2 > 128) { // 2 for key_len and value_len bytes
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Response too long
        return;
    }

    uint8_t response_payload[response_key_len + response_value_len + 2];
    int idx = 0;
    response_payload[idx++] = response_key_len;
    memcpy(&response_payload[idx], display_key_str.c_str(), response_key_len);
    idx += response_key_len;
    response_payload[idx++] = response_value_len;
    memcpy(&response_payload[idx], value_str.c_str(), response_value_len);
    idx += response_value_len;

    _send_msp_response(MSP_GET_SETTING, response_payload, idx);
}

void SerialManagerTask::_handle_msp_set_setting() {
    if (_msp_payload_size < 3) { // Need at least key_len, key, value_len, value
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid payload
        return;
    }

    // MSP payload for set setting: [key_length (1 byte)] [key_string (variable)] [value_length (1 byte)] [value_string (variable)]
    uint8_t key_len = _msp_payload_buffer[0];
    if (key_len == 0 || key_len > (_msp_payload_size - 2)) { // -2 for key_len and value_len bytes
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid key length
        return;
    }

    char key_cstr[key_len + 1];
    memcpy(key_cstr, &_msp_payload_buffer[1], key_len);
    key_cstr[key_len] = '\0';
    String display_key_str = String(key_cstr);

    uint8_t value_len_idx = 1 + key_len;
    uint8_t value_len = _msp_payload_buffer[value_len_idx];
    if (value_len == 0 || (value_len_idx + 1 + value_len) > _msp_payload_size) {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid value length
        return;
    }

    char value_cstr[value_len + 1];
    memcpy(value_cstr, &_msp_payload_buffer[value_len_idx + 1], value_len);
    value_cstr[value_len] = '\0';
    String value_str = String(value_cstr);

    const char *internal_key = _settings_manager->getInternalKeyFromDisplayKey(display_key_str.c_str());

    if (internal_key == nullptr) {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: unknown setting
        return;
    }

    if (_settings_manager->setSettingValue(internal_key, value_str)) {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Acknowledge success
    } else {
        // Could send an error code in payload if MSP supports it, for now just empty response
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Acknowledge failure
    }
}

void SerialManagerTask::_handle_msp_pid_get()
{
    if (!_pid_task) {
        // No log here, MSP clients don't read logs. Just send empty response.
        _send_msp_response(MSP_PID, nullptr, 0); 
        return;
    }

    uint8_t payload[MSP_PID_PAYLOAD_SIZE]; // 3 axes * 3 gains (P,I,D) = 9 bytes
    int i = 0;

    PidGains roll_gains = _pid_task->getGains(PidAxis::ROLL);
    payload[i++] = (uint8_t)(roll_gains.p * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(roll_gains.i * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(roll_gains.d * PID_SCALE_FACTOR);

    PidGains pitch_gains = _pid_task->getGains(PidAxis::PITCH);
    payload[i++] = (uint8_t)(pitch_gains.p * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(pitch_gains.i * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(pitch_gains.d * PID_SCALE_FACTOR);

    PidGains yaw_gains = _pid_task->getGains(PidAxis::YAW);
    payload[i++] = (uint8_t)(yaw_gains.p * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(yaw_gains.i * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(yaw_gains.d * PID_SCALE_FACTOR);

    _send_msp_response(MSP_PID, payload, 9);
}

void SerialManagerTask::_handle_msp_pid_set()
{
    if (!_pid_task || _msp_payload_size != MSP_PID_PAYLOAD_SIZE) {
        _send_msp_response(MSP_SET_PID, nullptr, 0); // Error or invalid payload size
        return;
    }

    int i = 0;
    PidGains roll_gains = _pid_task->getGains(PidAxis::ROLL);
    roll_gains.p = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    roll_gains.i = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    roll_gains.d = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::ROLL, roll_gains);

    PidGains pitch_gains = _pid_task->getGains(PidAxis::PITCH);
    pitch_gains.p = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    pitch_gains.i = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    pitch_gains.d = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::PITCH, pitch_gains);

    PidGains yaw_gains = _pid_task->getGains(PidAxis::YAW);
    yaw_gains.p = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    yaw_gains.i = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    yaw_gains.d = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::YAW, yaw_gains);

    _send_msp_response(MSP_SET_PID, nullptr, 0); // Acknowledge receipt
}
