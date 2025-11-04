#include "terminal.h"
#include "../firmware.h"
#include "../config.h"
#include "../com_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define the command table
const Command TerminalTask::_commands[] = {
    {"help", &TerminalTask::_handle_help, "Shows this help message."},
    {"status", &TerminalTask::_handle_status, "Shows firmware information."},
    {"tasks", &TerminalTask::_handle_tasks, "Shows information about running tasks."},
    {"mem", &TerminalTask::_handle_mem, "Shows current memory usage."},
    {"reboot", &TerminalTask::_handle_reboot, "Reboots the ESP32."}
};

// Calculate the number of commands
const int TerminalTask::_num_commands = sizeof(TerminalTask::_commands) / sizeof(Command);

TerminalTask::TerminalTask(const char* name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID)
    : TaskBase(name, stackSize, priority, coreID), _input_buffer("") {}

void TerminalTask::setup() {
    _input_buffer.reserve(TERMINAL_INPUT_BUFFER_SIZE); 
    com_send_log(LOG_INFO, "Terminal task initialized. Type 'help' for a list of commands.");
}

void TerminalTask::run() {
    while (Serial.available() > 0) {
        char incoming_char = Serial.read();
        if (incoming_char == '\n' || incoming_char == '\r') {
            if (_input_buffer.length() > 0) {
                _parse_command(_input_buffer);
                _input_buffer = "";
            }
        } else {
            _input_buffer += incoming_char;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(TERMINAL_TASK_DELAY_MS));
}

void TerminalTask::_parse_command(String &command) {
    command.trim();

    // Separate command and arguments
    String command_name = command;
    String args = "";
    int space_index = command.indexOf(' ');
    if (space_index != -1) {
        command_name = command.substring(0, space_index);
        args = command.substring(space_index + 1);
        args.trim();
    }

    // Find and execute the command
    for (int i = 0; i < _num_commands; i++) {
        if (command_name.equalsIgnoreCase(_commands[i].name)) {
            (this->*_commands[i].handler)(args);
            return;
        }
    }

    com_send_log(LOG_WARN, "Unknown command: '%s'", command_name.c_str());
}

// --- Command Handlers ---

void TerminalTask::_handle_help(String &args) {
    com_send_log(TERMINAL_OUTPUT, ""); // Empty line for visual separation
    com_send_log(TERMINAL_OUTPUT, "--- Available Commands ---");
    for (int i = 0; i < _num_commands; i++) {
        com_send_log(TERMINAL_OUTPUT, "  %s - %s", _commands[i].name, _commands[i].help);
    }
    com_send_log(TERMINAL_OUTPUT, "--------------------------");
}

void TerminalTask::_handle_status(String &args) {
    com_send_log(TERMINAL_OUTPUT, "Flight32 Firmware v%s", get_firmware_version());
}

// Helper to convert task state enum to a readable string
const char* _get_task_state_string(eTaskState state) {
    switch (state) {
        case eRunning:   return "Running";
        case eReady:     return "Ready";
        case eBlocked:   return "Blocked";
        case eSuspended: return "Suspended";
        case eDeleted:   return "Deleted";
        default:         return "Invalid";
    }
}

void TerminalTask::_handle_tasks(String &args) {
    // Get the number of tasks
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_status_array = (TaskStatus_t *)pvPortMalloc(num_tasks * sizeof(TaskStatus_t));
    uint32_t total_run_time;

    if (task_status_array == NULL) {
        com_send_log(LOG_ERROR, "Failed to allocate memory for task list.");
        return;
    }

    // Get task states and total run time
    num_tasks = uxTaskGetSystemState(task_status_array, num_tasks, &total_run_time);

    // Print header
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "%-16s %-10s %-6s %-8s %s", "Task Name", "State", "Prio", "CPU %%", "Stack HWM (bytes)");
    com_send_log(TERMINAL_OUTPUT, "-------------------------------------------------------------------");

    // Print each relevant task's status
    for (UBaseType_t i = 0; i < num_tasks; i++) {
        const char* name = task_status_array[i].pcTaskName;
        // Filter out irrelevant tasks
        if (strcmp(name, "IDLE0") == 0 || strcmp(name, "IDLE1") == 0 || strcmp(name, "Tmr Svc") == 0) {
            continue;
        }

        float cpu_percentage = 0.0f;
        if (total_run_time > 0) {
            cpu_percentage = (float)task_status_array[i].ulRunTimeCounter / total_run_time * 100.0f;
        }

        char cpu_str[8];
        snprintf(cpu_str, sizeof(cpu_str), "%.2f", cpu_percentage);

        com_send_log(TERMINAL_OUTPUT, "%-16s %-10s %-6u %-8s %u",
                     name,
                     _get_task_state_string(task_status_array[i].eCurrentState),
                     task_status_array[i].uxBasePriority,
                     cpu_str,
                     task_status_array[i].usStackHighWaterMark);
    }
    com_send_log(TERMINAL_OUTPUT, "-------------------------------------------------------------------");

    // Free the allocated memory
    vPortFree(task_status_array);
}

void TerminalTask::_handle_mem(String &args) {
    com_send_log(TERMINAL_OUTPUT, "Memory (Heap):");
    com_send_log(TERMINAL_OUTPUT, "  Total: %u bytes", ESP.getHeapSize());
    com_send_log(TERMINAL_OUTPUT, "  Free:  %u bytes", ESP.getFreeHeap());
    com_send_log(TERMINAL_OUTPUT, "  Min Free: %u bytes", ESP.getMinFreeHeap());
}

void TerminalTask::_handle_reboot(String &args) {
    com_send_log(TERMINAL_OUTPUT, "Rebooting...");
    ESP.restart();
}