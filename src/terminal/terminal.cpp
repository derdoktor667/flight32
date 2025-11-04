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
    {"reboot", &TerminalTask::_handle_reboot, "Reboots the ESP32."}};

// Calculate the number of commands
const int TerminalTask::_num_commands = sizeof(TerminalTask::_commands) / sizeof(Command);

TerminalTask::TerminalTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms), _input_buffer(""), _scheduler(scheduler) {}

//
void TerminalTask::setup()
{
    _input_buffer.reserve(TERMINAL_INPUT_BUFFER_SIZE);

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(LOG_INFO, "Welcome, type 'help' for a list of commands.");
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
                _parse_command(_input_buffer);
                _input_buffer = "";
            }
        }
        else
        {
            _input_buffer += incoming_char;
        }
    }
}

//
void TerminalTask::_parse_command(String &command)
{
    command.trim();

    // Separate command and arguments
    String command_name = command;
    String args = "";
    int space_index = command.indexOf(' ');
    if (space_index != -1)
    {
        command_name = command.substring(0, space_index);
        args = command.substring(space_index + 1);
        args.trim();
    }

    // Find and execute the command
    for (int i = 0; i < _num_commands; i++)
    {
        if (command_name.equalsIgnoreCase(_commands[i].name))
        {
            (this->*_commands[i].handler)(args);
            return;
        }
    }

    com_send_log(LOG_WARN, "Unknown command: '%s'", command_name.c_str());
}

// --- Command Handlers ---

//
void TerminalTask::_handle_help(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "--- Available Commands ---");
    for (int i = 0; i < _num_commands; i++)
    {
        com_send_log(TERMINAL_OUTPUT, "  %s - %s", _commands[i].name, _commands[i].help);
    }
    com_send_log(TERMINAL_OUTPUT, "--------------------------");
}

void TerminalTask::_handle_status(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Flight32 Firmware v%s", get_firmware_version());
}

// Helper to convert task state enum to a readable string
const char *_get_task_state_string(eTaskState state)
{
    switch (state)
    {
    case eRunning:
        return "Running";
    case eReady:
        return "Ready";
    case eBlocked:
        return "Blocked";
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
    com_send_log(TERMINAL_OUTPUT, "%-16s %-10s %-6s %-8s %-10s %-10s %-10s %s",
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
        if (strcmp(name, "IDLE0") == 0 || strcmp(name, "IDLE1") == 0 || strcmp(name, "Tmr Svc") == 0)
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

        com_send_log(TERMINAL_OUTPUT, "%-16s %-10s %-6u %-8s %-10u %-10u %-10u %u",
                     name,
                     _get_task_state_string(freertos_status.eCurrentState),
                     freertos_status.uxBasePriority,
                     cpu_str,
                     current_task->getLoopTime(),
                     current_task->getAvgLoopTime(),
                     current_task->getMaxLoopTime(),
                     freertos_status.usStackHighWaterMark);
    }
    com_send_log(TERMINAL_OUTPUT, "-------------------------------------------------------------------------------------------------------------------");

    // Free the allocated memory
    vPortFree(freertos_task_status_array);
}

void TerminalTask::_handle_mem(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Memory (Heap):");
    com_send_log(TERMINAL_OUTPUT, "  Total: %u bytes", ESP.getHeapSize());
    com_send_log(TERMINAL_OUTPUT, "  Free:  %u bytes", ESP.getFreeHeap());
    com_send_log(TERMINAL_OUTPUT, "  Min Free: %u bytes", ESP.getMinFreeHeap());
}

void TerminalTask::_handle_reboot(String &args)
{
    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "Rebooting...");

    // ...short break before restarting
    delayMicroseconds(1000000);

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "");

    ESP.restart();
}
