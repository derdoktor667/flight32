#pragma once

#include "../scheduler/task.h"
#include <Arduino.h>

// Forward declaration of TerminalTask is needed for the function pointer type
class TerminalTask;

// Define a function pointer type for command handlers.
// It points to a member function of TerminalTask that takes a String reference and returns void.
using CommandHandler = void (TerminalTask::*)(String &args);

// Struct to define the properties of a terminal command
struct Command {
    const char* name;       // The command string (e.g., "help")
    CommandHandler handler; // Pointer to the handler function
    const char* help;       // The help text for the command
};

class TerminalTask : public TaskBase {
public:
    TerminalTask(const char* name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID);
    void setup() override;
    void run() override;

private:
    String _input_buffer;
    void _parse_command(String &command);

    // Command handler functions
    void _handle_help(String &args);
    void _handle_status(String &args);
    void _handle_tasks(String &args);
    void _handle_mem(String &args);
    void _handle_reboot(String &args);

    // Command table definition
    static const Command _commands[];
    static const int _num_commands;
};