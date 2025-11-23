/**
 * @file com_manager.h
 * @brief Defines the communication manager interface for serial logging and terminal output.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdarg>
#include "config/com_manager_config.h"

enum class ComMessageType
{
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    TERMINAL_OUTPUT,
    TERMINAL_PROMPT,
    TERMINAL_FLUSH
};

typedef struct
{
    ComMessageType type;
    union
    {
        char content[COM_MESSAGE_MAX_LENGTH];
    };
} com_message_t;

// Define serial communication modes
enum class ComSerialMode
{
    TERMINAL, // Default mode: handles CLI commands.
    MSP,      // MultiWii Serial Protocol mode: handles configurator communication.
    PASSTHROUGH // ESC Passthrough mode: raw serial data to/from ESCs.
};

extern QueueHandle_t com_queue;
extern QueueHandle_t com_flush_signal_queue;

void com_task(void *pvParameters);

// Initializes the communication manager's FreeRTOS queues.
void com_manager_init();

void com_send_log(ComMessageType type, const char *format, ...);
void com_send_prompt(const char *prompt);
void com_flush_output();
const char *com_format_bytes(uint32_t bytes);

// Function to set the current serial mode
void com_set_serial_mode(ComSerialMode mode);

extern bool _is_passthrough_active; // Global flag to indicate passthrough mode
