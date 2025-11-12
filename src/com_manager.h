/**
 * @file com_manager.h
 * @brief Defines the communication manager interface for serial logging and terminal output.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdarg>
#include "config.h"



typedef enum
{
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    TERMINAL_OUTPUT,
    TERMINAL_PROMPT,
    TERMINAL_FLUSH
} com_message_type_t;

typedef struct
{
    com_message_type_t type;
    union
    {
        char content[COM_MESSAGE_MAX_LENGTH];
    };
} com_message_t;

extern QueueHandle_t com_queue;
extern QueueHandle_t com_flush_signal_queue;

void com_task(void *pvParameters);

void com_send_log(com_message_type_t type, const char *format, ...);
void com_send_prompt(const char *prompt);
void com_flush_output();
const char *com_format_bytes(uint32_t bytes);