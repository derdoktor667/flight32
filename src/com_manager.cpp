/**
 * @file com_manager.cpp
 * @brief Implements the communication manager for serial logging and terminal output.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "com_manager.h"
#include "config.h"
#include <Arduino.h>

QueueHandle_t com_queue;
QueueHandle_t com_flush_signal_queue;

void com_task(void *pvParameters)
{
    com_queue = xQueueCreate(COM_QUEUE_LENGTH, sizeof(com_message_t));
    com_flush_signal_queue = xQueueCreate(1, sizeof(uint8_t)); // Queue for signaling flush completion

    com_message_t msg;
    uint8_t dummy = 0;

    while (1)
    {
        if (xQueueReceive(com_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            switch (msg.type)
            {
            case LOG_INFO:
                Serial.print("[INFO] ");
                Serial.println(msg.content);
                break;
            case LOG_WARN:
                Serial.print("[WARN] ");
                Serial.println(msg.content);
                break;
            case LOG_ERROR:
                Serial.print("[ERROR] ");
                Serial.println(msg.content);
                break;
            case TERMINAL_OUTPUT:
                Serial.println(msg.content);
                break;
            case TERMINAL_PROMPT:
                Serial.print(msg.content);
                break;
            case TERMINAL_FLUSH:
                // Signal that flush message has been processed
                xQueueSend(com_flush_signal_queue, &dummy, 0);
                break;
            default:
                break;
            }
        }
    }
}

void com_send_log(com_message_type_t type, const char *format, ...)
{
    com_message_t msg;
    msg.type = type;

    va_list args;
    va_start(args, format);
    vsnprintf(msg.content, COM_MESSAGE_MAX_LENGTH, format, args);
    va_end(args);

    xQueueSend(com_queue, &msg, 0);
}

void com_send_prompt(const char *prompt)
{
    com_message_t msg;
    msg.type = TERMINAL_PROMPT;
    strncpy(msg.content, prompt, COM_MESSAGE_MAX_LENGTH);
    xQueueSend(com_queue, &msg, portMAX_DELAY);
}

void com_flush_output()
{
    com_message_t msg;
    msg.type = TERMINAL_FLUSH;
    uint8_t dummy;
    // Send flush message and wait for it to be processed
    xQueueSend(com_queue, &msg, portMAX_DELAY);
    xQueueReceive(com_flush_signal_queue, &dummy, portMAX_DELAY);
}