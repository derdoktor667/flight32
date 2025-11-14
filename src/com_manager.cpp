/**
 * @file com_manager.cpp
 * @brief Implements the communication manager for serial logging and terminal output.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "com_manager.h"
#include "com_manager_config.h"
#include "terminal/terminal_config.h"
#include <Arduino.h>

QueueHandle_t com_queue;
QueueHandle_t com_flush_signal_queue;

static ComSerialMode _current_com_mode = ComSerialMode::TERMINAL; // Default mode

void com_task(void *pvParameters)
{
    com_queue = xQueueCreate(COM_QUEUE_LENGTH, sizeof(com_message_t));
    com_flush_signal_queue = xQueueCreate(COM_FLUSH_QUEUE_LENGTH, sizeof(uint8_t));

    com_message_t msg;
    uint8_t dummy = 0;

    while (1)
    {
        if (xQueueReceive(com_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            // Only print anything if in TERMINAL mode
            if (_current_com_mode == ComSerialMode::TERMINAL)
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
                    xQueueSend(com_flush_signal_queue, &dummy, 0);
                    break;
                default:
                    break;
                }
            }
            else if (msg.type == TERMINAL_FLUSH)
            {
                // Even in MSP mode, we might need to acknowledge a flush if it was sent
                xQueueSend(com_flush_signal_queue, &dummy, 0);
            }
        }
    }
}

void com_send_log(com_message_type_t type, const char *format, ...)
{
    // Only send log messages to the queue if in TERMINAL mode
    if (_current_com_mode == ComSerialMode::TERMINAL)
    {
        com_message_t msg;
        msg.type = type;

        va_list args;
        va_start(args, format);
        vsnprintf(msg.content, COM_MESSAGE_MAX_LENGTH, format, args);
        va_end(args);

        xQueueSend(com_queue, &msg, 0);
    }
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
    xQueueSend(com_queue, &msg, portMAX_DELAY);
    xQueueReceive(com_flush_signal_queue, &dummy, portMAX_DELAY);
}

static char _com_byte_buffer[BYTE_BUFFER_SIZE];

const char *com_format_bytes(uint32_t bytes)
{
    if (bytes < BYTES_IN_KB)
    {
        snprintf(_com_byte_buffer, sizeof(_com_byte_buffer), "%lu B", bytes);
    }
    else if (bytes < BYTES_IN_MB)
    {
        snprintf(_com_byte_buffer, sizeof(_com_byte_buffer), "%.1f KB", (float)bytes / BYTES_IN_KB);
    }
    else
    {
        snprintf(_com_byte_buffer, sizeof(_com_byte_buffer), "%.1f MB", (float)bytes / BYTES_IN_MB);
    }
    return _com_byte_buffer;
}

void com_set_serial_mode(ComSerialMode mode)
{
    _current_com_mode = mode;
}
