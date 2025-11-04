#include "com_manager.h"
#include "config.h"
#include <Arduino.h>

void com_task(void *pvParameters)
{
    com_message_t msg;

    while (1)
    {
        if (xQueueReceive(com_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            switch (msg.type)
            {
            case LOG_INFO:
                Serial.print("[INFO] ");
                break;
            case LOG_WARN:
                Serial.print("[WARN] ");
                break;
            case LOG_ERROR:
                Serial.print("[ERROR] ");
                break;
            case TERMINAL_OUTPUT:
            default:
                break;
            }
            Serial.println(msg.content);
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
