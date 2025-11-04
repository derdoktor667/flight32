#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdarg>

#define COM_QUEUE_LENGTH 10
#define COM_MESSAGE_MAX_LENGTH 256

typedef enum {
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    TERMINAL_OUTPUT
} com_message_type_t;

typedef struct {
    com_message_type_t type;
    char content[COM_MESSAGE_MAX_LENGTH];
} com_message_t;

extern QueueHandle_t com_queue;

void com_task(void *pvParameters);

void com_send_log(com_message_type_t type, const char *format, ...);