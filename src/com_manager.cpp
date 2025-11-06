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
            case MPU6050_DATA:
                Serial.printf("Acc: x=%.2f, y=%.2f, z=%.2f | Gyro: x=%.2f, y=%.2f, z=%.2f | Temp: %.2f C\n",
                              msg.mpu6050_data.accelerometer.x,
                              msg.mpu6050_data.accelerometer.y,
                              msg.mpu6050_data.accelerometer.z,
                              msg.mpu6050_data.gyroscope.x,
                              msg.mpu6050_data.gyroscope.y,
                              msg.mpu6050_data.gyroscope.z,
                              msg.mpu6050_data.temperature_celsius);
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

void com_send_mpu6050_data(const SensorReadings &data)
{
    com_message_t msg;
    msg.type = MPU6050_DATA;
    msg.mpu6050_data = data;
    xQueueSend(com_queue, &msg, 0);
}

void com_send_prompt(const char *prompt)
{
    com_message_t msg;
    msg.type = TERMINAL_PROMPT;
    strncpy(msg.content, prompt, COM_MESSAGE_MAX_LENGTH);
    xQueueSend(com_queue, &msg, 0);
}
