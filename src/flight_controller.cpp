#include "flight_controller.h"
#include "scheduler/scheduler.h"
#include "tasks/terminal_task.h"
#include "com_manager.h"
#include "config.h"
#include "firmware.h"
#include <Arduino.h>
#include "tasks/mpu6050_task.h"
#include <Wire.h>

// The global queue handle, declared as extern in com_manager.h
QueueHandle_t com_queue;

FlightController::FlightController() : _scheduler(nullptr) {}

void FlightController::setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Create the IO message queue and assign it to the global handle
    com_queue = xQueueCreate(COM_QUEUE_LENGTH, sizeof(com_message_t));

    // Create the IO manager task
    xTaskCreate(
        com_task,                // Task function
        "com_task",              // Task name
        COM_TASK_STACK_SIZE,     // Stack size
        NULL,                    // Parameter
        COM_TASK_PRIORITY,       // Priority
        NULL                     // Task handle
    );

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(LOG_INFO, "Flight32 Firmware v%s starting...", get_firmware_version());

    // --- MPU6050 Initialization ---
    com_send_log(LOG_INFO, "Initializing MPU6050...");
    delay(100); // Add a small delay to allow the sensor to power up
    if (!_mpu6050.begin())
    {
        com_send_log(LOG_ERROR, "Failed to find MPU6050 chip");
        // Handle error, maybe halt or blink an LED
    }
    else
    {
        com_send_log(LOG_INFO, "MPU6050 Found! Calibrating...");
        _mpu6050.calibrate();
        com_send_log(LOG_INFO, "MPU6050 Calibration complete.");
    }

    _scheduler = new Scheduler();

    Mpu6050Task *mpu6050Task = new Mpu6050Task(
        "MPU6050",
        MPU6050_TASK_STACK_SIZE,
        (UBaseType_t)MPU6050_TASK_PRIORITY,
        (BaseType_t)MPU6050_TASK_CORE_ID,
        (uint32_t)MPU6050_TASK_DELAY_MS,
        _mpu6050);
    _scheduler->addTask(mpu6050Task);

    TerminalTask *terminalTask = new TerminalTask(
        "CLI / Terminal",
        TERMINAL_TASK_STACK_SIZE,
        (UBaseType_t)TERMINAL_TASK_PRIORITY,
        (BaseType_t)TERMINAL_TASK_CORE_ID,
        (uint32_t)TERMINAL_TASK_DELAY_MS,
        _scheduler,
        &_mpu6050);
    _scheduler->addTask(terminalTask);

    _scheduler->start();
}