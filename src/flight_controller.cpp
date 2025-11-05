#include "flight_controller.h"
#include "scheduler/scheduler.h"
#include "tasks/terminal_task.h"
#include "com_manager.h"
#include "config.h"
#include "firmware.h"
#include <Arduino.h>
#include "tasks/mpu6050_task.h"
#include "tasks/motor_task.h"
#include <Wire.h>

// The global queue handle, declared as extern in com_manager.h
QueueHandle_t com_queue;

// Define motor pins
static const int motor_pins[NUM_MOTORS] = {MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4};

FlightController::FlightController() : _scheduler(), // Initialize as an instance
                                       _mpu6050_sensor(),
                                       _mpu6050_task("MPU6050", MPU6050_TASK_STACK_SIZE, (UBaseType_t)MPU6050_TASK_PRIORITY, (BaseType_t)MPU6050_TASK_CORE_ID, (uint32_t)MPU6050_TASK_DELAY_MS, _mpu6050_sensor),
                                       _ibus_receiver_task("IBus", IBUS_TASK_STACK_SIZE, (UBaseType_t)IBUS_TASK_PRIORITY, (BaseType_t)IBUS_TASK_CORE_ID, (uint32_t)IBUS_TASK_DELAY_MS),
    _terminal_task("CLI / Terminal", TERMINAL_TASK_STACK_SIZE, (UBaseType_t)TERMINAL_TASK_PRIORITY, (BaseType_t)TERMINAL_TASK_CORE_ID, (uint32_t)TERMINAL_TASK_DELAY_MS, &_scheduler, &_mpu6050_sensor, &_ibus_receiver_task, &_motor_task),
                                       _motor_task("Motors", MOTOR_TASK_STACK_SIZE, (UBaseType_t)MOTOR_TASK_PRIORITY, (BaseType_t)MOTOR_TASK_CORE_ID, (uint32_t)MOTOR_TASK_DELAY_MS, motor_pins, DSHOT_PROTOCOL)
{
}

void FlightController::setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Create the IO message queue and assign it to the global handle
    com_queue = xQueueCreate(COM_QUEUE_LENGTH, sizeof(com_message_t));

    // Create the IO manager task
    xTaskCreate(
        com_task,            // Task function
        "com_task",          // Task name
        COM_TASK_STACK_SIZE, // Stack size
        NULL,                // Parameter
        COM_TASK_PRIORITY,   // Priority
        NULL                 // Task handle
    );

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(LOG_INFO, "Flight32 Firmware v%s starting...", get_firmware_version());

    // --- MPU6050 Initialization ---
    com_send_log(LOG_INFO, "Initializing MPU6050...");
    delay(100); // Add a small delay to allow the sensor to power up
    if (!_mpu6050_sensor.begin())
    {
        com_send_log(LOG_ERROR, "Failed to find MPU6050 chip");
        // Handle error, maybe halt or blink an LED
    }
    else
    {
        com_send_log(LOG_INFO, "MPU6050 Found! Calibrating...");
        _mpu6050_sensor.calibrate();
        com_send_log(LOG_INFO, "MPU6050 Calibration complete.");
    }

    // Add tasks to scheduler
    _scheduler.addTask(&_ibus_receiver_task);
    _scheduler.addTask(&_mpu6050_task);
    _scheduler.addTask(&_terminal_task);
    _scheduler.addTask(&_motor_task);

    _scheduler.start();
}