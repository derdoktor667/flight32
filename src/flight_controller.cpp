#include "flight_controller.h"
#include "scheduler/scheduler.h"
#include "tasks/terminal_task.h"
#include "com_manager.h"
#include "config.h"
#include "firmware.h"
#include "settings_manager.h"
#include <Arduino.h>
#include "tasks/mpu6050_task.h"
#include "tasks/motor_task.h"
#include <Wire.h>

// The global queue handle, declared as extern in com_manager.h
QueueHandle_t com_queue;

FlightController::~FlightController()
{
    delete _mpu6050_task;
    delete _ibus_receiver_task;
    delete _terminal_task;
    delete _motor_task;
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

    _settings_manager.begin();

    // --- MPU6050 Initialization ---
    delay(100); // Add a small delay to allow the sensor to power up

    if (!_mpu6050_sensor.begin((GyroRange)_settings_manager.getSettingValue(SettingsManager::KEY_MPU_GYRO_RANGE).toInt(), AccelRange::ACCEL_RANGE_2G, LpfBandwidth::LPF_188HZ_N_2MS))
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

    // --- Task Creation ---
    _mpu6050_task = new Mpu6050Task("GYRO / MPU6050", MPU6050_TASK_STACK_SIZE, MPU6050_TASK_PRIORITY, MPU6050_TASK_CORE_ID, MPU6050_TASK_DELAY_MS, _mpu6050_sensor);
    _ibus_receiver_task = new IbusTask("RX / IBUS", IBUS_TASK_STACK_SIZE, IBUS_TASK_PRIORITY, IBUS_TASK_CORE_ID, IBUS_TASK_DELAY_MS);
    _motor_task = new MotorTask("MOTORS / DShot", MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY, MOTOR_TASK_CORE_ID, MOTOR_TASK_DELAY_MS, MOTOR_PINS, DSHOT_PROTOCOL);
    _terminal_task = new TerminalTask("CLI / Terminal", TERMINAL_TASK_STACK_SIZE, TERMINAL_TASK_PRIORITY, TERMINAL_TASK_CORE_ID, TERMINAL_TASK_DELAY_MS, &_scheduler, &_mpu6050_sensor, _ibus_receiver_task, _motor_task, &_settings_manager);

    // Add tasks to scheduler
    _scheduler.addTask(_ibus_receiver_task);
    _scheduler.addTask(_mpu6050_task);
    _scheduler.addTask(_terminal_task);
    _scheduler.addTask(_motor_task);

    _scheduler.start();
}
