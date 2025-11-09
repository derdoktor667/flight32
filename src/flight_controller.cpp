/**
 * @file flight_controller.cpp
 * @brief Initializes and manages all flight controller components and FreeRTOS tasks.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

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
#include "tasks/rx_task.h" // Include the generic RX task
#include "rx_protocol.h"   // Include the RX protocol interface
#include "rx_ibus_protocol.h" // Include the IBUS protocol implementation
#include <Wire.h>

FlightController::~FlightController()
{
    delete _mpu6050_task;
    delete _rx_task;
    delete _rx_protocol; // Delete the dynamically allocated protocol
    delete _terminal_task;
    delete _motor_task;
    delete _pid_task;
}

void FlightController::setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Create the IO manager task
    xTaskCreate(
        com_task,
        COM_TASK_NAME,
        COM_TASK_STACK_SIZE,
        NULL,
        COM_TASK_PRIORITY,
        NULL
    );

    vTaskDelay(pdMS_TO_TICKS(COM_TASK_STARTUP_DELAY_MS));

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(LOG_INFO, "Flight32 Firmware v%s starting...", get_firmware_version());

    _settings_manager.begin();

    // --- MPU6050 Initialization ---
    delay(SENSOR_POWER_UP_DELAY_MS); // Add a small delay to allow the sensor to power up

    if (!_mpu6050_sensor.begin((GyroRange)DEFAULT_GYRO_RANGE, AccelRange::ACCEL_RANGE_2G, LpfBandwidth::LPF_188HZ_N_2MS))
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

    // --- RX Protocol Selection and Task Creation ---
    RxProtocolType rx_protocol_type = (RxProtocolType)_settings_manager.getSettingValue(KEY_RX_PROTOCOL).toInt();

    switch (rx_protocol_type)
    {
    case RxProtocolType::IBUS:
        _rx_protocol = new RxIbusProtocol();
        com_send_log(LOG_INFO, "Selected RX Protocol: IBUS");
        break;
    case RxProtocolType::PPM:
        // _rx_protocol = new RxPpmProtocol(); // Placeholder for future PPM implementation
        com_send_log(LOG_WARN, "Selected RX Protocol: PPM (Not yet implemented, falling back to IBUS)");
        _rx_protocol = new RxIbusProtocol(); // Fallback
        break;
    default:
        com_send_log(LOG_ERROR, "Unknown RX Protocol selected, falling back to IBUS");
        _rx_protocol = new RxIbusProtocol(); // Fallback
        break;
    }

    // --- Task Creation ---
    _mpu6050_task = new Mpu6050Task(MPU6050_TASK_NAME, MPU6050_TASK_STACK_SIZE, MPU6050_TASK_PRIORITY, MPU6050_TASK_CORE, MPU6050_TASK_DELAY_MS, _mpu6050_sensor);
    _rx_task = new RxTask(RX_TASK_NAME, RX_TASK_STACK_SIZE, RX_TASK_PRIORITY, RX_TASK_CORE, RX_TASK_DELAY_MS, _rx_protocol); // Use RX_TASK_NAME and _rx_protocol
    _motor_task = new MotorTask(MOTOR_TASK_NAME, MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY, MOTOR_TASK_CORE, MOTOR_TASK_DELAY_MS, MOTOR_PINS_ARRAY, &_settings_manager);
    _pid_task = new PidTask(PID_TASK_NAME, PID_TASK_STACK_SIZE, PID_TASK_PRIORITY, PID_TASK_CORE, PID_TASK_DELAY_MS, _mpu6050_task, _rx_task, _motor_task, &_settings_manager); // Pass _rx_task
    _terminal_task = new TerminalTask(TERMINAL_TASK_NAME, TERMINAL_TASK_STACK_SIZE, TERMINAL_TASK_PRIORITY, TERMINAL_TASK_CORE, TERMINAL_TASK_DELAY_MS, &_scheduler, &_mpu6050_sensor, _rx_task, _motor_task, _pid_task, &_settings_manager); // Pass _rx_task

    // Add tasks to scheduler
    _scheduler.addTask(_rx_task); // Add _rx_task
    _scheduler.addTask(_mpu6050_task);
    _scheduler.addTask(_motor_task);
    _scheduler.addTask(_pid_task);
    _scheduler.addTask(_terminal_task);

    for (uint8_t i = 0; i < _scheduler.getTaskCount(); i++)

    {

        _scheduler.getTask(i)->setup();
    }

    com_send_log(LOG_INFO, "Welcome, type 'help' for a list of commands.");

    com_flush_output();

    _terminal_task->_show_prompt();

    _scheduler.start();
}