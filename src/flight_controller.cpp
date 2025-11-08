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

FlightController::~FlightController()
{
    delete _mpu6050_task;
    delete _ibus_receiver_task;
    delete _terminal_task;
    delete _motor_task;
    delete _pid_task;
}

void FlightController::setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Create the IO manager task
    xTaskCreate(
        com_task,            // Task function
        COM_TASK_NAME,       // Task name
        COM_TASK_STACK_SIZE, // Stack size
        NULL,                // Parameter
        COM_TASK_PRIORITY,   // Priority
        NULL                 // Task handle
    );

    vTaskDelay(pdMS_TO_TICKS(10));

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(LOG_INFO, "Flight32 Firmware v%s starting...", get_firmware_version());

    _settings_manager.begin();

    // --- MPU6050 Initialization ---
    delay(SENSOR_POWER_UP_DELAY_MS); // Add a small delay to allow the sensor to power up

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
    _mpu6050_task = new Mpu6050Task(MPU6050_TASK_NAME, MPU6050_TASK_STACK_SIZE, MPU6050_TASK_PRIORITY, MPU6050_TASK_CORE, MPU6050_TASK_DELAY_MS, _mpu6050_sensor);
    _ibus_receiver_task = new IbusTask(IBUS_TASK_NAME, IBUS_TASK_STACK_SIZE, IBUS_TASK_PRIORITY, IBUS_TASK_CORE, IBUS_TASK_DELAY_MS);
    _motor_task = new MotorTask(MOTOR_TASK_NAME, MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY, MOTOR_TASK_CORE, MOTOR_TASK_DELAY_MS, MOTOR_PINS_ARRAY, &_settings_manager);
    _pid_task = new PidTask(PID_TASK_NAME, PID_TASK_STACK_SIZE, PID_TASK_PRIORITY, PID_TASK_CORE, PID_TASK_DELAY_MS, _mpu6050_task, _ibus_receiver_task, _motor_task, &_settings_manager);
    _terminal_task = new TerminalTask(TERMINAL_TASK_NAME, TERMINAL_TASK_STACK_SIZE, TERMINAL_TASK_PRIORITY, TERMINAL_TASK_CORE, TERMINAL_TASK_DELAY_MS, &_scheduler, &_mpu6050_sensor, _ibus_receiver_task, _motor_task, _pid_task, &_settings_manager);

    // Add tasks to scheduler
    _scheduler.addTask(_ibus_receiver_task);
    _scheduler.addTask(_mpu6050_task);
    _scheduler.addTask(_motor_task);
    _scheduler.addTask(_pid_task);
    _scheduler.addTask(_terminal_task);

        // Manually call setup for each task

        for (uint8_t i = 0; i < _scheduler.getTaskCount(); i++)

        {

            _scheduler.getTask(i)->setup();

        }

    

        com_send_log(LOG_INFO, "Welcome, type 'help' for a list of commands.");

        com_flush_output();

        _terminal_task->_show_prompt();

    

        _scheduler.start();
}
