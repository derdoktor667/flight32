/**
 * @file flight_controller.cpp
 * @brief Initializes and manages all flight controller components and FreeRTOS tasks.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "flight_controller.h"
#include "scheduler/scheduler.h"
#include "tasks/serial_manager_task.h"
#include "com_manager.h"
#include "config/serial_config.h"
#include "config/pid_config.h"
#include "utils/version_info.h"
#include "settings_manager.h"
#include "utils/task_names.h"
#include <Arduino.h>
#include "tasks/imu_task.h"
#include "imu/sensors/imu_mpu6050.h"
#include "tasks/motor_task.h"
#include "tasks/rx_task.h"
#include "config/rx_config.h"
#include <Wire.h>

FlightController::~FlightController()
{
    delete _imu_sensor;
    delete _imu_task;
    delete _rx_task;
    delete _serial_manager_task;
    delete _motor_task;
    delete _pid_task;
}

void FlightController::setup()
{
    Wire.begin();
    Wire.setClock(MPU6050_I2C_CLOCK_SPEED); // Set I2C clock to 1MHz (Fast Mode Plus)

    Serial.begin(SERIAL_BAUD_RATE);

    xTaskCreate(
        com_task,
        COM_TASK_NAME,
        COM_TASK_STACK_SIZE,
        NULL,
        COM_TASK_PRIORITY,
        NULL);

    vTaskDelay(pdMS_TO_TICKS(COM_TASK_STARTUP_DELAY_MS));

    com_send_log(TERMINAL_OUTPUT, "");
    com_send_log(TERMINAL_OUTPUT, "========================================");
    com_send_log(TERMINAL_OUTPUT, " Flight32 Flight Controller");
    com_send_log(TERMINAL_OUTPUT, "========================================");
    com_send_log(LOG_INFO, "Firmware v%s starting...", FIRMWARE_VERSION);

    _settings_manager.begin();

    delay(SENSOR_POWER_UP_DELAY_MS);

    String imu_type_str = _settings_manager.getSettingValue(KEY_IMU_TYPE);
    ImuType imu_type;
    if (imu_type_str.length() == 0)
    {
        imu_type = DEFAULT_IMU_TYPE;
    }
    else
    {
        imu_type = (ImuType)imu_type_str.toInt();
    }

    switch (imu_type)
    {
    case ImuType::MPU6050:
        _imu_sensor = new ImuMpu6050();
        com_send_log(LOG_INFO, "Using MPU6050 IMU.");
        break;
    default:
        com_send_log(LOG_ERROR, "No valid IMU type configured.");
        // Handle error, maybe loop forever or use a dummy sensor
        return;
    }

    uint8_t lpf_bandwidth_index = _settings_manager.getSettingValue(KEY_IMU_LPF_BANDWIDTH).toInt();
    LpfBandwidth lpf_bandwidth = ImuMpu6050::getLpfBandwidthFromIndex(lpf_bandwidth_index);

    if (!_imu_sensor->begin(MPU6050_I2C_CLOCK_SPEED, IMU_DMP_ENABLED_DEFAULT, GYRO_RANGE_2000DPS, ACCEL_RANGE_16G, lpf_bandwidth))
    {
        // Error message is already printed in the sensor's begin() method
        // Handle error, maybe loop forever
        return;
    }

    ImuAxisData gyro_offsets = _settings_manager.getGyroOffsets();
    ImuAxisData accel_offsets = _settings_manager.getAccelOffsets();

    com_send_log(LOG_INFO, "Starting IMU calibration...");
    _imu_sensor->calibrate();
    _settings_manager.setGyroOffsets(_imu_sensor->getGyroscopeOffset());
    _settings_manager.setAccelOffsets(_imu_sensor->getAccelerometerOffset());
    _settings_manager.saveSettings();
    com_send_log(LOG_INFO, "IMU calibration complete and offsets saved.");

    _imu_task = new ImuTask(IMU_TASK_NAME, IMU_TASK_STACK_SIZE, IMU_TASK_PRIORITY, IMU_TASK_CORE, IMU_TASK_DELAY_MS, *_imu_sensor);
    _rx_task = new RxTask(RX_TASK_NAME, RX_TASK_STACK_SIZE, RX_TASK_PRIORITY, RX_TASK_CORE, RX_TASK_DELAY_MS, &_settings_manager);
    _motor_task = new MotorTask(MOTOR_TASK_NAME, MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY, MOTOR_TASK_CORE, MOTOR_TASK_DELAY_MS, MOTOR_PINS_ARRAY, &_settings_manager);
    _pid_task = new PidTask(PID_TASK_NAME, PID_TASK_STACK_SIZE, PID_TASK_PRIORITY, PID_TASK_CORE, PID_TASK_DELAY_MS, _imu_task, _rx_task, _motor_task, &_settings_manager);

    _scheduler.addTask(_rx_task);
    _scheduler.addTask(_imu_task);
    _scheduler.addTask(_motor_task);
    _scheduler.addTask(_pid_task);

    _serial_manager_task = new SerialManagerTask(SERIAL_MANAGER_TASK_NAME, SERIAL_MANAGER_TASK_STACK_SIZE, SERIAL_MANAGER_TASK_PRIORITY, SERIAL_MANAGER_TASK_CORE, SERIAL_MANAGER_TASK_DELAY_MS, &_scheduler, _imu_task, _rx_task, _motor_task, _pid_task, &_settings_manager);
    _scheduler.addTask(_serial_manager_task);

    for (uint8_t i = 0; i < _scheduler.getTaskCount(); i++)
    {
        _scheduler.getTask(i)->setup();
    }

    com_send_log(LOG_INFO, "Welcome, type 'help' for a list of commands.");
    com_flush_output();
    _serial_manager_task->showPrompt();

    _scheduler.start();
}
