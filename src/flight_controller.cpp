/**
 * @file flight_controller.cpp
 * @brief Initializes and manages all flight controller components and FreeRTOS tasks.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include <Arduino.h>
#include "flight_controller.h"
#include "scheduler/scheduler.h"
#include "tasks/serial_manager_task.h"
#include "com_manager.h"
#include "config/serial_config.h"
#include "config/pid_config.h"
#include "utils/version_info.h"
#include "settings_manager.h"
#include "utils/task_names.h"
#include "tasks/imu_task.h"
#include "imu/sensors/imu_mpu6050.h"
#include "tasks/motor_task.h"
#include "tasks/rx_task.h"
#include "config/rx_config.h"
#include <Wire.h>

// Helper function to convert SystemState enum to a string for logging.
const char *systemStateToString(SystemState state)
{
    switch (state)
    {
    case SystemState::INITIALIZING:
        return "INITIALIZING";
    case SystemState::CALIBRATING:
        return "CALIBRATING";
    case SystemState::READY:
        return "READY";
    case SystemState::ARMED:
        return "ARMED";
    case SystemState::IN_FLIGHT:
        return "IN_FLIGHT";
    case SystemState::FAILSAFE:
        return "FAILSAFE";
    case SystemState::ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}

FlightController::FlightController() : _system_state(SystemState::INITIALIZING)
{
    // Constructor body can be empty if all initialization is done in the member initializer list.
}

SystemState FlightController::getSystemState() const
{
    return _system_state;
}

void FlightController::setSystemState(SystemState new_state)
{
    if (_system_state != new_state)
    {
        _system_state = new_state;
    }
}

// Setup function for the FlightController, initializes hardware and FreeRTOS tasks.
void FlightController::setup()
{
    Wire.begin();
    Wire.setClock(MPU6050_I2C_CLOCK_SPEED);

    Serial.begin(SERIAL_BAUD_RATE);

    // Create and start the communication task.
    xTaskCreate(
        com_task,
        COM_TASK_NAME,
        COM_TASK_STACK_SIZE,
        nullptr,
        COM_TASK_PRIORITY,
        nullptr);

    // Delay to allow the communication task to initialize.
    vTaskDelay(pdMS_TO_TICKS(COM_TASK_STARTUP_DELAY_MS));

    setSystemState(SystemState::INITIALIZING);

    // Print startup messages to the serial terminal.
    com_send_log(ComMessageType::TERMINAL_OUTPUT, "");
    com_send_log(ComMessageType::TERMINAL_OUTPUT, "========================================");
    com_send_log(ComMessageType::TERMINAL_OUTPUT, " Flight32 Flight Controller");
    com_send_log(ComMessageType::TERMINAL_OUTPUT, "========================================");
    com_send_log(ComMessageType::LOG_INFO, "Firmware v%s starting...", FIRMWARE_VERSION);

    _settings_manager.begin();

    // Delay to allow sensors to power up.
    delay(SENSOR_POWER_UP_DELAY_MS);

    // Retrieve IMU type from settings.
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

    // Initialize the appropriate IMU sensor based on settings.
    switch (imu_type)
    {
    case ImuType::MPU6050:
        _imu_sensor = std::make_unique<ImuMpu6050>();
        com_send_log(ComMessageType::LOG_INFO, "Using MPU6050 IMU.");
        break;
    default:
        com_send_log(ComMessageType::LOG_ERROR, "No valid IMU type configured. Halting execution.");
        setSystemState(SystemState::ERROR);
        while (true)
        {
            vTaskDelay(portMAX_DELAY);
        } // Halt
    }

    // Get LPF bandwidth from settings and convert to appropriate enum.
    uint8_t lpf_bandwidth_index = _settings_manager.getSettingValue(KEY_IMU_LPF_BANDWIDTH).toInt();
    ImuLpfBandwidthIndex lpf_bandwidth = static_cast<ImuLpfBandwidthIndex>(lpf_bandwidth_index);

    // Initialize the IMU sensor with configured parameters.
    if (!_imu_sensor->begin(MPU6050_I2C_CLOCK_SPEED, IMU_DMP_ENABLED_DEFAULT, ImuGyroRangeIndex::GYRO_RANGE_2000DPS, ImuAccelRangeIndex::ACCEL_RANGE_16G, lpf_bandwidth))
    {
        // Error message is already printed in the sensor's begin() method.
        setSystemState(SystemState::ERROR);
        com_send_log(ComMessageType::LOG_ERROR, "IMU initialization failed. Halting execution.");
        while (true)
        {
            vTaskDelay(portMAX_DELAY);
        } // Halt
    }

    ImuAxisData gyro_offsets = _settings_manager.getGyroOffsets();
    ImuAxisData accel_offsets = _settings_manager.getAccelOffsets();

    setSystemState(SystemState::CALIBRATING);
    _imu_sensor->calibrate();
    // Save new offsets to settings.
    _settings_manager.setGyroOffsets(_imu_sensor->getGyroscopeOffset());
    _settings_manager.setAccelOffsets(_imu_sensor->getAccelerometerOffset());
    _settings_manager.saveSettings();
    com_send_log(ComMessageType::LOG_INFO, "IMU calibration complete and offsets saved.");

    // Create FreeRTOS task objects.
    _imu_task = std::make_unique<ImuTask>(IMU_TASK_NAME, IMU_TASK_STACK_SIZE, IMU_TASK_PRIORITY, IMU_TASK_CORE, IMU_TASK_DELAY_MS, *_imu_sensor, &_settings_manager);
    _rx_task = std::make_unique<RxTask>(RX_TASK_NAME, RX_TASK_STACK_SIZE, RX_TASK_PRIORITY, RX_TASK_CORE, RX_TASK_DELAY_MS, &_settings_manager);
    _motor_task = std::make_unique<MotorTask>(MOTOR_TASK_NAME, MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY, MOTOR_TASK_CORE, MOTOR_TASK_DELAY_MS, MOTOR_PINS_ARRAY, &_settings_manager);
    _pid_task = std::make_unique<PidTask>(PID_TASK_NAME, PID_TASK_STACK_SIZE, PID_TASK_PRIORITY, PID_TASK_CORE, PID_TASK_DELAY_MS, _imu_task.get(), _rx_task.get(), _motor_task.get(), &_settings_manager);

    // Add tasks to the scheduler.
    _scheduler.addTask(_rx_task.get());
    _scheduler.addTask(_imu_task.get());
    _scheduler.addTask(_motor_task.get());
    _scheduler.addTask(_pid_task.get());

    // Create and add the Serial Manager task.
    _serial_manager_task = std::make_unique<SerialManagerTask>(SERIAL_MANAGER_TASK_NAME, SERIAL_MANAGER_TASK_STACK_SIZE, SERIAL_MANAGER_TASK_PRIORITY, SERIAL_MANAGER_TASK_CORE, SERIAL_MANAGER_TASK_DELAY_MS, &_scheduler, _imu_task.get(), _rx_task.get(), _motor_task.get(), _pid_task.get(), &_settings_manager);
    _scheduler.addTask(_serial_manager_task.get());

    // Call setup() for each registered task.
    for (uint8_t i = 0; i < _scheduler.getTaskCount(); i++)
    {
        _scheduler.getTask(i)->setup();
    }

    // Display welcome message and prompt.
    com_send_log(ComMessageType::LOG_INFO, "Welcome, type 'help' for a list of commands.");
    com_flush_output();
    _serial_manager_task->showPrompt();

    setSystemState(SystemState::READY);

    _scheduler.start(); // Start the FreeRTOS scheduler, beginning task execution.
}
