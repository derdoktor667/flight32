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
#include "tasks/com_manager_task.h" // Include the new ComManagerTask header
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

void FlightController::handleFatalError(const char *message)
{
    com_send_log(ComMessageType::LOG_ERROR, message);
    setSystemState(SystemState::ERROR);
    while (true)
    {
        vTaskDelay(portMAX_DELAY); // Halt
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

void FlightController::enterEscPassthrough()
{
    if (_system_state == SystemState::ARMED || _system_state == SystemState::IN_FLIGHT)
    {
        com_send_log(ComMessageType::LOG_ERROR, "Cannot enter ESC Passthrough mode while armed or in flight.");
        return;
    }

    com_send_log(ComMessageType::LOG_INFO, "FlightController: Entering ESC Passthrough mode.");
    setSystemState(SystemState::ESC_PASSTHROUGH);

    // Suspend relevant tasks
    if (_pid_task)
    {
        _pid_task->suspend();
    }
    if (_rx_task)
    {
        _rx_task->suspend();
    }

    // Inform MotorTask to enter passthrough mode
    if (_motor_task)
    {
        _motor_task->enterPassthroughMode(); // Need to implement this method in MotorTask
    }

    // Inform SerialManagerTask to switch to passthrough mode
    if (_serial_manager_task)
    {
        _serial_manager_task->enterPassthroughMode(); // Need to implement this method in SerialManagerTask
    }
}

void FlightController::exitEscPassthrough()
{
    com_send_log(ComMessageType::LOG_INFO, "FlightController: Exiting ESC Passthrough mode.");
    setSystemState(SystemState::READY);

    // Resume relevant tasks
    if (_pid_task)
    {
        _pid_task->resume();
    }
    if (_rx_task)
    {
        _rx_task->resume();
    }

    // Inform MotorTask to exit passthrough mode
    if (_motor_task)
    {
        _motor_task->exitPassthroughMode();
    }

    // Inform SerialManagerTask to exit passthrough mode
    if (_serial_manager_task)
    {
        _serial_manager_task->exitPassthroughMode();
    }
}

// Setup function for the FlightController, initializes hardware and FreeRTOS tasks.
void FlightController::setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // just to be sure I2C running maximum speed
    Wire.setClock(MPU6050_I2C_CLOCK_SPEED);

    com_manager_init(); // Initialize communication queues early

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
    String imu_type_str = _settings_manager.getSettingValue(NVS_KEY_IMU_TYPE);
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
        handleFatalError("No valid IMU type configured. Halting execution.");
    }

    // Get LPF bandwidth from settings and convert to appropriate enum.
    uint8_t lpf_bandwidth_index = _settings_manager.getSettingValue(NVS_KEY_IMU_LPF_BANDWIDTH).toInt();
    ImuLpfBandwidthIndex lpf_bandwidth = static_cast<ImuLpfBandwidthIndex>(lpf_bandwidth_index);

    // Initialize the IMU sensor with configured parameters.
    // The useDMP parameter is now handled internally by ImuMpu6050::begin()
    // The I2C clock speed is set globally by Wire.setClock()

    if (!_imu_sensor->begin(false, ImuGyroRangeIndex::GYRO_RANGE_2000DPS, ImuAccelRangeIndex::ACCEL_RANGE_16G, lpf_bandwidth))
    {
        // Error message is already printed in the sensor's begin() method.
        handleFatalError("IMU initialization failed. Halting execution.");
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
    _com_manager_task = std::make_unique<ComManagerTask>(COM_TASK_NAME, COM_TASK_STACK_SIZE, COM_TASK_PRIORITY, COM_TASK_CORE, COM_TASK_DELAY_MS);
    _imu_task = std::make_unique<ImuTask>(IMU_TASK_NAME, IMU_TASK_STACK_SIZE, IMU_TASK_PRIORITY, IMU_TASK_CORE, IMU_TASK_DELAY_MS, *_imu_sensor, &_settings_manager);
    _rx_task = std::make_unique<RxTask>(RX_TASK_NAME, RX_TASK_STACK_SIZE, RX_TASK_PRIORITY, RX_TASK_CORE, RX_TASK_DELAY_MS, &_settings_manager);
    _motor_task = std::make_unique<MotorTask>(MOTOR_TASK_NAME, MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY, MOTOR_TASK_CORE, MOTOR_TASK_DELAY_MS, MOTOR_PINS_ARRAY, &_settings_manager);
    _pid_task = std::make_unique<PidTask>(PidConfig::TASK_NAME, PidConfig::TASK_STACK_SIZE, PidConfig::TASK_PRIORITY, PidConfig::TASK_CORE, PidConfig::TASK_DELAY_MS, _imu_task.get(), _rx_task.get(), _motor_task.get(), &_settings_manager); // Removed extra _pid_task.get()

    // Add tasks to the scheduler.
    _scheduler.addTask(_com_manager_task.get());
    _scheduler.addTask(_rx_task.get());
    _scheduler.addTask(_imu_task.get());
    _scheduler.addTask(_motor_task.get());
    _scheduler.addTask(_pid_task.get());

    // Create and add the Serial Manager task.
    _serial_manager_task = std::make_unique<SerialManagerTask>(SERIAL_MANAGER_TASK_NAME, SERIAL_MANAGER_TASK_STACK_SIZE, SERIAL_MANAGER_TASK_PRIORITY, SERIAL_MANAGER_TASK_CORE, SERIAL_MANAGER_TASK_DELAY_MS, &_scheduler, this, _imu_task.get(), _rx_task.get(), _motor_task.get(), _pid_task.get(), &_settings_manager);
    _scheduler.addTask(_serial_manager_task.get());

    // Call setup() for each registered task.
    for (uint8_t i = 0; i < _scheduler.getTaskCount(); i++)
    {
        _scheduler.getTask(i)->setup();
    }

    setSystemState(SystemState::READY);

    _scheduler.start(); // Start the FreeRTOS scheduler, beginning task execution.

    // Display welcome message and prompt AFTER scheduler has started and tasks are running.
    com_send_log(ComMessageType::LOG_INFO, "Welcome, type 'help' for a list of commands.");
    com_send_log(ComMessageType::TERMINAL_OUTPUT, "");
    com_flush_output();
    _serial_manager_task->showPrompt();
}