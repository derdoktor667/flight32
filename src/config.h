/**
 * @file config.h
 * @brief Central configuration file for the Flight32 firmware.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <cstdint>
#include <DShotRMT.h>

// --- Serial Configuration ---
static constexpr auto SERIAL_BAUD_RATE = 115200;

// --- Scheduler Configuration ---
static constexpr uint8_t MAX_TASKS = 10;
static constexpr uint32_t SCHEDULER_TASK_STACK_SIZE = 4096;
static constexpr uint8_t SCHEDULER_TASK_PRIORITY = 2;
static constexpr int8_t SCHEDULER_TASK_CORE = 0;

// --- Terminal Task Configuration ---
static constexpr uint32_t TERMINAL_TASK_STACK_SIZE = 8192;
static constexpr uint8_t TERMINAL_TASK_PRIORITY = 1;
static constexpr int8_t TERMINAL_TASK_CORE = 0;
static constexpr uint16_t TERMINAL_INPUT_BUFFER_SIZE = 128;
static constexpr uint8_t TERMINAL_TASK_DELAY_MS = 50;

// --- Com Task Configuration ---
static constexpr uint32_t COM_TASK_STACK_SIZE = 4096;
static constexpr uint8_t COM_TASK_PRIORITY = 2;
static constexpr uint8_t COM_QUEUE_LENGTH = 20;
static constexpr uint16_t COM_MESSAGE_MAX_LENGTH = 256;
static constexpr uint32_t COM_TASK_STARTUP_DELAY_MS = 10;

// --- MPU6050 Task Configuration ---
static constexpr uint32_t MPU6050_TASK_STACK_SIZE = 4096;
static constexpr uint8_t MPU6050_TASK_PRIORITY = 5;
static constexpr int8_t MPU6050_TASK_CORE = 1;
static constexpr uint8_t MPU6050_TASK_DELAY_MS = 10;
static constexpr uint8_t MPU6050_I2C_SDA = 21;
static constexpr uint8_t MPU6050_I2C_SCL = 22;
static constexpr uint32_t MPU6050_I2C_CLOCK_SPEED = 400000;

// --- IBUS Task Configuration ---
static constexpr uint32_t RX_TASK_STACK_SIZE = 4096;
static constexpr uint8_t RX_TASK_PRIORITY = 4;
static constexpr int8_t RX_TASK_CORE = 1;
static constexpr uint8_t RX_TASK_DELAY_MS = 20;
static constexpr uint8_t IBUS_UART_NUM = 2;
static constexpr uint8_t IBUS_RX_PIN = 16;
static constexpr uint8_t IBUS_TX_PIN = 17;
static constexpr uint32_t IBUS_BAUD_RATE = 115200;

// --- RX Protocol Configuration ---
enum class RxProtocolType : uint8_t
{
    IBUS = 0,
    PPM = 1, // Placeholder for future PPM implementation
    // Add other protocols here
};

static constexpr const char *KEY_RX_PROTOCOL = "rx.protocol";
static constexpr RxProtocolType DEFAULT_RX_PROTOCOL = RxProtocolType::IBUS;

// --- Motor Task Configuration ---
static constexpr uint32_t MOTOR_TASK_STACK_SIZE = 4096;
static constexpr uint8_t MOTOR_TASK_PRIORITY = 3;
static constexpr int8_t MOTOR_TASK_CORE = 1;
static constexpr uint8_t MOTOR_TASK_DELAY_MS = 10;
static constexpr uint8_t NUM_MOTORS = 4;
static constexpr uint8_t MOTOR_PIN_FR = 25;              // Front Right
static constexpr uint8_t MOTOR_PIN_RL = 26;              // Rear Left
static constexpr uint8_t MOTOR_PIN_FL = 33;              // Front Left
static constexpr uint8_t MOTOR_PIN_RR = 27;              // Rear Right
static constexpr dshot_mode_t DSHOT_PROTOCOL = DSHOT300; // DSHOT150, DSHOT300, DSHOT600, DSHOT1200
static const int MOTOR_PINS_ARRAY[NUM_MOTORS] = {MOTOR_PIN_RR, MOTOR_PIN_FR, MOTOR_PIN_RL, MOTOR_PIN_FL};

// --- PID Task Configuration ---
static constexpr uint32_t PID_TASK_STACK_SIZE = 4096;
static constexpr uint8_t PID_TASK_PRIORITY = 4;
static constexpr int8_t PID_TASK_CORE = 1;
static constexpr uint8_t PID_TASK_DELAY_MS = 10;

// --- PID Gain Settings ---
static constexpr const char *KEY_PID_ROLL_P = "pid.roll.p";
static constexpr const char *KEY_PID_ROLL_I = "pid.roll.i";
static constexpr const char *KEY_PID_ROLL_D = "pid.roll.d";
static constexpr const char *KEY_PID_PITCH_P = "pid.pitch.p";
static constexpr const char *KEY_PID_PITCH_I = "pid.pitch.i";
static constexpr const char *KEY_PID_PITCH_D = "pid.pitch.d";
static constexpr const char *KEY_PID_YAW_P = "pid.yaw.p";
static constexpr const char *KEY_PID_YAW_I = "pid.yaw.i";
static constexpr const char *KEY_PID_YAW_D = "pid.yaw.d";

static constexpr float PID_SCALE_FACTOR = 100.0f; // Scaling factor for PID gains

static constexpr float DEFAULT_PID_ROLL_P = 0.2f;
static constexpr float DEFAULT_PID_ROLL_I = 0.02f;
static constexpr float DEFAULT_PID_ROLL_D = 0.005f;
static constexpr float DEFAULT_PID_PITCH_P = 0.2f;
static constexpr float DEFAULT_PID_PITCH_I = 0.02f;
static constexpr float DEFAULT_PID_PITCH_D = 0.005f;
static constexpr float DEFAULT_PID_YAW_P = 0.4f;
static constexpr float DEFAULT_PID_YAW_I = 0.04f;
static constexpr float DEFAULT_PID_YAW_D = 0.01f;

// --- Motor Protocol Settings ---
static constexpr const char *KEY_MOTOR_PROTOCOL = "motor.protocol";
static constexpr uint8_t DEFAULT_MOTOR_PROTOCOL = 1; // Corresponds to DSHOT300
static constexpr const char *DSHOT_PROTOCOL_STRINGS[] = {"DSHOT150", "DSHOT300", "DSHOT600", "DSHOT1200"};
static constexpr uint8_t NUM_DSHOT_PROTOCOLS = sizeof(DSHOT_PROTOCOL_STRINGS) / sizeof(DSHOT_PROTOCOL_STRINGS[0]);

// --- Settings Manager ---
static constexpr const char *SETTINGS_NAMESPACE = "flight32";
static constexpr uint16_t CURRENT_SCHEMA_VERSION = 2;
static constexpr const char *KEY_SCHEMA_VERSION = "schema_ver";
static constexpr uint8_t DEFAULT_GYRO_RANGE = 0; // Corresponds to 250_DPS in GYRO_RANGE_STRINGS

// --- Sensor ---
static constexpr uint16_t SENSOR_POWER_UP_DELAY_MS = 100;

// --- Terminal/Utility ---
static constexpr uint32_t ONE_SECOND_MICROSECONDS = 1000000;
static constexpr uint16_t MAX_THROTTLE_VALUE = 2047;
static constexpr uint16_t BYTES_IN_KB = 1024;
static constexpr uint32_t BYTES_IN_MB = (1024 * 1024);
static constexpr uint8_t ASCII_BACKSPACE = 127;
static constexpr int8_t INVALID_SETTING_VALUE = -1;
static constexpr uint8_t UINT8_MAX_VALUE = 255;

// --- Terminal Formatting Constants ---
static constexpr uint8_t TASK_NAME_COLUMN_WIDTH = 16;
static constexpr uint8_t TASK_STATE_COLUMN_WIDTH = 10;
static constexpr uint8_t TASK_PRIO_COLUMN_WIDTH = 6;
static constexpr uint8_t TASK_CPU_COLUMN_WIDTH = 8;
static constexpr uint8_t TASK_LOOP_COLUMN_WIDTH = 10;
static constexpr uint8_t TASK_AVG_LOOP_COLUMN_WIDTH = 10;
static constexpr uint8_t TASK_MAX_LOOP_COLUMN_WIDTH = 10;
static constexpr uint8_t TASK_STACK_HWM_COLUMN_WIDTH = 17;

// New constants for terminal output formatting and channel indexing
static constexpr uint8_t TERMINAL_MIN_CHANNEL_INDEX = 1;
static constexpr uint8_t TERMINAL_MAX_CHANNEL_INDEX = 14;
static constexpr uint8_t TERMINAL_COLUMN_BUFFER_WIDTH = 2;
static constexpr uint8_t TERMINAL_RX_DATA_DISPLAY_CHANNELS = 10;
static constexpr uint8_t TERMINAL_RX_SINGLE_DESC_WIDTH = 25; // "RX Flight Mode (CH14)" is 22 chars, plus buffer

// --- IBUS Channel Constants ---
static constexpr int16_t IBUS_CHANNEL_MIN_RAW = 1000;
static constexpr int16_t IBUS_CHANNEL_MAX_RAW = 2000;
static constexpr int16_t IBUS_CHANNEL_DEFAULT_MID = 1500;
static constexpr int16_t IBUS_CHANNEL_DEFAULT_MIN = 1000;
static constexpr int16_t IBUS_CHANNEL_DEFAULT_MAX = 2000;
static constexpr float THROTTLE_DEADZONE_THRESHOLD = 0.01f;
static constexpr float MS_TO_SECONDS_FACTOR = 1000.0f;

// --- Task Names ---
static constexpr const char *COM_TASK_NAME = "com_task";
static constexpr const char *MPU6050_TASK_NAME = "GYRO / MPU6050";
static constexpr const char *RX_TASK_NAME = "RX / Receiver"; // New generic RX task name
static constexpr const char *IBUS_TASK_NAME = "RX / IBUS";
static constexpr const char *MOTOR_TASK_NAME = "MOTORS / DShot";
static constexpr const char *PID_TASK_NAME = "PID Controller";
static constexpr const char *TERMINAL_TASK_NAME = "CLI / Terminal";
static constexpr const char *IDLE_TASK_NAME_0 = "IDLE0";
static constexpr const char *IDLE_TASK_NAME_1 = "IDLE1";
static constexpr const char *TIMER_SERVICE_TASK_NAME = "Tmr Svc";