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
static constexpr uint16_t TERMINAL_STATS_BUFFER_SIZE = 2048;
static constexpr uint8_t TERMINAL_TASK_DELAY_MS = 50;

// --- Com Task Configuration ---
static constexpr uint32_t COM_TASK_STACK_SIZE = 4096;
static constexpr uint8_t COM_TASK_PRIORITY = 2;
static constexpr uint8_t COM_QUEUE_LENGTH = 20;

// --- MPU6050 Task Configuration ---
static constexpr uint32_t MPU6050_TASK_STACK_SIZE = 4096;
static constexpr uint8_t MPU6050_TASK_PRIORITY = 5;
static constexpr int8_t MPU6050_TASK_CORE = 1;
static constexpr uint8_t MPU6050_TASK_DELAY_MS = 10;
static constexpr uint8_t MPU6050_I2C_SDA = 21;
static constexpr uint8_t MPU6050_I2C_SCL = 22;
static constexpr uint32_t MPU6050_I2C_CLOCK_SPEED = 400000;

// --- IBUS Task Configuration ---
static constexpr uint32_t IBUS_TASK_STACK_SIZE = 4096;
static constexpr uint8_t IBUS_TASK_PRIORITY = 4;
static constexpr int8_t IBUS_TASK_CORE = 1;
static constexpr uint8_t IBUS_TASK_DELAY_MS = 20;
static constexpr uint8_t IBUS_UART_NUM = 2;
static constexpr uint8_t IBUS_RX_PIN = 16;
static constexpr uint8_t IBUS_TX_PIN = 17;
static constexpr uint32_t IBUS_BAUD_RATE = 115200;

// --- Motor Task Configuration ---
static constexpr uint32_t MOTOR_TASK_STACK_SIZE = 4096;
static constexpr uint8_t MOTOR_TASK_PRIORITY = 3;
static constexpr int8_t MOTOR_TASK_CORE = 1;
static constexpr uint8_t MOTOR_TASK_DELAY_MS = 10;
static constexpr uint8_t NUM_MOTORS = 4;
static constexpr uint16_t MOTOR_PWM_FREQUENCY = 500;
static constexpr uint8_t MOTOR_PWM_RESOLUTION = 10;
static constexpr uint8_t MOTOR_PWM_CHANNEL_0 = 0;
static constexpr uint8_t MOTOR_PWM_CHANNEL_1 = 1;
static constexpr uint8_t MOTOR_PWM_CHANNEL_2 = 2;
static constexpr uint8_t MOTOR_PWM_CHANNEL_3 = 3;
static constexpr uint8_t MOTOR_PIN_0 = 25;
static constexpr uint8_t MOTOR_PIN_1 = 26;
static constexpr uint8_t MOTOR_PIN_2 = 27;
static constexpr uint8_t MOTOR_PIN_3 = 14;
static constexpr dshot_mode_t DSHOT_PROTOCOL = DSHOT300; // DSHOT150, DSHOT300, DSHOT600, DSHOT1200
static const int MOTOR_PINS_ARRAY[NUM_MOTORS] = {27, 25, 26, 33}; // Renamed to avoid conflict with MOTOR_PIN_X

// --- Motor Protocol Settings ---
static constexpr const char *KEY_MOTOR_PROTOCOL = "motor.protocol";
static constexpr uint8_t DEFAULT_MOTOR_PROTOCOL = 1; // Corresponds to DSHOT300
static constexpr const char *DSHOT_PROTOCOL_STRINGS[] = {"DSHOT150", "DSHOT300", "DSHOT600", "DSHOT1200"};
static constexpr uint8_t NUM_DSHOT_PROTOCOLS = sizeof(DSHOT_PROTOCOL_STRINGS) / sizeof(DSHOT_PROTOCOL_STRINGS[0]);

// --- Settings Manager ---
static constexpr const char* SETTINGS_NAMESPACE = "flight32";
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

// --- Task Names ---
static constexpr const char* COM_TASK_NAME = "com_task";
static constexpr const char* MPU6050_TASK_NAME = "GYRO / MPU6050";
static constexpr const char* IBUS_TASK_NAME = "RX / IBUS";
static constexpr const char* MOTOR_TASK_NAME = "MOTORS / DShot";
static constexpr const char* TERMINAL_TASK_NAME = "CLI / Terminal";
static constexpr const char* IDLE_TASK_NAME_0 = "IDLE0";
static constexpr const char* IDLE_TASK_NAME_1 = "IDLE1";
static constexpr const char* TIMER_SERVICE_TASK_NAME = "Tmr Svc";

