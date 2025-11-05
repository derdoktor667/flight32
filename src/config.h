#pragma once

#include <cstdint>

// --- Serial Configuration ---
static constexpr auto SERIAL_BAUD_RATE = 115200;

// --- Scheduler Configuration ---
static constexpr uint8_t MAX_TASKS = 10;

// --- Terminal Task Configuration ---
static constexpr uint32_t TERMINAL_TASK_STACK_SIZE = 4096;
static constexpr uint8_t TERMINAL_TASK_PRIORITY = 1;
static constexpr int8_t TERMINAL_TASK_CORE_ID = 1;
static constexpr uint16_t TERMINAL_INPUT_BUFFER_SIZE = 128;
static constexpr uint16_t TERMINAL_STATS_BUFFER_SIZE = 2048;
static constexpr uint8_t TERMINAL_TASK_DELAY_MS = 10;

// --- Com Task Configuration ---
static constexpr uint32_t COM_TASK_STACK_SIZE = 4096;
static constexpr uint8_t COM_TASK_PRIORITY = 2;

// --- MPU6050 Task Configuration ---
static constexpr uint32_t MPU6050_TASK_STACK_SIZE = 4096;
static constexpr uint8_t MPU6050_TASK_PRIORITY = 2;
static constexpr int8_t MPU6050_TASK_CORE_ID = 0;
static constexpr uint8_t MPU6050_TASK_DELAY_MS = 10;

// --- IBUS Task Configuration ---
#define IBUS_TASK_STACK_SIZE 2048
#define IBUS_TASK_PRIORITY 5
#define IBUS_TASK_CORE_ID 1
#define IBUS_TASK_DELAY_MS 10
#define IBUS_SERIAL_BAUDRATE 115200

// --- Motor Task Configuration ---
#define MOTOR_TASK_STACK_SIZE 2048
#define MOTOR_TASK_PRIORITY 5
#define MOTOR_TASK_CORE_ID 1
#define MOTOR_TASK_DELAY_MS 10

// --- DShot Configuration ---
#define DSHOT_PROTOCOL DSHOT300 // DSHOT150, DSHOT300, DSHOT600, DSHOT1200
#define MOTOR_PIN_1 27
#define MOTOR_PIN_2 25
#define MOTOR_PIN_3 26
#define MOTOR_PIN_4 33
