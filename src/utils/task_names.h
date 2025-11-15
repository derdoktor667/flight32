/**
 * @file task_names.h
 * @brief Defines constant strings for FreeRTOS task names for identification and debugging.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- Task Names ---
constexpr const char *COM_TASK_NAME = "com_task";
constexpr const char *IMU_TASK_NAME = "IMU / Sensor";
constexpr const char *RX_TASK_NAME = "RX / Receiver";
constexpr const char *IBUS_TASK_NAME = "RX / IBUS";
constexpr const char *MOTOR_TASK_NAME = "MOTORS / DShot";
constexpr const char *PID_TASK_NAME = "PID Controller";
constexpr const char *SERIAL_MANAGER_TASK_NAME = "Serial Manager";
constexpr const char *IDLE_TASK_NAME_0 = "IDLE0";
constexpr const char *IDLE_TASK_NAME_1 = "IDLE1";
constexpr const char *TIMER_SERVICE_TASK_NAME = "Tmr Svc";
