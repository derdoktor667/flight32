/**
 * @file pid_config.h
 * @brief Configuration settings for the PID controller, including task parameters and default gains.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- PID Task Configuration ---
constexpr uint32_t PID_TASK_STACK_SIZE = 4096;
constexpr uint8_t PID_TASK_PRIORITY = 4;
constexpr int8_t PID_TASK_CORE = 1;
constexpr uint8_t PID_TASK_DELAY_MS = 10;

// --- PID Gain Settings ---
constexpr const char *KEY_PID_ROLL_P = "pid.roll.p";
constexpr const char *KEY_PID_ROLL_I = "pid.roll.i";
constexpr const char *KEY_PID_ROLL_D = "pid.roll.d";
constexpr const char *KEY_PID_PITCH_P = "pid.pitch.p";
constexpr const char *KEY_PID_PITCH_I = "pid.pitch.i";
constexpr const char *KEY_PID_PITCH_D = "pid.pitch.d";
constexpr const char *KEY_PID_YAW_P = "pid.yaw.p";
constexpr const char *KEY_PID_YAW_I = "pid.yaw.i";
constexpr const char *KEY_PID_YAW_D = "pid.yaw.d";

// PID scaling: 1/100 for precision
constexpr float PID_SCALE_FACTOR = 100.0f; // 1 unit = 0.01 gain

constexpr float DEFAULT_PID_ROLL_P = 0.2f;
constexpr float DEFAULT_PID_ROLL_I = 0.02f;
constexpr float DEFAULT_PID_ROLL_D = 0.005f;
constexpr float DEFAULT_PID_PITCH_P = 0.2f;
constexpr float DEFAULT_PID_PITCH_I = 0.02f;
constexpr float DEFAULT_PID_PITCH_D = 0.005f;
constexpr float DEFAULT_PID_YAW_P = 0.4f;
constexpr float DEFAULT_PID_YAW_I = 0.04f;
constexpr float DEFAULT_PID_YAW_D = 0.01f;