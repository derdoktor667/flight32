/**
 * @file pid_config.h
 * @brief Configuration settings for the PID controller, including task parameters and default gains.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

namespace PidConfig
{
    // --- PID Task Configuration ---
    constexpr uint32_t TASK_STACK_SIZE = 4096;
    constexpr uint8_t TASK_PRIORITY = 4;
    constexpr int8_t TASK_CORE = 1;
    constexpr uint8_t TASK_DELAY_MS = 10;
    constexpr float MAX_ANGLE_DEGREES = 30.0f; // Maximum angle in degrees for stabilized mode

    // PID scaling: 1/100 for precision
    constexpr float SCALE_FACTOR = 100.0f; // 1 unit = 0.01 gain
    constexpr float ANGLE_D_GAIN_DISABLED = 0.0f;

    constexpr float DEFAULT_RATE_P = 0.2f;
    constexpr float DEFAULT_RATE_I = 0.02f;
    constexpr float DEFAULT_RATE_D = 0.005f;

    constexpr float DEFAULT_YAW_P = 0.4f;
    constexpr float DEFAULT_YAW_I = 0.04f;
    constexpr float DEFAULT_YAW_D = 0.01f;

    constexpr float DEFAULT_ANGLE_P = 0.5f;
    constexpr float DEFAULT_ANGLE_I = 0.0f;
} // namespace PidConfig

// --- PID Gain Settings ---
constexpr const char *NVS_KEY_PID_R_P = "pid.r.p";
constexpr const char *NVS_KEY_PID_R_I = "pid.r.i";
constexpr const char *NVS_KEY_PID_R_D = "pid.r.d";
constexpr const char *NVS_KEY_PID_P_P = "pid.p.p";
constexpr const char *NVS_KEY_PID_P_I = "pid.p.i";
constexpr const char *NVS_KEY_PID_P_D = "pid.p.d";
constexpr const char *NVS_KEY_PID_Y_P = "pid.y.p";
constexpr const char *NVS_KEY_PID_Y_I = "pid.y.i";
constexpr const char *NVS_KEY_PID_Y_D = "pid.y.d";

constexpr const char *NVS_KEY_PID_AR_P = "pid.ar.p";
constexpr const char *NVS_KEY_PID_AR_I = "pid.ar.i";
constexpr const char *NVS_KEY_PID_AP_P = "pid.ap.p";
constexpr const char *NVS_KEY_PID_AP_I = "pid.ap.i";
