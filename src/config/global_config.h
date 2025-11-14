/**
 * @file global_config.h
 * @brief Central configuration file for the Flight32 firmware.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>
#include <DShotRMT.h> // Keep this for now, might be moved later

#include "version_info.h"
#include "scheduler/scheduler_config.h"
#include "terminal/terminal_config.h"
#include "serial_config.h"
#include "com_manager_config.h"
#include "imu_config.h" // New include for IMU config

#include "imu_config.h" // New include for IMU config
#include "motor_config.h"
#include "pid/pid_config.h"
#include "rx_config.h"

#include "rx_config.h"
#include "settings_config.h"

// --- Time Constants ---
static constexpr float MS_TO_SECONDS_FACTOR = 1000.0f;

#include "settings_config.h"
#include "task_names.h"

// --- Time Constants ---
