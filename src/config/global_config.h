/**
 * @file global_config.h
 * @brief Central configuration file for the Flight32 firmware.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>
#include <DShotRMT.h> // Keep this for now, might be moved later

#include "../utils/version_info.h"
#include "scheduler_config.h"
#include "terminal_config.h"
#include "serial_config.h"
#include "com_manager_config.h"
#include "imu_config.h"
#include "motor_config.h"
#include "pid_config.h"
#include "rx_config.h"
#include "settings_config.h"

static constexpr float RADIANS_TO_DEGREES = 180.0f / M_PI;

#include "../utils/task_names.h"
