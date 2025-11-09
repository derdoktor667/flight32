/**
 * @file rc_config.h
 * @brief Configuration for RC channel mappings and interpretation.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- IBUS Channel Mapping ---

// --- RC Channel Interpretation ---
static constexpr float RC_CHANNEL_CENTER = 1500.0f;
static constexpr float RC_CHANNEL_RANGE_SYMMETRIC = 500.0f; // For Roll, Pitch, Yaw (1500 +/- 500)
static constexpr float RC_CHANNEL_MIN = 1000.0f;
static constexpr float RC_CHANNEL_RANGE_THROTTLE = 1000.0f; // For Throttle (1000 to 2000)
