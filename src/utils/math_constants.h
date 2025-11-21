/**
 * @file math_constants.h
 * @brief Centralized header for generic mathematical constants and unit conversion factors.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cmath>
#include <math.h>

// Conversion constants
constexpr float RADIANS_TO_DEGREES = 180.0f / M_PI;

// Time-related constants
constexpr float MS_PER_SECOND = 1000.0f;
constexpr uint64_t ONE_SECOND_MICROSECONDS = 1000000ULL;

constexpr float MS_TO_SECONDS_FACTOR = 0.001f;

// Angle-related constants
constexpr float HALF_PI_RADIANS = M_PI / 2.0f;

// Filter Calculation Constants
constexpr float BUTTERWORTH_Q_FACTOR = 0.7071f;
constexpr float BIQUAD_ALPHA_DENOMINATOR_FACTOR = 2.0f;

// CPU usage constants
constexpr float CPU_PERCENTAGE_FACTOR = 100.0f;
