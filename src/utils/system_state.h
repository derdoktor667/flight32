/**
 * @file system_state.h
 * @brief Defines the core system states for the flight controller.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <stdint.h>

// Defines the main states of the flight controller.
enum class SystemState : uint8_t
{
    INITIALIZING, // The system is in the process of starting up.
    CALIBRATING,  // The system is performing sensor calibration.
    READY,        // All systems are initialized and ready to be armed.
    ARMED,        // The motors are armed and ready for throttle input.
    IN_FLIGHT,    // The drone is in the air.
    FAILSAFE,     // A critical error has occurred, and the system has entered a safe mode.
    ERROR         // A non-recoverable error has occurred, execution is halted.
};
