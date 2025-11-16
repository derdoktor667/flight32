/**
 * @file serial_config.h
 * @brief Configuration settings for serial communication parameters.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- Serial Configuration ---
constexpr auto SERIAL_BAUD_RATE = 115200;

// --- MSP FC Variant ---
constexpr char MSP_FC_VARIANT_NAME[] = "FL32";

// --- MSP Board Info ---
constexpr char MSP_BOARD_NAME[] = "Flight32";
constexpr char MSP_MANUFACTURER_ID[] = "FL32";
constexpr char MSP_BOARD_TYPE[] = "FC";
constexpr char MSP_HARDWARE_REVISION[] = "1.0";