#pragma once

#include <cstdint>

// --- MSP Protocol Constants ---
static constexpr uint8_t MSP_PROTOCOL_VERSION = 0;
static constexpr uint8_t MSP_API_VERSION_MAJOR = 1;
static constexpr uint8_t MSP_API_VERSION_MINOR = 0;

// --- MSP Constants ---
static constexpr uint8_t MSP_MAX_PAYLOAD_SIZE = 128;
static constexpr uint8_t MSP_PID_PAYLOAD_SIZE = 9;

// --- MSP Command IDs ---
static constexpr uint8_t MSP_API_VERSION = 1;
static constexpr uint8_t MSP_FC_VARIANT = 2;
static constexpr uint8_t MSP_FC_VERSION = 3;
static constexpr uint8_t MSP_BOARD_INFO = 4;
static constexpr uint8_t MSP_BUILD_INFO = 5;
static constexpr uint8_t MSP_REBOOT = 6;
static constexpr uint8_t MSP_STATUS = 7;
static constexpr uint8_t MSP_MEM_STATS = 8;
static constexpr uint8_t MSP_GET_SETTING = 9;
static constexpr uint8_t MSP_SET_SETTING = 10;
static constexpr uint8_t MSP_PID = 11;
static constexpr uint8_t MSP_RAW_IMU = 102;
static constexpr uint8_t MSP_MOTOR = 104;
static constexpr uint8_t MSP_RC = 105;
static constexpr uint8_t MSP_ATTITUDE = 108;
static constexpr uint8_t MSP_EEPROM_WRITE = 200;
static constexpr uint8_t MSP_RESET_SETTINGS = 201;
static constexpr uint8_t MSP_SET_PID = 202;

// --- Flight Controller Version Constants ---
static constexpr uint8_t FC_VERSION_MAJOR = 0;
static constexpr uint8_t FC_VERSION_MINOR = 3;
static constexpr uint8_t FC_VERSION_PATCH = 0;
