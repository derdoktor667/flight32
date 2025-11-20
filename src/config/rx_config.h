/**
 * @file rx_config.h
 * @brief Configuration settings for RC receiver protocols (IBUS, PPM) and channel mappings.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <cstdint>

// --- IBUS Task Configuration ---
inline HardwareSerial &IBUS_SERIAL_PORT = Serial2;
constexpr uint32_t RX_TASK_STACK_SIZE = 4096;
constexpr uint8_t RX_TASK_PRIORITY = 4;
constexpr int8_t RX_TASK_CORE = 1;
constexpr uint8_t RX_TASK_DELAY_MS = 20;
constexpr uint8_t IBUS_UART_NUM = 2;
constexpr uint8_t IBUS_RX_PIN = 16;
constexpr uint8_t IBUS_TX_PIN = 17;
constexpr uint32_t IBUS_BAUD_RATE = 115200;

// --- Generic RX Pin Configuration ---
constexpr uint8_t DEFAULT_RX_PIN = 16; // Example default pin for generic RX input
constexpr const char *KEY_RX_PIN = "rx.pin";

// --- RX Protocol Configuration ---
enum class RcProtocolType : uint8_t
{
    IBUS = 0,
    PPM = 1,
    NONE = 255 // Represents no protocol selected or an error state
};

constexpr const char *KEY_RC_PROTOCOL_TYPE = "rc.protocol";
constexpr RcProtocolType DEFAULT_RC_PROTOCOL_TYPE = RcProtocolType::IBUS;

// --- PPM Protocol Configuration ---
constexpr uint8_t PPM_MAX_CHANNELS = 18;
constexpr uint32_t PPM_FRAME_LENGTH_MIN = 18000;     // microseconds (18ms)
constexpr uint8_t PPM_CONNECTION_TIMEOUT_FACTOR = 2; // Multiplier for PPM_FRAME_LENGTH_MIN
constexpr uint16_t PPM_MIN_PULSE_WIDTH = 1000;       // microseconds
constexpr uint16_t PPM_MAX_PULSE_WIDTH = 2000;       // microseconds

// --- RC Channel Mapping Keys ---
constexpr const char *KEY_RC_CHANNEL_ROLL = "rc.ch.roll";
constexpr const char *KEY_RC_CHANNEL_PITCH = "rc.ch.pitch";
constexpr const char *KEY_RC_CHANNEL_THRO = "rc.ch.thro";
constexpr const char *KEY_RC_CHANNEL_YAW = "rc.ch.yaw";
constexpr const char *KEY_RC_CHANNEL_ARM = "rc.ch.arm";
constexpr const char *KEY_RC_CHANNEL_FMODE = "rc.ch.fmode";
constexpr const char *KEY_RC_CHANNEL_AUX1 = "rc.ch.aux1";
constexpr const char *KEY_RC_CHANNEL_AUX2 = "rc.ch.aux2";
constexpr const char *KEY_RC_CHANNEL_AUX3 = "rc.ch.aux3";
constexpr const char *KEY_RC_CHANNEL_AUX4 = "rc.ch.aux4";

// --- Generic RC Channel Constants ---
constexpr int16_t RC_CHANNEL_MIN_RAW = 1000;
constexpr int16_t RC_CHANNEL_MAX_RAW = 2000;
constexpr int16_t RC_CHANNEL_CENTER = (RC_CHANNEL_MIN_RAW + RC_CHANNEL_MAX_RAW) / 2;
constexpr float RC_CHANNEL_RANGE_SYMMETRIC = (float)(RC_CHANNEL_MAX_RAW - RC_CHANNEL_MIN_RAW) / 2.0f;
constexpr float RC_CHANNEL_RANGE_THROTTLE = (float)(RC_CHANNEL_MAX_RAW - RC_CHANNEL_MIN_RAW);
constexpr int16_t RC_CHANNEL_INDEX_OFFSET = 1;
constexpr int16_t INVALID_CHANNEL_VALUE = 0;

// --- RC Arming Threshold ---
constexpr int16_t RC_CHANNEL_ARM_THRESHOLD = 1800;
constexpr int16_t RC_CHANNEL_FMODE_STABILIZED_THRESHOLD = 1800;

// --- Default RC Channel Mappings (0-based index) ---
constexpr uint8_t DEFAULT_RC_CHANNEL_ROLL = 1;
constexpr uint8_t DEFAULT_RC_CHANNEL_PITCH = 0;
constexpr uint8_t DEFAULT_RC_CHANNEL_THRO = 2;
constexpr uint8_t DEFAULT_RC_CHANNEL_YAW = 3;
constexpr uint8_t DEFAULT_RC_CHANNEL_ARM = 4;
constexpr uint8_t DEFAULT_RC_CHANNEL_FMODE = 5;
constexpr uint8_t DEFAULT_RC_CHANNEL_AUX1 = 6;
constexpr uint8_t DEFAULT_RC_CHANNEL_AUX2 = 7;
constexpr uint8_t DEFAULT_RC_CHANNEL_AUX3 = 8;
constexpr uint8_t DEFAULT_RC_CHANNEL_AUX4 = 9;