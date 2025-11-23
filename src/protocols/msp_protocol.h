/**
 * @file msp_protocol.h
 * @brief Defines constants and command IDs for the MultiWii Serial Protocol (MSP).
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cstdint>

// --- MSP Protocol Constants ---
static constexpr uint8_t MSP_SYNC_BYTE = '$';
static constexpr uint8_t MSP_V1_IDENTIFIER = 'M';
static constexpr uint8_t MSP_V2_IDENTIFIER = 'X';
static constexpr uint8_t MSP_DIRECTION_HOST = '<'; // Host to FC
static constexpr uint8_t MSP_DIRECTION_FC = '>';   // FC to Host

static constexpr uint8_t MSP_PROTOCOL_VERSION = 0;        // Changed to 0 for Betaflight compatibility
static constexpr uint8_t MSP_API_VERSION_MAJOR = 1;       // From Betaflight example
static constexpr uint8_t MSP_API_VERSION_MINOR = 43;      // From Betaflight example
static constexpr uint8_t MSP_CAPABILITY = 1;              // From Betaflight example
static constexpr char MSP_FC_IDENTIFIER = 'F';            // 'F' for Flight32
static constexpr uint8_t MSP_RESPONSE_OVERHEAD_BYTES = 2; // 1 byte for payload size, 1 byte for command ID
static constexpr uint8_t MSP_MIN_PAYLOAD_SIZE = 3;        // Minimum payload size for MSP commands (e.g., API_VERSION)

// --- MSP Constants ---
static constexpr uint8_t MSP_MAX_PAYLOAD_SIZE = 128;
static constexpr uint8_t MSP_PID_PAYLOAD_SIZE = 18;   // 9x PID gains (uint16_t = 2 bytes each)

// --- MSP Payload Sizes ---
static constexpr uint8_t MSP_API_VERSION_PAYLOAD_SIZE = 3;
static constexpr uint8_t MSP_FC_VARIANT_PAYLOAD_SIZE = 0;
static constexpr uint8_t MSP_FC_VERSION_PAYLOAD_SIZE = 3;
static constexpr uint8_t MSP_MEM_STATS_PAYLOAD_SIZE = 4;
static constexpr uint8_t MSP_SENSOR_STATUS_PAYLOAD_SIZE = 3;
static constexpr uint8_t MSP_RAW_IMU_PAYLOAD_SIZE = 18; // 3x Accel, 3x Gyro, 3x Mag (int16_t = 2 bytes each)
static constexpr uint8_t MSP_ATTITUDE_PAYLOAD_SIZE = 6; // 3x angles (int16_t = 2 bytes each)
static constexpr uint8_t MSP_RC_PAYLOAD_SIZE = 36;      // 18x RC channels (int16_t = 2 bytes each)
static constexpr uint8_t MSP_RX_CONFIG_PAYLOAD_SIZE = 20; // Example size, adjust as needed
static constexpr uint8_t MSP_MOTOR_PAYLOAD_SIZE = 16;   // 8x motor outputs (int16_t = 2 bytes each)
static constexpr uint8_t MSP_MAX_MOTORS = 8;            // Max number of motors expected by MSP_MOTOR command
static constexpr uint16_t MSP_RC_NEUTRAL_CHANNEL_VALUE = 1500;
static constexpr uint8_t MSP_BOXNAMES_PAYLOAD_SIZE = 0;
static constexpr uint8_t MSP_MODE_RANGES_PAYLOAD_SIZE = 20;   // 5 mode ranges * 4 bytes each
static constexpr uint8_t MSP_MOTOR_CONFIG_PAYLOAD_SIZE = 6;   // minthrottle (U16) + maxthrottle (U16) + mincommand (U16) - Python script only expects 6 bytes
static constexpr uint8_t MSP_FILTER_CONFIG_PAYLOAD_SIZE = 20; // 5 floats * 4 bytes each
static constexpr uint8_t MSP_BOX_PAYLOAD_SIZE = 4;            // 4-byte bitmask for active modes
static constexpr uint8_t MSP_UID_PAYLOAD_SIZE = 12;           // 3x uint32_t for unique ID
static constexpr uint8_t MSP_STATUS_PAYLOAD_SIZE = 11;        // cycletime, i2c_errors, sensors, flightmode, profile
static constexpr uint8_t MSP_IDENT_PAYLOAD_SIZE = 7;          // version, fc_type, fc_version[3], board_type, capabilities

// Additional MSP Payload and Buffer Sizes
static constexpr uint8_t MSP_BOARD_INFO_PAYLOAD_SIZE = 12; // FLT3 (4 bytes) + HW_REV (2 bytes) + BOARD_TYPE (1 byte) + TARGET_CAP (4 bytes) + Board Name (null terminator)
static constexpr uint8_t MSP_BUILD_INFO_BUFFER_SIZE = 27;  // 26 characters + null terminator for build info string
static constexpr uint8_t MSP_RC_CHANNEL_COUNT = 18;        // Total number of RC channels supported by MSP
static constexpr uint8_t MSP_RAW_IMU_MAG_BYTES = 6;        // Number of bytes for magnetometer data (3x int16_t)

// MSP Capability Flags (for MSP_IDENT)
static constexpr uint8_t MSP_CAPABILITY_ACC_BIT = (1 << 0);
static constexpr uint8_t MSP_CAPABILITY_GYRO_BIT = (1 << 1); // Add Gyro capability
static constexpr uint8_t MSP_CAPABILITY_MAG_BIT = (1 << 2);
static constexpr uint8_t MSP_CAPABILITY_GPS_BIT = (1 << 3);
static constexpr uint8_t MSP_CAPABILITY_SONAR_BIT = (1 << 4);
static constexpr uint8_t MSP_CAPABILITY_ARM_BIT = (1 << 5);
static constexpr uint8_t MSP_CAPABILITY_NAVC_BIT = (1 << 6);
static constexpr uint8_t MSP_CAPABILITY_LED_STRIP_BIT = (1 << 7); // For Flight32, this is effectively our capabilities mask

// MSP FC and Board Types for MSP_IDENT
static constexpr uint8_t MSP_FC_TYPE_FLIGHT32 = 2; // Mimicking a common Betaflight/Multiwii type
static constexpr uint8_t MSP_BOARD_TYPE_FLIGHT32 = 7; // Using 7 as a generic board type (e.g., from older MultiWii/Cleanflight)

// MSP Status Flags
static constexpr uint8_t MSP_STATUS_ACCEL_SENSOR_FLAG = (1 << 0); // Bitmask for Accelerometer present
static constexpr uint8_t MSP_STATUS_GYRO_SENSOR_FLAG = (1 << 1);  // Bitmask for Gyroscope present
static constexpr uint8_t MSP_BRAKING_ACTION_NONE = 0; // Value for no braking action in MSP_STATUS payload


// MSP Board Info Payload Indices
static constexpr uint8_t BOARD_INFO_HW_REV_BYTE_INDEX = 4;
static constexpr uint8_t BOARD_INFO_BOARD_TYPE_BYTE_INDEX = 6;
static constexpr uint8_t BOARD_INFO_TARGET_CAP_START_INDEX = 7;
static constexpr uint8_t BOARD_INFO_BOARD_NAME_NULL_TERMINATOR_INDEX = 11;

// --- Betaflight Permanent IDs for Modes ---
static constexpr uint8_t BF_PERMANENT_ID_ANGLE = 1;
static constexpr uint8_t BF_PERMANENT_ID_ARM = 0;                 // From Betaflight msp_box.c
static constexpr uint8_t BF_PERMANENT_ID_HORIZON = 2;
static constexpr uint8_t BF_PERMANENT_ID_HEADFREE = 3;
static constexpr uint8_t BF_PERMANENT_ID_HEADADJ = 4;
static constexpr uint8_t BF_PERMANENT_ID_BEEPER = 8;
static constexpr uint8_t BF_PERMANENT_ID_OSDDISABLESW = 15;
static constexpr uint8_t BF_PERMANENT_ID_BLACKBOX = 18;
static constexpr uint8_t BF_PERMANENT_ID_TELEMETRY = 20;          // From Betaflight msp_box.c
static constexpr uint8_t BF_PERMANENT_ID_FPVANGLEMIX = 24;
static constexpr uint8_t BF_PERMANENT_ID_BLACKBOXERASE = 25;
static constexpr uint8_t BF_PERMANENT_ID_FAILSAFE = 27;           // From Betaflight msp_box.c
static constexpr uint8_t BF_PERMANENT_ID_FLIPOVERAFTERCRASH = 35; // From Betaflight msp_box.c
static constexpr uint8_t BF_PERMANENT_ID_LAUNCHCONTROL = 37;
static constexpr uint8_t BF_PERMANENT_ID_PREARM = 13;
static constexpr uint8_t BF_PERMANENT_ID_PARALYZE = 14;
static constexpr uint8_t BF_PERMANENT_ID_ACRO = 53;               // Placeholder for Acro, as Betaflight doesn't have explicit Acro box

// --- MSP Mode Range Constants ---
static constexpr uint8_t MSP_MODE_RANGE_ARM_AUX_CHANNEL = 1;
static constexpr uint8_t MSP_MODE_RANGE_ARM_START_STEP = 5;  // Corresponds to ~1020 PWM
static constexpr uint8_t MSP_MODE_RANGE_ARM_END_STEP = 15;   // Corresponds to ~1060 PWM

static constexpr uint8_t MSP_MODE_RANGE_ANGLE_AUX_CHANNEL = 1; // Often same channel as ARM for mode switching
static constexpr uint8_t MSP_MODE_RANGE_ANGLE_START_STEP = 5; // Corresponds to ~1020 PWM
static constexpr uint8_t MSP_MODE_RANGE_ANGLE_END_STEP = 15;  // Corresponds to ~1060 PWM

static constexpr uint8_t MSP_MODE_RANGE_FAILSAFE_AUX_CHANNEL = 7;
static constexpr uint8_t MSP_MODE_RANGE_FAILSAFE_START_STEP = 33; // Corresponds to ~1130 PWM
static constexpr uint8_t MSP_MODE_RANGE_FAILSAFE_END_STEP = 48;   // Corresponds to ~1190 PWM

static constexpr uint8_t MSP_MODE_RANGE_TELEMETRY_AUX_CHANNEL = 26;
static constexpr uint8_t MSP_MODE_RANGE_TELEMETRY_START_STEP = 77; // Corresponds to ~1300 PWM
static constexpr uint8_t MSP_MODE_RANGE_TELEMETRY_END_STEP = 79;   // Corresponds to ~1310 PWM

static constexpr uint8_t MSP_MODE_RANGE_FLIPOVERAFTERCRASH_AUX_CHANNEL = 36;
static constexpr uint8_t MSP_MODE_RANGE_FLIPOVERAFTERCRASH_START_STEP = 115; // Corresponds to ~1450 PWM
static constexpr uint8_t MSP_MODE_RANGE_FLIPOVERAFTERCRASH_END_STEP = 125;   // Corresponds to ~1490 PWM

// --- MSP Flight Mode IDs ---
static constexpr uint8_t MSP_BOX_ACRO_ID = 0;
static constexpr uint8_t MSP_BOX_STABILIZED_ID = 1;

// --- MSP Scaling Factors ---
static constexpr float MSP_ATTITUDE_SCALE_FACTOR = 10.0f; // Roll/Pitch scaled by 10

// --- MSP Setting Payload Overheads ---
static constexpr uint8_t MSP_SETTING_KEY_VALUE_OVERHEAD_BYTES = 2; // 1 byte for key_len, 1 byte for value_len

// --- MSP Command IDs ---
static constexpr uint8_t MSP_API_VERSION = 1;
static constexpr uint8_t MSP_FC_VARIANT = 2;
static constexpr uint8_t MSP_FC_VERSION = 3;
static constexpr uint8_t MSP_BOARD_INFO = 4;
static constexpr uint8_t MSP_BUILD_INFO = 5;
static constexpr uint8_t MSP_REBOOT = 6;
static constexpr uint8_t MSP_GET_SETTING = 9;
// MSP_SET_SETTING is 10 in the old file, but MSP_NAME is 10 in the tester.
// Let's assume the tester is the source of truth for compatibility.
static constexpr uint8_t MSP_NAME = 10;
static constexpr uint8_t MSP_SET_SETTING = 254; // Common value for this in BF

static constexpr uint8_t MSP_MODE_RANGES = 34;
static constexpr uint8_t MSP_FEATURE_CONFIG = 36;
static constexpr uint8_t MSP_RX_CONFIG = 44;
static constexpr uint8_t MSP_ARMING_CONFIG = 61;
static constexpr uint8_t MSP_RX_MAP = 64;
static constexpr uint8_t MSP_FAILSAFE_CONFIG = 75;
static constexpr uint8_t MSP_OSD_CONFIG = 84;
static constexpr uint8_t MSP_VTX_CONFIG = 88;
static constexpr uint8_t MSP_ADVANCED_CONFIG = 90;
static constexpr uint8_t MSP_FILTER_CONFIG = 92;

static constexpr uint8_t MSP_IDENT = 100;
static constexpr uint8_t MSP_STATUS = 101;
// MSP_PID is 102 in old file, but 112 in tester
static constexpr uint8_t MSP_BOX = 113; // A common ID for MSP_BOX
static constexpr uint8_t MSP_MOTOR = 104;
static constexpr uint8_t MSP_RC = 105;
static constexpr uint8_t MSP_RAW_IMU = 106;
static constexpr uint8_t MSP_ATTITUDE = 108;
static constexpr uint8_t MSP_ALTITUDE = 109;
static constexpr uint8_t MSP_ANALOG = 110;
static constexpr uint8_t MSP_RC_TUNING = 111;
static constexpr uint8_t MSP_PID = 112;

static constexpr uint8_t MSP_BOXNAMES = 116;
static constexpr uint8_t MSP_PIDNAMES = 117;
static constexpr uint8_t MSP_BOXIDS = 119;

static constexpr uint8_t MSP_MOTOR_CONFIG = 131;
static constexpr uint8_t MSP_BATTERY_STATE = 130;
static constexpr uint8_t MSP_STATUS_EX = 150;
static constexpr uint8_t MSP_UID = 160;

static constexpr uint8_t MSP_EEPROM_WRITE = 250; // common value for this in BF

// These were legacy values, replacing with more modern ones
static constexpr uint8_t MSP_RESET_SETTINGS = 201; // This might need verification
static constexpr uint8_t MSP_SET_PID = 202;
static constexpr uint8_t MSP_GET_FILTER_CONFIG = 203;
static constexpr uint8_t MSP_SET_FILTER_CONFIG = 204;
static constexpr uint8_t MSP_SET_BOX = 205;
static constexpr uint8_t MSP_MEM_STATS = 208;
static constexpr uint8_t MSP_SENSOR_STATUS = 209;

