/**
 * @file serial_4way_protocol.h
 * @brief Defines constants for the serial 4-way ESC passthrough protocol.
 * @author Based on Betaflight's serial_4way.c
 */

#pragma once

#include <stdint.h>

// Send Structure
// ESC + CMD + PARAM_LEN + [PARAM (if len > 0)] + CRC16_Hi + CRC16_Lo
// Return
// ESC + CMD + PARAM_LEN + [PARAM (if len > 0)] + ACK (uint8_t OK or ERR) + CRC16_Hi + CRC16_Lo

constexpr uint8_t cmd_Remote_Escape = 0x2E; // '.'
constexpr uint8_t cmd_Local_Escape  = 0x2F; // '/'

// Commands
constexpr uint8_t cmd_InterfaceTestAlive = 0x30; // '0' alive
constexpr uint8_t cmd_ProtocolGetVersion = 0x31; // '1' version
constexpr uint8_t cmd_InterfaceGetName = 0x32;   // '2' name
constexpr uint8_t cmd_InterfaceGetVersion = 0x33; // '3' version
constexpr uint8_t cmd_InterfaceExit = 0x34;       // '4' exit
constexpr uint8_t cmd_DeviceReset = 0x35;         // '5' reset
constexpr uint8_t cmd_DeviceInitFlash = 0x37;     // '7' init flash access
constexpr uint8_t cmd_DeviceEraseAll = 0x38;      // '8' erase all
constexpr uint8_t cmd_DevicePageErase = 0x39;     // '9' page erase
constexpr uint8_t cmd_DeviceRead = 0x3A;          // ':' read Device
constexpr uint8_t cmd_DeviceWrite = 0x3B;         // ';' write
constexpr uint8_t cmd_DeviceC2CK_LOW = 0x3C;      // '<'
constexpr uint8_t cmd_DeviceReadEEprom = 0x3D;    // '=' read Device EEPROM
constexpr uint8_t cmd_DeviceWriteEEprom = 0x3E;   // '>' write Device EEPROM
constexpr uint8_t cmd_InterfaceSetMode = 0x3F;    // '?' set interface mode
constexpr uint8_t cmd_DeviceVerify = 0x40;        // '@' verify

// BLHeli_S Bootloader Commands (from BLHeliBootLoad.inc)
constexpr uint8_t READ_VERSION = 0x02;
constexpr uint8_t ERASE_CODE_PAGE = 0x04;
constexpr uint8_t WRITE_CODE_WORD = 0x06;
constexpr uint8_t GET_WORD = 0x0A;
constexpr uint8_t GET_ID = 0x0B;
constexpr uint8_t GO = 0x0C;
constexpr uint8_t WRITE_DATA_BLOCK = 0x0D;
constexpr uint8_t READ_DATA_BLOCK = 0x0E;
constexpr uint8_t GET_CHIP_ID = 0x0F;

// Responses
constexpr uint8_t ACK_OK = 0x00;
constexpr uint8_t ACK = 0x01; // Success acknowledgment (used by BLHeli bootloader)
constexpr uint8_t NACK = 0x00; // Failure acknowledgment (used by BLHeli bootloader)
constexpr uint8_t ACK_I_INVALID_CMD = 0x02;
constexpr uint8_t ACK_I_INVALID_CRC = 0x03;
constexpr uint8_t ACK_I_VERIFY_ERROR = 0x04;
constexpr uint8_t ACK_I_INVALID_CHANNEL = 0x08;
constexpr uint8_t ACK_I_INVALID_PARAM = 0x09;
constexpr uint8_t ACK_D_GENERAL_ERROR = 0x0F;

constexpr uint8_t START_BYTE = 0x08;

// Interface Modes
constexpr uint8_t imC2 = 0;
constexpr uint8_t imSIL_BLB = 1;
constexpr uint8_t imATM_BLB = 2;
constexpr uint8_t imSK = 3;
constexpr uint8_t imARM_BLB = 4; // Not in the original file, but used in switch cases.
