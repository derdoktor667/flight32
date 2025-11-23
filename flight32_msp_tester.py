#!/usr/bin/env python3

"""
/**
 * @file flight32_msp_tester.py
 * @brief Flight32 MSP Communication Tester.
 * @author Wastl Kraus
 * @license MIT
 */
"""

import argparse
import serial
import struct
import time

# --- ANSI Color Codes ---
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

# --- Icons ---
ICON_SUCCESS = "✅"
ICON_FAILURE = "❌"
ICON_WARNING = "⚠️"
ICON_INFO = "ℹ️"
ICON_COMMAND = "⚙️"

# --- Configuration ---
BAUD_RATE = 115200
TIMEOUT = 2  # seconds for serial read timeout

# --- MSP Command Codes ---
MSP_COMMANDS = {
    # --- System Info ---
    "MSP_API_VERSION": 1,
    "MSP_FC_VARIANT": 2,
    "MSP_FC_VERSION": 3,
    "MSP_BOARD_INFO": 4,
    "MSP_BUILD_INFO": 5,
    "MSP_NAME": 10,
    "MSP_UID": 160,
    # --- Config Dumps ---
    "MSP_FEATURE_CONFIG": 36,
    "MSP_ARMING_CONFIG": 61,
    "MSP_RX_CONFIG": 44,
    "MSP_RX_MAP": 64,
    "MSP_FAILSAFE_CONFIG": 75,
    "MSP_PID": 112,
    "MSP_RC_TUNING": 111,
    "MSP_ADVANCED_CONFIG": 90,
    "MSP_FILTER_CONFIG": 92,
    # --- Live Status & Data ---
    "MSP_STATUS_EX": 150,
    "MSP_ATTITUDE": 108,
    "MSP_ALTITUDE": 109,
    "MSP_ANALOG": 110,
    "MSP_RC": 105,
    "MSP_MOTOR": 104,
    "MSP_MOTOR_CONFIG": 131,
    "MSP_BATTERY_STATE": 130,
    # --- Modes ---
    "MSP_BOXNAMES": 116,
    "MSP_BOXIDS": 119,
    "MSP_MODE_RANGES": 34,
    # --- OSD, VTX, etc. ---
    "MSP_OSD_CONFIG": 84,
    "MSP_VTX_CONFIG": 88,
    "MSP_PIDNAMES": 117,
    # --- Deprecated but for testing ---
    "MSP_IDENT": 100,
}


# --- MSP Protocol Functions ---
def _crc8_dvb_s2(crc, ch):
    """Calculates CRC8 for MSP V2."""
    crc ^= ch
    for _ in range(8):
        if crc & 0x80:
            crc = ((crc << 1) & 0xFF) ^ 0xD5
        else:
            crc = (crc << 1) & 0xFF
    return crc


def serialize_msp_command(command_code, data=b"", protocol_version=1):
    if protocol_version == 1:
        size = len(data)
        checksum = size ^ command_code
        for byte in data:
            checksum ^= byte

        packet = b"$M<"
        packet += struct.pack("<BB", size, command_code) + data
        packet += struct.pack("<B", checksum)
    else:  # MSP V2
        flag = 0  # Currently always 0 for future use
        size = len(data)

        crc = 0
        crc = _crc8_dvb_s2(crc, flag)
        crc = _crc8_dvb_s2(crc, command_code & 0xFF)
        crc = _crc8_dvb_s2(crc, (command_code >> 8) & 0xFF)
        crc = _crc8_dvb_s2(crc, size & 0xFF)
        crc = _crc8_dvb_s2(crc, (size >> 8) & 0xFF)
        for byte in data:
            crc = _crc8_dvb_s2(crc, byte)

        packet = b"$X>"  # MSP V2 header for response
        packet += struct.pack(
            "<BHHH", flag, command_code, size, 0
        )  # Placeholder for payload size high byte in struct, will be overwritten by direct bytes
        packet = packet[:-2]  # Remove placeholder size high byte, command high byte
        packet += struct.pack("<B", flag)
        packet += struct.pack("<H", command_code)  # Command ID (16-bit)
        packet += struct.pack("<H", size)  # Payload Size (16-bit)
        packet += data
        packet += struct.pack("<B", crc)
    return packet


def deserialize_msp_response(response_bytes):
    if not response_bytes or len(response_bytes) < 3:
        return None, None, None, None  # command_code, data, is_error, protocol_version

    header = response_bytes[0:3]
    is_error = False
    protocol_version = 1

    if header == b"$M>":
        protocol_version = 1
        if len(response_bytes) < 6:
            print(
                f"{YELLOW}{ICON_WARNING} Incomplete MSP V1 response header/minimal length.{RESET}"
            )
            return None, None, None, protocol_version

        size = response_bytes[3]
        command_code = response_bytes[4]

        if len(response_bytes) < 6 + size:
            print(f"{YELLOW}{ICON_WARNING} Incomplete MSP V1 response payload.{RESET}")
            return None, None, None, protocol_version

        data = response_bytes[5 : 5 + size]
        received_checksum = response_bytes[5 + size]

        calculated_checksum = size ^ command_code
        for byte in data:
            calculated_checksum ^= byte

        if calculated_checksum != received_checksum:
            print(
                f"{RED}{ICON_FAILURE} Checksum mismatch for V1 command {command_code}. Expected {calculated_checksum}, got {received_checksum}{RESET}"
            )
            return None, None, None, protocol_version

    elif header == b"$X>":
        protocol_version = 2
        # V2 header: $X> (3 bytes), flag (1 byte), command (2 bytes), size (2 bytes)
        if len(response_bytes) < 9:
            print(
                f"{YELLOW}{ICON_WARNING} Incomplete MSP V2 response header/minimal length.{RESET}"
            )
            return None, None, None, protocol_version

        flag = response_bytes[3]
        command_code = struct.unpack("<H", response_bytes[4:6])[0]
        size = struct.unpack("<H", response_bytes[6:8])[0]

        if len(response_bytes) < 9 + size:
            print(f"{YELLOW}{ICON_WARNING} Incomplete MSP V2 response payload.{RESET}")
            return None, None, None, protocol_version

        data = response_bytes[8 : 8 + size]
        received_checksum = response_bytes[8 + size]

        # Calculate V2 CRC
        calculated_crc = 0
        calculated_crc = _crc8_dvb_s2(calculated_crc, flag)
        calculated_crc = _crc8_dvb_s2(calculated_crc, command_code & 0xFF)
        calculated_crc = _crc8_dvb_s2(calculated_crc, (command_code >> 8) & 0xFF)
        calculated_crc = _crc8_dvb_s2(calculated_crc, size & 0xFF)
        calculated_crc = _crc8_dvb_s2(calculated_crc, (size >> 8) & 0xFF)
        for byte in data:
            calculated_crc = _crc8_dvb_s2(calculated_crc, byte)

        if calculated_crc != received_checksum:
            print(
                f"{RED}{ICON_FAILURE} Checksum mismatch for V2 command {command_code}. Expected {calculated_crc}, got {received_checksum}{RESET}"
            )
            return None, None, None, protocol_version

    elif header == b"$M!":  # V1 error
        protocol_version = 1
        is_error = True
        # Parse minimal error response for V1
        if len(response_bytes) < 6:
            print(f"{YELLOW}{ICON_WARNING} Incomplete MSP V1 error response.{RESET}")
            return None, None, is_error, protocol_version

        size = response_bytes[3]
        command_code = response_bytes[4]
        data = response_bytes[
            5 : 5 + size
        ]  # Should be empty for error, but just in case

        # Checksum for error is usually on the header as well
        # For simplicity, we'll just check if it's an error response
        print(
            f"{RED}{ICON_FAILURE} Received V1 Error for command {command_code}.{RESET}"
        )
        return command_code, data, is_error, protocol_version

    elif header == b"$X!":  # V2 error
        protocol_version = 2
        is_error = True
        # Parse minimal error response for V2
        if len(response_bytes) < 9:  # $X! + flag + cmd (2) + size (2) + crc (1)
            print(f"{YELLOW}{ICON_WARNING} Incomplete MSP V2 error response.{RESET}")
            return None, None, is_error, protocol_version

        flag = response_bytes[3]
        command_code = struct.unpack("<H", response_bytes[4:6])[0]
        size = struct.unpack("<H", response_bytes[6:8])[0]  # Should be 0 for error
        data = response_bytes[8 : 8 + size]  # Should be empty for error

        print(
            f"{RED}{ICON_FAILURE} Received V2 Error for command {command_code}.{RESET}"
        )
        return command_code, data, is_error, protocol_version

    else:
        print(f"{YELLOW}{ICON_WARNING} Unexpected MSP response header: {header}{RESET}")
        return None, None, None, None

    return command_code, data, is_error, protocol_version


# --- MSP Response Parsing Helpers ---
def _parse_system_info(command_code, response_data):
    """
    Parses and prints common system information commands (API_VERSION, FC_VARIANT, etc.).
    These commands provide essential identification and version details of the flight controller.
    """
    if not response_data:
        print(
            f"  {YELLOW}{ICON_WARNING} No data received for system info command.{RESET}"
        )
        return

    if command_code == MSP_COMMANDS["MSP_API_VERSION"] and len(response_data) >= 3:
        # MSP_API_VERSION format: protocol version (byte), API major (byte), API minor (byte)
        print(
            f"  {BOLD}MSP Protocol Version:{RESET} {response_data[0]}, API Version: {response_data[1]}.{response_data[2]}"
        )
    elif command_code == MSP_COMMANDS["MSP_FC_VARIANT"]:
        # MSP_FC_VARIANT: typically a 4-char ASCII string representing the firmware variant (e.g., "BTFL", "INAV").
        print(
            f"  {BOLD}FC Variant:{RESET} {response_data.decode('ascii', errors='ignore')}"
        )
    elif command_code == MSP_COMMANDS["MSP_FC_VERSION"] and len(response_data) == 3:
        # MSP_FC_VERSION format: major (byte), minor (byte), patch (byte)
        print(
            f"  {BOLD}FC Version:{RESET} {response_data[0]}.{response_data[1]}.{response_data[2]}"
        )
    elif command_code == MSP_COMMANDS["MSP_BOARD_INFO"] and len(response_data) >= 4:
        # MSP_BOARD_INFO: 4-char board identifier, hardware revision (uint16), optional board name.
        board_identifier = response_data[0:4].decode("ascii", errors="ignore")
        hardware_revision = (
            struct.unpack("<H", response_data[4:6])[0]
            if len(response_data) >= 6
            else "N/A"
        )
        board_name = (
            response_data[7:].decode("ascii", errors="ignore")
            if len(response_data) >= 7 and response_data[6] > 0
            else "N/A"
        )
        print(
            f"  {BOLD}Board Identifier:{RESET} {board_identifier}, {BOLD}HW Rev:{RESET} {hardware_revision}, {BOLD}Name:{RESET} {board_name}"
        )
    elif command_code == MSP_COMMANDS["MSP_BUILD_INFO"]:
        # MSP_BUILD_INFO: build date, time, and short git hash as an ASCII string.
        print(
            f"  {BOLD}Build Info:{RESET} {response_data.decode('ascii', errors='ignore')}"
        )
    elif command_code == MSP_COMMANDS["MSP_NAME"]:
        # MSP_NAME: user-defined name for the flight controller, as an ASCII string.
        print(f"  {BOLD}Name:{RESET} {response_data.decode('ascii', errors='ignore')}")
    elif command_code == MSP_COMMANDS["MSP_IDENT"] and len(response_data) >= 7:
        # MSP_IDENT: Legacy command from MultiWii/Cleanflight, largely superseded by other commands.
        # It typically returns mixerMode + multiwii version + protocol version + capability variable.
        (
            version,
            fc_type,
            fc_version_major,
            fc_version_minor,
            fc_version_patch,
            board_type,
            capabilities,
        ) = struct.unpack("<BBBBBBB", response_data)
        print(
            f"  {BOLD}MSP Version:{RESET} {version}, {BOLD}FC Type:{RESET} {fc_type}, {BOLD}FC Version:{RESET} {fc_version_major}.{fc_version_minor}.{fc_version_patch}, {BOLD}Board Type:{RESET} {board_type}, {BOLD}Capabilities:{RESET} {bin(capabilities)}"
        )
    else:
        # Fallback for unexpected data length or format for system info commands.
        print(
            f"  {YELLOW}{ICON_WARNING} Decoded (unparsed system info):{RESET} {response_data}"
        )


def _parse_attitude(response_data):
    """
    Parses and prints attitude data (Roll, Pitch, Yaw).
    Attitude values are signed 16-bit integers, typically scaled by 10 for Roll/Pitch.
    """
    if response_data and len(response_data) == 6:
        roll, pitch, yaw = struct.unpack(
            "<hhh", response_data
        )  # <hhh means three little-endian signed shorts
        # Roll and Pitch are usually scaled by 10, Yaw is in degrees.
        print(
            f"  {BOLD}Roll:{RESET} {roll/10.0:.2f} deg, {BOLD}Pitch:{RESET} {pitch/10.0:.2f} deg, {BOLD}Yaw:{RESET} {yaw:.2f} deg"
        )
    else:
        # Warn if data is missing or malformed.
        print(
            f"  {YELLOW}{ICON_WARNING} Decoded (unparsed attitude):{RESET} {response_data}"
        )


def _parse_motor_values(response_data):
    """
    Parses and prints motor values (PWM/Dshot).
    Motor values are typically unsigned 16-bit integers (uint16) for each motor.
    """
    if response_data and len(response_data) % 2 == 0:
        motors = struct.unpack(
            "<" + "H" * (len(response_data) // 2), response_data
        )  # Each motor value is uint16
        print(f"  {BOLD}Motor Values (PWM/Dshot):{RESET} {', '.join(map(str, motors))}")
    else:
        # Warn if data is missing or malformed.
        print(
            f"  {YELLOW}{ICON_WARNING} Decoded (unparsed motor data):{RESET} {response_data}"
        )


def _parse_motor_config(response_data):
    """
    Parses and prints motor configuration parameters like min_throttle, max_throttle, and min_command.
    These are typically unsigned 16-bit integers.
    """
    if response_data and len(response_data) >= 6:  # Expecting at least 3 uint16 values
        min_throttle, max_throttle, min_command = struct.unpack(
            "<HHH", response_data[0:6]
        )
        print(
            f"  {BOLD}Min Throttle:{RESET} {min_throttle}, {BOLD}Max Throttle:{RESET} {max_throttle}, {BOLD}Min Command:{RESET} {min_command}"
        )
    else:
        # Warn if data is missing or malformed.
        print(
            f"  {YELLOW}{ICON_WARNING} Decoded (unparsed motor config):{RESET} {response_data}"
        )


def _parse_pid_values(response_data):
    """
    Parses and prints PID values.
    PID values can be raw bytes, depending on the firmware version and specific PID command.
    Printing as raw bytes provides a generic representation.
    """
    if response_data:
        pids = list(
            response_data
        )  # PID values are typically sent as a flat list of bytes.
        print(f"  {BOLD}PID Values (raw bytes):{RESET} {', '.join(map(str, pids))}")
    else:
        # Warn if data is missing or empty.
        print(
            f"  {YELLOW}{ICON_WARNING} Decoded (unparsed PID data):{RESET} {response_data}"
        )


def _parse_names_list(response_data):
    """
    Parses and prints lists of names (e.g., Box Names, PID Names).
    These are typically ASCII strings, semicolon-separated.
    """
    if response_data:
        # Decode as ASCII, strip leading/trailing whitespace and semicolons, then split by semicolon.
        names = (
            response_data.decode("ascii", errors="ignore").strip().strip(";").split(";")
        )
        print(f"  {BOLD}Names:{RESET} {', '.join(names)}")
    else:
        # Warn if no data is received.
        print(f"  {YELLOW}{ICON_WARNING} No names data received.{RESET}")


def _parse_box_ids(response_data):
    """
    Parses and prints Box IDs.
    Box IDs are typically a list of single bytes representing each active mode ID.
    """
    if response_data:
        ids = ", ".join(map(str, response_data))  # Box IDs are a list of bytes
        print(f"  {BOLD}Box IDs:{RESET} {ids}")
    else:
        # Warn if no data is received.
        print(f"  {YELLOW}{ICON_WARNING} No Box IDs data received.{RESET}")


def _parse_mode_ranges(response_data):
    """
    Parses and prints mode range data.
    Each mode range is typically represented by 4 bytes: Mode ID, AUX channel index, Start percent, End percent.
    The Start/End percentages are scaled to PWM values (e.g., 900-2100).
    """
    if response_data and len(response_data) % 4 == 0:
        mode_ranges = []
        for i in range(
            0, len(response_data), 4
        ):  # Iterate over 4-byte chunks for each mode range
            mode_id, aux_channel, start, end = struct.unpack(
                "<BBBB", response_data[i : i + 4]
            )
            # PWM range for mode ranges is typically 900-2100.
            # The start/end values are usually scaled from 0-200 in legacy MSP.
            # (2100 - 900) / 200 = 6. So each step is 6 PWM units.
            start_pwm = 900 + start * 6
            end_pwm = 900 + end * 6
            mode_ranges.append(
                f"ID:{mode_id} Aux:{aux_channel} Range:{start_pwm}-{end_pwm}"
            )
        print(f"  {BOLD}Mode Ranges:{RESET}\n    " + "\n    ".join(mode_ranges))
    else:
        # Warn if the payload length is not a multiple of 4, indicating malformed or unexpected data.
        print(
            f"  {YELLOW}{ICON_WARNING} Malformed MSP_MODE_RANGES payload (expected length multiple of 4):{RESET} {response_data.hex() if response_data else 'None'}"
        )


def _parse_status_ex(response_data):
    """
    Parses and prints MSP_STATUS_EX data.
    Provides detailed flight controller status including cycle time, I2C errors,
    present sensors, active flight modes, and configuration profile.
    """
    if response_data and len(response_data) >= 11:
        # MSP_STATUS_EX (150) structure from Betaflight source (approx. v4.x):
        # cycleTime (uint16), i2cErrors (uint16), sensorsPresentFlags (uint16),
        # flightModeFlags (uint32), configProfileIndex (uint8)
        offset = 0
        cycle_time = struct.unpack("<H", response_data[offset : offset + 2])[0]
        offset += 2
        i2c_errors = struct.unpack("<H", response_data[offset : offset + 2])[0]
        offset += 2
        sensors_present_flags = struct.unpack("<H", response_data[offset : offset + 2])[
            0
        ]
        offset += 2
        flight_mode_flags = struct.unpack("<I", response_data[offset : offset + 4])[0]
        offset += 4
        config_profile_index = response_data[offset]
        offset += 1

        print(f"  {BOLD}Cycle Time:{RESET} {cycle_time} us")
        print(f"  {BOLD}I2C Errors:{RESET} {i2c_errors}")
        print(
            f"  {BOLD}Sensors Present Flags:{RESET} {bin(sensors_present_flags)}"
        )  # Binary representation for flags
        print(f"  {BOLD}Flight Mode Flags:{RESET} {flight_mode_flags} (raw)")
        print(f"  {BOLD}Config Profile Index:{RESET} {config_profile_index}")
    else:
        # Warn if data is missing or malformed.
        print(
            f"  {YELLOW}{ICON_WARNING} Decoded (unparsed MSP_STATUS_EX):{RESET} {response_data.hex() if response_data else 'None'}"
        )


def _parse_battery_state(response_data):
    """
    Parses and prints battery state information.
    The structure for this command can vary significantly between Betaflight versions.
    For robustness, it currently prints the raw hex data with a warning.
    """
    if response_data:
        # The structure of MSP_BATTERY_STATE varies significantly across BF versions (e.g., vbat_cells, vbat_capacity, etc.).
        # Detailed parsing would require version-specific logic. For a general tester, raw hex is informative.
        print(
            f"  {YELLOW}{ICON_WARNING} Raw parsing for MSP_BATTERY_STATE: needs specific struct based on FC version.{RESET}"
        )
        print(f"  {BOLD}Battery Data:{RESET} {response_data.hex()}")
    else:
        # Warn if data is missing.
        print(
            f"  {YELLOW}{ICON_WARNING} No battery data received or malformed:{RESET} {response_data.hex() if response_data else 'None'}"
        )


def _parse_simple_raw_data(command_name, response_data):
    """
    Generic parser for commands that return raw configuration bytes.
    Used for commands where detailed structured parsing is not yet implemented
    or where the data is simply a dump of settings.
    """
    if response_data:
        print(f"  {BOLD}{command_name} Data:{RESET} {response_data.hex()}")
    else:
        print(f"  {YELLOW}{ICON_WARNING} No data received for {command_name}.{RESET}")


# --- Dispatch table for MSP command parsing ---
# Maps MSP command codes to their respective parsing functions for better readability and maintainability.
# This approach replaces a long if/elif chain, making it easier to add or modify parsers.
COMMAND_PARSERS = {
    # System Information Commands (parsed by a single helper function as their logic is similar)
    MSP_COMMANDS["MSP_API_VERSION"]: _parse_system_info,
    MSP_COMMANDS["MSP_FC_VARIANT"]: _parse_system_info,
    MSP_COMMANDS["MSP_FC_VERSION"]: _parse_system_info,
    MSP_COMMANDS["MSP_BOARD_INFO"]: _parse_system_info,
    MSP_COMMANDS["MSP_BUILD_INFO"]: _parse_system_info,
    MSP_COMMANDS["MSP_NAME"]: _parse_system_info,
    MSP_COMMANDS[
        "MSP_IDENT"
    ]: _parse_system_info,  # Deprecated, but supported for older FCs.
    # Detailed Status and Live Data
    MSP_COMMANDS[
        "MSP_STATUS_EX"
    ]: _parse_status_ex,  # Provides extended status information.
    MSP_COMMANDS["MSP_ATTITUDE"]: _parse_attitude,  # Roll, Pitch, Yaw.
    MSP_COMMANDS["MSP_MOTOR"]: _parse_motor_values,  # PWM/Dshot values for motors.
    MSP_COMMANDS[
        "MSP_MOTOR_CONFIG"
    ]: _parse_motor_config,  # Min/Max throttle, Min command.
    MSP_COMMANDS[
        "MSP_PID"
    ]: _parse_pid_values,  # P, I, D gains (raw bytes for flexibility).
    MSP_COMMANDS[
        "MSP_BATTERY_STATE"
    ]: _parse_battery_state,  # Battery voltage, current, etc.
    # Mode Configuration
    MSP_COMMANDS["MSP_BOXNAMES"]: _parse_names_list,  # Names of flight modes/switches.
    MSP_COMMANDS["MSP_PIDNAMES"]: _parse_names_list,  # Names of PID tuning parameters.
    MSP_COMMANDS["MSP_BOXIDS"]: _parse_box_ids,  # IDs of flight modes/switches.
    MSP_COMMANDS[
        "MSP_MODE_RANGES"
    ]: _parse_mode_ranges,  # AUX channel ranges for modes.
    # Generic Raw Data Parsers
    # For commands where specific structured parsing is not yet implemented, or not critical
    # for a basic connection test, the raw hexadecimal data is printed.
    MSP_COMMANDS["MSP_FEATURE_CONFIG"]: lambda d: _parse_simple_raw_data(
        "MSP_FEATURE_CONFIG", d
    ),
    MSP_COMMANDS["MSP_ARMING_CONFIG"]: lambda d: _parse_simple_raw_data(
        "MSP_ARMING_CONFIG", d
    ),
    MSP_COMMANDS["MSP_RX_CONFIG"]: lambda d: _parse_simple_raw_data("MSP_RX_CONFIG", d),
    MSP_COMMANDS["MSP_RX_MAP"]: lambda d: _parse_simple_raw_data("MSP_RX_MAP", d),
    MSP_COMMANDS["MSP_FAILSAFE_CONFIG"]: lambda d: _parse_simple_raw_data(
        "MSP_FAILSAFE_CONFIG", d
    ),
    MSP_COMMANDS["MSP_RC_TUNING"]: lambda d: _parse_simple_raw_data("MSP_RC_TUNING", d),
    MSP_COMMANDS["MSP_ADVANCED_CONFIG"]: lambda d: _parse_simple_raw_data(
        "MSP_ADVANCED_CONFIG", d
    ),
    MSP_COMMANDS["MSP_FILTER_CONFIG"]: lambda d: _parse_simple_raw_data(
        "MSP_FILTER_CONFIG", d
    ),
    MSP_COMMANDS["MSP_OSD_CONFIG"]: lambda d: _parse_simple_raw_data(
        "MSP_OSD_CONFIG", d
    ),
    MSP_COMMANDS["MSP_VTX_CONFIG"]: lambda d: _parse_simple_raw_data(
        "MSP_VTX_CONFIG", d
    ),
    MSP_COMMANDS["MSP_ALTITUDE"]: lambda d: _parse_simple_raw_data("MSP_ALTITUDE", d),
    MSP_COMMANDS["MSP_ANALOG"]: lambda d: _parse_simple_raw_data("MSP_ANALOG", d),
    MSP_COMMANDS["MSP_RC"]: lambda d: _parse_simple_raw_data(
        "MSP_RC", d
    ),  # RC channel values, handled as raw data if no specific parser.
}


def send_and_receive(
    ser, command_name, command_code, data=b"", test_counts=None, protocol_version=1
):
    """
    Sends an MSP command to the flight controller and waits for a response.
    It handles serialization, deserialization, checksums, and basic error detection.
    The received data is then passed to a command-specific parser for interpretation.

    Args:
        ser (serial.Serial): The serial port object connected to the flight controller.
        command_name (str): The name of the MSP command (for logging).
        command_code (int): The integer code of the MSP command.
        data (bytes): Optional payload data to send with the command.
        test_counts (list): A mutable list [successful, failed] to track test results.
        protocol_version (int): The MSP protocol version to use (1 or 2).

    Returns:
        None
    """
    print(
        f"\n{BLUE}{BOLD}{ICON_COMMAND} Testing {command_name} (Code: {command_code}, Protocol: V{protocol_version}){RESET}"
    )

    # Serialize the MSP command into a byte packet, including header and checksum.
    command_packet = serialize_msp_command(command_code, data, protocol_version)
    ser.write(command_packet)

    # Clear the input buffer before reading to ensure we only get the current response.
    # A small delay is added for MSP_RC as it might sometimes send unwanted bytes initially.
    if command_code == MSP_COMMANDS["MSP_RC"]:
        time.sleep(0.05)  # Give some time for unwanted bytes to arrive
    ser.reset_input_buffer()

    response_buffer = b""
    start_time = time.time()
    # Wait for response bytes until timeout or a complete packet is received.
    while time.time() - start_time < TIMEOUT:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            response_buffer += byte
            # Check for a complete MSP V1 or V2 packet by inspecting header and size.
            if len(response_buffer) >= 3 and (
                response_buffer.startswith(b"$M") or response_buffer.startswith(b"$X")
            ):
                header_third_byte = response_buffer[2]
                if header_third_byte == ord(">") or header_third_byte == ord(
                    "!"
                ):  # Valid response or error header
                    if response_buffer.startswith(b"$M"):  # MSP V1 packet
                        if len(response_buffer) >= 4:  # Size byte present
                            size = response_buffer[3]
                            if (
                                len(response_buffer) >= 6 + size
                            ):  # Header (3) + size (1) + command (1) + data (size) + checksum (1)
                                break  # Complete V1 packet received
                    elif response_buffer.startswith(b"$X"):  # MSP V2 packet
                        if (
                            len(response_buffer) >= 8
                        ):  # Flag (1) + Command (2) + Size (2) bytes present
                            # Unpack size to determine full packet length
                            size = struct.unpack("<H", response_buffer[6:8])[0]
                            if (
                                len(response_buffer) >= 9 + size
                            ):  # Header (3) + flag (1) + command (2) + size (2) + data (size) + checksum (1)
                                break  # Complete V2 packet received
        time.sleep(0.001)  # Small delay to prevent busy-waiting and reduce CPU usage

    if not response_buffer:
        # If no response is received within the timeout, log a failure.
        print(f"{RED}{ICON_FAILURE} No response received for {command_name}.{RESET}")
        if test_counts is not None:
            test_counts[1] += 1  # Increment failed count
        return

    # Attempt to deserialize the received byte buffer into command code, data, error status, and protocol version.
    command_code_rx, response_data, is_error, received_protocol_version = (
        deserialize_msp_response(response_buffer)
    )

    if command_code_rx is not None:
        # Log success and update test counts.
        print(
            f"{GREEN}{ICON_SUCCESS} Received response for {command_name} (V{received_protocol_version}):{RESET}"
        )
        if test_counts is not None:
            test_counts[0] += 1  # Increment successful count

        # Always print raw data for debugging purposes.
        if response_data:
            print(f"  {CYAN}Raw Data (hex):{RESET} {response_data.hex()}")
        else:
            print(f"  {CYAN}Raw Data (hex):{RESET} None")

        # Handle error responses by simply noting them (details already printed by deserialize_msp_response).
        if is_error:
            pass
        else:
            # Dispatch to the appropriate parsing function using the COMMAND_PARSERS table.
            parser_func = COMMAND_PARSERS.get(command_code)
            if parser_func:
                # System info commands require the command code for specific interpretation
                # (due to the shared _parse_system_info helper function).
                if command_code in [
                    MSP_COMMANDS["MSP_API_VERSION"],
                    MSP_COMMANDS["MSP_FC_VARIANT"],
                    MSP_COMMANDS["MSP_FC_VERSION"],
                    MSP_COMMANDS["MSP_BOARD_INFO"],
                    MSP_COMMANDS["MSP_BUILD_INFO"],
                    MSP_COMMANDS["MSP_NAME"],
                    MSP_COMMANDS["MSP_IDENT"],
                ]:
                    # The _parse_system_info function handles multiple related commands.
                    parser_func(command_code, response_data)
                else:
                    # Other commands only need the response data.
                    parser_func(response_data)
            else:
                # If no specific parser is defined, indicate that the raw hex data is displayed.
                print(
                    f"  {YELLOW}{ICON_INFO} No specific parser for {command_name}. Raw hex data displayed.{RESET}"
                )
    else:
        # Log failure if deserialization failed (e.g., checksum mismatch or incomplete packet).
        print(f"{RED}{ICON_FAILURE} Failed to get valid MSP response.{RESET}")
        if test_counts is not None:
            test_counts[1] += 1  # Increment failed count


# --- Main Script ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test MSP communication with the flight32 Flight Controller."
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyUSB1",
        help="Serial port connected to the Flight Controller (default: /dev/ttyUSB1)",
    )
    parser.add_argument(
        "--command",
        type=str,
        help="Test a specific MSP command by its name (e.g., MSP_BOX). If omitted, all commands are tested.",
    )
    args = parser.parse_args()

    SERIAL_PORT_TO_USE = args.port
    COMMAND_TO_TEST = args.command

    print(
        f"{ICON_INFO} Attempting to connect to {BOLD}{SERIAL_PORT_TO_USE}{RESET} at {BOLD}{BAUD_RATE}{RESET} baud..."
    )
    test_results = [0, 0]  # [successful_tests, failed_tests]
    try:
        ser = serial.Serial(SERIAL_PORT_TO_USE, BAUD_RATE, timeout=TIMEOUT)
        ser.reset_input_buffer()  # Clear any leftover data in the buffer
        print(f"{GREEN}{ICON_SUCCESS} Serial port opened successfully.{RESET}")
        time.sleep(
            1
        )  # Add a delay to allow the FC to boot and send any initial debug messages
        time.sleep(2)  # Give the FC some time to initialize after connection

        commands_to_run = {}
        if COMMAND_TO_TEST:
            if COMMAND_TO_TEST in MSP_COMMANDS:
                commands_to_run[COMMAND_TO_TEST] = MSP_COMMANDS[COMMAND_TO_TEST]
            else:
                print(
                    f"{RED}{ICON_FAILURE} Error: Command '{COMMAND_TO_TEST}' not found in the list of available MSP commands.{RESET}"
                )
                test_results[1] = 1  # Mark as a failed test overall for invalid command
                raise SystemExit  # Exit if command is not found
        else:
            commands_to_run = MSP_COMMANDS

        for command_name, command_code in commands_to_run.items():
            # Special handling for MSP_GET_SETTING to send a key
            if command_name == "MSP_GET_SETTING":
                setting_key = "pid.roll.p"
                key_bytes = setting_key.encode("ascii")
                # Payload: [key_length (1 byte)] [key_string (variable length)]
                payload = bytes([len(key_bytes)]) + key_bytes
                send_and_receive(
                    ser,
                    command_name,
                    command_code,
                    data=payload,
                    test_counts=test_results,
                )
                time.sleep(0.5)  # Added delay
                # Test MSP_SET_SETTING as well
                time.sleep(0.1)  # Small delay

            else:
                send_and_receive(
                    ser, command_name, command_code, test_counts=test_results
                )
                time.sleep(0.5)  # Added delay

    except serial.SerialException as e:
        print(
            f"{RED}{ICON_FAILURE} Error: Could not open serial port {BOLD}{SERIAL_PORT_TO_USE}{RESET}: {e}{RESET}"
        )
        print(
            f"{YELLOW}{ICON_WARNING} Please ensure the flight controller is connected and not in use by another application.{RESET}"
        )
    except KeyboardInterrupt:
        print(f"\n{YELLOW}{ICON_WARNING} Script interrupted by user.{RESET}")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()
            print(f"\n{ICON_INFO} Serial port closed.{RESET}")

        print(f"\n{BLUE}{BOLD}--- Test Summary ---{RESET}")
        print(f"{GREEN}{ICON_SUCCESS} Successful tests: {test_results[0]}{RESET}")
        print(f"{RED}{ICON_FAILURE} Failed tests: {test_results[1]}{RESET}")
        print(f"{BLUE}{BOLD}--------------------{RESET}")
