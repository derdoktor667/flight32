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
    "MSP_API_VERSION": 1,
    "MSP_FC_VARIANT": 2,
    "MSP_FC_VERSION": 3,
    "MSP_BOARD_INFO": 4,
    "MSP_BUILD_INFO": 5,
    "MSP_STATUS": 101,
    "MSP_MOTOR": 104,
    "MSP_ATTITUDE": 108,
    "MSP_PID": 102,
    "MSP_BOXNAMES": 116,
    "MSP_BOX": 117,
    "MSP_MODE_RANGES": 119,
    "MSP_MOTOR_CONFIG": 124,
    "MSP_UID": 160,
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
        packet += struct.pack("<BHHH", flag, command_code, size, 0) # Placeholder for payload size high byte in struct, will be overwritten by direct bytes
        packet = packet[:-2] # Remove placeholder size high byte, command high byte
        packet += struct.pack("<B", flag)
        packet += struct.pack("<H", command_code) # Command ID (16-bit)
        packet += struct.pack("<H", size) # Payload Size (16-bit)
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
            print(f"{YELLOW}{ICON_WARNING} Incomplete MSP V1 response header/minimal length.{RESET}")
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
            print(f"{RED}{ICON_FAILURE} Checksum mismatch for V1 command {command_code}. Expected {calculated_checksum}, got {received_checksum}{RESET}")
            return None, None, None, protocol_version

    elif header == b"$X>":
        protocol_version = 2
        # V2 header: $X> (3 bytes), flag (1 byte), command (2 bytes), size (2 bytes)
        if len(response_bytes) < 9:
            print(f"{YELLOW}{ICON_WARNING} Incomplete MSP V2 response header/minimal length.{RESET}")
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
            print(f"{RED}{ICON_FAILURE} Checksum mismatch for V2 command {command_code}. Expected {calculated_crc}, got {received_checksum}{RESET}")
            return None, None, None, protocol_version

    elif header == b"$M!": # V1 error
        protocol_version = 1
        is_error = True
        # Parse minimal error response for V1
        if len(response_bytes) < 6:
            print(f"{YELLOW}{ICON_WARNING} Incomplete MSP V1 error response.{RESET}")
            return None, None, is_error, protocol_version
        
        size = response_bytes[3]
        command_code = response_bytes[4]
        data = response_bytes[5 : 5 + size] # Should be empty for error, but just in case
        
        # Checksum for error is usually on the header as well
        # For simplicity, we'll just check if it's an error response
        print(f"{RED}{ICON_FAILURE} Received V1 Error for command {command_code}.{RESET}")
        return command_code, data, is_error, protocol_version

    elif header == b"$X!": # V2 error
        protocol_version = 2
        is_error = True
        # Parse minimal error response for V2
        if len(response_bytes) < 9: # $X! + flag + cmd (2) + size (2) + crc (1)
            print(f"{YELLOW}{ICON_WARNING} Incomplete MSP V2 error response.{RESET}")
            return None, None, is_error, protocol_version
        
        flag = response_bytes[3]
        command_code = struct.unpack("<H", response_bytes[4:6])[0]
        size = struct.unpack("<H", response_bytes[6:8])[0] # Should be 0 for error
        data = response_bytes[8 : 8 + size] # Should be empty for error
        
        print(f"{RED}{ICON_FAILURE} Received V2 Error for command {command_code}.{RESET}")
        return command_code, data, is_error, protocol_version

    else:
        print(f"{YELLOW}{ICON_WARNING} Unexpected MSP response header: {header}{RESET}")
        return None, None, None, None

    return command_code, data, is_error, protocol_version


def send_and_receive(ser, command_name, command_code, data=b"", test_counts=None, protocol_version=1):
    """Sends an MSP command and reads the response, updating test_counts."""
    print(f"\n{BLUE}{BOLD}{ICON_COMMAND} Testing {command_name} (Code: {command_code}, Protocol: V{protocol_version}){RESET}")
    command_packet = serialize_msp_command(command_code, data, protocol_version)
    ser.write(command_packet)
    if command_name == "MSP_RC":
        time.sleep(0.01) # Give some time for unwanted bytes to arrive
        ser.reset_input_buffer() # Clear them
        time.sleep(0.01) # Another small delay
        ser.reset_input_buffer() # Clear again, just in case
    else:
        ser.reset_input_buffer() # Clear input buffer before reading response

    response_buffer = b""
    start_time = time.time()
    while time.time() - start_time < TIMEOUT:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            response_buffer += byte
            # Check for V1 header ($M) or V2 header ($X)
            if len(response_buffer) >= 3 and (response_buffer.startswith(b"$M") or response_buffer.startswith(b"$X")):
                header_third_byte = response_buffer[2]
                if header_third_byte == ord(">") or header_third_byte == ord("!"):
                    if response_buffer.startswith(b"$M"): # V1
                        if len(response_buffer) >= 4: # Size byte present
                            size = response_buffer[3]
                            if len(response_buffer) >= 6 + size: # Header (3) + size (1) + command (1) + data (size) + checksum (1)
                                break
                    elif response_buffer.startswith(b"$X"): # V2
                        if len(response_buffer) >= 8: # Flag (1) + Command (2) + Size (2) bytes present
                            size = struct.unpack("<H", response_buffer[6:8])[0]
                            if len(response_buffer) >= 9 + size: # Header (3) + flag (1) + command (2) + size (2) + data (size) + checksum (1)
                                break
        time.sleep(0.001)  # Small delay to prevent busy-waiting

    if not response_buffer:
        print(f"{RED}{ICON_FAILURE} No response received for {command_name}.{RESET}")
        if test_counts is not None:
            test_counts[1] += 1  # Increment failed count
        return

    command_code_rx, response_data, is_error, received_protocol_version = deserialize_msp_response(response_buffer)

    if command_code_rx is not None:
        print(f"{GREEN}{ICON_SUCCESS} Received response for {command_name} (V{received_protocol_version}):{RESET}")
        if test_counts is not None:
            test_counts[0] += 1  # Increment successful count
        print(f"  {CYAN}Raw Data (hex):{RESET} {response_data.hex() if response_data is not None else 'None'}")
        
        # Add basic interpretation for some known commands
        if command_code == MSP_COMMANDS["MSP_API_VERSION"]:
            if response_data is not None and len(response_data) >= 3:
                print(f"  {BOLD}MSP Protocol Version:{RESET} {response_data[0]}")
                print(f"  {BOLD}Capability:{RESET} {response_data[1]} {response_data[2]}")
            else:
                print(f"  {YELLOW}{ICON_WARNING} Decoded (unparsed API version):{RESET} {response_data}")
        elif command_code == MSP_COMMANDS["MSP_FC_VARIANT"]:
            if response_data:
                print(f"  {BOLD}FC Variant:{RESET} {response_data.decode('ascii', errors='ignore')}")
        elif command_code == MSP_COMMANDS["MSP_FC_VERSION"]:
            if response_data is not None and len(response_data) == 3:
                print(f"  {BOLD}FC Version:{RESET} {response_data[0]}.{response_data[1]}.{response_data[2]}")
        elif command_code == MSP_COMMANDS["MSP_BOARD_INFO"]:
            if response_data is not None and len(response_data) >= 7:
                board_identifier = response_data[0:4].decode("ascii", errors="ignore")
                hardware_revision = struct.unpack("<H", response_data[4:6])[0]
                board_name_len = response_data[6]
                if len(response_data) >= 7 + board_name_len:
                    board_name = response_data[7 : 7 + board_name_len].decode("ascii", errors="ignore")
                else:
                    board_name = response_data[7:].decode("ascii", errors="ignore")

                print(f"  {BOLD}Board Identifier:{RESET} {board_identifier}")
                print(f"  {BOLD}Hardware Revision:{RESET} {hardware_revision}")
                print(f"  {BOLD}Board Name:{RESET} {board_name}")
        elif command_code == MSP_COMMANDS["MSP_BUILD_INFO"]:
            if response_data:
                build_info = response_data.decode("ascii", errors="ignore")
                print(f"  {BOLD}Build Info:{RESET} {build_info}")
        elif command_code == MSP_COMMANDS["MSP_STATUS"]:
            if response_data is not None and len(response_data) >= 11:
                # Based on msp.js and common MSP_STATUS v1.43:
                # cycleTime (uint16)
                # i2cErrors (uint16)
                # sensors (uint16) - sensorsPresentFlags bitmask
                # flightModeFlags (uint32)
                # configProfileIndex (uint8)

                offset = 0
                cycle_time = struct.unpack("<H", response_data[offset : offset + 2])[0]
                offset += 2
                i2c_errors = struct.unpack("<H", response_data[offset : offset + 2])[0]
                offset += 2
                sensors_present_flags = struct.unpack("<H", response_data[offset : offset + 2])[0]
                offset += 2  # sensors present bitmask
                flight_mode_flags = struct.unpack("<I", response_data[offset : offset + 4])[0]
                offset += 4  # uint32
                config_profile_index = response_data[offset]
                offset += 1

                print(f"  {BOLD}Cycle Time:{RESET} {cycle_time} us")
                print(f"  {BOLD}I2C Errors:{RESET} {i2c_errors}")
                print(f"  {BOLD}Sensors Present Flags:{RESET} {bin(sensors_present_flags)}")
                print(f"  {BOLD}Flight Mode Flags:{RESET} {flight_mode_flags} (raw)")
                print(f"  {BOLD}Config Profile Index:{RESET} {config_profile_index}")
        elif command_code == MSP_COMMANDS["MSP_ATTITUDE"]:
            if response_data is not None and len(response_data) == 6:
                roll, pitch, yaw = struct.unpack("<hhh", response_data)  # int16, int16, int16
                # Assuming same scaling factor as web app (10.0)
                print(f"  {BOLD}Roll:{RESET} {roll/10.0:.2f} deg")
                print(f"  {BOLD}Pitch:{RESET} {pitch/10.0:.2f} deg")
                print(f"  {BOLD}Yaw:{RESET} {yaw/10.0:.2f} deg")
            else:
                print(f"  {YELLOW}{ICON_WARNING} Decoded (unparsed attitude):{RESET} {response_data}")
        elif command_code == MSP_COMMANDS["MSP_BOXNAMES"]:
            if response_data:
                box_names = response_data.decode("ascii", errors="ignore").strip(";").split(";")
                print(f"  {BOLD}Box Names:{RESET} {', '.join(box_names)}")
        elif command_code == MSP_COMMANDS["MSP_BOX"]:
            if response_data:
                # This should be a uint32 bitmask, unpack as such
                box_flags = struct.unpack("<I", response_data[0:4])[0]
                print(f"  {BOLD}Box Flags:{RESET} {bin(box_flags)}")
            else:
                print(f"  {YELLOW}{ICON_WARNING} Decoded (unparsed BOX data):{RESET} {response_data}")
        elif command_code == MSP_COMMANDS["MSP_MODE_RANGES"]:
            if response_data is not None and len(response_data) % 4 == 0: # 4 bytes per range
                mode_ranges = []
                for i in range(0, len(response_data), 4): # Increment by 4
                    mode_id, aux_channel, start_percent, end_percent = struct.unpack("<BBBB", response_data[i : i + 4])

                    start_pwm = 1000 + (start_percent * 10)
                    end_pwm = 1000 + (end_percent * 10)

                    start_pwm = max(1000, min(2000, start_pwm))
                    end_pwm = max(1000, min(2000, end_pwm))

                    mode_ranges.append(f"ID:{mode_id} Aux:{aux_channel} Start:{start_pwm} End:{end_pwm}")
                print(f"  {BOLD}Mode Ranges:{RESET}")
                for r in mode_ranges:
                    print(f"    - {r}")
            else:
                print(f"  {YELLOW}{ICON_WARNING} Decoded (unparsed mode ranges):{RESET} {response_data}")
        elif command_code == MSP_COMMANDS["MSP_MOTOR"]:
            if response_data is not None and len(response_data) % 2 == 0:
                motors = struct.unpack("<" + "H" * (len(response_data) // 2), response_data)
                print(f"  {BOLD}Motor Values (PWM/Dshot):{RESET} {', '.join(map(str, motors))}")
            else:
                print(f"  {YELLOW}{ICON_WARNING} Decoded (unparsed motor data):{RESET} {response_data}")
        elif command_code == MSP_COMMANDS["MSP_PID"]:
            if response_data is not None and len(response_data) >= 18: # 9 PID values, each 2 bytes (int16)
                # Assuming format from msp.js: 9 int16 values (P,I,D for Roll, Pitch, Yaw)
                pid_gains = struct.unpack("<HHHHHHHHH", response_data[0:18])
                # Apply PID_SCALE_FACTOR as per msp.js (100.0)
                print(f"  {BOLD}PID Gains (Roll):{RESET} P:{pid_gains[0]/100.0:.3f} I:{pid_gains[1]/100.0:.3f} D:{pid_gains[2]/100.0:.3f}")
                print(f"  {BOLD}PID Gains (Pitch):{RESET} P:{pid_gains[3]/100.0:.3f} I:{pid_gains[4]/100.0:.3f} D:{pid_gains[5]/100.0:.3f}")
                print(f"  {BOLD}PID Gains (Yaw):{RESET} P:{pid_gains[6]/100.0:.3f} I:{pid_gains[7]/100.0:.3f} D:{pid_gains[8]/100.0:.3f}")
            else:
                print(f"  {YELLOW}{ICON_WARNING} Decoded (unparsed PID data):{RESET} {response_data}")
        elif command_code == MSP_COMMANDS["MSP_MOTOR_CONFIG"]:
            if response_data is not None and len(response_data) == 6: # Only 3 uint16 values (min_throttle, max_throttle, min_command)
                min_throttle = struct.unpack("<H", response_data[0:2])[0]
                max_throttle = struct.unpack("<H", response_data[2:4])[0]
                min_command = struct.unpack("<H", response_data[4:6])[0]

                print(f"  {BOLD}Min Throttle:{RESET} {min_throttle}")
                print(f"  {BOLD}Max Throttle:{RESET} {max_throttle}")
                print(f"  {BOLD}Min Command:{RESET} {min_command}")
            else:
                print(f"  {YELLOW}{ICON_WARNING} Decoded (unparsed motor config):{RESET} {response_data}")

    else:
        print(f"{RED}{ICON_FAILURE} Failed to get valid MSP response.{RESET}")
        if test_counts is not None:
            test_counts[1] += 1  # Increment failed count


# --- Main Script ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test MSP communication with the flight32 Flight Controller.")
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port connected to the Flight Controller (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--command",
        type=str,
        help="Test a specific MSP command by its name (e.g., MSP_BOX). If omitted, all commands are tested.",
    )
    args = parser.parse_args()

    SERIAL_PORT_TO_USE = args.port
    COMMAND_TO_TEST = args.command

    print(f"{ICON_INFO} Attempting to connect to {BOLD}{SERIAL_PORT_TO_USE}{RESET} at {BOLD}{BAUD_RATE}{RESET} baud...")
    test_results = [0, 0]  # [successful_tests, failed_tests]
    try:
        ser = serial.Serial(SERIAL_PORT_TO_USE, BAUD_RATE, timeout=TIMEOUT)
        ser.reset_input_buffer()  # Clear any leftover data in the buffer
        print(f"{GREEN}{ICON_SUCCESS} Serial port opened successfully.{RESET}")
        time.sleep(1)  # Add a delay to allow the FC to boot and send any initial debug messages
        time.sleep(2)  # Give the FC some time to initialize after connection

        commands_to_run = {}
        if COMMAND_TO_TEST:
            if COMMAND_TO_TEST in MSP_COMMANDS:
                commands_to_run[COMMAND_TO_TEST] = MSP_COMMANDS[COMMAND_TO_TEST]
            else:
                print(f"{RED}{ICON_FAILURE} Error: Command '{COMMAND_TO_TEST}' not found in the list of available MSP commands.{RESET}")
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
                send_and_receive(ser, command_name, command_code, data=payload, test_counts=test_results)
                time.sleep(0.5)  # Added delay
                # Test MSP_SET_SETTING as well
                time.sleep(0.1)  # Small delay

            else:
                send_and_receive(ser, command_name, command_code, test_counts=test_results)
                time.sleep(0.5)  # Added delay

    except serial.SerialException as e:
        print(f"{RED}{ICON_FAILURE} Error: Could not open serial port {BOLD}{SERIAL_PORT_TO_USE}{RESET}: {e}{RESET}")
        print(f"{YELLOW}{ICON_WARNING} Please ensure the flight controller is connected and not in use by another application.{RESET}")
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