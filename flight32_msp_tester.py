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
# Note: MSP_UID is not a standard direct command. MSP_IDENT (100) usually provides
# board information including a unique ID. Using MSP_IDENT for this purpose.
MSP_COMMANDS = {
    "MSP_API_VERSION": 1,
    "MSP_FC_VARIANT": 2,
    "MSP_FC_VERSION": 3,
    "MSP_BOARD_INFO": 4,
    "MSP_BUILD_INFO": 5,
    "MSP_GET_SETTING": 9,
    "MSP_PID": 11,
    "MSP_IDENT": 100,
    "MSP_STATUS": 101,
    "MSP_PID": 102,
    "MSP_MOTOR": 104,
    "MSP_RC_CHANNELS": 105,
    "MSP_SENSOR_STATUS": 108,
    "MSP_BOXNAMES": 116,
    "MSP_BOX": 117,
    "MSP_MODE_RANGES": 119,
    "MSP_MOTOR_CONFIG": 124,
    "MSP_GET_FILTER_CONFIG": 203,
    "MSP_SET_FILTER_CONFIG": 204,
}


# --- MSP Protocol Functions ---
def serialize_msp_command(command_code, data=b""):
    """Constructs an MSP command packet."""
    size = len(data)
    checksum = size ^ command_code
    for byte in data:
        checksum ^= byte

    packet = b"$M<"
    packet += struct.pack("<BB", size, command_code) + data
    packet += struct.pack("<B", checksum)
    return packet


def deserialize_msp_response(response_bytes):
    """Parses an MSP response packet."""
    if not response_bytes:
        return None, None

    # Check for valid MSP response headers: $, M (API v1), > (Slave to Master) or ! (Error)
    if response_bytes.startswith(b"$M>"):
        # Normal response, proceed with parsing
        pass
    elif response_bytes.startswith(b"$M!"):  # The FC explicitly sent an error
        print(
            f"{RED}{ICON_FAILURE} FC reported an error (MSP header: {response_bytes[:3]}).{RESET}"
        )
        # We consider this a handled failure from the FC itself.
        # Further data in response_bytes could be an error message, but for now we just report the header.
        return None, None
    else:  # Truly unexpected header
        print(
            f"{YELLOW}{ICON_WARNING} Warning: Unexpected MSP response header: {response_bytes[:3]}{RESET}"
        )
        return None, None

    try:
        # Extract size, command, data, and checksum
        size = response_bytes[3]
        command_code = response_bytes[4]
        data = response_bytes[5 : 5 + size]
        received_checksum = response_bytes[5 + size]

        # Calculate expected checksum
        calculated_checksum = size ^ command_code
        for byte in data:
            calculated_checksum ^= byte

        if calculated_checksum != received_checksum:
            print(
                f"{YELLOW}{ICON_WARNING} Warning: Checksum mismatch for command {command_code}. Expected {calculated_checksum}, got {received_checksum}.{RESET}"
            )
            return None, None

        return command_code, data
    except IndexError:
        print(f"{RED}{ICON_FAILURE} Error: Incomplete MSP response packet.{RESET}")
        return None, None
    except Exception as e:
        print(f"{RED}{ICON_FAILURE} Error parsing MSP response: {e}{RESET}")
        return None, None


def send_and_receive(ser, command_name, command_code, data=b"", test_counts=None):
    """Sends an MSP command and reads the response, updating test_counts."""
    print(
        f"\n{BLUE}{BOLD}{ICON_COMMAND} Testing {command_name} (Code: {command_code}){RESET}"
    )
    command_packet = serialize_msp_command(command_code, data)
    ser.write(command_packet)

    response_buffer = b""
    start_time = time.time()
    while time.time() - start_time < TIMEOUT:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            response_buffer += byte
            if len(response_buffer) >= 5 and response_buffer.startswith(
                b"$M"
            ):  # Check for header ($M) + size + command
                # Read the third byte to determine if it's a regular response (>) or error (!)
                header_third_byte = response_buffer[2]
                if header_third_byte == ord(">") or header_third_byte == ord("!"):
                    size = response_buffer[3]
                    # If we've received enough bytes for header (3) + size (1) + command (1) + data (size) + checksum (1)
                    if len(response_buffer) >= 6 + size:
                        break
        time.sleep(0.001)  # Small delay to prevent busy-waiting

    if not response_buffer:
        print(f"{RED}{ICON_FAILURE} No response received for {command_name}.{RESET}")
        if test_counts is not None:
            test_counts[1] += 1  # Increment failed count
        return

    command_code_rx, response_data = deserialize_msp_response(response_buffer)

    if command_code_rx is not None:
        print(f"{GREEN}{ICON_SUCCESS} Received response for {command_name}:{RESET}")
        print(
            f"  {CYAN}Raw Data (hex):{RESET} {response_data.hex() if response_data is not None else 'None'}"
        )
        # Add basic interpretation for some known commands
        if command_code == MSP_COMMANDS["MSP_API_VERSION"]:
            if response_data is not None and len(response_data) >= 3:
                print(f"  {BOLD}MSP Protocol Version:{RESET} {response_data[0]}")
                print(
                    f"  {BOLD}Capability:{RESET} {response_data[1]} {response_data[2]}"
                )
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed API version):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_FC_VARIANT"]:
            if response_data:
                print(
                    f"  {BOLD}FC Variant:{RESET} {response_data.decode('ascii', errors='ignore')}"
                )
        elif command_code == MSP_COMMANDS["MSP_FC_VERSION"]:
            if response_data is not None and len(response_data) == 3:
                print(
                    f"  {BOLD}FC Version:{RESET} {response_data[0]}.{response_data[1]}.{response_data[2]}"
                )
        elif command_code == MSP_COMMANDS["MSP_BOARD_INFO"]:
            if response_data is not None and len(response_data) >= 7:
                board_identifier = response_data[0:4].decode("ascii", errors="ignore")
                hardware_revision = struct.unpack("<H", response_data[4:6])[0]
                board_name_len = response_data[6]
                # Ensure we don't slice beyond the available data
                if len(response_data) >= 7 + board_name_len:
                    board_name = response_data[7 : 7 + board_name_len].decode(
                        "ascii", errors="ignore"
                    )
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
                flight_mode_flags = struct.unpack("<H", response_data[0:2])[0]
                cycle_time = struct.unpack("<H", response_data[2:4])[0]
                i2c_errors = struct.unpack("<H", response_data[4:6])[0]
                sensor_present = response_data[6]
                box_setup = struct.unpack("<H", response_data[7:9])[0]

                sensor_names = []
                if sensor_present & (1 << 0):
                    sensor_names.append("ACC")
                if sensor_present & (1 << 1):
                    sensor_names.append("BARO")
                if sensor_present & (1 << 2):
                    sensor_names.append("MAG")
                if sensor_present & (1 << 3):
                    sensor_names.append("GPS")
                if sensor_present & (1 << 4):
                    sensor_names.append("SONAR")

                print(f"  {BOLD}Flight Mode Flags:{RESET} {flight_mode_flags}")
                print(f"  {BOLD}Cycle Time:{RESET} {cycle_time} us")
                print(f"  {BOLD}I2C Errors:{RESET} {i2c_errors}")
                print(
                    f"  {BOLD}Sensor Present:{RESET} {', '.join(sensor_names) if sensor_names else 'None'} ({bin(sensor_present)})"
                )
                print(f"  {BOLD}Box Setup:{RESET} {box_setup}")

        elif command_code == MSP_COMMANDS["MSP_SENSOR_STATUS"]:
            # Ensure response_data is not None before calling len() to avoid TypeError
            if response_data is not None and len(response_data) >= 3:
                acc_health, gyro_health, mag_health = struct.unpack(
                    "<BBB", response_data[0:3]
                )
                print(f"  {BOLD}Accelerometer Health:{RESET} {acc_health}")
                print(f"  {BOLD}Gyroscope Health:{RESET} {gyro_health}")
                print(f"  {BOLD}Magnetometer Health:{RESET} {mag_health}")
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed sensor status):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_BOXNAMES"]:
            if response_data:
                box_names = (
                    response_data.decode("ascii", errors="ignore").strip(";").split(";")
                )
                print(f"  {BOLD}Box Names:{RESET} {', '.join(box_names)}")
        elif command_code == MSP_COMMANDS["MSP_BOX"]:
            if response_data:
                print(
                    f"  {BOLD}Decoded (unparsed BOX data):{RESET} {response_data.decode('ascii', errors='ignore')}"
                )
        elif command_code == MSP_COMMANDS["MSP_MODE_RANGES"]:
            if response_data is not None and len(response_data) % 6 == 0:
                mode_ranges = []
                for i in range(0, len(response_data), 6):
                    mode_id, aux_channel, start_percent, end_percent, flags = (
                        struct.unpack("<BBBBH", response_data[i : i + 6])
                    )
                    mode_ranges.append(
                        f"ID:{mode_id} Aux:{aux_channel} Start:{start_percent}% End:{end_percent}% Flags:{flags}"
                    )
                print(f"  {BOLD}Mode Ranges:{RESET}")
                for r in mode_ranges:
                    print(f"    - {r}")
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed mode ranges):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_RC_CHANNELS"]:
            if response_data is not None and len(response_data) % 2 == 0:
                channels = struct.unpack(
                    "<" + "H" * (len(response_data) // 2), response_data
                )
                print(f"  {BOLD}RC Channels:{RESET} {', '.join(map(str, channels))}")
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed RC channels):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_MOTOR"]:
            if response_data is not None and len(response_data) % 2 == 0:
                motors = struct.unpack(
                    "<" + "H" * (len(response_data) // 2), response_data
                )
                print(
                    f"  {BOLD}Motor Values (PWM/Dshot):{RESET} {', '.join(map(str, motors))}"
                )
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed motor data):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_PID"]:
            if response_data is not None and len(response_data) >= 9:
                pid_gains = struct.unpack("<BBBBBBBBB", response_data[0:9])
                print(
                    f"  {BOLD}PID Gains (Roll):{RESET} P:{pid_gains[0]} I:{pid_gains[1]} D:{pid_gains[2]}"
                )
                print(
                    f"  {BOLD}PID Gains (Pitch):{RESET} P:{pid_gains[3]} I:{pid_gains[4]} D:{pid_gains[5]}"
                )
                print(
                    f"  {BOLD}PID Gains (Yaw):{RESET} P:{pid_gains[6]} I:{pid_gains[7]} D:{pid_gains[8]}"
                )
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed PID data):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_MOTOR_CONFIG"]:
            if response_data is not None and len(response_data) >= 2:
                motor_count, motor_poles = struct.unpack("<BB", response_data[0:2])
                print(f"  {BOLD}Motor Count:{RESET} {motor_count}")
                print(f"  {BOLD}Motor Poles:{RESET} {motor_poles}")
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed motor config):{RESET} {response_data}"
                )
        else:
            print(f"  {ICON_INFO} Decoded (generic):{RESET} {response_data}")

        if test_counts is not None:
            test_counts[0] += 1  # Increment successful count
    else:
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

    print(
        f"{ICON_INFO} Attempting to connect to {BOLD}{SERIAL_PORT_TO_USE}{RESET} at {BOLD}{BAUD_RATE}{RESET} baud..."
    )
    test_results = [0, 0]  # [successful_tests, failed_tests]
    try:
        ser = serial.Serial(SERIAL_PORT_TO_USE, BAUD_RATE, timeout=TIMEOUT)
        ser.reset_input_buffer() # Clear any leftover data in the buffer
        print(f"{GREEN}{ICON_SUCCESS} Serial port opened successfully.{RESET}")
        time.sleep(1) # Add a delay to allow the FC to boot and send any initial debug messages
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
                key_bytes = setting_key.encode('ascii')
                # Payload: [key_length (1 byte)] [key_string (variable length)]
                payload = bytes([len(key_bytes)]) + key_bytes
                send_and_receive(ser, command_name, command_code, data=payload, test_counts=test_results)
            else:
                send_and_receive(ser, command_name, command_code, test_counts=test_results)
            time.sleep(0.1)  # Small delay between commands

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
