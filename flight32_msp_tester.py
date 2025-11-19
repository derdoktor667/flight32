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
    "MSP_PID": 102,
    "MSP_MOTOR": 104,
    "MSP_RC_CHANNELS": 105,
    "MSP_SENSOR_STATUS": 108,
    "MSP_BOXNAMES": 116,
    "MSP_BOX": 117,
    "MSP_MODE_RANGES": 119,
    "MSP_MOTOR_CONFIG": 124,
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


# --- Individual MSP Response Decoders ---


def _decode_msp_api_version(response_data):
    if response_data is not None and len(response_data) >= 3:
        return {
            "msp_protocol_version": response_data[0],
            "capability_hr": response_data[1],
            "capability_lr": response_data[2],
        }
    return None


def _decode_msp_fc_variant(response_data):
    if response_data:
        return {"fc_variant": response_data.decode("ascii", errors="ignore")}
    return None


def _decode_msp_fc_version(response_data):
    if response_data is not None and len(response_data) == 3:
        return {
            "major": response_data[0],
            "minor": response_data[1],
            "patch": response_data[2],
        }
    return None


def _decode_msp_board_info(response_data):
    if response_data is not None and len(response_data) >= 7:
        board_identifier = response_data[0:4].decode("ascii", errors="ignore")
        hardware_revision = struct.unpack("<H", response_data[4:6])[0]
        board_name_len = response_data[6]
        if len(response_data) >= 7 + board_name_len:
            board_name = response_data[7 : 7 + board_name_len].decode(
                "ascii", errors="ignore"
            )
        else:
            board_name = response_data[7:].decode("ascii", errors="ignore")
        return {
            "board_identifier": board_identifier,
            "hardware_revision": hardware_revision,
            "board_name": board_name,
        }
    return None


def _decode_msp_build_info(response_data):
    if response_data:
        return {"build_info": response_data.decode("ascii", errors="ignore")}
    return None


def _decode_msp_status(response_data):
    if response_data is not None and len(response_data) >= 11:
        flight_mode_flags = struct.unpack("<H", response_data[0:2])[0]
        cycle_time = struct.unpack("<H", response_data[2:4])[0]
        i2c_errors = struct.unpack("<H", response_data[4:6])[0]
        sensor_present = response_data[6]
        box_status = struct.unpack("<I", response_data[7:11])[
            0
        ]  # Assuming uint32 for boxStatus based on modern MSP

        sensor_names = []
        if sensor_present & (1 << 0):
            sensor_names.append("ACC")
        if sensor_present & (1 << 1):
            sensor_names.append("BARO")
        # Add other sensors if needed (MAG, GPS, SONAR as per original script comments)

        return {
            "flight_mode_flags": flight_mode_flags,
            "cycle_time_us": cycle_time,
            "i2c_errors": i2c_errors,
            "sensor_present": sensor_present,
            "sensor_names": ", ".join(sensor_names) if sensor_names else "None",
            "box_status": box_status,
        }
    return None


def _decode_msp_sensor_status(response_data):
    if response_data is not None and len(response_data) >= 3:
        acc_health, gyro_health, mag_health = struct.unpack("<BBB", response_data[0:3])
        return {
            "accelerometer_health": acc_health,
            "gyroscope_health": gyro_health,
            "magnetometer_health": mag_health,
        }
    return None


def _decode_msp_boxnames(response_data):
    if response_data:
        box_names = response_data.decode("ascii", errors="ignore").strip(";").split(";")
        return {"box_names": box_names}
    return None


def _decode_msp_box(response_data):
    if response_data:
        return {"box_data": response_data.decode("ascii", errors="ignore")}
    return None


def _decode_msp_mode_ranges(response_data):
    if response_data is None:
        return None

    entry_len = 4
    if len(response_data) % entry_len != 0:
        return None  # Indicate parsing error due to unexpected length

    mode_ranges = []
    for i in range(0, len(response_data) // entry_len):
        offset = i * entry_len
        permanent_id, aux_channel_index, start_step, end_step = struct.unpack(
            "<BBBB", response_data[offset : offset + entry_len]
        )
        mode_ranges.append(
            {
                "permanent_id": permanent_id,
                "aux_channel_index": aux_channel_index,
                "start_step": start_step,
                "end_step": end_step,
            }
        )

    if mode_ranges:
        return {"mode_ranges": mode_ranges}
    return None


def _decode_msp_rc_channels(response_data):
    if response_data is not None and len(response_data) % 2 == 0:
        channels = struct.unpack("<" + "H" * (len(response_data) // 2), response_data)
        return {"rc_channels": list(channels)}
    return None


def _decode_msp_motor(response_data):
    if response_data is not None and len(response_data) % 2 == 0:
        motors = struct.unpack("<" + "H" * (len(response_data) // 2), response_data)
        return {"motor_values": list(motors)}
    return None


def _decode_msp_pid(response_data):
    if response_data is not None and len(response_data) >= 9:
        pid_gains = struct.unpack("<BBBBBBBBB", response_data[0:9])
        return {
            "roll_p": pid_gains[0],
            "roll_i": pid_gains[1],
            "roll_d": pid_gains[2],
            "pitch_p": pid_gains[3],
            "pitch_i": pid_gains[4],
            "pitch_d": pid_gains[5],
            "yaw_p": pid_gains[6],
            "yaw_i": pid_gains[7],
            "yaw_d": pid_gains[8],
        }
    return None


def _decode_msp_motor_config(response_data):
    if response_data is None:
        return None

    expected_len = 6
    if len(response_data) < expected_len:
        return None  # Indicate parsing error due to incomplete data

    minthrottle = struct.unpack("<H", response_data[0:2])[0]
    maxthrottle = struct.unpack("<H", response_data[2:4])[0]
    mincommand = struct.unpack("<H", response_data[4:6])[0]

    decoded = {
        "min_throttle": minthrottle,
        "max_throttle": maxthrottle,
        "min_command": mincommand,
    }

    if len(response_data) > expected_len:
        decoded["unparsed_trailing_data_hex"] = response_data[expected_len:].hex()

    return decoded


def _decode_msp_generic(response_data):
    return {"raw_data": response_data.hex() if response_data is not None else "None"}


def _decode_msp_raw_imu(response_data):
    if response_data is None or len(response_data) != 18:  # 3x Accel, 3x Gyro, 3x Mag (2 bytes each = 18 bytes)
        return None

    # Unpack 9 signed short integers (int16_t)
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = struct.unpack(
        "<hhhhhhhhh", response_data
    )
    return {
        "accel_x": accel_x,
        "accel_y": accel_y,
        "accel_z": accel_z,
        "gyro_x": gyro_x,
        "gyro_y": gyro_y,
        "gyro_z": gyro_z,
        "mag_x": mag_x,
        "mag_y": mag_y,
        "mag_z": mag_z,
    }

# Mapping of MSP command codes to their decoding functions
MSP_DECODERS = {
    MSP_COMMANDS["MSP_API_VERSION"]: _decode_msp_api_version,
    MSP_COMMANDS["MSP_FC_VARIANT"]: _decode_msp_fc_variant,
    MSP_COMMANDS["MSP_FC_VERSION"]: _decode_msp_fc_version,
    MSP_COMMANDS["MSP_BOARD_INFO"]: _decode_msp_board_info,
    MSP_COMMANDS["MSP_BUILD_INFO"]: _decode_msp_build_info,
    MSP_COMMANDS["MSP_STATUS"]: _decode_msp_status,
    MSP_COMMANDS["MSP_SENSOR_STATUS"]: _decode_msp_sensor_status,
    MSP_COMMANDS["MSP_BOXNAMES"]: _decode_msp_boxnames,
    MSP_COMMANDS["MSP_BOX"]: _decode_msp_box,
    MSP_COMMANDS["MSP_MODE_RANGES"]: _decode_msp_mode_ranges,
    MSP_COMMANDS["MSP_RC_CHANNELS"]: _decode_msp_rc_channels,
    MSP_COMMANDS["MSP_MOTOR"]: _decode_msp_motor,
    102: _decode_msp_pid, # Map actual code 102 to PID decoder
    # MSP_COMMANDS["MSP_RAW_IMU"] will use decoder_override
    MSP_COMMANDS["MSP_MOTOR_CONFIG"]: _decode_msp_motor_config,
}


# Helper function to print decoded data
def _print_decoded_data(command_name, decoded_data, raw_data_hex):
    if decoded_data:
        print(f"  {BOLD}Decoded {command_name} Data:{RESET}")
        for key, value in decoded_data.items():
            if key == "mode_ranges":  # Special handling for mode_ranges list
                print(f"    - {key.replace('_', ' ').title()}:")
                for item in value:
                    print(
                        f"      - Permanent ID:{item['permanent_id']} Aux Channel:{item['aux_channel_index']} Start Step:{item['start_step']} End Step:{item['end_step']}"
                    )
            elif key == "box_names":  # Special handling for box_names list
                print(f"    - {key.replace('_', ' ').title()}: {', '.join(value)}")
            elif (
                key == "rc_channels" or key == "motor_values"
            ):  # Special handling for lists
                print(
                    f"    - {key.replace('_', ' ').title()}: {', '.join(map(str, value))}"
                )
            elif key == "box_status" and isinstance(
                value, int
            ):  # Print Box Status in binary
                print(f"    - {key.replace('_', ' ').title()}: {bin(value)}")
            else:
                print(f"    - {key.replace('_', ' ').title()}: {value}")
        if "unparsed_trailing_data_hex" in decoded_data:
            print(
                f"  {YELLOW}{ICON_WARNING} Unparsed trailing data (hex):{RESET} {decoded_data['unparsed_trailing_data_hex']}"
            )
    elif command_name == "generic":
        print(f"  {ICON_INFO} Decoded (generic):{RESET} {raw_data_hex}")
    else:
        print(
            f"  {YELLOW}{ICON_WARNING} Decoded (unparsed {command_name}):{RESET} {raw_data_hex}"
        )


def send_and_receive(ser, command_name, command_code, data=b"", decoder_override=None):
    """
    Sends an MSP command and receives the response.
    Returns a tuple (success, decoded_data) where success is a boolean and decoded_data is the parsed response.
    """
    print(
        f"{ICON_COMMAND} Sending {BOLD}{command_name}{RESET} (code: {command_code})..."
    )

    try:
        # Serialize and send the command
        packet = serialize_msp_command(command_code, data)
        ser.write(packet)

        # Read response
        response_bytes = ser.read(1024)  # Read up to 1024 bytes

        if not response_bytes:
            print(f"{RED}{ICON_FAILURE} No response received from FC.{RESET}")
            return False, None

        # Deserialize the response
        command_code_response, response_data = deserialize_msp_response(response_bytes)

        if command_code_response is None:
            return False, None

        # Get the appropriate decoder function
        if decoder_override:
            decoder_func = decoder_override
        else:
            decoder_func = MSP_DECODERS.get(command_code_response, _decode_msp_generic)
        decoded_data = decoder_func(response_data)

        # Print the results
        raw_data_hex = response_data.hex() if response_data else "None"
        _print_decoded_data(command_name, decoded_data, raw_data_hex)

        print(f"{GREEN}{ICON_SUCCESS} {command_name} command successful.{RESET}")
        return True, decoded_data

    except Exception as e:
        print(f"{RED}{ICON_FAILURE} Error sending/receiving {command_name}: {e}{RESET}")
        return False, None


def run_test_case(ser, command_name, command_code, data=b"", assertions=None, decoder_override=None):
    """
    Runs a single test case for an MSP command.
    Returns True if the test passes, False otherwise.
    """
    success, decoded_data = send_and_receive(ser, command_name, command_code, data, decoder_override)
    test_passed = success

    if assertions and decoded_data:
        # Check for expected keys to exist in decoded_data
        for key in assertions.keys():
            if key not in decoded_data:
                print(
                    f"{RED}{ICON_FAILURE} Assertion Failed for {command_name}: Key '{key}' not found in decoded data.{RESET}"
                )
                test_passed = False

        # Perform specific value assertions
        if test_passed:  # Only proceed if all keys are present
            for key, expected_value in assertions.items():
                if key == "mode_ranges":  # Special handling for mode_ranges list
                    if not isinstance(decoded_data[key], list):
                        print(
                            f"{RED}{ICON_FAILURE} Assertion Failed for {command_name} - {key}: Expected a list, but got {type(decoded_data[key])}.{RESET}"
                        )
                        test_passed = False
                        continue
                    if len(decoded_data[key]) != len(expected_value):
                        print(
                            f"{RED}{ICON_FAILURE} Assertion Failed for {command_name} - {key}: Expected {len(expected_value)} mode ranges, but got {len(decoded_data[key])}.{RESET}"
                        )
                        test_passed = False
                    else:
                        for i, expected_item in enumerate(expected_value):
                            if decoded_data[key][i] != expected_item:
                                print(
                                    f"{RED}{ICON_FAILURE} Assertion Failed for {command_name} - {key} item {i}: Expected '{expected_item}', but got '{decoded_data[key][i]}'.{RESET}"
                                )
                                test_passed = False
                elif isinstance(
                    expected_value, dict
                ):  # For nested dictionaries (e.g., if we had sub-assertions)
                    for sub_key, sub_expected_value in expected_value.items():
                        if sub_key in decoded_data[key]:
                            if decoded_data[key][sub_key] != sub_expected_value:
                                print(
                                    f"{RED}{ICON_FAILURE} Assertion Failed for {command_name} - {key}: Expected '{sub_key}' to be '{sub_expected_value}', but got '{decoded_data[key][sub_key]}'.{RESET}"
                                )
                                test_passed = False
                        else:
                            print(
                                f"{RED}{ICON_FAILURE} Assertion Failed for {command_name} - {key}: Sub-key '{sub_key}' not found in decoded data.{RESET}"
                            )
                            test_passed = False
                elif decoded_data[key] != expected_value:
                    print(
                        f"{RED}{ICON_FAILURE} Assertion Failed for {command_name}: Expected '{key}' to be '{expected_value}', but got '{decoded_data[key]}'.{RESET}"
                    )
                    test_passed = False
    elif assertions and not decoded_data:
        print(
            f"{RED}{ICON_FAILURE} Assertion Failed for {command_name}: No decoded data to assert against.{RESET}"
        )
        test_passed = False

    return test_passed


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
    successful_tests = 0
    failed_tests = 0
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
                failed_tests += 1  # Mark as a failed test overall for invalid command
                raise SystemExit  # Exit if command is not found
        else:
            commands_to_run = MSP_COMMANDS

        for command_name_in_loop, command_code_in_loop in commands_to_run.items():
            # Define assertions here for each command
            assertions = None
            
            if command_code_in_loop == 102: # Handle shared ID for PID and RAW_IMU (which is MSP_PID)
                # Test MSP_PID first
                print(f"\n{ICON_INFO} Testing {BOLD}MSP_PID{RESET} (code: {command_code_in_loop})...")
                test_passed_pid = run_test_case(ser, "MSP_PID", command_code_in_loop, assertions=assertions, decoder_override=_decode_msp_pid)
                if not test_passed_pid:
                    failed_tests += 1
                else:
                    successful_tests += 1
                time.sleep(0.1)

                # Then test MSP_RAW_IMU
                print(f"\n{ICON_INFO} Testing {BOLD}MSP_RAW_IMU{RESET} (code: {command_code_in_loop})...")
                test_passed_raw_imu = run_test_case(ser, "MSP_RAW_IMU", command_code_in_loop, assertions=assertions, decoder_override=_decode_msp_raw_imu)
                if not test_passed_raw_imu:
                    failed_tests += 1
                else:
                    successful_tests += 1
                time.sleep(0.1)
                continue # Skip normal processing for this item from commands_to_run

            # Normal processing for other commands
            test_passed = run_test_case(ser, command_name_in_loop, command_code_in_loop, assertions=assertions)
            if test_passed:
                successful_tests += 1
            else:
                failed_tests += 1
            time.sleep(0.1)

    except serial.SerialException as e:
        print(
            f"{RED}{ICON_FAILURE} Error: Could not open serial port {BOLD}{SERIAL_PORT_TO_USE}{RESET}: {e}{RESET}"
        )
        print(
            f"{YELLOW}{ICON_WARNING} Please ensure the flight controller is connected and not in use by another application.{RESET}"
        )
        failed_tests += 1
    except KeyboardInterrupt:
        print(f"\n{YELLOW}{ICON_WARNING} Script interrupted by user.{RESET}")
        failed_tests += 1
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()
            print(f"\n{ICON_INFO} Serial port closed.{RESET}")

        print(f"\n{BLUE}{BOLD}--- Test Summary ---{RESET}")
        print(f"{GREEN}{ICON_SUCCESS} Successful tests: {successful_tests}{RESET}")
        print(f"{RED}{ICON_FAILURE} Failed tests: {failed_tests}{RESET}")
        print(f"{BLUE}{BOLD}--------------------{RESET}")
