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
    "MSP_NAME": 10,
    "MSP_STATUS": 101,
    "MSP_RAW_IMU": 102,
    "MSP_GET_SETTING": 9,
    "MSP_MOTOR": 104,
    "MSP_RC": 105,
    "MSP_ATTITUDE": 108, # Changed from 100 back to 108
    "MSP_PID": 112,
    "MSP_BOXNAMES": 116,
    "MSP_BOX": 117,
    "MSP_MODE_RANGES": 119,
    "MSP_MOTOR_CONFIG": 131,
    "MSP_UID": 160,
    "MSP_REBOOT": 68,
    "MSP_SENSOR_STATUS": 209, # Added MSP_SENSOR_STATUS
}


# --- MSP Protocol Functions ---
def serialize_msp_command(command_code, data=b""):
    size = len(data)
    checksum = size ^ command_code
    for byte in data:
        checksum ^= byte

    packet = b"$M<"
    packet += struct.pack("<BB", size, command_code) + data
    packet += struct.pack("<B", checksum)
    return packet


def deserialize_msp_response(response_bytes):
    if not response_bytes or len(response_bytes) < 3:
        return None, None, None  # Return error status as well

    # Check for header
    header = response_bytes[0:3]
    is_error = False
    if header == b"$M>":
        pass
    elif header == b"$M!":
        is_error = True
    else:
        print(f"{YELLOW}{ICON_WARNING} Unexpected MSP response header: {header}{RESET}")
        return None, None, None

    if len(response_bytes) < 6:  # minimal length ($M> + size + command + checksum)
        print(
            f"{YELLOW}{ICON_WARNING} Incomplete MSP response packet header/minimal length.{RESET}"
        )
        return None, None, None

    size = response_bytes[3]
    command_code = response_bytes[4]

    if len(response_bytes) < 6 + size:
        print(f"{YELLOW}{ICON_WARNING} Incomplete MSP response packet payload.{RESET}")
        return None, None, None

    data = response_bytes[5 : 5 + size]
    received_checksum = response_bytes[5 + size]

    calculated_checksum = size ^ command_code
    for byte in data:
        calculated_checksum ^= byte

    if calculated_checksum != received_checksum:
        print(
            f"{RED}{ICON_FAILURE} Checksum mismatch for command {command_code}. Expected {calculated_checksum}, got {received_checksum}{RESET}"
        )
        return None, None, None  # Return None on checksum mismatch

    return command_code, data, is_error


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

    command_code_rx, response_data, is_error = deserialize_msp_response(response_buffer)

    if command_code_rx is not None:
        print(f"{GREEN}{ICON_SUCCESS} Received response for {command_name}:{RESET}")
        if test_counts is not None:
            test_counts[0] += 1  # Increment successful count
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
                sensors_present_flags = struct.unpack(
                    "<H", response_data[offset : offset + 2]
                )[0]
                offset += 2  # sensors present bitmask
                flight_mode_flags = struct.unpack(
                    "<I", response_data[offset : offset + 4]
                )[0]
                offset += 4  # uint32
                config_profile_index = response_data[offset]
                offset += 1

                print(f"  {BOLD}Cycle Time:{RESET} {cycle_time} us")
                print(f"  {BOLD}I2C Errors:{RESET} {i2c_errors}")
                print(
                    f"  {BOLD}Sensors Present Flags:{RESET} {bin(sensors_present_flags)}"
                )
                print(f"  {BOLD}Flight Mode Flags:{RESET} {flight_mode_flags} (raw)")
                print(f"  {BOLD}Config Profile Index:{RESET} {config_profile_index}")
        elif command_code == MSP_COMMANDS["MSP_SENSOR_STATUS"]: # New block for MSP_SENSOR_STATUS
            if response_data is not None and len(response_data) == 3:
                acc_health, gyro_health, mag_health = struct.unpack("<BBB", response_data)
                print(f"  {BOLD}ACC Health:{RESET} {acc_health}")
                print(f"  {BOLD}GYRO Health:{RESET} {gyro_health}")
                print(f"  {BOLD}MAG Health:{RESET} {mag_health}")
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed sensor status):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_ATTITUDE"]:
            if response_data is not None and len(response_data) == 6:
                roll, pitch, yaw = struct.unpack(
                    "<hhh", response_data
                )  # int16, int16, int16
                # Assuming same scaling factor as web app (10.0)
                print(f"  {BOLD}Roll:{RESET} {roll/10.0:.2f} deg")
                print(f"  {BOLD}Pitch:{RESET} {pitch/10.0:.2f} deg")
                print(f"  {BOLD}Yaw:{RESET} {yaw/10.0:.2f} deg")
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed attitude):{RESET} {response_data}"
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
            if response_data is not None and len(response_data) % 4 == 0:
                mode_ranges = []
                for i in range(0, len(response_data), 6):
                    mode_id, aux_channel, start_percent, end_percent = struct.unpack(
                        "<BBBB", response_data[i : i + 4]
                    )

                    start_pwm = 1000 + (start_percent * 10)
                    end_pwm = 1000 + (end_percent * 10)

                    start_pwm = max(1000, min(2000, start_pwm))
                    end_pwm = max(1000, min(2000, end_pwm))

                    mode_ranges.append(
                        f"ID:{mode_id} Aux:{aux_channel} Start:{start_pwm} End:{end_pwm}"
                    )
                print(f"  {BOLD}Mode Ranges:{RESET}")
                for r in mode_ranges:
                    print(f"    - {r}")
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed mode ranges):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_RC"]:
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
            if (
                response_data is not None and len(response_data) >= 18
            ):  # 9 PID values, each 2 bytes (int16)
                # Assuming format from msp.js: 9 int16 values (P,I,D for Roll, Pitch, Yaw)
                pid_gains = struct.unpack("<HHHHHHHHH", response_data[0:18])
                # Apply PID_SCALE_FACTOR as per msp.js (100.0)
                print(
                    f"  {BOLD}PID Gains (Roll):{RESET} P:{pid_gains[0]/100.0:.3f} I:{pid_gains[1]/100.0:.3f} D:{pid_gains[2]/100.0:.3f}"
                )
                print(
                    f"  {BOLD}PID Gains (Pitch):{RESET} P:{pid_gains[3]/100.0:.3f} I:{pid_gains[4]/100.0:.3f} D:{pid_gains[5]/100.0:.3f}"
                )
                print(
                    f"  {BOLD}PID Gains (Yaw):{RESET} P:{pid_gains[6]/100.0:.3f} I:{pid_gains[7]/100.0:.3f} D:{pid_gains[8]/100.0:.3f}"
                )
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed PID data):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_RAW_IMU"]:
            if response_data is not None and len(response_data) == 18:
                # Parse 3x accel + 3x gyro + 3x mag (each 2 bytes, signed int16)
                (
                    accel_x,
                    accel_y,
                    accel_z,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                    mag_x,
                    mag_y,
                    mag_z,
                ) = struct.unpack("<hhhhhhhhh", response_data)
                print(
                    f"  {BOLD}Accelerometer:{RESET} X:{accel_x:6d} Y:{accel_y:6d} Z:{accel_z:6d}"
                )
                print(
                    f"  {BOLD}Gyroscope:{RESET}     X:{gyro_x:6d} Y:{gyro_y:6d} Z:{gyro_z:6d}"
                )
                print(
                    f"  {BOLD}Magnetometer:{RESET}   X:{mag_x:6d} Y:{mag_y:6d} Z:{mag_z:6d}"
                )
            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed raw IMU):{RESET} {response_data}"
                )
        elif command_code == MSP_COMMANDS["MSP_MOTOR_CONFIG"]:
            if response_data is not None and len(response_data) >= 7:
                min_throttle = struct.unpack("<H", response_data[0:2])[0]
                max_throttle = struct.unpack("<H", response_data[2:4])[0]
                min_command = struct.unpack("<H", response_data[4:6])[0]
                motor_poles = response_data[6]

                print(f"  {BOLD}Min Throttle:{RESET} {min_throttle}")
                print(f"  {BOLD}Max Throttle:{RESET} {max_throttle}")
                print(f"  {BOLD}Min Command:{RESET} {min_command}")
                print(f"  {BOLD}Motor Poles:{RESET} {motor_poles}")

            else:
                print(
                    f"  {YELLOW}{ICON_WARNING} Decoded (unparsed motor config):{RESET} {response_data}"
                )

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
                # Test MSP_SET_SETTING as well
                time.sleep(0.1)  # Small delay
                
            else:
                send_and_receive(
                    ser, command_name, command_code, test_counts=test_results
                )

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