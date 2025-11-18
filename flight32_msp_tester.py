#!/usr/bin/env python3

"""
/**
 * @file flight32_msp_tester.py
 * @brief Flight32 MSP Communication Tester.
 * @author Wastl Kraus
 * @license MIT
 */
"""

import serial
import struct
import sys
import time
from enum import Enum
from dataclasses import dataclass


# ANSI Color Codes
class Colors:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    GRAY = "\033[90m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    WHITE = "\033[97m"


def colored(text: str, color: str = "", bold: bool = False) -> str:
    """Apply color to text"""
    if not color:
        return text
    prefix = Colors.BOLD if bold else ""
    return f"{prefix}{color}{text}{Colors.RESET}"


# Print functions
def print_header():
    """Print header"""
    print(f"\n{colored('━' * 40, Colors.BLUE)}")
    print(f"{colored('     Flight32 MSP Protocol Tester', Colors.BLUE, bold=True)}")
    print(f"{colored('━' * 40, Colors.BLUE)}\n")


def print_section(title: str):
    """Print section"""
    print(f"{colored('▸ ' + title, Colors.BLUE, bold=True)}")


def print_success(message: str, details: str = ""):
    """Print success message with optional details"""
    print(f"{colored('✓', Colors.GREEN)} {message}")
    if details:
        print(f"    {colored(details, Colors.GRAY)}")


def print_error(message: str):
    """Print error message"""
    print(f"{colored('✗', Colors.RED)} {message}")


def print_info(message: str):
    """Print info message"""
    print(f"{colored('ℹ', Colors.BLUE)} {message}")


def print_result_table(results: list):
    """Print results in a table"""
    print(f"\n{colored('Test Results', Colors.BLUE, bold=True)}")
    print(colored("─" * 50, Colors.GRAY))

    passed = sum(1 for _, result, _ in results if result)
    total = len(results)

    for test_name, result, details in results:
        status = colored("✓", Colors.GREEN) if result else colored("✗", Colors.RED)
        print(f"  {status} {test_name}")
        if details and result:  # Only show details for passed tests
            print(f"    {colored(details, Colors.GRAY)}")

    print(colored("─" * 50, Colors.GRAY))
    percent = (passed / total) * 100
    status_color = Colors.GREEN if passed == total else Colors.RED
    print(
        f"  {colored(f'{passed}/{total} passed ({percent:.0f}%)', status_color, bold=True)}"
    )


# MSP Commands
class MSPCommand(Enum):
    MSP_API_VERSION = 1
    MSP_FC_VARIANT = 2
    MSP_FC_VERSION = 3
    MSP_BOARD_INFO = 4
    MSP_BUILD_INFO = 5
    MSP_REBOOT = 6
    MSP_MEM_STATS = 8
    MSP_GET_SETTING = 9
    MSP_SET_SETTING = 10
    MSP_PID = 11
    MSP_STATUS = 101
    MSP_RAW_IMU = 102
    MSP_MOTOR = 104
    MSP_RC = 105
    MSP_ATTITUDE = 108
    MSP_BOX = 113
    MSP_UID = 160
    MSP_SENSOR_STATUS = 212
    MSP_SET_PID = 202
    MSP_GET_FILTER_CONFIG = 203
    MSP_SET_FILTER_CONFIG = 204


# Configuration
PORT = "/dev/ttyUSB0"
# PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TIMEOUT = 2.0


@dataclass
class MSPResponse:
    """Represents an MSP protocol response"""

    command: int
    payload: bytes
    is_valid: bool
    crc_error: bool = False
    parse_error: bool = False


class MSPTester:
    """MSP Protocol Tester for Flight32"""

    def __init__(self, port: str, baud_rate: int = 115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.in_msp_mode = False
        self.test_results = []
        self.fw_version = None

    def connect(self) -> bool:
        """Connect to the serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port, baudrate=self.baud_rate, timeout=TIMEOUT
            )
            print_success(f"Connected to {self.port} @ {self.baud_rate} baud")
            # Give the FC a moment to be ready after connection
            time.sleep(1.5)
            self.serial.reset_input_buffer()
            return True
        except serial.SerialException as e:
            print_error(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial:
            self.serial.close()
            print_success("Disconnected")

    def exit_msp_mode(self) -> bool:
        """Exit MSP mode by waiting for timeout"""
        print_section("Exit - Returning to Terminal Mode")
        print_info("Waiting for MSP timeout (2 seconds)...")
        time.sleep(2.1)
        self.in_msp_mode = False
        print_success("Terminal mode restored")
        return True

    def send_msp_command(self, cmd: int, payload: bytes = b"") -> MSPResponse:
        """Send an MSP command and receive response"""
        if not self.serial:
            return MSPResponse(cmd, b"", False, parse_error=True)

        try:
            self.serial.reset_input_buffer()
            time.sleep(0.05)  # Give FC a moment to send any buffered output

            size = len(payload)
            crc = size ^ cmd
            for byte in payload:
                crc ^= byte

            request = b"$M<" + bytes([size, cmd]) + payload + bytes([crc])
            try:
                cmd_name = MSPCommand(cmd).name
            except ValueError:
                cmd_name = f"0x{cmd:02X}"
            print_info(
                f"Sending MSP command {cmd_name} (0x{cmd:02X}) with payload size {size}. Request: {request.hex()}"
            )
            self.serial.write(request)
            time.sleep(0.02)  # Small delay to allow FC to process and respond
            response = self._read_msp_response()
            return response
        except Exception as e:
            print_error(f"Error sending MSP command {cmd}: {e}")
            return MSPResponse(cmd, b"", False, parse_error=True)

    def _read_msp_response(self) -> MSPResponse:
        """Read and parse MSP response"""
        if not self.serial:
            return MSPResponse(0, b"", False, parse_error=True)

        try:
            # Wait for the '$M>' header
            header_buffer = b""
            start_time = time.time()
            while b"$M>" not in header_buffer:
                byte = self.serial.read(1)
                if not byte:
                    if (time.time() - start_time) > TIMEOUT:
                        print_error("Timeout waiting for $M> header.")
                        return MSPResponse(0, b"", False, parse_error=True)  # Timeout
                    continue
                header_buffer += byte
                if len(header_buffer) > 10:  # Prevent buffer from growing indefinitely
                    header_buffer = header_buffer[-10:]

            # Once '$M>' is found, discard everything before it
            header_buffer = header_buffer[header_buffer.find(b"$M>") :]

            # Read size and command
            while len(header_buffer) < 5:  # Need $M> + size + cmd
                byte = self.serial.read(1)
                if not byte:
                    if (time.time() - start_time) > TIMEOUT:
                        print_error("Timeout waiting for size and command bytes.")
                        return MSPResponse(0, b"", False, parse_error=True)  # Timeout
                    continue
                header_buffer += byte

            size = header_buffer[3]
            cmd = header_buffer[4]

            # Read payload and CRC
            expected_total_len = 5 + size + 1  # $M> + size + cmd + payload + crc
            while len(header_buffer) < expected_total_len:
                byte = self.serial.read(1)
                if not byte:
                    if (time.time() - start_time) > TIMEOUT:
                        print_error(
                            f"Timeout waiting for payload and CRC for command {cmd}."
                        )
                        return MSPResponse(0, b"", False, parse_error=True)  # Timeout
                    continue
                header_buffer += byte

            # Extract components
            payload = header_buffer[5 : 5 + size]
            crc_byte = header_buffer[5 + size]

            # Calculate CRC (size ^ cmd ^ payload bytes) - match send side
            crc = size ^ cmd
            for byte in payload:
                crc ^= byte

            is_valid = crc == crc_byte
            try:
                cmd_name = MSPCommand(cmd).name
            except ValueError:
                cmd_name = f"0x{cmd:02X}"
            print_info(
                f"Received MSP response for command {cmd_name} (0x{cmd:02X}). Raw: {header_buffer.hex()}"
            )
            print_info(
                f"  Payload size: {size}, Valid: {is_valid}, CRC Error: {not is_valid}"
            )

            return MSPResponse(cmd, payload, is_valid, crc_error=(not is_valid))
        except Exception as e:
            print_error(f"Error reading MSP response: {e}")
            return MSPResponse(0, b"", False, parse_error=True)

    # Test methods
    def test_api_version(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_API_VERSION.value)
        if not response.is_valid or len(response.payload) < 3:
            self.test_results.append(("API Version", False, ""))
            return False

        msp_version = response.payload[0]
        protocol_version = response.payload[1]
        api_version_major = response.payload[2]
        details = f"MSP Version: {msp_version}, Protocol: {protocol_version}, API Major: {api_version_major}"
        self.test_results.append(("API Version", True, details))
        return True

    def test_fc_variant(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_FC_VARIANT.value)
        if not response.is_valid:
            self.test_results.append(("FC Variant", False, ""))
            return False

        fc_variant = response.payload.decode("ascii", errors="ignore").strip()
        details = f"FC Variant: {fc_variant}"
        self.test_results.append(("FC Variant", True, details))
        return True

    def test_fc_version(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_FC_VERSION.value)
        if not response.is_valid or len(response.payload) < 3:
            self.test_results.append(("FC Version", False, ""))
            return False
        major, minor, patch = response.payload[:3]
        self.fw_version = f"v{major}.{minor}.{patch}"
        details = f"Firmware Version: {self.fw_version}"
        self.test_results.append(("FC Version", True, details))
        return True

    def test_board_info(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_BOARD_INFO.value)
        if not response.is_valid:
            self.test_results.append(("Board Info", False, ""))
            return False
        # Payload often contains multiple NUL-terminated text fields and unused bytes.
        raw = response.payload
        parts = raw.split(b"\x00")
        fields = []
        for p in parts:
            try:
                s = p.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            # Remove non-printable characters
            s = "".join(ch for ch in s if ch.isprintable())
            if not s:
                continue
            # Truncate each field to reasonable length
            if len(s) > 120:
                s = s[:117] + "..."
            fields.append(s)

        if not fields:
            details = f"Board Raw: {raw.hex()}"
        else:
            # Format as clean list (no numbering) and indent fields for correct display
            details = "Board Fields:\n    " + "\n    ".join(fields)
        self.test_results.append(("Board Info", True, details))
        return True

    def test_raw_imu(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_RAW_IMU.value)
        if not response.is_valid or len(response.payload) < 18:
            self.test_results.append(("Raw IMU", False, ""))
            return False

        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = struct.unpack(
            "<hhhhhhhhh", response.payload[:18]
        )
        details = (
            f"Acc: ({acc_x}, {acc_y}, {acc_z}), Gyro: ({gyro_x}, {gyro_y}, {gyro_z}), Mag: ({mag_x}, {mag_y}, {mag_z})"
        )
        self.test_results.append(("Raw IMU", True, details))
        return True

    def test_rc_channels(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_RC.value)
        if (
            not response.is_valid or len(response.payload) < 16
        ):  # Assuming 8 channels * 2 bytes
            self.test_results.append(("RC Channels", False, ""))
            return False

        rc_channels = struct.unpack(
            "<HHHHHHHH", response.payload[:16]
        )  # 8 unsigned shorts
        details = f"RC Channels: {rc_channels}"
        self.test_results.append(("RC Channels", True, details))
        return True

    def test_attitude(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_ATTITUDE.value)
        if not response.is_valid or len(response.payload) < 6:
            self.test_results.append(("Attitude", False, ""))
            return False

        roll, pitch, yaw = struct.unpack("<hhh", response.payload[:6])
        details = f"Roll: {roll/10.0:.1f}°, Pitch: {pitch/10.0:.1f}°, Yaw: {yaw:.0f}°"
        self.test_results.append(("Attitude", True, details))
        return True

    def test_motor_output(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_MOTOR.value)
        if (
            not response.is_valid or len(response.payload) < 8
        ):  # Assuming 4 motors * 2 bytes
            self.test_results.append(("Motor Output", False, ""))
            return False

        motor_values = struct.unpack("<HHHH", response.payload[:8])  # 4 unsigned shorts
        details = f"Motors: {motor_values}"
        self.test_results.append(("Motor Output", True, details))
        return True

    def test_msp_uid(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_UID.value)
        if not response.is_valid or len(response.payload) < 12:
            self.test_results.append(("MSP UID", False, ""))
            return False

        uid = response.payload.hex()
        details = f"UID: {uid}"
        self.test_results.append(("MSP UID", True, details))
        return True

    def test_msp_status(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_STATUS.value)
        if not response.is_valid or len(response.payload) < 11:
            self.test_results.append(("MSP Status", False, ""))
            return False

        cycle_time, i2c_errors, sensors, flight_mode_flags, profile = struct.unpack(
            "<HHHIB", response.payload
        )
        details = f"Cycle Time: {cycle_time}μs, I2C Errors: {i2c_errors}, Sensors: {sensors}, Flight Modes: {flight_mode_flags}, Profile: {profile}"
        self.test_results.append(("MSP Status", True, details))
        return True

    def test_msp_sensor_status(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_SENSOR_STATUS.value)
        if not response.is_valid or len(response.payload) < 12:
            self.test_results.append(("MSP Sensor Status", False, ""))
            return False

        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = struct.unpack(
            "<hhhhhh", response.payload
        )
        details = (
            f"Acc: ({acc_x}, {acc_y}, {acc_z}), Gyro: ({gyro_x}, {gyro_y}, {gyro_z})"
        )
        self.test_results.append(("MSP Sensor Status", True, details))
        return True

    def run_all_tests(self) -> bool:
        """Run all tests"""
        if not self.connect():
            return False

        print_section("Running MSP Tests")
        try:
            tests = [
                self.test_api_version,
                self.test_fc_variant,
                self.test_fc_version,
                self.test_board_info,
                self.test_raw_imu,
                self.test_rc_channels,
                self.test_attitude,
                self.test_motor_output,
                self.test_msp_uid,
                self.test_msp_status,
                self.test_msp_sensor_status,
            ]

            for test_func in tests:
                test_func()
        finally:
            self.disconnect()

        self._print_summary()
        return all(result for _, result, _ in self.test_results)
    
    def run_single_test(self, test_func_name: str) -> bool:
        """Run a single test by command name"""
        if not self.connect():
            return False

        print_section(f"Running MSP Test for {test_func_name}")
        try:
            # Construct the full test function name, e.g., 'test_msp_uid'
            test_func = getattr(self, test_func_name, None)
            if test_func:
                test_func()
            else:
                print_error(f"Test function '{test_func_name}' not found.")
        finally:
            self.disconnect()

        self._print_summary()
        # Check if any test was actually run and passed
        if not self.test_results:
            return False
        return all(result for _, result, _ in self.test_results)

    def _print_summary(self):
        """Print test summary"""
        if not self.test_results:
            return

        passed = sum(1 for _, result, _ in self.test_results if result)
        total = len(self.test_results)

        print_result_table(self.test_results)


        print()
        if self.fw_version:
            print(
                f"{colored('  Firmware Version:', Colors.GREEN)} {colored(self.fw_version, Colors.WHITE, bold=True)}"
            )
            print()

        if passed == total:
            print(
                colored(
                    "✓ All tests PASSED - MSP is working correctly!",
                    Colors.GREEN,
                    bold=True,
                )
            )
        else:
            print(
                colored(
                    f"✗ {total - passed} test(s) FAILED - Check firmware!",
                    Colors.RED,
                    bold=True,
                )
            )
        print()


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="Flight32 MSP Tester")
    parser.add_argument(
        "--device",
        type=str,
        default=PORT,
        help=f"Serial device path (default: {PORT})",
    )
    parser.add_argument("--command", type=str, help="Run a single test by MSP command name (e.g., MSP_UID)")
    args = parser.parse_args()

    try:
        print_header()
        tester = MSPTester(args.device, BAUD_RATE)
        if args.command:
            # The test function name is expected to be in the format 'test_msp_...'
            # The command name from the CLI should be like 'MSP_UID'
            test_name = f"test_{args.command.lower()}"
            success = tester.run_single_test(test_name)
        else:
            success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print(f"\n{colored('✗ Test aborted by user', Colors.RED)}")
        sys.exit(1)


if __name__ == "__main__":
    main()
