#!/usr/bin/env python3
"""
Flight32 MSP Communication Tester
Tests the corrected MSP protocol implementation on Flight32 firmware.
"""

import serial
import struct
import sys
import time
from enum import Enum
from dataclasses import dataclass

# ANSI Color Codes
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    GRAY = '\033[90m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    WHITE = '\033[97m'

def colored(text: str, color: str = "", bold: bool = False) -> str:
    """Apply color to text"""
    if not color:
        return text
    prefix = Colors.BOLD if bold else ""
    return f"{prefix}{color}{text}{Colors.RESET}"

# Print functions
def print_header():
    """Print header"""
    print(f"\n{colored('━' * 60, Colors.BLUE)}")
    print(f"{colored('Flight32 MSP Protocol Tester v0.1', Colors.BLUE, bold=True)}")
    print(f"{colored('━' * 60, Colors.BLUE)}\n")

def print_section(title: str):
    """Print section"""
    print(f"{colored('▸ ' + title, Colors.BLUE, bold=True)}")

def print_success(message: str):
    """Print success message"""
    print(f"{colored('✓', Colors.GREEN)} {message}")

def print_error(message: str):
    """Print error message"""
    print(f"{colored('✗', Colors.RED)} {message}")

def print_info(message: str):
    """Print info message"""
    print(f"{colored('ℹ', Colors.BLUE)} {message}")

def print_result_table(results: list):
    """Print results in a table"""
    print(f"\n{colored('Test Results', Colors.BLUE, bold=True)}")
    print(colored('─' * 50, Colors.GRAY))
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = colored("✓", Colors.GREEN) if result else colored("✗", Colors.RED)
        print(f"  {status} {test_name}")
    
    print(colored('─' * 50, Colors.GRAY))
    percent = (passed / total) * 100
    status_color = Colors.GREEN if passed == total else Colors.RED
    print(f"  {colored(f'{passed}/{total} passed ({percent:.0f}%)', status_color, bold=True)}")

# MSP Commands
class MSPCommand(Enum):
    MSP_API_VERSION = 1
    MSP_FC_VARIANT = 2
    MSP_FC_VERSION = 3
    MSP_BOARD_INFO = 4
    MSP_BUILD_INFO = 5
    MSP_REBOOT = 6
    MSP_STATUS = 7
    MSP_MEM_STATS = 8
    MSP_GET_SETTING = 9
    MSP_SET_SETTING = 10
    MSP_PID = 11
    MSP_RAW_IMU = 102
    MSP_ATTITUDE = 108
    MSP_RC = 105
    MSP_MOTOR = 104
    MSP_SET_PID = 202

# Configuration
PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
TIMEOUT = 2.0
MSP_HANDSHAKE = b"$RMSP"

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
                port=self.port,
                baudrate=self.baud_rate,
                timeout=TIMEOUT
            )
            print_success(f"Connected to {self.port} @ {self.baud_rate} baud")
            time.sleep(1)
            return True
        except serial.SerialException as e:
            print_error(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial:
            self.serial.close()
            print_success("Disconnected")
    
    def enter_msp_mode(self) -> bool:
        """Enter MSP mode by sending handshake"""
        print_section("Handshake - Entering MSP Mode")
        try:
            self.serial.reset_input_buffer()
            self.serial.write(MSP_HANDSHAKE)
            print_info(f"Sending handshake: {MSP_HANDSHAKE}")
            
            response = self.serial.read_until(b'\n', size=100)
            if b"MSP mode" in response:
                response_text = response.decode(errors='ignore').strip()
                print_success(response_text)
                self.in_msp_mode = True
                return True
            else:
                print_error(f"Unexpected response: {response}")
                return False
        except Exception as e:
            print_error(f"Handshake failed: {e}")
            return False
    
    def exit_msp_mode(self) -> bool:
        """Exit MSP mode by waiting for timeout"""
        print_section("Exit - Returning to Terminal Mode")
        print_info("Waiting for MSP timeout (2 seconds)...")
        time.sleep(2.1)
        self.in_msp_mode = False
        print_success("Terminal mode restored")
        return True
    
    def send_msp_command(self, cmd: int, payload: bytes = b'') -> MSPResponse:
        """Send an MSP command and receive response"""
        try:
            size = len(payload)
            crc = size ^ cmd
            for byte in payload:
                crc ^= byte
            
            request = b'$M' + bytes([size, cmd]) + payload + bytes([crc])
            self.serial.write(request)
            response = self._read_msp_response()
            return response
        except Exception:
            return MSPResponse(cmd, b'', False, parse_error=True)
    
    def _read_msp_response(self) -> MSPResponse:
        """Read and parse MSP response"""
        try:
            header = self.serial.read(2)
            if header != b'$M':
                return MSPResponse(0, b'', False, parse_error=True)
            
            size_cmd = self.serial.read(2)
            if len(size_cmd) < 2:
                return MSPResponse(0, b'', False, parse_error=True)
            
            size = size_cmd[0]
            cmd = size_cmd[1]
            
            payload = self.serial.read(size) if size > 0 else b''
            crc_byte = self.serial.read(1)
            if len(crc_byte) < 1:
                return MSPResponse(cmd, payload, False, parse_error=True)
            
            crc = size ^ cmd
            for byte in payload:
                crc ^= byte
            
            is_valid = (crc == crc_byte[0])
            return MSPResponse(cmd, payload, is_valid, crc_error=(not is_valid))
        except Exception:
            return MSPResponse(0, b'', False, parse_error=True)
    
    # Test methods
    def test_api_version(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_API_VERSION.value)
        if not response.is_valid or len(response.payload) < 3:
            self.test_results.append(("API Version", False))
            return False
        self.test_results.append(("API Version", True))
        return True
    
    def test_fc_variant(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_FC_VARIANT.value)
        if not response.is_valid:
            self.test_results.append(("FC Variant", False))
            return False
        self.test_results.append(("FC Variant", True))
        return True
    
    def test_fc_version(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_FC_VERSION.value)
        if not response.is_valid or len(response.payload) < 3:
            self.test_results.append(("FC Version", False))
            return False
        major, minor, patch = response.payload[:3]
        self.fw_version = f"v{major}.{minor}.{patch}"
        self.test_results.append(("FC Version", True))
        return True
    
    def test_board_info(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_BOARD_INFO.value)
        if not response.is_valid:
            self.test_results.append(("Board Info", False))
            return False
        self.test_results.append(("Board Info", True))
        return True
    
    def test_mem_stats(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_MEM_STATS.value)
        if not response.is_valid or len(response.payload) < 4:
            self.test_results.append(("Memory Stats", False))
            return False
        self.test_results.append(("Memory Stats", True))
        return True
    
    def test_raw_imu(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_RAW_IMU.value)
        if not response.is_valid or len(response.payload) < 18:
            self.test_results.append(("Raw IMU (CRITICAL)", False))
            return False
        self.test_results.append(("Raw IMU (CRITICAL)", True))
        return True
    
    def test_rc_channels(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_RC.value)
        if not response.is_valid or len(response.payload) < 16:
            self.test_results.append(("RC Channels", False))
            return False
        self.test_results.append(("RC Channels", True))
        return True
    
    def test_attitude(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_ATTITUDE.value)
        if not response.is_valid or len(response.payload) < 6:
            self.test_results.append(("Attitude", False))
            return False
        self.test_results.append(("Attitude", True))
        return True
    
    def test_motor_output(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_MOTOR.value)
        if not response.is_valid or len(response.payload) < 8:
            self.test_results.append(("Motor Output", False))
            return False
        self.test_results.append(("Motor Output", True))
        return True
    
    def test_pid_get(self) -> bool:
        response = self.send_msp_command(MSPCommand.MSP_PID.value)
        if not response.is_valid:
            self.test_results.append(("PID Get (CRITICAL)", False))
            return False
        if len(response.payload) == 18:
            self.test_results.append(("PID Get (CRITICAL)", True))
            return True
        else:
            self.test_results.append(("PID Get (CRITICAL)", False))
            return False
    
    def run_all_tests(self) -> bool:
        """Run all tests"""
        if not self.connect():
            return False
        
        if not self.enter_msp_mode():
            self.disconnect()
            return False
        
        try:
            tests = [
                self.test_api_version,
                self.test_fc_variant,
                self.test_fc_version,
                self.test_board_info,
                self.test_mem_stats,
                self.test_raw_imu,
                self.test_rc_channels,
                self.test_attitude,
                self.test_motor_output,
                self.test_pid_get,
            ]
            
            for test_func in tests:
                test_func()
        finally:
            self.exit_msp_mode()
            self.disconnect()
        
        self._print_summary()
        return all(result for _, result in self.test_results)
    
    def _print_summary(self):
        """Print test summary"""
        passed = sum(1 for _, result in self.test_results if result)
        total = len(self.test_results)
        
        print_result_table(self.test_results)
        
        print()
        if self.fw_version:
            print(f"{colored('Firmware Version:', Colors.BLUE)} {colored(self.fw_version, Colors.WHITE, bold=True)}")
        
        if passed == total:
            print(colored("✓ All tests PASSED - MSP is working correctly!", Colors.GREEN, bold=True))
        else:
            print(colored(f"✗ {total - passed} test(s) FAILED - Check firmware!", Colors.RED, bold=True))
        print()

def main():
    """Main entry point"""
    try:
        print_header()
        tester = MSPTester(PORT, BAUD_RATE)
        success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print(f"\n{colored('✗ Test aborted by user', Colors.RED)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
    