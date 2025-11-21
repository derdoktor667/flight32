/**
 * @file serial_manager_task.cpp
 * @brief Implements the SerialManagerTask class for managing serial communication.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "serial_manager_task.h"
#include "../utils/version_info.h"
#include <cmath>
#include "../utils/math_constants.h"
#include "imu_task.h"
#include "../config/terminal_config.h"
#include "../config/serial_config.h"
#include "../config/filter_config.h"
#include "../config/com_manager_config.h"
#include "../config/imu_config.h"
#include "../config/motor_config.h"
#include "../config/settings_config.h"
#include "../config/rx_config.h"
#include <cstring>
#include <cstdio>



SerialManagerTask::SerialManagerTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, FlightController* flightController, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms),
      _scheduler(scheduler),
      _flightController(flightController),
      _imu_task(imu_task),
      _rx_task(rx_task),
      _motor_task(motor_task),
      _pid_task(pid_task),
      _settings_manager(settings_manager)
{
    _terminal = std::make_unique<Terminal>(_scheduler, _imu_task, _rx_task, _motor_task, _pid_task, _settings_manager);
    _msp_processor = std::make_unique<MspProcessor>(_flightController, _pid_task, _imu_task, _rx_task, _motor_task, _settings_manager);
}

void SerialManagerTask::setup()
{
    // No setup needed for _terminal_input_buffer as it's now handled by Terminal class
}

void SerialManagerTask::run()
{
    while (Serial.available() > 0)
    {
        uint8_t c = Serial.read();
        // Always attempt to parse as MSP first
        bool consumed_by_msp = _parse_msp_char(c);

        // If not consumed by MSP and in terminal mode, send to terminal
        if (!consumed_by_msp && _current_mode == ComSerialMode::TERMINAL)
        {
            _terminal->handleInput(c);
            _should_show_prompt = true; // Allow prompt in terminal mode
        }
    }

    // MSP Timeout logic - applies only if currently in MSP mode
    if (_current_mode == ComSerialMode::MSP && (millis() - _last_msp_activity_ms > MSP_TIMEOUT_MS))
    {
        _current_mode = ComSerialMode::TERMINAL;
        com_set_serial_mode(ComSerialMode::TERMINAL);

        // Reset MSP parser state
        _msp_state = MspState::IDLE;
        _should_show_prompt = true; // Re-enable prompt when switching back to terminal

        // Clear any partial input from the terminal buffer
        _terminal->clearInputBuffer();
    }

    // If in terminal mode and should quit, signal scheduler
    if (_current_mode == ComSerialMode::TERMINAL && _terminal->shouldQuit())
    {
        _should_show_prompt = false; // Prevent prompt when quitting
    }
}

void SerialManagerTask::showPrompt()
{
    if (_should_show_prompt)
    {
        _terminal->showPrompt();
    }
}

// MSP Mode Functions
bool SerialManagerTask::_parse_msp_char(uint8_t c)
{
    // MspState prev_state = _msp_state; // For debugging if needed
    bool consumed_by_msp = true;

    _last_msp_activity_ms = millis(); // Update activity on any MSP parsing attempt

    switch (_msp_state)
    {
    case MspState::IDLE:
        _msp_message_length_received = 0; // Reset payload counter
        _msp_command_id = 0;              // Reset command ID
        _msp_flag_v2 = 0;                 // Reset V2 flag
        _msp_message_length_expected = 0; // Reset expected length
        _msp_protocol_version = 0;        // Reset protocol version
        _msp_crc = 0;                     // Reset CRC
        _msp_payload_index = 0;           // Reset payload index

        if (c == '$')
        {
            _msp_state = MspState::PROTO_IDENTIFIER;
        }
        else
        {
            consumed_by_msp = false; // Not an MSP start character
        }
        break;

    case MspState::PROTO_IDENTIFIER:
        if (c == 'M')
        {
            _msp_protocol_version = 1;
            _msp_state = MspState::DIRECTION_V1;
            _msp_crc = 0; // V1 CRC initialization
        }
        else if (c == 'X')
        {
            _msp_protocol_version = 2;
            _msp_state = MspState::DIRECTION_V2;
            // V2 CRC is calculated over the entire frame (excluding $X and CRC byte itself)
            // It starts with the flag byte.
            _msp_crc = 0; // Reset CRC, will be calculated from flag
        }
        else
        {
            _msp_state = MspState::IDLE; // Unknown protocol, reset
            consumed_by_msp = false;
        }
        break;

    case MspState::DIRECTION_V1:
    case MspState::DIRECTION_V2:
        if (c == '<') // Incoming command
        {
            if (_current_mode == ComSerialMode::TERMINAL)
            {
                _current_mode = ComSerialMode::MSP;
                com_set_serial_mode(ComSerialMode::MSP);
                _should_show_prompt = false; // Disable prompt when entering MSP mode
            }

            if (_msp_protocol_version == 1)
            {
                _msp_state = MspState::PAYLOAD_LENGTH_V1;
            }
            else // MSP v2
            {
                _msp_state = MspState::FLAG_V2;
            }
        }
        else
        {
            _msp_state = MspState::IDLE; // Not an incoming command, reset
            consumed_by_msp = false;
        }
        break;

    case MspState::FLAG_V2:
        _msp_flag_v2 = c;
        _msp_crc = _crc8_dvb_s2(_msp_crc, _msp_flag_v2); // Add flag to CRC
        _msp_state = MspState::CODE_V2_LOW;
        break;

    case MspState::PAYLOAD_LENGTH_V1: // MSP v1 payload length (1 byte)
        _msp_message_length_expected = c;
        if (_msp_message_length_expected == 255)
        { // Jumbo frame
            _msp_state = MspState::CODE_JUMBO_V1;
        }
        else
        {
            _msp_crc ^= _msp_message_length_expected; // Add length to V1 CRC
            _msp_state = MspState::CODE_V1;
        }
        break;

    case MspState::CODE_V1: // MSP v1 command ID (1 byte)
        _msp_command_id = c;
        _msp_crc ^= _msp_command_id; // Add command ID to V1 CRC
        _msp_payload_index = 0;

        if (_msp_message_length_expected > 0)
        {
            _msp_state = MspState::PAYLOAD_V1;
        }
        else
        {
            _msp_state = MspState::CHECKSUM_V1; // No payload, go straight to checksum
        }
        break;

    case MspState::CODE_JUMBO_V1: // MSP v1 Jumbo Frame, read first byte of command ID
        _msp_command_id = c;
        _msp_crc ^= _msp_command_id; // Add command ID to V1 CRC
        _msp_state = MspState::PAYLOAD_LENGTH_JUMBO_LOW;
        break;

    case MspState::PAYLOAD_LENGTH_JUMBO_LOW: // MSP v1 Jumbo frame payload length (low byte)
        _msp_message_length_expected = c;
        _msp_crc ^= c; // Add length to V1 CRC
        _msp_state = MspState::PAYLOAD_LENGTH_JUMBO_HIGH;
        break;

    case MspState::PAYLOAD_LENGTH_JUMBO_HIGH: // MSP v1 Jumbo frame payload length (high byte)
        _msp_message_length_expected |= (c << 8);
        _msp_crc ^= c; // Add length to V1 CRC
        _msp_payload_index = 0;
        _msp_state = MspState::PAYLOAD_V1;
        break;

    case MspState::CODE_V2_LOW: // MSP v2 command ID (low byte)
        _msp_command_id = c;
        _msp_crc = _crc8_dvb_s2(_msp_crc, c); // Add to V2 CRC
        _msp_state = MspState::CODE_V2_HIGH;
        break;

    case MspState::CODE_V2_HIGH: // MSP v2 command ID (high byte)
        _msp_command_id |= (c << 8);
        _msp_crc = _crc8_dvb_s2(_msp_crc, c); // Add to V2 CRC
        _msp_state = MspState::PAYLOAD_LENGTH_V2_LOW;
        break;

    case MspState::PAYLOAD_LENGTH_V2_LOW: // MSP v2 payload length (low byte)
        _msp_message_length_expected = c;
        _msp_crc = _crc8_dvb_s2(_msp_crc, c); // Add to V2 CRC
        _msp_state = MspState::PAYLOAD_LENGTH_V2_HIGH;
        break;

    case MspState::PAYLOAD_LENGTH_V2_HIGH: // MSP v2 payload length (high byte)
        _msp_message_length_expected |= (c << 8);
        _msp_crc = _crc8_dvb_s2(_msp_crc, c); // Add to V2 CRC
        _msp_payload_index = 0;

        if (_msp_message_length_expected > 0)
        {
            _msp_state = MspState::PAYLOAD_V2;
        }
        else
        {
            _msp_state = MspState::CHECKSUM_V2; // No payload, go straight to checksum
        }
        break;

    case MspState::PAYLOAD_V1:
    case MspState::PAYLOAD_V2:
        if (_msp_payload_index < MSP_MAX_PAYLOAD_SIZE)
        { // Prevent buffer overflow
            _msp_payload_buffer[_msp_payload_index++] = c;
            if (_msp_protocol_version == 1)
            {
                _msp_crc ^= c; // Add to V1 CRC
            }
            else
            {
                _msp_crc = _crc8_dvb_s2(_msp_crc, c); // Add to V2 CRC
            }
            
            if (_msp_payload_index >= _msp_message_length_expected)
            {
                if (_msp_protocol_version == 1)
                {
                    _msp_state = MspState::CHECKSUM_V1;
                }
                else
                {
                    _msp_state = MspState::CHECKSUM_V2;
                }
            }
        }
        else
        {
            // Payload too large, reset state to avoid buffer overflow
            // com_send_log(ComMessageType::LOG_ERROR, "MSP Payload too large!");
            _msp_state = MspState::IDLE;
        }
        break;

    case MspState::CHECKSUM_V1:
        if (_msp_crc == c)
        {
            _process_msp_message();
        }
        else
        {
            // com_send_log(ComMessageType::LOG_ERROR, "MSP V1 CRC error! Exp: %02X, Got: %02X", _msp_crc, c);
        }
        _msp_state = MspState::IDLE;
        break;

    case MspState::CHECKSUM_V2:
        if (_msp_crc == c)
        {
            _process_msp_message();
        }
        else
        {
            // com_send_log(ComMessageType::LOG_ERROR, "MSP V2 CRC error! Exp: %02X, Got: %02X", _msp_crc, c);
        }
        _msp_state = MspState::IDLE;
        break;

    default:
        _msp_state = MspState::IDLE; // Should not happen
        break;
    }
    return consumed_by_msp;
}

void SerialManagerTask::_process_msp_message()
{
    // Delegate the MSP command processing to the MspProcessor instance
    if (_msp_processor) {
        _msp_processor->process_msp_command(_msp_command_id, _msp_payload_buffer, _msp_message_length_received, _msp_protocol_version);
    }
}



uint8_t SerialManagerTask::_crc8_dvb_s2(uint8_t crc, uint8_t ch) {
    crc ^= ch;
    for (uint8_t ii = 0; ii < 8; ii++) {
        if (crc & 0x80) {
            crc = ((crc << 1) & 0xFF) ^ 0xD5;
        } else {
            crc = (crc << 1) & 0xFF;
        }
    }
    return crc;
}
