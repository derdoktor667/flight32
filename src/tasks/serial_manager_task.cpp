/**
 * @file serial_manager_task.cpp
 * @brief Implements the SerialManagerTask class for managing serial communication.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "serial_manager_task.h"
#include "../flight_controller.h" // Add this include
#include "../protocols/serial_4way_protocol.h"



#include "imu_task.h"








#include <cstring>

#include "../protocols/msp_protocol.h" // Added for MSP_ATTITUDE and MSP_MAX_BYTE_VALUE



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
    _terminal = std::make_unique<Terminal>(_scheduler, _flightController, _imu_task, _rx_task, _motor_task, _pid_task, _settings_manager);
    _msp_processor = std::make_unique<MspProcessor>(_flightController, _pid_task, _imu_task, _rx_task, _motor_task, _settings_manager);
}

void SerialManagerTask::setup()
{
    // No setup needed for _terminal_input_buffer as it's now handled by Terminal class
}

void SerialManagerTask::run()
{
    if (_inPassthroughMode)
    {
        while (Serial.available() > 0)
        {
            uint8_t c = Serial.read();
            _parse_passthrough_char(c);
        }
        return; // Exit early as we are in passthrough mode
    }

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

void SerialManagerTask::enterPassthroughMode()
{
    _inPassthroughMode = true;
    _current_mode = ComSerialMode::PASSTHROUGH;
    com_set_serial_mode(ComSerialMode::PASSTHROUGH);
    _should_show_prompt = false; // Disable prompt in passthrough mode
    com_send_log(ComMessageType::LOG_INFO, "SerialManagerTask: Entered passthrough mode.");
}

void SerialManagerTask::exitPassthroughMode()
{
    _inPassthroughMode = false;
    _current_mode = ComSerialMode::TERMINAL;
    com_set_serial_mode(ComSerialMode::TERMINAL);
    _should_show_prompt = true; // Re-enable prompt
    _msp_state = MspState::IDLE; // Reset MSP parser state
    _terminal->clearInputBuffer(); // Clear any partial input from the terminal buffer
    com_send_log(ComMessageType::LOG_INFO, "SerialManagerTask: Exited passthrough mode.");
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
        if (_msp_message_length_expected == MSP_MAX_BYTE_VALUE)
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

void SerialManagerTask::_parse_passthrough_char(uint8_t c)
{
    switch (_passthrough_state)
    {
    case PassthroughParserState::IDLE:
        if (c == cmd_Local_Escape)
        {
            _passthrough_crc_calculated = 0;
            _passthrough_crc_calculated = _crc_xmodem_update(_passthrough_crc_calculated, c);
            _passthrough_state = PassthroughParserState::COMMAND;
        }
        break;
    case PassthroughParserState::COMMAND:
        _passthrough_cmd = c;
        _passthrough_crc_calculated = _crc_xmodem_update(_passthrough_crc_calculated, c);
        _passthrough_state = PassthroughParserState::ADDR_H;
        break;
    case PassthroughParserState::ADDR_H:
        _passthrough_addr = (uint16_t)c << 8;
        _passthrough_crc_calculated = _crc_xmodem_update(_passthrough_crc_calculated, c);
        _passthrough_state = PassthroughParserState::ADDR_L;
        break;
    case PassthroughParserState::ADDR_L:
        _passthrough_addr |= c;
        _passthrough_crc_calculated = _crc_xmodem_update(_passthrough_crc_calculated, c);
        _passthrough_state = PassthroughParserState::PARAM_LEN;
        break;
    case PassthroughParserState::PARAM_LEN:
        _passthrough_param_len = c;
        _passthrough_crc_calculated = _crc_xmodem_update(_passthrough_crc_calculated, c);
        _passthrough_param_index = 0;
        if (_passthrough_param_len > 0)
        {
            _passthrough_state = PassthroughParserState::PARAMS;
        }
        else
        {
            _passthrough_state = PassthroughParserState::CRC_H;
        }
        break;
    case PassthroughParserState::PARAMS:
        if (_passthrough_param_index < sizeof(_passthrough_params))
        {
            _passthrough_params[_passthrough_param_index++] = c;
            _passthrough_crc_calculated = _crc_xmodem_update(_passthrough_crc_calculated, c);
        }
        if (_passthrough_param_index >= _passthrough_param_len)
        {
            _passthrough_state = PassthroughParserState::CRC_H;
        }
        break;
    case PassthroughParserState::CRC_H:
        _passthrough_crc_expected = (uint16_t)c << 8;
        _passthrough_state = PassthroughParserState::CRC_L;
        break;
    case PassthroughParserState::CRC_L:
        _passthrough_crc_expected |= c;
        if (_passthrough_crc_expected == _passthrough_crc_calculated)
        {
            _process_passthrough_message();
        }
        _passthrough_state = PassthroughParserState::IDLE;
        break;
    }
}

void SerialManagerTask::_process_passthrough_message()
{
    uint8_t ack = ACK_OK;
    uint8_t o_param_len = 0;
    uint8_t o_param[PASSTHROUGH_BUFFER_SIZE];

    switch (_passthrough_cmd)
    {
    case cmd_InterfaceTestAlive:
        // For now, always return OK
        break;
    case cmd_ProtocolGetVersion:
        o_param[0] = MSP_ATTITUDE; // Use MSP_ATTITUDE constant
        o_param_len = 1;
        break;
    case cmd_InterfaceGetName:
    {
        const char *name = "FL32";
        o_param_len = strlen(name);
        memcpy(o_param, name, o_param_len);
        break;
    }
    case cmd_InterfaceGetVersion:
        o_param[0] = 0; // SERIAL_4WAY_VERSION_HI
        o_param[1] = 1; // SERIAL_4WAY_VERSION_LO
        o_param_len = 2;
        break;
    case cmd_InterfaceExit:
        if (_flightController)
        {
            // The exit from passthrough mode is handled by the flight controller
            // which will resume the tasks.
            _flightController->exitEscPassthrough();
        }
        break;
    case cmd_DeviceRead:
        if (_motor_task)
        {
            ack = _motor_task->passthroughRead(_passthrough_params[0], _passthrough_addr, o_param, _passthrough_param_len);
        }
        else
        {
            ack = ACK_D_GENERAL_ERROR;
        }
        break;
    case cmd_DeviceWrite:
        if (_motor_task)
        {
            ack = _motor_task->passthroughWrite(_passthrough_params[0], _passthrough_addr, _passthrough_params + 1, _passthrough_param_len - 1);
        }
        else
        {
            ack = ACK_D_GENERAL_ERROR;
        }
        break;
    case cmd_DeviceEraseAll:
        if (_motor_task)
        {
            ack = _motor_task->passthroughErase(_passthrough_params[0], _passthrough_addr);
        }
        else
        {
            ack = ACK_D_GENERAL_ERROR;
        }
        break;
    default:
        ack = ACK_I_INVALID_CMD;
        break;
    }

    _send_passthrough_response(_passthrough_cmd, _passthrough_addr, o_param, o_param_len, ack);
}

void SerialManagerTask::_send_passthrough_response(uint8_t cmd, uint16_t addr, const uint8_t *param, uint8_t param_len, uint8_t ack)
{
    uint16_t crc = 0;
    Serial.write(cmd_Remote_Escape);
    crc = _crc_xmodem_update(crc, cmd_Remote_Escape);
    Serial.write(cmd);
    crc = _crc_xmodem_update(crc, cmd);
    Serial.write((uint8_t)(addr >> 8));
    crc = _crc_xmodem_update(crc, (uint8_t)(addr >> 8));
    Serial.write((uint8_t)addr);
    crc = _crc_xmodem_update(crc, (uint8_t)addr);
    Serial.write(param_len);
    crc = _crc_xmodem_update(crc, param_len);

    for (int i = 0; i < param_len; i++)
    {
        Serial.write(param[i]);
        crc = _crc_xmodem_update(crc, param[i]);
    }

    Serial.write(ack);
    crc = _crc_xmodem_update(crc, ack);

    Serial.write((uint8_t)(crc >> 8));
    Serial.write((uint8_t)crc);
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

uint16_t SerialManagerTask::_crc_xmodem_update(uint16_t crc, uint8_t data)
{
    int i;
    crc = crc ^ ((uint16_t)data << 8);
    for (i = 0; i < 8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}
