/**
 * @file serial_manager_task.cpp
 * @brief Implements the SerialManagerTask class for managing serial communication.
 * @author Your Name
 * @date 2025-11-12
 * @license MIT
 */

#include "serial_manager_task.h"
#include "../firmware.h"



SerialManagerTask::SerialManagerTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms),
      _scheduler(scheduler),
      _imu_task(imu_task),
      _rx_task(rx_task),
      _motor_task(motor_task),
      _pid_task(pid_task),
      _settings_manager(settings_manager)
{
    _terminal = new Terminal(_scheduler, _imu_task, _rx_task, _motor_task, _pid_task, _settings_manager);
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

        // Handshake logic: Check for MSP preamble
        if (_current_mode == SerialMode::TERMINAL) {
            // Look for '$', 'R', 'M', 'S', 'P' sequence
            static uint8_t handshake_state = 0;
            const char msp_handshake[] = "$RMSP";

            if (c == msp_handshake[handshake_state]) {
                handshake_state++;
                if (handshake_state == strlen(msp_handshake)) {
                    _current_mode = SerialMode::MSP;
                    _last_msp_activity_ms = millis();
                    com_send_log(LOG_INFO, "Switched to MSP mode.");
                    handshake_state = 0; // Reset handshake state
                    continue; // Don't process this char as terminal input
                }
            } else {
                handshake_state = 0; // Reset if sequence is broken
            }
        }

        if (_current_mode == SerialMode::TERMINAL)
        {
            _terminal->handleInput(c);
        }
        else // MSP Mode
        {
            _parse_msp_char(c);
            _last_msp_activity_ms = millis(); // Update activity on any MSP char
        }
    }

    // MSP Timeout logic
    if (_current_mode == SerialMode::MSP && (millis() - _last_msp_activity_ms > MSP_TIMEOUT_MS)) {
        _current_mode = SerialMode::TERMINAL;
        com_send_log(LOG_INFO, "MSP timeout. Switched back to Terminal mode.");
        showPrompt();
    }

    // If in terminal mode and should quit, signal scheduler
    if (_current_mode == SerialMode::TERMINAL && _terminal->shouldQuit()) {
        // _scheduler->stop(); // Scheduler does not have a stop method. Implement system halt if needed.
    }
}

void SerialManagerTask::showPrompt()
{
    _terminal->showPrompt();
}





// MSP Mode Functions
void SerialManagerTask::_parse_msp_char(uint8_t c)
{
    switch (_msp_state)
    {
    case MspState::IDLE:
        if (c == '$') {
            _msp_state = MspState::HEADER_START;
        }
        break;
    case MspState::HEADER_START:
        _msp_state = (c == 'M') ? MspState::HEADER_SIZE : MspState::IDLE;
        break;
    case MspState::HEADER_SIZE:
        _msp_payload_size = c;
        _msp_crc = c;
        _msp_payload_index = 0;
        _msp_state = MspState::HEADER_CMD;
        break;
    case MspState::HEADER_CMD:
        _msp_command_id = c;
        _msp_crc ^= c;
        _msp_state = (_msp_payload_size > 0) ? MspState::PAYLOAD : MspState::CRC;
        break;
    case MspState::PAYLOAD:
        _msp_payload_buffer[_msp_payload_index++] = c;
        _msp_crc ^= c;
        if (_msp_payload_index >= _msp_payload_size) {
            _msp_state = MspState::CRC;
        }
        break;
    case MspState::CRC:
        if (_msp_crc == c) {
            _process_msp_message();
        }
        _msp_state = MspState::IDLE;
        break;
    default:
        _msp_state = MspState::IDLE;
        break;
    }
}

void SerialManagerTask::_process_msp_message()
{
    switch (_msp_command_id)
    {
    case MSP_API_VERSION:
        _handle_msp_api_version();
        break;
    case MSP_FC_VARIANT:
        _handle_msp_fc_variant();
        break;
    case MSP_FC_VERSION:
        _handle_msp_fc_version();
        break;
    case MSP_BOARD_INFO:
        _handle_msp_board_info();
        break;
    case MSP_BUILD_INFO:
        _handle_msp_build_info();
        break;
    case MSP_REBOOT:
        _handle_msp_reboot();
        break;
    case MSP_EEPROM_WRITE:
        _handle_msp_eeprom_write();
        break;
    case MSP_RESET_SETTINGS:
        _handle_msp_reset_settings();
        break;
    case MSP_STATUS:
        _handle_msp_status();
        break;
    case MSP_MEM_STATS:
        _handle_msp_mem_stats();
        break;
    case MSP_GET_SETTING:
        _handle_msp_get_setting();
        break;
    case MSP_SET_SETTING:
        _handle_msp_set_setting();
        break;
    case MSP_PID:
        _handle_msp_pid_get();
        break;
    case MSP_SET_PID:
        _handle_msp_pid_set();
        break;
    // Add more command handlers here
    default:
        // Send empty response for unsupported command
        _send_msp_response(_msp_command_id, nullptr, 0);
        break;
    }
}

void SerialManagerTask::_send_msp_response(uint8_t cmd, uint8_t *payload, uint8_t size)
{
    Serial.write('$');
    Serial.write('M');
    Serial.write('>');
    uint8_t crc = 0;
    Serial.write(size);
    crc ^= size;
    Serial.write(cmd);
    crc ^= cmd;
    for (uint8_t i = 0; i < size; i++)
    {
        Serial.write(payload[i]);
        crc ^= payload[i];
    }
    Serial.write(crc);
}

// MSP Command Handlers
void SerialManagerTask::_handle_msp_api_version()
{
    uint8_t payload[3];
    payload[0] = MSP_PROTOCOL_VERSION;
    payload[1] = MSP_API_VERSION_MAJOR;
    payload[2] = MSP_API_VERSION_MINOR;
    _send_msp_response(MSP_API_VERSION, payload, 3);
}

void SerialManagerTask::_handle_msp_fc_variant()
{
    uint8_t payload[4] = {'F', 'L', '3', '2'};
    _send_msp_response(MSP_FC_VARIANT, payload, 4);
}

void SerialManagerTask::_handle_msp_fc_version()
{
    uint8_t payload[3] = {FC_VERSION_MAJOR, FC_VERSION_MINOR, FC_VERSION_PATCH};
    _send_msp_response(MSP_FC_VERSION, payload, 3);
}

void SerialManagerTask::_handle_msp_board_info()
{
    uint8_t payload[8] = {'F', 'L', '3', '2', 0, 0, 0, 0};
    _send_msp_response(MSP_BOARD_INFO, payload, 8);
}

void SerialManagerTask::_handle_msp_build_info()
{
    uint8_t payload[20] = "Nov 12 202512:00:00";
    _send_msp_response(MSP_BUILD_INFO, payload, 19);
}

void SerialManagerTask::_handle_msp_reboot() {
    _send_msp_response(MSP_REBOOT, nullptr, 0); // Acknowledge
    delayMicroseconds(ONE_SECOND_MICROSECONDS);
    ESP.restart();
}

void SerialManagerTask::_handle_msp_eeprom_write() {
    _settings_manager->saveSettings();
    _send_msp_response(MSP_EEPROM_WRITE, nullptr, 0); // Acknowledge
}

void SerialManagerTask::_handle_msp_reset_settings() {
    _settings_manager->factoryReset();
    _send_msp_response(MSP_RESET_SETTINGS, nullptr, 0); // Acknowledge
    delayMicroseconds(ONE_SECOND_MICROSECONDS);
    ESP.restart();
}

void SerialManagerTask::_handle_msp_status() {
    // For simplicity, just send firmware version for now
    // More detailed status would require a custom payload structure
    String version_str = get_firmware_version();
    uint8_t payload[version_str.length() + 1];
    memcpy(payload, version_str.c_str(), version_str.length());
    payload[version_str.length()] = 0; // Null terminator
    _send_msp_response(MSP_STATUS, payload, version_str.length() + 1);
}

void SerialManagerTask::_handle_msp_mem_stats() {
    // Send free heap size (4 bytes)
    uint32_t free_heap = ESP.getFreeHeap();
    uint8_t payload[4];
    payload[0] = (free_heap >> 0) & 0xFF;
    payload[1] = (free_heap >> 8) & 0xFF;
    payload[2] = (free_heap >> 16) & 0xFF;
    payload[3] = (free_heap >> 24) & 0xFF;
    _send_msp_response(MSP_MEM_STATS, payload, 4);
}

void SerialManagerTask::_handle_msp_get_setting() {
    if (_msp_payload_size == 0) {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: no key provided
        return;
    }

    // MSP payload for get setting: [key_length (1 byte)] [key_string (variable length)]
    uint8_t key_len = _msp_payload_buffer[0];
    if (key_len == 0 || key_len > (_msp_payload_size - 1)) {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: invalid key length
        return;
    }

    char key_cstr[key_len + 1];
    memcpy(key_cstr, &_msp_payload_buffer[1], key_len);
    key_cstr[key_len] = '\0';
    String display_key_str = String(key_cstr);

    const char *internal_key = _settings_manager->getInternalKeyFromDisplayKey(display_key_str.c_str());

    if (internal_key == nullptr) {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: unknown setting
        return;
    }

    String value_str = _settings_manager->getSettingValueHumanReadable(internal_key);
    
    // Response payload: [key_length (1 byte)] [key_string (variable)] [value_length (1 byte)] [value_string (variable)]
    // Max payload size is 128, so need to be careful with string lengths
    uint8_t response_key_len = display_key_str.length();
    uint8_t response_value_len = value_str.length();

    if (response_key_len + response_value_len + 2 > 128) { // 2 for key_len and value_len bytes
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Response too long
        return;
    }

    uint8_t response_payload[response_key_len + response_value_len + 2];
    int idx = 0;
    response_payload[idx++] = response_key_len;
    memcpy(&response_payload[idx], display_key_str.c_str(), response_key_len);
    idx += response_key_len;
    response_payload[idx++] = response_value_len;
    memcpy(&response_payload[idx], value_str.c_str(), response_value_len);
    idx += response_value_len;

    _send_msp_response(MSP_GET_SETTING, response_payload, idx);
}

void SerialManagerTask::_handle_msp_set_setting() {
    if (_msp_payload_size < 3) { // Need at least key_len, key, value_len, value
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid payload
        return;
    }

    // MSP payload for set setting: [key_length (1 byte)] [key_string (variable)] [value_length (1 byte)] [value_string (variable)]
    uint8_t key_len = _msp_payload_buffer[0];
    if (key_len == 0 || key_len > (_msp_payload_size - 2)) { // -2 for key_len and value_len bytes
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid key length
        return;
    }

    char key_cstr[key_len + 1];
    memcpy(key_cstr, &_msp_payload_buffer[1], key_len);
    key_cstr[key_len] = '\0';
    String display_key_str = String(key_cstr);

    uint8_t value_len_idx = 1 + key_len;
    uint8_t value_len = _msp_payload_buffer[value_len_idx];
    if (value_len == 0 || (value_len_idx + 1 + value_len) > _msp_payload_size) {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid value length
        return;
    }

    char value_cstr[value_len + 1];
    memcpy(value_cstr, &_msp_payload_buffer[value_len_idx + 1], value_len);
    value_cstr[value_len] = '\0';
    String value_str = String(value_cstr);

    const char *internal_key = _settings_manager->getInternalKeyFromDisplayKey(display_key_str.c_str());

    if (internal_key == nullptr) {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: unknown setting
        return;
    }

    if (_settings_manager->setSettingValue(internal_key, value_str)) {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Acknowledge success
    } else {
        // Could send an error code in payload if MSP supports it, for now just empty response
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Acknowledge failure
    }
}

void SerialManagerTask::_handle_msp_pid_get()
{
    if (!_pid_task) {
        // No log here, MSP clients don't read logs. Just send empty response.
        _send_msp_response(MSP_PID, nullptr, 0); 
        return;
    }

    uint8_t payload[MSP_PID_PAYLOAD_SIZE]; // 3 axes * 3 gains (P,I,D) = 9 bytes
    int i = 0;

    PidGains roll_gains = _pid_task->getGains(PidAxis::ROLL);
    payload[i++] = (uint8_t)(roll_gains.p * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(roll_gains.i * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(roll_gains.d * PID_SCALE_FACTOR);

    PidGains pitch_gains = _pid_task->getGains(PidAxis::PITCH);
    payload[i++] = (uint8_t)(pitch_gains.p * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(pitch_gains.i * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(pitch_gains.d * PID_SCALE_FACTOR);

    PidGains yaw_gains = _pid_task->getGains(PidAxis::YAW);
    payload[i++] = (uint8_t)(yaw_gains.p * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(yaw_gains.i * PID_SCALE_FACTOR);
    payload[i++] = (uint8_t)(yaw_gains.d * PID_SCALE_FACTOR);

    _send_msp_response(MSP_PID, payload, 9);
}

void SerialManagerTask::_handle_msp_pid_set()
{
    if (!_pid_task || _msp_payload_size != MSP_PID_PAYLOAD_SIZE) {
        _send_msp_response(MSP_SET_PID, nullptr, 0); // Error or invalid payload size
        return;
    }

    int i = 0;
    PidGains roll_gains = _pid_task->getGains(PidAxis::ROLL);
    roll_gains.p = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    roll_gains.i = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    roll_gains.d = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::ROLL, roll_gains);

    PidGains pitch_gains = _pid_task->getGains(PidAxis::PITCH);
    pitch_gains.p = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    pitch_gains.i = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    pitch_gains.d = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::PITCH, pitch_gains);

    PidGains yaw_gains = _pid_task->getGains(PidAxis::YAW);
    yaw_gains.p = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    yaw_gains.i = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    yaw_gains.d = (float)_msp_payload_buffer[i++] / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::YAW, yaw_gains);

    _send_msp_response(MSP_SET_PID, nullptr, 0); // Acknowledge receipt
}
