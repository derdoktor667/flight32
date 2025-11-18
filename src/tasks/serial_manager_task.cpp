/**
 * @file serial_manager_task.cpp
 * @brief Implements the SerialManagerTask class for managing serial communication.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "serial_manager_task.h"
#include "../utils/version_info.h"
#include <cmath>
#include "imu_task.h"
#include "../config/terminal_config.h"
#include "../config/serial_config.h"
#include "../config/filter_config.h"
#include "../config/com_manager_config.h" // Include for MSP_RESPONSE_DELAY_US and UNIQUE_ID_BUFFER_SIZE
#include "../config/imu_config.h"         // Include for MSP_ACCEL_SCALING_FACTOR and MSP_GYRO_SCALING_FACTOR
#include <cstring>                        // For strlen and memcpy
#include <cstdio>                         // For snprintf

static constexpr float RADIANS_TO_DEGREES = (180.0f / M_PI);

// Helper function to convert quaternion to Euler angles (roll, pitch, yaw)
// Angles are in degrees
void quaternionToEuler(float w, float x, float y, float z, float *roll, float *pitch, float *yaw)
{
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp) * RADIANS_TO_DEGREES;

    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1)
    {                                                                  // Check for singularity
        *pitch = copysign(HALF_PI_RADIANS, sinp) * RADIANS_TO_DEGREES; // Use 90 degrees if out of range
    }
    else
    {
        *pitch = asin(sinp) * RADIANS_TO_DEGREES;
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp) * RADIANS_TO_DEGREES;
}

// Helper function to write float to payload
void SerialManagerTask::_write_float_to_payload(uint8_t *payload, int &index, float value)
{
    uint8_t *float_bytes = (uint8_t *)&value;
    payload[index++] = float_bytes[0];
    payload[index++] = float_bytes[1];
    payload[index++] = float_bytes[2];
    payload[index++] = float_bytes[3];
}

// Helper function to read float from payload
float SerialManagerTask::_read_float_from_payload(const uint8_t *payload, int &index)
{
    float value;
    uint8_t *float_bytes = (uint8_t *)&value;
    float_bytes[0] = payload[index++];
    float_bytes[1] = payload[index++];
    float_bytes[2] = payload[index++];
    float_bytes[3] = payload[index++];
    return value;
}

SerialManagerTask::SerialManagerTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms),
      _scheduler(scheduler),
      _imu_task(imu_task),
      _rx_task(rx_task),
      _motor_task(motor_task),
      _pid_task(pid_task),
      _settings_manager(settings_manager)
{
    _terminal = std::make_unique<Terminal>(_scheduler, _imu_task, _rx_task, _motor_task, _pid_task, _settings_manager);
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
        _parse_msp_char(c); // Unified parser
    }

    // MSP Timeout logic
    if (_current_mode == ComSerialMode::MSP && (millis() - _last_msp_activity_ms > MSP_TIMEOUT_MS))
    {
        _current_mode = ComSerialMode::TERMINAL;
        com_set_serial_mode(ComSerialMode::TERMINAL);

        // Reset MSP parser state
        _msp_state = MspState::IDLE;

        // Clear any partial input
        _terminal->clearInputBuffer();
        // Serial.println("[INFO] MSP timeout. Switched back to Terminal mode."); // Removed to prevent interference with MSP communication

        showPrompt();
    }

    // If in terminal mode and should quit, signal scheduler
    if (_current_mode == ComSerialMode::TERMINAL && _terminal->shouldQuit())
    {
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
        if (c == '$')
        {
            _msp_state = MspState::HEADER_START;
            _msp_crc = 0; // Initialize CRC for incoming message
        }
        else
        {
            // Not starting an MSP message, so it's for the terminal
            if (_current_mode == ComSerialMode::TERMINAL)
            {
                _terminal->handleInput(c);
            }
        }
        break;
    case MspState::HEADER_START:
        if (c == 'M')
        {
            _msp_state = MspState::HEADER_DIR;
            // _msp_crc = 'M'; // CRC starts with size byte, not 'M'
            _last_msp_activity_ms = millis();
        }
        else
        {
            // False alarm. It was not "$M".
            // We need to send the '$' and the current char 'c' to the terminal.
            if (_current_mode == ComSerialMode::TERMINAL)
            {
                _terminal->handleInput('$');
                _terminal->handleInput(c);
            }
            _msp_state = MspState::IDLE; // Reset state
        }
        break;
    case MspState::HEADER_DIR:
        if (c == '<') // Expecting '<' for incoming commands
        {
            _msp_state = MspState::HEADER_SIZE;
            // _msp_crc ^= c; // Direction byte is not part of CRC for incoming message
            if (_current_mode == ComSerialMode::TERMINAL)
            {
                _current_mode = ComSerialMode::MSP;
                com_set_serial_mode(ComSerialMode::MSP);
                // Serial.println("\n[INFO] Switched to MSP mode."); // Removed for clean MSP communication
            }
        }
        else
        {
            // Invalid direction byte, reset MSP state
            if (_current_mode == ComSerialMode::TERMINAL)
            {
                _terminal->handleInput('$');
                _terminal->handleInput('M');
                _terminal->handleInput(c);
            }
            _msp_state = MspState::IDLE;
        }
        _last_msp_activity_ms = millis();
        break;
    case MspState::HEADER_SIZE:
        _msp_payload_size = c;
        _msp_crc ^= c;
        _msp_payload_index = 0;
        _msp_state = MspState::HEADER_CMD;
        _last_msp_activity_ms = millis();
        break;
    case MspState::HEADER_CMD:
        _msp_command_id = c;
        _msp_crc ^= c;
        _msp_state = (_msp_payload_size > 0) ? MspState::PAYLOAD : MspState::CRC;
        _last_msp_activity_ms = millis();
        break;
    case MspState::PAYLOAD:
        _msp_payload_buffer[_msp_payload_index++] = c;
        _msp_crc ^= c;
        if (_msp_payload_index >= _msp_payload_size)
        {
            _msp_state = MspState::CRC;
        }
        _last_msp_activity_ms = millis();
        break;
    case MspState::CRC:
        if (_msp_crc == c)
        {
            _process_msp_message();
        }
        _msp_state = MspState::IDLE;
        _last_msp_activity_ms = millis();
        break;
    default:
        _msp_state = MspState::IDLE;
        break;
    }
}

void SerialManagerTask::_process_msp_message()
{
    // Serial.printf("[DEBUG] _process_msp_message called for command: 0x%02X\n", _msp_command_id);
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
    case MSP_GET_FILTER_CONFIG:
        _handle_msp_get_filter_config();
        break;
    case MSP_SET_FILTER_CONFIG:
        _handle_msp_set_filter_config();
        break;
    // Add new MSP command handlers
    case MSP_RAW_IMU:
        _handle_msp_raw_imu();
        break;
    case MSP_ATTITUDE:
        _handle_msp_attitude();
        break;
    case MSP_RC:
        _handle_msp_rc();
        break;
    case MSP_MOTOR:
        _handle_msp_motor();
        break;
    case MSP_BOX:
        _handle_msp_box_get();
        break;
    case MSP_SET_BOX:
        _handle_msp_box_set();
        break;
    case MSP_UID:
        _handle_msp_uid();
        break;
    case MSP_SENSOR_STATUS:
        _handle_msp_sensor_status();
        break;
    default:
        // Send empty response for unsupported command
        _send_msp_response(_msp_command_id, nullptr, 0);
        break;
    }
}

void SerialManagerTask::_handle_msp_sensor_status()
{
    if (!_imu_task)
    {
        _send_msp_response(MSP_SENSOR_STATUS, nullptr, 0);
        return;
    }

    ImuData imu_data = _imu_task->getImuSensor().getData();

    // Scale factors from Betaflight for raw IMU data (these are approximate)
    // Accel: 1G = 512 (approx, depends on sensor and range)
    // Gyro: 1 deg/s = 4 (approx, depends on sensor and range)
    int16_t accX = (int16_t)(imu_data.accelX * MSP_ACCEL_SCALING_FACTOR);
    int16_t accY = (int16_t)(imu_data.accelY * MSP_ACCEL_SCALING_FACTOR);
    int16_t accZ = (int16_t)(imu_data.accelZ * MSP_ACCEL_SCALING_FACTOR);

    int16_t gyroX = (int16_t)(imu_data.gyroX * MSP_GYRO_SCALING_FACTOR);
    int16_t gyroY = (int16_t)(imu_data.gyroY * MSP_GYRO_SCALING_FACTOR);
    int16_t gyroZ = (int16_t)(imu_data.gyroZ * MSP_GYRO_SCALING_FACTOR);

    uint8_t payload[MSP_SENSOR_STATUS_PAYLOAD_SIZE]; // 3x Accel, 3x Gyro (int16_t = 2 bytes each)
    int i = 0;

    _write_int16_to_payload(payload, i, accX);
    _write_int16_to_payload(payload, i, accY);
    _write_int16_to_payload(payload, i, accZ);

    _write_int16_to_payload(payload, i, gyroX);
    _write_int16_to_payload(payload, i, gyroY);
    _write_int16_to_payload(payload, i, gyroZ);

    _send_msp_response(MSP_SENSOR_STATUS, payload, MSP_SENSOR_STATUS_PAYLOAD_SIZE);
}


// CORRECTED: Removed '$M>' and changed to proper '$M' format
void SerialManagerTask::_send_msp_response(uint8_t cmd, uint8_t *payload, uint8_t size)
{
    // Serial.printf("[DEBUG] _send_msp_response called for cmd: 0x%02X, size: %d\n", cmd, size);
    Serial.write('$');
    Serial.write('M');
    Serial.write('>'); // Direction byte for response

    uint8_t crc = 0; // Initialize CRC with 0
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
    delayMicroseconds(MSP_RESPONSE_DELAY_US); // Small delay to ensure the entire response is sent
}

// MSP Command Handlers
void SerialManagerTask::_handle_msp_api_version()
{
    uint8_t payload[MSP_API_VERSION_PAYLOAD_SIZE];
    payload[0] = MSP_PROTOCOL_VERSION;
    payload[1] = MSP_CAPABILITY;
    payload[2] = MSP_FC_IDENTIFIER;
    payload[3] = MSP_API_VERSION_MAJOR;
    payload[4] = MSP_API_VERSION_MINOR;
    _send_msp_response(MSP_API_VERSION, payload, MSP_API_VERSION_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_fc_variant()
{
    _send_msp_response(MSP_FC_VARIANT, (uint8_t *)MSP_FC_VARIANT_NAME, MSP_FC_VARIANT_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_fc_version()
{
    uint8_t payload[3] = {FC_VERSION_MAJOR, FC_VERSION_MINOR, FC_VERSION_PATCH};
    _send_msp_response(MSP_FC_VERSION, payload, MSP_FC_VERSION_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_board_info()
{
    uint8_t payload[MSP_BOARD_INFO_PAYLOAD_SIZE] = {0};
    int idx = 0;

    // Board Identifier (4 chars)
    memcpy(&payload[idx], MSP_BOARD_IDENTIFIER, 4);
    idx += 4;

    // Hardware Revision (uint16_t)
    payload[idx++] = (uint8_t)(MSP_HARDWARE_REVISION_VALUE & 0xFF);
    payload[idx++] = (uint8_t)((MSP_HARDWARE_REVISION_VALUE >> 8) & 0xFF);

    // Board Name (length-prefixed string)
    uint8_t board_name_len = strlen(MSP_BOARD_NAME);
    payload[idx++] = board_name_len;
    memcpy(&payload[idx], MSP_BOARD_NAME, board_name_len);
    idx += board_name_len;

    _send_msp_response(MSP_BOARD_INFO, payload, idx);
}

void SerialManagerTask::_handle_msp_build_info()
{
    char build_info_str[MSP_BUILD_INFO_PAYLOAD_SIZE];
    snprintf(build_info_str, sizeof(build_info_str), "%s %s", __DATE__, __TIME__);
    _send_msp_response(MSP_BUILD_INFO, (uint8_t *)build_info_str, strlen(build_info_str));
}

void SerialManagerTask::_handle_msp_reboot()
{
    _send_msp_response(MSP_REBOOT, nullptr, 0); // Acknowledge
    delayMicroseconds(ONE_SECOND_MICROSECONDS);
    ESP.restart();
}

void SerialManagerTask::_handle_msp_eeprom_write()
{
    _settings_manager->saveSettings();
    _send_msp_response(MSP_EEPROM_WRITE, nullptr, 0); // Acknowledge
}

void SerialManagerTask::_handle_msp_reset_settings()
{
    _settings_manager->factoryReset();
    _send_msp_response(MSP_RESET_SETTINGS, nullptr, 0); // Acknowledge
    delayMicroseconds(ONE_SECOND_MICROSECONDS);
    ESP.restart();
}

void SerialManagerTask::_handle_msp_status()
{
    uint8_t payload[MSP_STATUS_PAYLOAD_SIZE];
    int i = 0;

    // cycleTime
    _write_int16_to_payload(payload, i, _pid_task->getCycleTime());

    // i2c_errors_count
    _write_int16_to_payload(payload, i, _imu_task->getImuSensor().getI2CErrorCount());

    // sensor
    uint16_t sensors = 0;
    if (_imu_task->getImuSensor().isSensorHealthy()) {
        sensors |= 1; // ACC
    }
    // Gyro is implicit
    // No baro or mag in this hardware
    _write_int16_to_payload(payload, i, sensors);

    // flightModeFlags
    uint32_t flightModeFlags = 0;
    if (_pid_task->isArmed()) {
        flightModeFlags |= (1 << 0); // ARM
    }
    if (_pid_task->getFlightMode() == FlightMode::STABILIZED) {
        flightModeFlags |= (1 << 1); // ANGLE
    }
    // Acro is the absence of other modes

    payload[i++] = (flightModeFlags >> 0) & 0xFF;
    payload[i++] = (flightModeFlags >> 8) & 0xFF;
    payload[i++] = (flightModeFlags >> 16) & 0xFF;
    payload[i++] = (flightModeFlags >> 24) & 0xFF;

    // configProfileIndex
    payload[i++] = 0;

    _send_msp_response(MSP_STATUS, payload, MSP_STATUS_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_mem_stats()
{
    // Send free heap size (4 bytes)
    uint32_t free_heap = ESP.getFreeHeap();
    uint8_t payload[MSP_MEM_STATS_PAYLOAD_SIZE];
    payload[0] = (free_heap >> 0) & 0xFF;
    payload[1] = (free_heap >> 8) & 0xFF;
    payload[2] = (free_heap >> 16) & 0xFF;
    payload[3] = (free_heap >> 24) & 0xFF;
    _send_msp_response(MSP_MEM_STATS, payload, MSP_MEM_STATS_PAYLOAD_SIZE);
    _send_msp_response(MSP_MEM_STATS, payload, MSP_MEM_STATS_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_get_setting()
{
    if (_msp_payload_size == 0)
    {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: no key provided
        return;
    }

    // MSP payload for get setting: [key_length (1 byte)] [key_string (variable length)]
    uint8_t key_len = _msp_payload_buffer[0];
    if (key_len == 0 || key_len > (_msp_payload_size - 1))
    {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: invalid key length
        return;
    }

    char key_cstr[key_len + 1];
    memcpy(key_cstr, &_msp_payload_buffer[1], key_len);
    key_cstr[key_len] = '\0';
    String display_key_str = String(key_cstr);

    const char *internal_key = _settings_manager->getInternalKeyFromDisplayKey(display_key_str.c_str());

    if (internal_key == nullptr)
    {
        com_send_log(ComMessageType::LOG_ERROR, "MSP_GET_SETTING: Unknown setting display key: %s", display_key_str.c_str());
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: unknown setting
        return;
    }

    String value_str = _settings_manager->getSettingValueHumanReadable(internal_key);
    com_send_log(ComMessageType::LOG_INFO, "MSP_GET_SETTING: Retrieved %s (internal: %s) = %s", display_key_str.c_str(), internal_key, value_str.c_str());

    // Response payload: [key_length (1 byte)] [key_string (variable)] [value_length (1 byte)] [value_string (variable)]
    // Max payload size is 128, so need to be careful with string lengths
    uint8_t response_key_len = display_key_str.length();
    uint8_t response_value_len = value_str.length();

    if (response_key_len + response_value_len + MSP_SETTING_KEY_VALUE_OVERHEAD_BYTES > MSP_MAX_PAYLOAD_SIZE)
    {
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Response too long
        return;
    }

    uint8_t response_payload[response_key_len + response_value_len + MSP_RESPONSE_OVERHEAD_BYTES];
    int idx = 0;
    response_payload[idx++] = response_key_len;
    memcpy(&response_payload[idx], display_key_str.c_str(), response_key_len);
    idx += response_key_len;
    response_payload[idx++] = response_value_len;
    memcpy(&response_payload[idx], value_str.c_str(), response_value_len);
    idx += response_value_len;

    _send_msp_response(MSP_GET_SETTING, response_payload, idx);
}

void SerialManagerTask::_handle_msp_set_setting()
{
    if (_msp_payload_size < MSP_MIN_PAYLOAD_SIZE)
    {                                                    // Need at least key_len, key, value_len, value
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid payload
        return;
    }

    // MSP payload for set setting: [key_length (1 byte)] [key_string (variable)] [value_length (1 byte)] [value_string (variable)]
    uint8_t key_len = _msp_payload_buffer[0];
    if (key_len == 0 || key_len > (_msp_payload_size - MSP_SETTING_KEY_VALUE_OVERHEAD_BYTES))
    {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid key length
        return;
    }

    char key_cstr[key_len + 1];
    memcpy(key_cstr, &_msp_payload_buffer[1], key_len);
    key_cstr[key_len] = '\0';
    String display_key_str = String(key_cstr);

    uint8_t value_len_idx = 1 + key_len;
    uint8_t value_len = _msp_payload_buffer[value_len_idx];
    if (value_len == 0 || (value_len_idx + 1 + value_len) > _msp_payload_size)
    {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: invalid value length
        return;
    }

    char value_cstr[value_len + 1];
    memcpy(value_cstr, &_msp_payload_buffer[value_len_idx + 1], value_len);
    value_cstr[value_len] = '\0';
    String value_str = String(value_cstr);

    const char *internal_key = _settings_manager->getInternalKeyFromDisplayKey(display_key_str.c_str());

    if (internal_key == nullptr)
    {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Error: unknown setting
        return;
    }

    if (_settings_manager->setSettingValue(internal_key, value_str))
    {
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Acknowledge success
    }
    else
    {
        // Could send an error code in payload if MSP supports it, for now just empty response
        _send_msp_response(MSP_SET_SETTING, nullptr, 0); // Acknowledge failure
    }
}

// MSP_GET_FILTER_CONFIG handler
void SerialManagerTask::_handle_msp_get_filter_config()
{
    uint8_t payload[MSP_FILTER_CONFIG_PAYLOAD_SIZE]; // 5 floats * 4 bytes each
    int i = 0;

    _write_float_to_payload(payload, i, _settings_manager->getFloat(NVS_KEY_GYRO_LPF_HZ));
    _write_float_to_payload(payload, i, _settings_manager->getFloat(NVS_KEY_NOTCH1_HZ));
    _write_float_to_payload(payload, i, _settings_manager->getFloat(NVS_KEY_NOTCH1_Q));
    _write_float_to_payload(payload, i, _settings_manager->getFloat(NVS_KEY_NOTCH2_HZ));
    _write_float_to_payload(payload, i, _settings_manager->getFloat(NVS_KEY_NOTCH2_Q));

    _send_msp_response(MSP_GET_FILTER_CONFIG, payload, MSP_FILTER_CONFIG_PAYLOAD_SIZE);
}

// MSP_SET_FILTER_CONFIG handler
void SerialManagerTask::_handle_msp_set_filter_config()
{
    if (_msp_payload_size != MSP_FILTER_CONFIG_PAYLOAD_SIZE) // Expect 5 floats * 4 bytes
    {
        _send_msp_response(MSP_SET_FILTER_CONFIG, nullptr, 0); // Error: invalid payload size
        return;
    }

    int i = 0;
    _settings_manager->setFloat(NVS_KEY_GYRO_LPF_HZ, _read_float_from_payload(_msp_payload_buffer, i));
    _settings_manager->setFloat(NVS_KEY_NOTCH1_HZ, _read_float_from_payload(_msp_payload_buffer, i));
    _settings_manager->setFloat(NVS_KEY_NOTCH1_Q, _read_float_from_payload(_msp_payload_buffer, i));
    _settings_manager->setFloat(NVS_KEY_NOTCH2_HZ, _read_float_from_payload(_msp_payload_buffer, i));
    _settings_manager->setFloat(NVS_KEY_NOTCH2_Q, _read_float_from_payload(_msp_payload_buffer, i));

    _send_msp_response(MSP_SET_FILTER_CONFIG, nullptr, 0); // Acknowledge receipt
}

// CORRECTED: Changed PID payload from 9 bytes (uint8_t) to 18 bytes (int16_t) to prevent overflow
void SerialManagerTask::_handle_msp_pid_get()
{
    if (!_pid_task)
    {
        // No log here, MSP clients don't read logs. Just send empty response.
        _send_msp_response(MSP_PID, nullptr, 0);
        return;
    }

    // CORRECTION: Use int16_t instead of uint8_t for better range (prevents overflow)
    // Payload: 9 values × 2 bytes = 18 bytes total
    uint8_t payload[MSP_PID_PAYLOAD_SIZE];
    int i = 0;

    PidGains roll_gains = _pid_task->getGains(PidAxis::ROLL);
    _write_int16_to_payload(payload, i, (int16_t)(roll_gains.p * PID_SCALE_FACTOR));
    _write_int16_to_payload(payload, i, (int16_t)(roll_gains.i * PID_SCALE_FACTOR));
    _write_int16_to_payload(payload, i, (int16_t)(roll_gains.d * PID_SCALE_FACTOR));

    PidGains pitch_gains = _pid_task->getGains(PidAxis::PITCH);
    _write_int16_to_payload(payload, i, (int16_t)(pitch_gains.p * PID_SCALE_FACTOR));
    _write_int16_to_payload(payload, i, (int16_t)(pitch_gains.i * PID_SCALE_FACTOR));
    _write_int16_to_payload(payload, i, (int16_t)(pitch_gains.d * PID_SCALE_FACTOR));

    PidGains yaw_gains = _pid_task->getGains(PidAxis::YAW);
    _write_int16_to_payload(payload, i, (int16_t)(yaw_gains.p * PID_SCALE_FACTOR));
    _write_int16_to_payload(payload, i, (int16_t)(yaw_gains.i * PID_SCALE_FACTOR));
    _write_int16_to_payload(payload, i, (int16_t)(yaw_gains.d * PID_SCALE_FACTOR));

    _send_msp_response(MSP_PID, payload, MSP_PID_PAYLOAD_SIZE); // CORRECTION: Changed from 9 to 18
}

// CORRECTED: Changed PID payload parsing from 9 bytes to 18 bytes
void SerialManagerTask::_handle_msp_pid_set()
{
    if (!_pid_task || _msp_payload_size != MSP_PID_PAYLOAD_SIZE) // CORRECTION: Changed from 9 to 18
    {
        _send_msp_response(MSP_SET_PID, nullptr, 0); // Error or invalid payload size
        return;
    }

    int i = 0;
    PidGains roll_gains = _pid_task->getGains(PidAxis::ROLL);
    roll_gains.p = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    roll_gains.i = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    roll_gains.d = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::ROLL, roll_gains);

    PidGains pitch_gains = _pid_task->getGains(PidAxis::PITCH);
    pitch_gains.p = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    pitch_gains.i = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    pitch_gains.d = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::PITCH, pitch_gains);

    PidGains yaw_gains = _pid_task->getGains(PidAxis::YAW);
    yaw_gains.p = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    yaw_gains.i = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    yaw_gains.d = (float)_read_int16_from_payload(_msp_payload_buffer, i) / PID_SCALE_FACTOR;
    _pid_task->setGains(PidAxis::YAW, yaw_gains);

    _send_msp_response(MSP_SET_PID, nullptr, 0); // Acknowledge receipt
}

void SerialManagerTask::_handle_msp_raw_imu()
{
    if (!_imu_task)
    {
        _send_msp_response(MSP_RAW_IMU, nullptr, 0);
        return;
    }

    ImuData imu_data = _imu_task->getImuSensor().getData();

    // Scaling factors similar to Betaflight for raw IMU data
    // Accel: 1G = 512 (approx, depends on sensor and range)
    // Gyro: 1 deg/s = 4 (approx, depends on sensor and range)
    int16_t accX = (int16_t)(imu_data.accelX * MSP_ACCEL_SCALING_FACTOR);
    int16_t accY = (int16_t)(imu_data.accelY * MSP_ACCEL_SCALING_FACTOR);
    int16_t accZ = (int16_t)(imu_data.accelZ * MSP_ACCEL_SCALING_FACTOR);

    int16_t gyroX = (int16_t)(imu_data.gyroX * MSP_GYRO_SCALING_FACTOR);
    int16_t gyroY = (int16_t)(imu_data.gyroY * MSP_GYRO_SCALING_FACTOR);
    int16_t gyroZ = (int16_t)(imu_data.gyroZ * MSP_GYRO_SCALING_FACTOR);

    int16_t magX = 0, magY = 0, magZ = 0; // MPU6050 does not have magnetometer

    uint8_t payload[MSP_RAW_IMU_PAYLOAD_SIZE]; // 3x Accel, 3x Gyro, 3x Mag (int16_t = 2 bytes each)
    int i = 0;

    _write_int16_to_payload(payload, i, accX);
    _write_int16_to_payload(payload, i, accY);
    _write_int16_to_payload(payload, i, accZ);

    _write_int16_to_payload(payload, i, gyroX);
    _write_int16_to_payload(payload, i, gyroY);
    _write_int16_to_payload(payload, i, gyroZ);

    _write_int16_to_payload(payload, i, magX);
    _write_int16_to_payload(payload, i, magY);
    _write_int16_to_payload(payload, i, magZ);

    _send_msp_response(MSP_RAW_IMU, payload, MSP_RAW_IMU_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_attitude()
{
    if (!_imu_task)
    {
        _send_msp_response(MSP_ATTITUDE, nullptr, 0);
        return;
    }

    ImuQuaternionData q = _imu_task->getImuSensor().getQuaternion();
    float roll_f, pitch_f, yaw_f;
    quaternionToEuler(q.w, q.x, q.y, q.z, &roll_f, &pitch_f, &yaw_f);

    int16_t msp_roll = (int16_t)(roll_f * MSP_ATTITUDE_SCALE_FACTOR);
    int16_t msp_pitch = (int16_t)(pitch_f * MSP_ATTITUDE_SCALE_FACTOR);
    int16_t msp_yaw = (int16_t)yaw_f; // Yaw is typically not scaled by 10 in MSP

    uint8_t payload[MSP_ATTITUDE_PAYLOAD_SIZE]; // 3x angles (int16_t = 2 bytes each)
    int i = 0;

    _write_int16_to_payload(payload, i, msp_roll);
    _write_int16_to_payload(payload, i, msp_pitch);
    _write_int16_to_payload(payload, i, msp_yaw);

    _send_msp_response(MSP_ATTITUDE, payload, MSP_ATTITUDE_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_rc()
{
    if (!_rx_task)
    {
        _send_msp_response(MSP_RC, nullptr, 0);
        return;
    }

    uint8_t payload[MSP_RC_PAYLOAD_SIZE]; // 8x RC channels (int16_t = 2 bytes each)
    int i = 0;

    for (int j = 0; j < PPM_MAX_CHANNELS; j++)
    {
        // Use getChannel to retrieve the individual channel value
        int16_t channel_value = _rx_task->getChannel(j);
        _write_int16_to_payload(payload, i, channel_value);
    }

    _send_msp_response(MSP_RC, payload, MSP_RC_PAYLOAD_SIZE);
}

void SerialManagerTask::_write_int16_to_payload(uint8_t *payload, int &index, int16_t value)
{
    payload[index++] = (value >> 0) & 0xFF;
    payload[index++] = (value >> 8) & 0xFF;
}

// Helper function to read int16_t from payload (for MSP_SET_PID)
int16_t SerialManagerTask::_read_int16_from_payload(const uint8_t *payload, int &index)
{
    int16_t value = 0;
    value |= ((int16_t)payload[index++] << 0);
    value |= ((int16_t)payload[index++] << 8);
    return value;
}

void SerialManagerTask::_handle_msp_motor()
{
    if (!_motor_task)
    {
        _send_msp_response(MSP_MOTOR, nullptr, 0);
        return;
    }

    uint8_t payload[MSP_MOTOR_PAYLOAD_SIZE]; // 4x motor outputs (int16_t = 2 bytes each)
    int i = 0;

    for (int j = 0; j < NUM_MOTORS; j++)
    {
        uint16_t motor_output = _motor_task->getMotorOutput(j);
        _write_int16_to_payload(payload, i, motor_output);
    }

    _send_msp_response(MSP_MOTOR, payload, MSP_MOTOR_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_box_get()
{
    if (!_pid_task)
    {
        _send_msp_response(MSP_BOX, nullptr, 0);
        return;
    }

    uint8_t payload[MSP_BOX_PAYLOAD_SIZE];
    int i = 0;

    FlightMode current_mode = _pid_task->getFlightMode();
    uint16_t mode_id = (current_mode == FlightMode::STABILIZED) ? MSP_BOX_STABILIZED_ID : MSP_BOX_ACRO_ID;

    _write_int16_to_payload(payload, i, mode_id);

    _send_msp_response(MSP_BOX, payload, MSP_BOX_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_box_set()
{
    if (!_pid_task || _msp_payload_size != MSP_BOX_PAYLOAD_SIZE)
    {
        _send_msp_response(MSP_SET_BOX, nullptr, 0);
        return;
    }

    int i = 0;
    uint16_t mode_id = _read_int16_from_payload(_msp_payload_buffer, i);

    if (mode_id == MSP_BOX_STABILIZED_ID)
    {
        _pid_task->setFlightMode(FlightMode::STABILIZED);
    }
    else
    {
        _pid_task->setFlightMode(FlightMode::ACRO);
    }

    _send_msp_response(MSP_SET_BOX, nullptr, 0); // Acknowledge
}

void SerialManagerTask::_handle_msp_uid()
{
    uint8_t payload[MSP_UID_PAYLOAD_SIZE] = {0};

    // UID is 3 x uint32_t
    // For Betaflight compatibility, the first 4 bytes are 0.
    // The next 8 bytes are the UID. We use a hardcoded value for now.
    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 0;
    payload[4] = 'F';
    payload[5] = 'L';
    payload[6] = '3';
    payload[7] = '2';
    payload[8] = '0';
    payload[9] = '0';
    payload[10] = '0';
    payload[11] = '1';

    _send_msp_response(MSP_UID, payload, MSP_UID_PAYLOAD_SIZE);
}
