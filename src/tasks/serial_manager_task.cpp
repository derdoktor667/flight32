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
        // com_send_log(ComMessageType::LOG_INFO, "MSP timeout. Switched back to Terminal mode."); // Re-commented
        // _terminal->showPrompt(); // Removed explicit call after timeout
    }

    // If in terminal mode and should quit, signal scheduler
    if (_current_mode == ComSerialMode::TERMINAL && _terminal->shouldQuit())
    {
        _should_show_prompt = false; // Prevent prompt when quitting
        // _scheduler->stop(); // Scheduler does not have a stop method. Implement system halt if needed.
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
    MspState prev_state = _msp_state;
    switch (_msp_state)
    {
    case MspState::IDLE:
        if (c == '$')
        {
            _msp_state = MspState::HEADER_START;
            _msp_crc = 0; // Initialize CRC for incoming message
            _last_msp_activity_ms = millis();
        }
        else
        {
            return false; // Not an MSP character, not consumed
        }
        break;
    case MspState::HEADER_START:
        if (c == 'M')
        {
            _msp_state = MspState::HEADER_DIR;
            _last_msp_activity_ms = millis(); // Update activity on 'M'
        }
        else
        {
            _msp_state = MspState::IDLE; // False alarm, reset state
        }
        break;
    case MspState::HEADER_DIR:
        if (c == '<') // Expecting '<' for incoming commands
        {
            _msp_state = MspState::HEADER_SIZE;
            if (_current_mode == ComSerialMode::TERMINAL)
            {
                _current_mode = ComSerialMode::MSP;
                com_set_serial_mode(ComSerialMode::MSP);
                _should_show_prompt = false; // Disable prompt when entering MSP mode
            }
        }
        else if (c == '>')
        {
            // This is likely an echo of our own MSP response, ignore it and reset.
            _msp_state = MspState::IDLE;
        }
        else
        {
            _msp_state = MspState::IDLE; // Invalid direction byte, reset MSP state
        }
        _last_msp_activity_ms = millis(); // Update activity on direction byte
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
    return true; // Character was part of an MSP parsing attempt
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
    case MSP_ATTITUDE: // New ID for MSP_ATTITUDE
        _handle_msp_attitude();
        break;
    case MSP_RC: // Formerly MSP_RC_CHANNELS in Python script's list
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
    case MSP_UID: // Firmware specific
        _handle_msp_uid();
        break;
    case MSP_SENSOR_STATUS: // New ID for MSP_SENSOR_STATUS
        _handle_msp_sensor_status();
        break;
    case MSP_BOXNAMES:
        _handle_msp_boxnames();
        break;
    case MSP_MODE_RANGES:
        _handle_msp_mode_ranges();
        break;
    case MSP_MOTOR_CONFIG:
        _handle_msp_motor_config();
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

    uint8_t payload[MSP_SENSOR_STATUS_PAYLOAD_SIZE]; // Expected 3 bytes (ACC, GYRO, MAG health)
    int i = 0;

    // Report 1 (healthy) or 0 (unhealthy) for each sensor
    payload[i++] = _imu_task->getImuSensor().isSensorHealthy() ? 1 : 0; // ACC health
    payload[i++] = _imu_task->getImuSensor().isSensorHealthy() ? 1 : 0; // GYRO health (assuming same health for now)
    payload[i++] = 0;                                                   // MAG health (MPU6050 does not have magnetometer)

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
    payload[1] = MSP_API_VERSION_MAJOR;
    payload[2] = MSP_API_VERSION_MINOR;
    _send_msp_response(MSP_API_VERSION, payload, MSP_API_VERSION_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_fc_variant()
{
    uint8_t variant_name_len = strlen(MSP_FC_VARIANT_NAME);
    _send_msp_response(MSP_FC_VARIANT, (uint8_t *)MSP_FC_VARIANT_NAME, variant_name_len);
}

void SerialManagerTask::_handle_msp_fc_version()
{
    uint8_t payload[3] = {FC_VERSION_MAJOR, FC_VERSION_MINOR, FC_VERSION_PATCH};
    _send_msp_response(MSP_FC_VERSION, payload, MSP_FC_VERSION_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_board_info()
{
    // Maximum possible size for the payload based on MSP_MAX_PAYLOAD_SIZE
    uint8_t payload[MSP_MAX_PAYLOAD_SIZE] = {0};
    int idx = 0;

    // Board Identifier (4 chars)
    memcpy(&payload[idx], MSP_BOARD_IDENTIFIER, strlen(MSP_BOARD_IDENTIFIER));
    idx += strlen(MSP_BOARD_IDENTIFIER);

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
    char build_info_str[MSP_MAX_PAYLOAD_SIZE];
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
    _write_to_payload<int16_t>(payload, i, _pid_task->getCycleTime());

    // i2c_errors_count
    _write_to_payload<int16_t>(payload, i, _imu_task->getImuSensor().getI2CErrorCount());

    // sensor
    uint16_t sensors = 0;
    if (_imu_task->getImuSensor().isSensorHealthy())
    {
        sensors |= (1 << 0); // ACC
        sensors |= (1 << 5); // GYRO (as per Betaflight's msp.c)
    }
    // No baro or mag in this hardware, so we don't set bits 2 (BARO) or 3 (MAG).
    _write_to_payload<int16_t>(payload, i, sensors);

    // flightModeFlags
    uint32_t flightModeFlags = 0;
    if (_pid_task->isArmed())
    {
        flightModeFlags |= (1 << 0); // ARM
    }
    if (_pid_task->getFlightMode() == FlightMode::STABILIZED)
    {
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
        // com_send_log(ComMessageType::LOG_ERROR, "MSP_GET_SETTING: Unknown setting display key: %s", display_key_str.c_str());
        _send_msp_response(MSP_GET_SETTING, nullptr, 0); // Error: unknown setting
        return;
    }

    String value_str = _settings_manager->getSettingValueHumanReadable(internal_key);
    // com_send_log(ComMessageType::LOG_INFO, "MSP_GET_SETTING: Retrieved %s (internal: %s) = %s", display_key_str.c_str(), internal_key, value_str.c_str());

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

    _write_to_payload<float>(payload, i, _settings_manager->getFloat(NVS_KEY_GYRO_LPF_HZ));
    _write_to_payload<float>(payload, i, _settings_manager->getFloat(NVS_KEY_NOTCH1_HZ));
    _write_to_payload<float>(payload, i, _settings_manager->getFloat(NVS_KEY_NOTCH1_Q));
    _write_to_payload<float>(payload, i, _settings_manager->getFloat(NVS_KEY_NOTCH2_HZ));
    _write_to_payload<float>(payload, i, _settings_manager->getFloat(NVS_KEY_NOTCH2_Q));

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
    _settings_manager->setFloat(NVS_KEY_GYRO_LPF_HZ, _read_from_payload<float>(_msp_payload_buffer, i));
    _settings_manager->setFloat(NVS_KEY_NOTCH1_HZ, _read_from_payload<float>(_msp_payload_buffer, i));
    _settings_manager->setFloat(NVS_KEY_NOTCH1_Q, _read_from_payload<float>(_msp_payload_buffer, i));
    _settings_manager->setFloat(NVS_KEY_NOTCH2_HZ, _read_from_payload<float>(_msp_payload_buffer, i));
    _settings_manager->setFloat(NVS_KEY_NOTCH2_Q, _read_from_payload<float>(_msp_payload_buffer, i));

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
    _write_to_payload<int16_t>(payload, i, (int16_t)(roll_gains.p * PidConfig::SCALE_FACTOR));
    _write_to_payload<int16_t>(payload, i, (int16_t)(roll_gains.i * PidConfig::SCALE_FACTOR));
    _write_to_payload<int16_t>(payload, i, (int16_t)(roll_gains.d * PidConfig::SCALE_FACTOR));

    PidGains pitch_gains = _pid_task->getGains(PidAxis::PITCH);
    _write_to_payload<int16_t>(payload, i, (int16_t)(pitch_gains.p * PidConfig::SCALE_FACTOR));
    _write_to_payload<int16_t>(payload, i, (int16_t)(pitch_gains.i * PidConfig::SCALE_FACTOR));
    _write_to_payload<int16_t>(payload, i, (int16_t)(pitch_gains.d * PidConfig::SCALE_FACTOR));

    PidGains yaw_gains = _pid_task->getGains(PidAxis::YAW);
    _write_to_payload<int16_t>(payload, i, (int16_t)(yaw_gains.p * PidConfig::SCALE_FACTOR));
    _write_to_payload<int16_t>(payload, i, (int16_t)(yaw_gains.i * PidConfig::SCALE_FACTOR));
    _write_to_payload<int16_t>(payload, i, (int16_t)(yaw_gains.d * PidConfig::SCALE_FACTOR));

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
    roll_gains.p = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
    roll_gains.i = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
    roll_gains.d = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
    _pid_task->setGains(PidAxis::ROLL, roll_gains);

    PidGains pitch_gains = _pid_task->getGains(PidAxis::PITCH);
    pitch_gains.p = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
    pitch_gains.i = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
    pitch_gains.d = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
    _pid_task->setGains(PidAxis::PITCH, pitch_gains);

    PidGains yaw_gains = _pid_task->getGains(PidAxis::YAW);
    yaw_gains.p = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
    yaw_gains.i = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
    yaw_gains.d = (float)_read_from_payload<int16_t>(_msp_payload_buffer, i) / PidConfig::SCALE_FACTOR;
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

    _write_to_payload<int16_t>(payload, i, accX);
    _write_to_payload<int16_t>(payload, i, accY);
    _write_to_payload<int16_t>(payload, i, accZ);

    _write_to_payload<int16_t>(payload, i, gyroX);
    _write_to_payload<int16_t>(payload, i, gyroY);
    _write_to_payload<int16_t>(payload, i, gyroZ);

    _write_to_payload<int16_t>(payload, i, magX);
    _write_to_payload<int16_t>(payload, i, magY);
    _write_to_payload<int16_t>(payload, i, magZ);

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
    int16_t msp_yaw = (int16_t)(yaw_f * MSP_ATTITUDE_SCALE_FACTOR); // Yaw should also be scaled by 10 in MSP, like roll and pitch

    uint8_t payload[MSP_ATTITUDE_PAYLOAD_SIZE]; // 3x angles (int16_t = 2 bytes each)
    int i = 0;

    _write_to_payload<int16_t>(payload, i, msp_roll);
    _write_to_payload<int16_t>(payload, i, msp_pitch);
    _write_to_payload<int16_t>(payload, i, msp_yaw);

    _send_msp_response(MSP_ATTITUDE, payload, MSP_ATTITUDE_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_rc()
{
    if (!_rx_task)
    {
        _send_msp_response(MSP_RC, nullptr, 0);
        return;
    }

    uint8_t payload[PPM_MAX_CHANNELS * 2]; // PPM_MAX_CHANNELS * 2 bytes each
    int i = 0;

    for (int j = 0; j < PPM_MAX_CHANNELS; j++)
    {
        int16_t channel_value = _rx_task->getChannel(j); // getChannel should return 1500 for unmapped/inactive channels
        _write_to_payload<int16_t>(payload, i, channel_value);
    }

    _send_msp_response(MSP_RC, payload, PPM_MAX_CHANNELS * 2);
}

void SerialManagerTask::_handle_msp_motor()
{
    if (!_motor_task)
    {
        _send_msp_response(MSP_MOTOR, nullptr, 0);
        return;
    }

    uint8_t payload[MSP_MAX_MOTORS * 2]; // 8x motor outputs (int16_t = 2 bytes each)
    int i = 0;

    for (int j = 0; j < MSP_MAX_MOTORS; j++)
    {
        uint16_t motor_output = 0;
        if (j < NUM_MOTORS)
        {
            motor_output = _motor_task->getMotorOutput(j);
        }
        _write_to_payload<int16_t>(payload, i, motor_output);
    }

    _send_msp_response(MSP_MOTOR, payload, MSP_MAX_MOTORS * 2);
}

void SerialManagerTask::_handle_msp_box_get()
{
    if (!_pid_task)
    {
        _send_msp_response(MSP_BOX, nullptr, 0);
        return;
    }

    uint32_t active_boxes_bitmask = 0;

    // Set ARM bit if armed
    if (_pid_task->isArmed())
    {
        active_boxes_bitmask |= (1 << BF_PERMANENT_ID_ARM);
    }

    // Set ANGLE bit if in STABILIZED mode
    if (_pid_task->getFlightMode() == FlightMode::STABILIZED)
    {
        active_boxes_bitmask |= (1 << BF_PERMANENT_ID_ANGLE);
    }

    // ACRO is typically the default mode when no other modes are active.
    // There isn't a direct "Acro" permanent ID that Betaflight uses in this context,
    // so we won't set a specific bit for BF_PERMANENT_ID_ACRO here unless it maps
    // to a specific Betaflight BOX ID in the `packFlightModeFlags` equivalent.

    uint8_t payload[MSP_BOX_PAYLOAD_SIZE];
    payload[0] = (active_boxes_bitmask >> 0) & 0xFF;
    payload[1] = (active_boxes_bitmask >> 8) & 0xFF;
    payload[2] = (active_boxes_bitmask >> 16) & 0xFF;
    payload[3] = (active_boxes_bitmask >> 24) & 0xFF;

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
    uint16_t mode_id = _read_from_payload<uint16_t>(_msp_payload_buffer, i);

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

void SerialManagerTask::_handle_msp_boxnames()
{
    // Build a semicolon-separated string of box names
    String box_names_str = "";

    // ARM
    box_names_str += "ARM;";
    // ANGLE (Stabilized)
    box_names_str += "ANGLE;";
    // ACRO
    box_names_str += "ACRO;";

    // We need to ensure the string is null-terminated and fits within MSP_MAX_PAYLOAD_SIZE
    if (box_names_str.length() >= MSP_MAX_PAYLOAD_SIZE)
    {
        // Handle error or truncate if necessary
        // For now, send empty response if too long
        _send_msp_response(MSP_BOXNAMES, nullptr, 0);
        return;
    }

    // Convert String to byte array (uint8_t*)
    uint8_t payload[box_names_str.length() + 1]; // +1 for null terminator
    box_names_str.getBytes(payload, box_names_str.length() + 1);

    _send_msp_response(MSP_BOXNAMES, payload, box_names_str.length());
}

void SerialManagerTask::_handle_msp_mode_ranges()
{
    // Payload format: [permanentId (1 byte)][auxChannelIndex (1 byte)][rangeStartStep (1 byte)][rangeEndStep (1 byte)]
    // The Python script expects a variable number of 4-byte entries, not a count-prefixed payload.
    uint8_t payload[MSP_MODE_RANGES_PAYLOAD_SIZE];
    int i = 0;

    // Mode 1: ARM
    payload[i++] = BF_PERMANENT_ID_ARM;
    payload[i++] = MSP_MODE_RANGE_ARM_AUX_CHANNEL;
    payload[i++] = MSP_MODE_RANGE_ARM_START_STEP;
    payload[i++] = MSP_MODE_RANGE_ARM_END_STEP;

    // Mode 2: FAILSAFE
    payload[i++] = BF_PERMANENT_ID_FAILSAFE;
    payload[i++] = MSP_MODE_RANGE_FAILSAFE_AUX_CHANNEL;
    payload[i++] = MSP_MODE_RANGE_FAILSAFE_START_STEP;
    payload[i++] = MSP_MODE_RANGE_FAILSAFE_END_STEP;

    // Mode 3: TELEMETRY
    payload[i++] = BF_PERMANENT_ID_TELEMETRY;
    payload[i++] = MSP_MODE_RANGE_TELEMETRY_AUX_CHANNEL;
    payload[i++] = MSP_MODE_RANGE_TELEMETRY_START_STEP;
    payload[i++] = MSP_MODE_RANGE_TELEMETRY_END_STEP;

    // Mode 4: FLIP OVER AFTER CRASH
    payload[i++] = BF_PERMANENT_ID_FLIPOVERAFTERCRASH;
    payload[i++] = MSP_MODE_RANGE_FLIPOVERAFTERCRASH_AUX_CHANNEL;
    payload[i++] = MSP_MODE_RANGE_FLIPOVERAFTERCRASH_START_STEP;
    payload[i++] = MSP_MODE_RANGE_FLIPOVERAFTERCRASH_END_STEP;

    _send_msp_response(MSP_MODE_RANGES, payload, MSP_MODE_RANGES_PAYLOAD_SIZE);
}

void SerialManagerTask::_handle_msp_motor_config()
{
    uint8_t payload[MSP_MOTOR_CONFIG_PAYLOAD_SIZE];
    int i = 0;

    // 1. minthrottle (U16) - In Flight32, this corresponds to MOTOR_MIN_THROTTLE_RAW
    _write_to_payload<uint16_t>(payload, i, MOTOR_MIN_THROTTLE_RAW);

    // 2. maxthrottle (U16) - In Flight32, this corresponds to MOTOR_MAX_THROTTLE_RAW
    _write_to_payload<uint16_t>(payload, i, MOTOR_MAX_THROTTLE_RAW);

    // 3. mincommand (U16) - In Flight32, this is also MOTOR_MIN_THROTTLE_RAW for simplicity
    _write_to_payload<uint16_t>(payload, i, MOTOR_MIN_THROTTLE_RAW);

    _send_msp_response(MSP_MOTOR_CONFIG, payload, MSP_MOTOR_CONFIG_PAYLOAD_SIZE);
}
