/**
 * @file msp_processor.cpp
 * @brief Processes MSP commands.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */
#include "msp_processor.h"
#include "Arduino.h"
#include "../settings_manager.h"
#include "../flight_controller.h"
#include "../utils/system_state.h"


#include "../imu/imu_sensor.h"
#include "../pid/pid_controller.h"
#include <esp_system.h>
#include "../config/rx_config.h"
#include "../config/pid_config.h"
#include "../config/motor_config.h"
#include "../config/filter_config.h"
#include "../config/serial_config.h"

#include <cmath>
#include <cstring>



MspProcessor::MspProcessor(FlightController *flightController, PidTask *pidTask, ImuTask *imuTask, RxTask *rxTask, MotorTask *motorTask, SettingsManager *settingsManager)
    : _flightController(flightController),
      _pidTask(pidTask),
      _imuTask(imuTask),
      _rxTask(rxTask),
      _motorTask(motorTask),
      _settingsManager(settingsManager)
{
}

// Helper function to convert quaternion to Euler angles (roll, pitch, yaw)
// Angles are in degrees
static void quaternionToEuler(float w, float x, float y, float z, float *roll, float *pitch, float *yaw)
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

// CRC8 for MSP V2
static uint8_t _crc8_dvb_s2(uint8_t crc, uint8_t ch)
{
    crc ^= ch;
    for (uint8_t ii = 0; ii < 8; ii++)
    {
        if (crc & 0x80)
        {
            crc = ((crc << 1) & 0xFF) ^ 0xD5;
        }
        else
        {
            crc = (crc << 1) & 0xFF;
        }
    }
    return crc;
}

void MspProcessor::send_msp_response(uint8_t command, const uint8_t *payload, uint8_t payload_size, uint8_t protocol_version)
{
    if (protocol_version == 1)
    { // MSP V1 Response
        Serial.write(MSP_SYNC_BYTE);
        Serial.write(MSP_V1_IDENTIFIER);
        Serial.write(MSP_DIRECTION_FC);

        uint8_t crc = 0;
        Serial.write((uint8_t)payload_size); // V1 size is 1 byte
        crc ^= (uint8_t)payload_size;
        Serial.write((uint8_t)command); // V1 command is 1 byte
        crc ^= (uint8_t)command;

        for (uint16_t i = 0; i < payload_size; i++)
        {
            Serial.write(payload[i]);
            crc ^= payload[i];
        }
        Serial.write(crc);
    }
    else
    { // MSP V2 Response
        // Flag byte (currently always 0, for future use)
        uint8_t flag = 0;

        // CRC calculation for MSP v2
        uint8_t crc = 0;
        crc = _crc8_dvb_s2(crc, flag);
        crc = _crc8_dvb_s2(crc, (uint8_t)(command & 0xFF));
        crc = _crc8_dvb_s2(crc, (uint8_t)((command >> 8) & 0xFF));
        crc = _crc8_dvb_s2(crc, (uint8_t)(payload_size & 0xFF));
        crc = _crc8_dvb_s2(crc, (uint8_t)((payload_size >> 8) & 0xFF));

        Serial.write(MSP_SYNC_BYTE);
        Serial.write(MSP_V2_IDENTIFIER); // MSP V2 Protocol identifier
        Serial.write(MSP_DIRECTION_FC);   // Direction byte for response (from FC to GUI)
        Serial.write(flag);
        Serial.write((uint8_t)(command & 0xFF));             // Command ID Low Byte
        Serial.write((uint8_t)((command >> 8) & 0xFF));      // Command ID High Byte
        Serial.write((uint8_t)(payload_size & 0xFF));        // Payload Size Low Byte
        Serial.write((uint8_t)((payload_size >> 8) & 0xFF)); // Payload Size High Byte

        for (uint16_t i = 0; i < payload_size; i++)
        {
            Serial.write(payload[i]);
            crc = _crc8_dvb_s2(crc, payload[i]); // Add payload byte to CRC
        }
        Serial.write(crc);
    }

    delayMicroseconds(MSP_RESPONSE_DELAY_US);
    Serial.flush();
}

void MspProcessor::send_msp_error(uint8_t command, uint8_t protocol_version)
{
    // For now, send an empty response for unsupported commands
    send_msp_response(command, nullptr, 0, protocol_version);
} // Added missing closing brace

// --- Command Processors ---

void MspProcessor::process_api_version_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_API_VERSION_PAYLOAD_SIZE];
    payload[0] = MSP_PROTOCOL_VERSION;
    payload[1] = MSP_API_VERSION_MAJOR;
    payload[2] = MSP_API_VERSION_MINOR;
    send_msp_response(MSP_API_VERSION, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_fc_variant_command(uint8_t protocol_version)
{
    // Example: "FLTX" for Flight32, X is a specific variant if needed
    // For now, send "BTFL" to match Betaflight's expected response
    uint8_t payload[4]; // "FLT32" is 5 chars, but MSP_FC_VARIANT expects 4.
    memcpy(payload, MSP_FC_VARIANT_NAME, 4);
    send_msp_response(MSP_FC_VARIANT, payload, 4, protocol_version);
}

void MspProcessor::process_fc_version_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_FC_VERSION_PAYLOAD_SIZE];
    payload[0] = FC_VERSION_MAJOR;
    payload[1] = FC_VERSION_MINOR;
    payload[2] = FC_VERSION_PATCH;
    send_msp_response(MSP_FC_VERSION, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_board_info_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_BOARD_INFO_PAYLOAD_SIZE]; // FLT3 (4 bytes) + HW_REV (2 bytes) + BOARD_TYPE (1 byte) + TARGET_CAP (4 bytes) + Board Name (null terminator)
    // Board Identifier (4 characters)
    memcpy(payload, MSP_FC_VARIANT_NAME, 4);

    // Hardware Revision (2 bytes, placeholder 0)
    payload[BOARD_INFO_HW_REV_BYTE_INDEX] = 0;
    payload[BOARD_INFO_HW_REV_BYTE_INDEX + 1] = 0;

    // Board Type (1 byte, placeholder 0)
    payload[BOARD_INFO_BOARD_TYPE_BYTE_INDEX] = 0;

    // Target Capabilities (4 bytes, placeholder 0)
    payload[BOARD_INFO_TARGET_CAP_START_INDEX] = 0;
    payload[BOARD_INFO_TARGET_CAP_START_INDEX + 1] = 0;
    payload[BOARD_INFO_TARGET_CAP_START_INDEX + 2] = 0;
    payload[BOARD_INFO_TARGET_CAP_START_INDEX + 3] = 0;

    // Board Name (null-terminated empty string - 1 byte for null terminator)
    payload[BOARD_INFO_BOARD_NAME_NULL_TERMINATOR_INDEX] = '\0';

    send_msp_response(MSP_BOARD_INFO, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_build_info_command(uint8_t protocol_version)
{
    // Format: "MMM DD YYYY HH:MM:SSFLT3"
    // Example: "Nov 21 2025 12:00:00FLT3"
    char build_info[MSP_BUILD_INFO_BUFFER_SIZE]; // 26 characters + null terminator
    snprintf(build_info, sizeof(build_info), "%s %s%s", __DATE__, __TIME__, MSP_FC_VARIANT_NAME);
    uint8_t payload_size = strlen(build_info);
    uint8_t payload[payload_size];
    memcpy(payload, build_info, payload_size);
    send_msp_response(MSP_BUILD_INFO, payload, payload_size, protocol_version);
}

void MspProcessor::process_reboot_command(uint8_t protocol_version)
{
    // This command reboots the FC.
    // For now, just send a response and don't actually reboot.
    send_msp_response(MSP_REBOOT, nullptr, 0, protocol_version);
    // ESP.restart(); // Uncomment to enable actual reboot
}

void MspProcessor::process_mem_stats_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_MEM_STATS_PAYLOAD_SIZE];
    uint32_t free_heap = ESP.getFreeHeap();
    payload[0] = free_heap & 0xFF;
    payload[1] = (free_heap >> 8) & 0xFF;
    payload[2] = (free_heap >> 16) & 0xFF;
    payload[3] = (free_heap >> 24) & 0xFF;
    send_msp_response(MSP_MEM_STATS, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_ident_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_IDENT_PAYLOAD_SIZE];
    uint8_t current_byte_idx = 0;

    // Protocol Version
    payload[current_byte_idx++] = MSP_PROTOCOL_VERSION;

    // Flight Controller Type (0 for Flight32, or specific value for Betaflight)
    payload[current_byte_idx++] = MSP_FC_TYPE_FLIGHT32; // Using custom ID for now

    // Flight Controller Version (Major, Minor, Patch)
    payload[current_byte_idx++] = FC_VERSION_MAJOR;
    payload[current_byte_idx++] = FC_VERSION_MINOR;
    payload[current_byte_idx++] = FC_VERSION_PATCH;

    // Board Type (0 for Flight32, or specific value for Betaflight)
    payload[current_byte_idx++] = MSP_BOARD_TYPE_FLIGHT32; // Using custom ID for now

    // Capabilities (bitmask)
    uint8_t capabilities = 0;
    // Assuming ACC and GYRO are always present with MPU6050
    capabilities |= MSP_CAPABILITY_ACC_BIT;
    capabilities |= MSP_CAPABILITY_GYRO_BIT; // Indicate Gyro presence
    // Note: Gyro is not typically a separate capability flag in MSP_IDENT, but rather IMU presence.
    // We indicate ACC as a general IMU presence.
    // If we had a magnetometer, we'd add MSP_CAPABILITY_MAG_BIT, etc.

    payload[current_byte_idx++] = capabilities;

    send_msp_response(MSP_IDENT, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_get_setting_command(const uint8_t *payload, uint8_t payload_size, uint8_t protocol_version)
{
    if (payload_size < 1)
    { // key_length
        send_msp_error(MSP_GET_SETTING, protocol_version);
        return;
    }

    uint8_t key_len = payload[0];
    if (key_len + 1 > payload_size || key_len == 0)
    { // +1 for key_len byte itself
        send_msp_error(MSP_GET_SETTING, protocol_version);
        return;
    }

    char key[key_len + 1];
    memcpy(key, payload + 1, key_len);
    key[key_len] = '\0'; // Null-terminate the key

    String value_str = _settingsManager->getSettingValue(key);
    if (value_str.length() > 0)
    {
        // Response format: key_len (1 byte) + key + value_len (1 byte) + value
        uint8_t response_key_len = key_len;
        uint8_t response_value_len = value_str.length();
        uint8_t response_payload_size = 1 + response_key_len + 1 + response_value_len;
        uint8_t response_payload[response_payload_size];

        response_payload[0] = response_key_len;
        memcpy(response_payload + 1, key, response_key_len);
        response_payload[1 + response_key_len] = response_value_len;
        memcpy(response_payload + 1 + response_key_len + 1, value_str.c_str(), response_value_len);

        send_msp_response(MSP_GET_SETTING, response_payload, response_payload_size, protocol_version);
    }
    else
    {
        send_msp_response(MSP_GET_SETTING, nullptr, 0, protocol_version); // Setting not found
    }
}

void MspProcessor::process_set_setting_command(const uint8_t *payload, uint8_t payload_size, uint8_t protocol_version)
{
    if (payload_size < MSP_SETTING_KEY_VALUE_OVERHEAD_BYTES)
    {
        send_msp_error(MSP_SET_SETTING, protocol_version);
        return;
    }

    uint8_t key_len = payload[0];
    if (key_len + 1 > payload_size)
    { // +1 for key_len byte
        send_msp_error(MSP_SET_SETTING, protocol_version);
        return;
    }

    char key[key_len + 1];
    memcpy(key, payload + 1, key_len);
    key[key_len] = '\0'; // Null-terminate the key

    uint8_t value_len_offset = 1 + key_len;
    if (value_len_offset >= payload_size)
    {
        send_msp_error(MSP_SET_SETTING, protocol_version);
        return;
    }

    uint8_t value_len = payload[value_len_offset];
    if (value_len_offset + 1 + value_len > payload_size)
    { // +1 for value_len byte
        send_msp_error(MSP_SET_SETTING, protocol_version);
        return;
    }

    char value[value_len + 1];
    memcpy(value, payload + value_len_offset + 1, value_len);
    value[value_len] = '\0'; // Null-terminate the value

    _settingsManager->setSettingValue(key, value);
    send_msp_response(MSP_SET_SETTING, nullptr, 0, protocol_version); // Acknowledge
}

void MspProcessor::process_pid_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_PID_PAYLOAD_SIZE];
    uint16_t p_gain, i_gain, d_gain;
    uint8_t current_byte_idx = 0;

    // Roll PID
    p_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_R_P) * PidConfig::SCALE_FACTOR);
    i_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_R_I) * PidConfig::SCALE_FACTOR);
    d_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_R_D) * PidConfig::SCALE_FACTOR);
    payload[current_byte_idx++] = p_gain & 0xFF;
    payload[current_byte_idx++] = (p_gain >> 8) & 0xFF;
    payload[current_byte_idx++] = i_gain & 0xFF;
    payload[current_byte_idx++] = (i_gain >> 8) & 0xFF;
    payload[current_byte_idx++] = d_gain & 0xFF;
    payload[current_byte_idx++] = (d_gain >> 8) & 0xFF;

    // Pitch PID
    p_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_P_P) * PidConfig::SCALE_FACTOR);
    i_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_P_I) * PidConfig::SCALE_FACTOR);
    d_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_P_D) * PidConfig::SCALE_FACTOR);
    payload[current_byte_idx++] = p_gain & 0xFF;
    payload[current_byte_idx++] = (p_gain >> 8) & 0xFF;
    payload[current_byte_idx++] = i_gain & 0xFF;
    payload[current_byte_idx++] = (i_gain >> 8) & 0xFF;
    payload[current_byte_idx++] = d_gain & 0xFF;
    payload[current_byte_idx++] = (d_gain >> 8) & 0xFF;

    // Yaw PID
    p_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_Y_P) * PidConfig::SCALE_FACTOR);
    i_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_Y_I) * PidConfig::SCALE_FACTOR);
    d_gain = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_PID_Y_D) * PidConfig::SCALE_FACTOR);
    payload[current_byte_idx++] = p_gain & 0xFF;
    payload[current_byte_idx++] = (p_gain >> 8) & 0xFF;
    payload[current_byte_idx++] = i_gain & 0xFF;
    payload[current_byte_idx++] = (i_gain >> 8) & 0xFF;
    payload[current_byte_idx++] = d_gain & 0xFF;
    payload[current_byte_idx++] = (d_gain >> 8) & 0xFF;

    send_msp_response(MSP_PID, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_status_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_STATUS_PAYLOAD_SIZE];
    uint16_t cycleTime = _pidTask->getCycleTime(); // Flight loop cycle time
    uint16_t i2cErrors = _imuTask->getImuSensor().getI2CErrorCount();

    payload[0] = cycleTime & 0xFF;
    payload[1] = (cycleTime >> 8) & 0xFF;
    payload[2] = i2cErrors & 0xFF;
    payload[3] = (i2cErrors >> 8) & 0xFF;

    // Sensors present (bitmask): 1 = ACC, 2 = BARO, 4 = MAG, 8 = SONAR, 16 = GPS, 32 = OPTIC_FLOW
    // MPU6050/6500 provides acc and gyro.
    payload[4] = MSP_STATUS_ACCEL_SENSOR_FLAG | MSP_STATUS_GYRO_SENSOR_FLAG; // ACCELEROMETER | GYROSCOPE

    uint32_t flight_mode_flags = 0;
    if (_pidTask->getFlightMode() == FlightMode::ANGLE || _pidTask->getFlightMode() == FlightMode::STABILIZED)
    {
        flight_mode_flags |= (1 << BF_PERMANENT_ID_ANGLE); // ANGLE mode (ID 1)
    }
    // Add other flight modes (like ACRO, if it has a permanent ID) as needed.

    payload[5] = flight_mode_flags & 0xFF;
    payload[6] = (flight_mode_flags >> 8) & 0xFF;
    payload[7] = (flight_mode_flags >> 16) & 0xFF;
    payload[8] = (flight_mode_flags >> 24) & 0xFF;

    payload[9] = 0;  // Current profile (always 0 for now)
    payload[10] = MSP_BRAKING_ACTION_NONE; // Braking action (0=none, 1=active)

    send_msp_response(MSP_STATUS, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_raw_imu_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_RAW_IMU_PAYLOAD_SIZE];

    // Accel
    int16_t accX = _imuTask->getImuSensor().getData().accelX;
    int16_t accY = _imuTask->getImuSensor().getData().accelY;
    int16_t accZ = _imuTask->getImuSensor().getData().accelZ;

    payload[0] = accX & 0xFF;
    payload[1] = (accX >> 8) & 0xFF;
    payload[2] = accY & 0xFF;
    payload[3] = (accY >> 8) & 0xFF;
    payload[4] = accZ & 0xFF;
    payload[5] = (accZ >> 8) & 0xFF;

    // Gyro
    int16_t gyroX = _imuTask->getImuSensor().getData().gyroX;
    int16_t gyroY = _imuTask->getImuSensor().getData().gyroY;
    int16_t gyroZ = _imuTask->getImuSensor().getData().gyroZ;

    payload[6] = gyroX & 0xFF;
    payload[7] = (gyroX >> 8) & 0xFF;
    payload[8] = gyroY & 0xFF;
    payload[9] = (gyroY >> 8) & 0xFF;
    payload[10] = gyroZ & 0xFF;
    payload[11] = (gyroZ >> 8) & 0xFF;

    // Mag (Magnetometer) - Not implemented yet, send zeros
    for (uint8_t i = 0; i < MSP_RAW_IMU_MAG_BYTES; i++)
    {
        payload[12 + i] = 0;
    }

    send_msp_response(MSP_RAW_IMU, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_motor_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_MOTOR_PAYLOAD_SIZE];
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        uint16_t motor_output = _motorTask->getMotorOutput(i);
        payload[i * 2] = motor_output & 0xFF;
        payload[i * 2 + 1] = (motor_output >> 8) & 0xFF;
    }
    // If less than MSP_MAX_MOTORS, fill with zeros
    for (uint8_t i = NUM_MOTORS; i < MSP_MAX_MOTORS; i++)
    {
        payload[i * 2] = 0;
        payload[i * 2 + 1] = 0;
    }
    send_msp_response(MSP_MOTOR, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_rc_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_RC_PAYLOAD_SIZE];
    for (uint8_t i = 0; i < PPM_MAX_CHANNELS; i++)
    {
        uint16_t rc_value = _rxTask->getChannel(i);
        payload[i * 2] = rc_value & 0xFF;
        payload[i * 2 + 1] = (rc_value >> 8) & 0xFF;
    }
    // Fill remaining channels with 1500 (neutral) if NUM_RC_CHANNELS < 18
    for (uint8_t i = PPM_MAX_CHANNELS; i < MSP_RC_CHANNEL_COUNT; i++)
    {
        payload[i * 2] = MSP_RC_NEUTRAL_CHANNEL_VALUE & 0xFF;     // 1500 & 0xFF
        payload[i * 2 + 1] = (MSP_RC_NEUTRAL_CHANNEL_VALUE >> 8) & 0xFF; // 1500 >> 8 & 0xFF
    }
    send_msp_response(MSP_RC, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_attitude_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_ATTITUDE_PAYLOAD_SIZE];
    ImuQuaternionData q = _imuTask->getImuSensor().getQuaternion();
    float roll_f, pitch_f, yaw_f;
    quaternionToEuler(q.w, q.x, q.y, q.z, &roll_f, &pitch_f, &yaw_f);

    int16_t roll = static_cast<int16_t>(roll_f * MSP_ATTITUDE_SCALE_FACTOR);
    int16_t pitch = static_cast<int16_t>(pitch_f * MSP_ATTITUDE_SCALE_FACTOR);
    int16_t yaw = static_cast<int16_t>(yaw_f * MSP_ATTITUDE_SCALE_FACTOR);

    payload[0] = roll & 0xFF;
    payload[1] = (roll >> 8) & 0xFF;
    payload[2] = pitch & 0xFF;
    payload[3] = (pitch >> 8) & 0xFF;
    payload[4] = yaw & 0xFF;
    payload[5] = (yaw >> 8) & 0xFF;

    send_msp_response(MSP_ATTITUDE, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_box_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_BOX_PAYLOAD_SIZE];
    uint32_t box_flags = 0;

    if (_flightController->getSystemState() == SystemState::ARMED)
    {
        box_flags |= (1 << BF_PERMANENT_ID_ARM);
    }

    FlightMode currentFlightMode = _pidTask->getFlightMode();
    if (currentFlightMode == FlightMode::ANGLE || currentFlightMode == FlightMode::STABILIZED)
    {
        box_flags |= (1 << BF_PERMANENT_ID_ANGLE);
    }
    else if (currentFlightMode == FlightMode::ACRO)
    {
        box_flags |= (1 << BF_PERMANENT_ID_ACRO);
    }
    // All other BF_PERMANENT_ID_ constants (HORIZON, HEADFREE, BEEPER, etc.) will have their flags as 0
    // unless explicitly implemented and active in the Flight32 firmware.

    payload[0] = box_flags & 0xFF;
    payload[1] = (box_flags >> 8) & 0xFF;
    payload[2] = (box_flags >> 16) & 0xFF;
    payload[3] = (box_flags >> 24) & 0xFF;

    send_msp_response(MSP_BOX, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_boxnames_command(uint8_t protocol_version)
{
    // This command sends a string of box names separated by null terminators.
    // Each box name is associated with an ID in Betaflight.
    // Format: "ID;NAME;ID;NAME;"

    String boxnames_str = "";
    boxnames_str += String(BF_PERMANENT_ID_ARM) + ";ARM;";
    boxnames_str += String(BF_PERMANENT_ID_ANGLE) + ";ANGLE;";
    boxnames_str += String(BF_PERMANENT_ID_HORIZON) + ";HORIZON;";
    boxnames_str += String(BF_PERMANENT_ID_HEADFREE) + ";HEADFREE;";
    boxnames_str += String(BF_PERMANENT_ID_HEADADJ) + ";HEADADJ;";
    boxnames_str += String(BF_PERMANENT_ID_FAILSAFE) + ";FAILSAFE;";
    boxnames_str += String(BF_PERMANENT_ID_BEEPER) + ";BEEPER;";
    boxnames_str += String(BF_PERMANENT_ID_OSDDISABLESW) + ";OSD DISABLE SW;";
    boxnames_str += String(BF_PERMANENT_ID_TELEMETRY) + ";TELEMETRY;";
    boxnames_str += String(BF_PERMANENT_ID_BLACKBOX) + ";BLACKBOX;";
    boxnames_str += String(BF_PERMANENT_ID_FPVANGLEMIX) + ";FPV ANGLE MIX;";
    boxnames_str += String(BF_PERMANENT_ID_BLACKBOXERASE) + ";BLACKBOX ERASE (>30s);";
    boxnames_str += String(BF_PERMANENT_ID_FLIPOVERAFTERCRASH) + ";FLIP OVER AFTER CRASH;";
    boxnames_str += String(BF_PERMANENT_ID_PREARM) + ";PREARM;";
    boxnames_str += String(BF_PERMANENT_ID_PARALYZE) + ";PARALYZE;";
    boxnames_str += String(BF_PERMANENT_ID_LAUNCHCONTROL) + ";LAUNCH CONTROL;";

    // The payload needs to be a byte array
    uint16_t payload_size = boxnames_str.length();
    uint8_t payload[payload_size];
    memcpy(payload, boxnames_str.c_str(), payload_size);

    send_msp_response(MSP_BOXNAMES, payload, payload_size, protocol_version);
}

void MspProcessor::process_mode_ranges_command(uint8_t protocol_version)
{
    // Each range is 4 bytes: permanent_id (1), aux_channel (1), range_start (1), range_end (1)
    // Betaflight typically sends 4 ranges (ARM, ANGLE, FAILSAFE, TELEMETRY, FLIPOVERAFTERCRASH)
    uint8_t payload[MSP_MODE_RANGES_PAYLOAD_SIZE * 5]; // 5 ranges * 4 bytes each
    uint8_t current_byte = 0;

    // ARM
    payload[current_byte++] = BF_PERMANENT_ID_ARM;
    payload[current_byte++] = MSP_MODE_RANGE_ARM_AUX_CHANNEL;
    payload[current_byte++] = MSP_MODE_RANGE_ARM_START_STEP;
    payload[current_byte++] = MSP_MODE_RANGE_ARM_END_STEP;

    // ANGLE
    payload[current_byte++] = BF_PERMANENT_ID_ANGLE;
    payload[current_byte++] = MSP_MODE_RANGE_ANGLE_AUX_CHANNEL;
    payload[current_byte++] = MSP_MODE_RANGE_ANGLE_START_STEP;
    payload[current_byte++] = MSP_MODE_RANGE_ANGLE_END_STEP;

    // FAILSAFE
    payload[current_byte++] = BF_PERMANENT_ID_FAILSAFE;
    payload[current_byte++] = MSP_MODE_RANGE_FAILSAFE_AUX_CHANNEL;
    payload[current_byte++] = MSP_MODE_RANGE_FAILSAFE_START_STEP;
    payload[current_byte++] = MSP_MODE_RANGE_FAILSAFE_END_STEP;

    // TELEMETRY
    payload[current_byte++] = BF_PERMANENT_ID_TELEMETRY;
    payload[current_byte++] = MSP_MODE_RANGE_TELEMETRY_AUX_CHANNEL;
    payload[current_byte++] = MSP_MODE_RANGE_TELEMETRY_START_STEP;
    payload[current_byte++] = MSP_MODE_RANGE_TELEMETRY_END_STEP;

    // FLIPOVERAFTERCRASH
    payload[current_byte++] = BF_PERMANENT_ID_FLIPOVERAFTERCRASH;
    payload[current_byte++] = MSP_MODE_RANGE_FLIPOVERAFTERCRASH_AUX_CHANNEL;
    payload[current_byte++] = MSP_MODE_RANGE_FLIPOVERAFTERCRASH_START_STEP;
    payload[current_byte++] = MSP_MODE_RANGE_FLIPOVERAFTERCRASH_END_STEP;

    send_msp_response(MSP_MODE_RANGES, payload, current_byte, protocol_version);
}

void MspProcessor::process_motor_config_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_MOTOR_CONFIG_PAYLOAD_SIZE];

    uint16_t min_throttle = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_MOTOR_MIN_THROTTLE));
    uint16_t max_throttle = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_MOTOR_MAX_THROTTLE));
    uint16_t min_command = static_cast<uint16_t>(_settingsManager->getFloat(NVS_KEY_MOTOR_MIN_COMMAND));

    payload[0] = min_throttle & 0xFF;
    payload[1] = (min_throttle >> 8) & 0xFF;
    payload[2] = max_throttle & 0xFF;
    payload[3] = (max_throttle >> 8) & 0xFF;
    payload[4] = min_command & 0xFF;
    payload[5] = (min_command >> 8) & 0xFF;

    send_msp_response(MSP_MOTOR_CONFIG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_uid_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_UID_PAYLOAD_SIZE];

    uint64_t chip_id = ESP.getEfuseMac(); // 64-bit MAC address

    payload[0] = (uint8_t)(chip_id);
    payload[1] = (uint8_t)(chip_id >> 8);
    payload[2] = (uint8_t)(chip_id >> 16);
    payload[3] = (uint8_t)(chip_id >> 24);
    payload[4] = (uint8_t)(chip_id >> 32);
    payload[5] = (uint8_t)(chip_id >> 40);
    payload[6] = (uint8_t)(chip_id >> 48);
    payload[7] = (uint8_t)(chip_id >> 56);

    // Fill remaining with zeros if MSP_UID_PAYLOAD_SIZE > 8 bytes
    for (uint8_t i = 8; i < MSP_UID_PAYLOAD_SIZE; i++)
    {
        payload[i] = 0;
    }

    send_msp_response(MSP_UID, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_sensor_status_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_SENSOR_STATUS_PAYLOAD_SIZE];
    payload[0] = 0; // Accel calibration status (placeholder)
    payload[1] = 0; // Gyro calibration status (placeholder)
    payload[2] = 0; // Magnetometer calibration status (not implemented)
    send_msp_response(MSP_SENSOR_STATUS, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_eeprom_write_command(uint8_t protocol_version)
{
    _settingsManager->saveSettings();
    send_msp_response(MSP_EEPROM_WRITE, nullptr, 0, protocol_version); // Acknowledge write
}

void MspProcessor::process_reset_settings_command(uint8_t protocol_version)
{
    _settingsManager->factoryReset();
    send_msp_response(MSP_RESET_SETTINGS, nullptr, 0, protocol_version); // Acknowledge reset
}

void MspProcessor::process_set_pid_command(const uint8_t *payload, uint8_t payload_size, uint8_t protocol_version)
{
    if (payload_size != MSP_PID_PAYLOAD_SIZE)
    {
        send_msp_error(MSP_SET_PID, protocol_version);
        return;
    }

    uint16_t p_gain, i_gain, d_gain;
    uint8_t current_byte_idx = 0;

    // Roll PID
    p_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    i_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    d_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    _settingsManager->setFloat(NVS_KEY_PID_R_P, static_cast<float>(p_gain) / PidConfig::SCALE_FACTOR);
    _settingsManager->setFloat(NVS_KEY_PID_R_I, static_cast<float>(i_gain) / PidConfig::SCALE_FACTOR);
    _settingsManager->setFloat(NVS_KEY_PID_R_D, static_cast<float>(d_gain) / PidConfig::SCALE_FACTOR);

    // Pitch PID
    p_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    i_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    d_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    _settingsManager->setFloat(NVS_KEY_PID_P_P, static_cast<float>(p_gain) / PidConfig::SCALE_FACTOR);
    _settingsManager->setFloat(NVS_KEY_PID_P_I, static_cast<float>(i_gain) / PidConfig::SCALE_FACTOR);
    _settingsManager->setFloat(NVS_KEY_PID_P_D, static_cast<float>(d_gain) / PidConfig::SCALE_FACTOR);

    // Yaw PID
    p_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    i_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    d_gain = payload[current_byte_idx++] | (payload[current_byte_idx++] << 8);
    _settingsManager->setFloat(NVS_KEY_PID_Y_P, static_cast<float>(p_gain) / PidConfig::SCALE_FACTOR);
    _settingsManager->setFloat(NVS_KEY_PID_Y_I, static_cast<float>(i_gain) / PidConfig::SCALE_FACTOR);
    _settingsManager->setFloat(NVS_KEY_PID_Y_D, static_cast<float>(d_gain) / PidConfig::SCALE_FACTOR);

    send_msp_response(MSP_SET_PID, nullptr, 0, protocol_version); // Acknowledge
}

void MspProcessor::process_get_filter_config_command(uint8_t protocol_version)
{
    uint8_t payload[MSP_FILTER_CONFIG_PAYLOAD_SIZE];

    // Each float is 4 bytes
    float gyro_lpf_hz = _settingsManager->getFloat(NVS_KEY_GYRO_LPF_HZ);
    float notch1_hz = _settingsManager->getFloat(NVS_KEY_NOTCH1_HZ);
    float notch1_q = _settingsManager->getFloat(NVS_KEY_NOTCH1_Q);
    float notch2_hz = _settingsManager->getFloat(NVS_KEY_NOTCH2_HZ);
    float notch2_q = _settingsManager->getFloat(NVS_KEY_NOTCH2_Q);

    memcpy(payload, &gyro_lpf_hz, 4);
    memcpy(payload + 4, &notch1_hz, 4);
    memcpy(payload + 8, &notch1_q, 4);
    memcpy(payload + 12, &notch2_hz, 4);
    memcpy(payload + 16, &notch2_q, 4);

    send_msp_response(MSP_GET_FILTER_CONFIG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_set_filter_config_command(const uint8_t *payload, uint8_t payload_size, uint8_t protocol_version)
{
    if (payload_size != MSP_FILTER_CONFIG_PAYLOAD_SIZE)
    {
        send_msp_error(MSP_SET_FILTER_CONFIG, protocol_version);
        return;
    }

    float gyro_lpf_hz, notch1_hz, notch1_q, notch2_hz, notch2_q;

    memcpy(&gyro_lpf_hz, payload, 4);
    memcpy(&notch1_hz, payload + 4, 4);
    memcpy(&notch1_q, payload + 8, 4);
    memcpy(&notch2_hz, payload + 12, 4);
    memcpy(&notch2_q, payload + 16, 4);

    _settingsManager->setFloat(NVS_KEY_GYRO_LPF_HZ, gyro_lpf_hz);
    _settingsManager->setFloat(NVS_KEY_NOTCH1_HZ, notch1_hz);
    _settingsManager->setFloat(NVS_KEY_NOTCH1_Q, notch1_q);
    _settingsManager->setFloat(NVS_KEY_NOTCH2_HZ, notch2_hz);
    _settingsManager->setFloat(NVS_KEY_NOTCH2_Q, notch2_q);

    // Apply immediately. Note: Filter::set_low_pass_filter_factor expects a single factor,
    // not individual cutoff frequencies. This needs careful consideration.
    // For now, I'll comment out the direct Filter::set... call as it seems mismatched with the settings being set.
    // If Filter class needs to be updated, that's a separate task.
    // Filter::set_low_pass_filter_factor(lp_factor); // Apply immediately

    send_msp_response(MSP_SET_FILTER_CONFIG, nullptr, 0, protocol_version); // Acknowledge
}

void MspProcessor::process_set_box_command(const uint8_t *payload, uint8_t payload_size, uint8_t protocol_version)
{
    // MSP_SET_BOX typically involves setting mode ranges based on permanent IDs
    // This is a complex command that needs careful implementation to map MSP box IDs
    // to actual flight modes and AUX channel ranges.
    // For now, it will acknowledge but not change anything.
    if (payload_size < 4)
    { // At least 1 permanent ID, 1 aux channel, 2 range bytes
        send_msp_error(MSP_SET_BOX, protocol_version);
        return;
    }

    // Example payload for a single box setting:
    // Permanent ID (1 byte)
    // AUX channel (1 byte)
    // Start Step (1 byte)
    // End Step (1 byte)
    // This command can have multiple such 4-byte entries.

    // Acknowledge for now
    send_msp_response(MSP_SET_BOX, nullptr, 0, protocol_version);
}

// --- Added Stubs for Unimplemented Commands ---

void MspProcessor::process_name_command(uint8_t protocol_version)
{
    // Send a default name for now
    const char* name = "Flight32";
    send_msp_response(MSP_NAME, (const uint8_t*)name, strlen(name), protocol_version);
}

void MspProcessor::process_feature_config_command(uint8_t protocol_version)
{
    // Define features as a bitmask, similar to Betaflight's approach.
    // Example: (1 << FEATURE_RX_SERIAL) | (1 << FEATURE_MOTOR_STOP)
    uint32_t features = 0;

    // Based on Flight32 core functionalities
    // FEATURE_RX_SERIAL for serial RX
    // FEATURE_MOTOR_STOP for motor arming/disarming
    // FEATURE_ACC for accelerometer presence
    // FEATURE_GYRO for gyroscope presence
    // FEATURE_PID_CONTROLLER for PID loop active
    // Define these in msp_protocol.h if needed, or use direct bitmasks.
    // For now, let's hardcode some common ones that are typically enabled.
    features |= (1 << 0); // Placeholder for a generic 'Flight Controller' feature
    features |= (1 << 1); // Placeholder for 'Accelerometer' feature
    features |= (1 << 2); // Placeholder for 'Gyroscope' feature
    features |= (1 << 3); // Placeholder for 'PID Controller' feature
    // You would map actual features to specific bits here.

    uint8_t payload[4]; // 32-bit feature mask
    payload[0] = features & 0xFF;
    payload[1] = (features >> 8) & 0xFF;
    payload[2] = (features >> 16) & 0xFF;
    payload[3] = (features >> 24) & 0xFF;

    send_msp_response(MSP_FEATURE_CONFIG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_arming_config_command(uint8_t protocol_version)
{
    // Payload: auto_disarm_delay (uint16_t), disarm_kill_switch (uint8_t)
    uint8_t payload[3];

    // auto_disarm_delay: NVS_KEY_ARM_AUTODISARM_DELAY is stored as UINT8, interpret as uint16_t
    uint16_t auto_disarm_delay = _settingsManager->getSettingValue(NVS_KEY_ARM_AUTODISARM_DELAY).toInt();
    payload[0] = auto_disarm_delay & 0xFF;
    payload[1] = (auto_disarm_delay >> 8) & 0xFF;

    // disarm_kill_switch: NVS_KEY_ARM_KILLSWITCH_STATE is stored as UINT8
    uint8_t disarm_kill_switch = _settingsManager->getSettingValue(NVS_KEY_ARM_KILLSWITCH_STATE).toInt();
    payload[2] = disarm_kill_switch;

    send_msp_response(MSP_ARMING_CONFIG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_rx_config_command(uint8_t protocol_version)
{
    // This is a complex one, will try to fill some fields.
    // Based on Betaflight's MSP_RX_CONFIG (code 44) structure:
    // rx_type (uint8)
    // stick_min (uint16)
    // stick_max (uint16)
    // stick_center (uint16)
    // rx_min_usec (uint16)
    // rx_max_usec (uint16)
    // rx_channel_count (uint8)
    // aux_channels (num_aux * uint8) - not implemented here
    // failsafe_delay (uint8)
    // failsafe_off_delay (uint8)
    // And more...

    uint8_t payload[MSP_RX_CONFIG_PAYLOAD_SIZE];
    uint8_t current_byte_idx = 0;

    // rx_type (uint8)
    payload[current_byte_idx++] = (uint8_t)_settingsManager->getSettingValue(NVS_KEY_RC_PROTOCOL_TYPE).toInt();

    // stick_min, stick_max, stick_center (uint16) - Use constants from RxConfig
    uint16_t stick_min = RC_CHANNEL_MIN_RAW;
    uint16_t stick_max = RC_CHANNEL_MAX_RAW;
    uint16_t stick_center = RC_CHANNEL_CENTER;

    payload[current_byte_idx++] = stick_min & 0xFF;
    payload[current_byte_idx++] = (stick_min >> 8) & 0xFF;
    payload[current_byte_idx++] = stick_max & 0xFF;
    payload[current_byte_idx++] = (stick_max >> 8) & 0xFF;
    payload[current_byte_idx++] = stick_center & 0xFF;
    payload[current_byte_idx++] = (stick_center >> 8) & 0xFF;

    // rx_min_usec, rx_max_usec (uint16)
    payload[current_byte_idx++] = RC_CHANNEL_MIN_RAW & 0xFF;
    payload[current_byte_idx++] = (RC_CHANNEL_MIN_RAW >> 8) & 0xFF;
    payload[current_byte_idx++] = RC_CHANNEL_MAX_RAW & 0xFF;
    payload[current_byte_idx++] = (RC_CHANNEL_MAX_RAW >> 8) & 0xFF;

    // rx_channel_count (uint8)
    payload[current_byte_idx++] = PPM_MAX_CHANNELS;

    // aux_channels (variable length based on rx_channel_count, not easy to map directly here)
    // We'll leave the rest as zeros for now, filling up to MSP_RX_CONFIG_PAYLOAD_SIZE if it's defined.
    // If MSP_RX_CONFIG_PAYLOAD_SIZE is not defined, calculate remaining bytes and fill.
    for (int i = current_byte_idx; i < 20; i++) // Assuming 20 bytes is enough for a reasonable default
    {
        payload[i] = 0;
    }

    send_msp_response(MSP_RX_CONFIG, payload, 20, protocol_version); // Send 20 bytes for now
}

void MspProcessor::process_rx_map_command(uint8_t protocol_version)
{
    // AETR1234
    // Get RC channel mappings from settings.
    uint8_t roll_idx = _settingsManager->getSettingValue(NVS_KEY_RC_ROLL).toInt();
    uint8_t pitch_idx = _settingsManager->getSettingValue(NVS_KEY_RC_PITCH).toInt();
    uint8_t thro_idx = _settingsManager->getSettingValue(NVS_KEY_RC_THRO).toInt();
    uint8_t yaw_idx = _settingsManager->getSettingValue(NVS_KEY_RC_YAW).toInt();
    uint8_t aux1_idx = _settingsManager->getSettingValue(NVS_KEY_RC_AUX1).toInt();
    uint8_t aux2_idx = _settingsManager->getSettingValue(NVS_KEY_RC_AUX2).toInt();
    uint8_t aux3_idx = _settingsManager->getSettingValue(NVS_KEY_RC_AUX3).toInt();
    uint8_t aux4_idx = _settingsManager->getSettingValue(NVS_KEY_RC_AUX4).toInt();
    
    // MSP_RX_MAP expects 8 bytes, typically representing the order of channels (0-based)
    uint8_t payload[8];
    payload[0] = roll_idx;
    payload[1] = pitch_idx;
    payload[2] = thro_idx;
    payload[3] = yaw_idx;
    payload[4] = aux1_idx;
    payload[5] = aux2_idx;
    payload[6] = aux3_idx;
    payload[7] = aux4_idx;

    send_msp_response(MSP_RX_MAP, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_failsafe_config_command(uint8_t protocol_version)
{
    // No explicit NVS keys for failsafe config in _settings_metadata.
    // Sending zeros for now.
    uint8_t payload[10] = {0};
    send_msp_response(MSP_FAILSAFE_CONFIG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_rc_tuning_command(uint8_t protocol_version)
{
    // rc_rate (uint8), rc_expo (uint8), roll_rate (uint8), pitch_rate (uint8), yaw_rate (uint8),
    // throttle_mid (uint8), throttle_expo (uint8) - total 7 bytes

    uint8_t payload[7];

    // These values are typically scaled by 100 or 10. For now, use default PID values directly or zeros.
    // RC Rate (P-gain for Roll)
    payload[0] = static_cast<uint8_t>(_settingsManager->getFloat(NVS_KEY_PID_R_P) * 100.0f); // Example scaling
    // RC Expo (P-gain for Pitch)
    payload[1] = static_cast<uint8_t>(_settingsManager->getFloat(NVS_KEY_PID_P_P) * 100.0f); // Example scaling
    // Roll Rate
    payload[2] = static_cast<uint8_t>(_settingsManager->getFloat(NVS_KEY_PID_R_D) * 10.0f);  // Example scaling
    // Pitch Rate
    payload[3] = static_cast<uint8_t>(_settingsManager->getFloat(NVS_KEY_PID_P_D) * 10.0f);  // Example scaling
    // Yaw Rate
    payload[4] = static_cast<uint8_t>(_settingsManager->getFloat(NVS_KEY_PID_Y_D) * 10.0f);  // Example scaling
    // Throttle Mid and Expo are not directly available, send zeros
    payload[5] = 0; // Throttle Mid
    payload[6] = 0; // Throttle Expo

    send_msp_response(MSP_RC_TUNING, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_advanced_config_command(uint8_t protocol_version)
{
    // No direct NVS keys for advanced config.
    // Sending zeros for now. (e.g. gyro_sync_denom, dterm_filter_type, debug_mode)
    uint8_t payload[3] = {0};
    send_msp_response(MSP_ADVANCED_CONFIG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_filter_config_command(uint8_t protocol_version)
{
    // This is different from GET/SET, it's the old combined command.
    // For now, let's just send back the same data as GET.
    process_get_filter_config_command(protocol_version);
}

void MspProcessor::process_status_ex_command(uint8_t protocol_version)
{
    // For now, just send the basic status response
    process_status_command(protocol_version);
}

void MspProcessor::process_altitude_command(uint8_t protocol_version)
{
    uint8_t payload[6] = {0}; // int32_t alt, int16_t vario
    send_msp_response(MSP_ALTITUDE, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_analog_command(uint8_t protocol_version)
{
    uint8_t payload[7] = {0}; // vbat, intPowerMeterSum, rssi, amperage
    send_msp_response(MSP_ANALOG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_battery_state_command(uint8_t protocol_version)
{
    uint8_t payload[3] = {0}; // cellCount, batteryState, flags
    send_msp_response(MSP_BATTERY_STATE, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_osd_config_command(uint8_t protocol_version)
{
    uint8_t payload[20] = {0};
    send_msp_response(MSP_OSD_CONFIG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_vtx_config_command(uint8_t protocol_version)
{
    uint8_t payload[20] = {0};
    send_msp_response(MSP_VTX_CONFIG, payload, sizeof(payload), protocol_version);
}

void MspProcessor::process_pidnames_command(uint8_t protocol_version)
{
    // Similar to BOXNAMES, but for PIDs.
    String pidnames_str = "ROLL;PITCH;YAW;ALT;POS;POSR;NAVR;LEVEL;MAG;VEL;";
    uint16_t payload_size = pidnames_str.length();
    uint8_t payload[payload_size];
    memcpy(payload, pidnames_str.c_str(), payload_size);
    send_msp_response(MSP_PIDNAMES, payload, payload_size, protocol_version);
}

void MspProcessor::process_boxids_command(uint8_t protocol_version)
{
    uint8_t box_ids[] = {
        BF_PERMANENT_ID_ARM,
        BF_PERMANENT_ID_ANGLE,
        BF_PERMANENT_ID_HORIZON,
        BF_PERMANENT_ID_HEADFREE,
        BF_PERMANENT_ID_HEADADJ,
        BF_PERMANENT_ID_FAILSAFE,
        BF_PERMANENT_ID_BEEPER,
        BF_PERMANENT_ID_OSDDISABLESW,
        BF_PERMANENT_ID_TELEMETRY,
        BF_PERMANENT_ID_BLACKBOX,
        BF_PERMANENT_ID_FPVANGLEMIX,
        BF_PERMANENT_ID_BLACKBOXERASE,
        BF_PERMANENT_ID_FLIPOVERAFTERCRASH,
        BF_PERMANENT_ID_PREARM,
        BF_PERMANENT_ID_PARALYZE,
        BF_PERMANENT_ID_LAUNCHCONTROL
    };
    send_msp_response(MSP_BOXIDS, box_ids, sizeof(box_ids), protocol_version);
}


void MspProcessor::process_msp_command(uint8_t command, const uint8_t *payload, uint8_t payload_size, uint8_t protocol_version)
{
    switch (command)
    {
    case MSP_API_VERSION:
        process_api_version_command(protocol_version);
        break;
    case MSP_FC_VARIANT:
        process_fc_variant_command(protocol_version);
        break;
    case MSP_FC_VERSION:
        process_fc_version_command(protocol_version);
        break;
    case MSP_BOARD_INFO:
        process_board_info_command(protocol_version);
        break;
    case MSP_BUILD_INFO:
        process_build_info_command(protocol_version);
        break;
    case MSP_NAME:
        process_name_command(protocol_version);
        break;
    case MSP_FEATURE_CONFIG:
        process_feature_config_command(protocol_version);
        break;
    case MSP_ARMING_CONFIG:
        process_arming_config_command(protocol_version);
        break;
    case MSP_RX_CONFIG:
        process_rx_config_command(protocol_version);
        break;
    case MSP_RX_MAP:
        process_rx_map_command(protocol_version);
        break;
    case MSP_FAILSAFE_CONFIG:
        process_failsafe_config_command(protocol_version);
        break;
    case MSP_RC_TUNING:
        process_rc_tuning_command(protocol_version);
        break;
    case MSP_ADVANCED_CONFIG:
        process_advanced_config_command(protocol_version);
        break;
    case MSP_FILTER_CONFIG:
        process_filter_config_command(protocol_version);
        break;
    case MSP_STATUS_EX:
        process_status_ex_command(protocol_version);
        break;
    case MSP_ALTITUDE:
        process_altitude_command(protocol_version);
        break;
    case MSP_ANALOG:
        process_analog_command(protocol_version);
        break;
    case MSP_BATTERY_STATE:
        process_battery_state_command(protocol_version);
        break;
    case MSP_OSD_CONFIG:
        process_osd_config_command(protocol_version);
        break;
    case MSP_VTX_CONFIG:
        process_vtx_config_command(protocol_version);
        break;
    case MSP_PIDNAMES:
        process_pidnames_command(protocol_version);
        break;
    case MSP_REBOOT:
        process_reboot_command(protocol_version);
        break;
    case MSP_MEM_STATS:
        process_mem_stats_command(protocol_version);
        break;
    case MSP_IDENT:
        process_ident_command(protocol_version);
        break;
    case MSP_GET_SETTING:
        process_get_setting_command(payload, payload_size, protocol_version);
        break;
    case MSP_SET_SETTING:
        process_set_setting_command(payload, payload_size, protocol_version);
        break;
    case MSP_PID:
        process_pid_command(protocol_version);
        break;
    case MSP_STATUS:
        process_status_command(protocol_version);
        break;
    case MSP_RAW_IMU:
        process_raw_imu_command(protocol_version);
        break;
    case MSP_MOTOR:
        process_motor_command(protocol_version);
        break;
    case MSP_RC:
        process_rc_command(protocol_version);
        break;
    case MSP_ATTITUDE:
        process_attitude_command(protocol_version);
        break;
    case MSP_BOX:
        process_box_command(protocol_version);
        break;
    case MSP_BOXNAMES:
        process_boxnames_command(protocol_version);
        break;
    case MSP_BOXIDS:
        process_boxids_command(protocol_version);
        break;
    case MSP_MODE_RANGES:
        process_mode_ranges_command(protocol_version);
        break;
    case MSP_MOTOR_CONFIG:
        process_motor_config_command(protocol_version);
        break;
    case MSP_UID:
        process_uid_command(protocol_version);
        break;
    case MSP_SENSOR_STATUS:
        process_sensor_status_command(protocol_version);
        break;
    case MSP_EEPROM_WRITE:
        process_eeprom_write_command(protocol_version);
        break;
    case MSP_RESET_SETTINGS:
        process_reset_settings_command(protocol_version);
        break;
    case MSP_SET_PID:
        process_set_pid_command(payload, payload_size, protocol_version);
        break;
    case MSP_GET_FILTER_CONFIG:
        process_get_filter_config_command(protocol_version);
        break;
    case MSP_SET_FILTER_CONFIG:
        process_set_filter_config_command(payload, payload_size, protocol_version);
        break;
    case MSP_SET_BOX:
        process_set_box_command(payload, payload_size, protocol_version);
        break;
    default:
        send_msp_error(command, protocol_version);
        break;
    }
}
