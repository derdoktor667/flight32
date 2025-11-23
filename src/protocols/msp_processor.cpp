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
#include "../utils/version_info.h"
#include "../utils/filter.h"
#include "../imu/imu_sensor.h"
#include "../pid/pid_controller.h"
#include <esp_system.h>
#include "../config/rx_config.h"
#include "../config/pid_config.h"
#include "../config/motor_config.h"
#include "../config/filter_config.h"
#include "../config/serial_config.h"
#include "../utils/math_constants.h"
#include <cmath>
#include <cstring>

// Add includes for the tasks/classes whose pointers are being passed
#include "../flight_controller.h"  // For FlightController
#include "../tasks/pid_task.h"     // For PidTask
#include "../tasks/imu_task.h"     // For ImuTask
#include "../tasks/rx_task.h"      // For RxTask
#include "../tasks/motor_task.h"   // For MotorTask
#include "../settings_manager.h"   // For SettingsManager
#include "../utils/system_state.h" // For SystemState enum and FlightMode (if defined there)
#include "../utils/flight_modes.h"

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
    // Assuming only ACC (MPU6050/6500 provides acc and gyro) is present for now.
    payload[4] = MSP_STATUS_ACCEL_SENSOR_FLAG; // ACCELEROMETER

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
    case MSP_REBOOT:
        process_reboot_command(protocol_version);
        break;
    case MSP_MEM_STATS:
        process_mem_stats_command(protocol_version);
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
