/**
 * @file settings_manager.cpp
 * @brief Manages persistent settings using ESP32's non-volatile storage (NVS).
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "config/settings_config.h"
#include "config/imu_config.h"
#include "config/rx_config.h"
#include "config/motor_config.h"
#include "config/pid_config.h"

#include "config/filter_config.h" // New include for filter settings
#include "config/scheduler_config.h"
#include "settings_manager.h"
#include "com_manager.h"
#include "terminal/terminal.h"

#include <cstdint>
#include <cstring>

// These string arrays are specific to SettingsManager for displaying options in a human-readable format.
const char *const GYRO_RANGE_STRINGS[] = {"250_DPS", "500_DPS", "1000_DPS", "2000_DPS"};
const uint8_t NUM_GYRO_RANGES = sizeof(GYRO_RANGE_STRINGS) / sizeof(GYRO_RANGE_STRINGS[0]);

const char *const RC_PROTOCOL_STRINGS[] = {"IBUS", "PPM"};
const uint8_t NUM_RC_PROTOCOLS = sizeof(RC_PROTOCOL_STRINGS) / sizeof(RC_PROTOCOL_STRINGS[0]);

const char *const IMU_TYPE_STRINGS[] = {"MPU6050", "NONE"};
const uint8_t NUM_IMU_TYPES = sizeof(IMU_TYPE_STRINGS) / sizeof(IMU_TYPE_STRINGS[0]);

const char *const BOOLEAN_STRINGS[] = {"false", "true"};
const uint8_t NUM_BOOLEAN_STRINGS = sizeof(BOOLEAN_STRINGS) / sizeof(BOOLEAN_STRINGS[0]);

const char *const IMU_LPF_BANDWIDTH_STRINGS[] = {
    "LPF_256HZ_N_0MS", "LPF_188HZ_N_2MS", "LPF_98HZ_N_3MS",
    "LPF_42HZ_N_5MS", "LPF_20HZ_N_10MS", "LPF_10HZ_N_13MS", "LPF_5HZ_N_18MS"};
const uint8_t NUM_IMU_LPF_BANDWIDTHS = sizeof(IMU_LPF_BANDWIDTH_STRINGS) / sizeof(IMU_LPF_BANDWIDTH_STRINGS[0]);

const char *const DSHOT_PROTOCOL_STRINGS[] = {"DSHOT150", "DSHOT300", "DSHOT600", "DSHOT1200"};
const uint8_t NUM_DSHOT_PROTOCOLS = sizeof(DSHOT_PROTOCOL_STRINGS) / sizeof(DSHOT_PROTOCOL_STRINGS[0]);

// Array containing metadata for all configurable settings.
const SettingsManager::SettingMetadata SettingsManager::_settings_metadata[] = {
    // Each entry defines a setting: internal key, display key, description, type, string map for options, default value, etc.
    {KEY_SYSTEM_NAME, "system.name", "Configurable model name", SettingsManager::STRING, nullptr, 0, 0, 0.0f, DEFAULT_SYSTEM_NAME},
    {KEY_MPU_GYRO_RANGE, "gyro.resolution", "MPU6050 Gyroscope Range", SettingsManager::UINT8, GYRO_RANGE_STRINGS, NUM_GYRO_RANGES, IMU_DEFAULT_GYRO_RANGE, 0.0f, nullptr},
    {NVS_KEY_IMU_TYPE, "imu.type", "IMU Sensor Type", SettingsManager::UINT8, IMU_TYPE_STRINGS, NUM_IMU_TYPES, (uint8_t)DEFAULT_IMU_TYPE, 0.0f, nullptr},
    {NVS_KEY_IMU_LPF_BANDWIDTH, "imu.lpf_bw", "IMU Low-Pass Filter Bandwidth", SettingsManager::UINT8, IMU_LPF_BANDWIDTH_STRINGS, NUM_IMU_LPF_BANDWIDTHS, DEFAULT_IMU_LPF_BANDWIDTH, 0.0f, nullptr},
    {NVS_KEY_IMU_DMP_EN, "imu.dmp_en", "IMU Digital Motion Processor (DMP) Enabled", SettingsManager::UINT8, BOOLEAN_STRINGS, NUM_BOOLEAN_STRINGS, IMU_DMP_ENABLED_DEFAULT, 0.0f, nullptr},
    {KEY_MPU_GYRO_OFF_X, "mpu.g_off.x", "Gyroscope X-axis offset", SettingsManager::FLOAT, nullptr, 0, 0, 0.0f, nullptr},
    {KEY_MPU_GYRO_OFF_Y, "mpu.g_off.y", "Gyroscope Y-axis offset", SettingsManager::FLOAT, nullptr, 0, 0, 0.0f, nullptr},
    {KEY_MPU_GYRO_OFF_Z, "mpu.g_off.z", "Gyroscope Z-axis offset", SettingsManager::FLOAT, nullptr, 0, 0, 0.0f, nullptr},
    {KEY_MPU_ACCEL_OFF_X, "mpu.a_off.x", "Accelerometer X-axis offset", SettingsManager::FLOAT, nullptr, 0, 0, 0.0f, nullptr},
    {KEY_MPU_ACCEL_OFF_Y, "mpu.a_off.y", "Accelerometer Y-axis offset", SettingsManager::FLOAT, nullptr, 0, 0, 0.0f, nullptr},
    {KEY_MPU_ACCEL_OFF_Z, "mpu.a_off.z", "Accelerometer Z-axis offset", SettingsManager::FLOAT, nullptr, 0, 0, 0.0f, nullptr},
    {NVS_KEY_RC_PROTOCOL_TYPE, "rc.proto", "RC Receiver Protocol", SettingsManager::UINT8, RC_PROTOCOL_STRINGS, NUM_RC_PROTOCOLS, (uint8_t)DEFAULT_RC_PROTOCOL_TYPE, 0.0f, nullptr},
    {NVS_KEY_RX_PIN, "rx.pin", "Generic RX Input Pin (GPIO)", SettingsManager::UINT8, nullptr, 0, DEFAULT_RX_PIN, 0.0f, nullptr},
    {NVS_KEY_RC_ROLL, "rc.roll", "RC Roll Channel Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_ROLL, 0.0f, nullptr},
    {NVS_KEY_RC_PITCH, "rc.pitch", "RC Pitch Channel Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_PITCH, 0.0f, nullptr},
    {NVS_KEY_RC_THRO, "rc.thro", "RC Throttle Channel Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_THRO, 0.0f, nullptr},
    {NVS_KEY_RC_YAW, "rc.yaw", "RC Yaw Channel Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_YAW, 0.0f, nullptr},
    {NVS_KEY_RC_ARM, "rc.arm", "RC Arming Channel Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_ARM, 0.0f, nullptr},
    {NVS_KEY_RC_FMODE, "rc.fmode", "RC Flight Mode Channel Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_FMODE, 0.0f, nullptr},
    {NVS_KEY_RC_AUX1, "rc.aux1", "RC Auxiliary Channel 1 Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_AUX1, 0.0f, nullptr},
    {NVS_KEY_RC_AUX2, "rc.aux2", "RC Auxiliary Channel 2 Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_AUX2, 0.0f, nullptr},
    {NVS_KEY_RC_AUX3, "rc.aux3", "RC Auxiliary Channel 3 Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_AUX3, 0.0f, nullptr},
    {NVS_KEY_RC_AUX4, "rc.aux4", "RC Auxiliary Channel 4 Index", SettingsManager::UINT8, nullptr, 0, DEFAULT_RC_CHANNEL_AUX4, 0.0f, nullptr},
    {NVS_KEY_MOTOR_PROTOCOL, "motor.proto", "DShot Motor Protocol", SettingsManager::UINT8, DSHOT_PROTOCOL_STRINGS, NUM_DSHOT_PROTOCOLS, DEFAULT_MOTOR_PROTOCOL, 0.0f, nullptr},
    {NVS_KEY_MOTOR_MIN_THROTTLE, "motor.min_thr", "Minimum motor throttle value", SettingsManager::FLOAT, nullptr, 0, 0, 1000.0f, nullptr},
    {NVS_KEY_MOTOR_MAX_THROTTLE, "motor.max_thr", "Maximum motor throttle value", SettingsManager::FLOAT, nullptr, 0, 0, 2000.0f, nullptr},
    {NVS_KEY_MOTOR_MIN_COMMAND, "motor.min_cmd", "Minimum motor command value", SettingsManager::FLOAT, nullptr, 0, 0, 1000.0f, nullptr},
    {NVS_KEY_PID_R_P, "pid.r.p", "PID Roll Proportional Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_RATE_P, nullptr},
    {NVS_KEY_PID_R_I, "pid.r.i", "PID Roll Integral Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_RATE_I, nullptr},
    {NVS_KEY_PID_R_D, "pid.r.d", "PID Roll Derivative Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_RATE_D, nullptr},
    {NVS_KEY_PID_P_P, "pid.p.p", "PID Pitch Proportional Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_RATE_P, nullptr},
    {NVS_KEY_PID_P_I, "pid.p.i", "PID Pitch Integral Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_RATE_I, nullptr},
    {NVS_KEY_PID_P_D, "pid.p.d", "PID Pitch Derivative Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_RATE_D, nullptr},
    {NVS_KEY_PID_Y_P, "pid.y.p", "PID Yaw Proportional Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_YAW_P, nullptr},
    {NVS_KEY_PID_Y_I, "pid.y.i", "PID Yaw Integral Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_YAW_I, nullptr},
    {NVS_KEY_PID_Y_D, "pid.y.d", "PID Yaw Derivative Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_YAW_D, nullptr},
    {NVS_KEY_PID_AR_P, "pid.ar.p", "PID Angle Roll Proportional Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_ANGLE_P, nullptr},
    {NVS_KEY_PID_AR_I, "pid.ar.i", "PID Angle Roll Integral Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_ANGLE_I, nullptr},
    {NVS_KEY_PID_AP_P, "pid.ap.p", "PID Angle Pitch Proportional Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_ANGLE_P, nullptr},
    {NVS_KEY_PID_AP_I, "pid.ap.i", "PID Angle Pitch Integral Gain", SettingsManager::FLOAT, nullptr, 0, 0, PidConfig::DEFAULT_ANGLE_I, nullptr},
    // Arming Settings
    {NVS_KEY_ARM_AUTODISARM_DELAY, "arm.auto_disarm", "Automatic disarm delay (10x seconds)", SettingsManager::UINT8, nullptr, 0, 50, 0.0f, nullptr},
    {NVS_KEY_ARM_KILLSWITCH_STATE, "arm.kill_switch", "Disarm Killswitch state (bitmask)", SettingsManager::UINT8, nullptr, 0, 25, 0.0f, nullptr},
    // Filter Settings
    {NVS_KEY_GYRO_LPF_HZ, "filter.lpf_hz", "Gyro Low-Pass Filter Cutoff Frequency", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_GYRO_LPF_HZ, nullptr},
    {NVS_KEY_NOTCH1_HZ, "filter.notch1_hz", "First Notch Filter Center Frequency", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_NOTCH1_HZ, nullptr},
    {NVS_KEY_NOTCH1_Q, "filter.notch1_q", "First Notch Filter Q-Factor", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_NOTCH1_Q, nullptr},
    {NVS_KEY_NOTCH2_HZ, "filter.notch2_hz", "Second Notch Filter Center Frequency", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_NOTCH2_HZ, nullptr},
    {NVS_KEY_NOTCH2_Q, "filter.notch2_q", "Second Notch Filter Q-Factor", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_NOTCH2_Q, nullptr}};
const int SettingsManager::_num_settings = sizeof(SettingsManager::_settings_metadata) / sizeof(SettingsManager::SettingMetadata);

SettingsManager::SettingsManager()
{
}

// Initializes the SettingsManager, starting NVS and loading/initializing settings.
void SettingsManager::begin()
{
    com_send_log(ComMessageType::LOG_INFO, "SettingsManager: begin()");
    _preferences.begin(NVS_NAMESPACE, false);
    _is_begun = true;
    loadOrInitSettings();
}

// Loads settings from NVS or initializes them if a schema mismatch is detected.
void SettingsManager::loadOrInitSettings()
{
    if (!_is_begun)
        return;

    uint16_t saved_version = _preferences.getUShort(NVS_KEY_SCHEMA_VERSION, NVS_DEFAULT_SCHEMA_VERSION);
    com_send_log(ComMessageType::LOG_INFO, "SettingsManager: NVS Schema Version - Saved: %u, Current: %u", saved_version, NVS_CURRENT_SCHEMA_VERSION);

    // Compare saved schema version with the current version.
    if (saved_version != NVS_CURRENT_SCHEMA_VERSION)
    {
        com_send_log(ComMessageType::LOG_WARN, "Settings: Schema mismatch or first run. Applying defaults.");
        factoryReset();
    }
    else
    {
        com_send_log(ComMessageType::LOG_INFO, "Settings: Loading existing configuration.");
    }
}

// Resets all settings to their factory defaults and saves them to NVS.
void SettingsManager::factoryReset()
{
    com_send_log(ComMessageType::LOG_INFO, "SettingsManager: factoryReset() - Clearing NVS namespace '%s'", NVS_NAMESPACE);
    if (!_is_begun)
        return;
    _preferences.clear();
    _write_defaults();
    saveSettings();
    com_send_log(ComMessageType::LOG_INFO, "SettingsManager: factoryReset() - Defaults applied and saved.");
}

// Writes default values for all defined settings to NVS.
void SettingsManager::_write_defaults()
{
    com_send_log(ComMessageType::LOG_INFO, "SettingsManager: _write_defaults() - Writing default settings to NVS.");
    for (int i = 0; i < _num_settings; ++i)
    {
        const char *key = _settings_metadata[i].key;
        switch (_settings_metadata[i].type)
        {
        case UINT8:
            _preferences.putUChar(key, (uint8_t)_settings_metadata[i].default_value);
            break;
        case INT32:
            _preferences.putInt(key, _settings_metadata[i].default_value);
            break;
        case FLOAT:
            _preferences.putFloat(key, _settings_metadata[i].default_float_value);
            break;
        case STRING:
            if (_settings_metadata[i].string_default != nullptr)
            {
                _preferences.putString(key, _settings_metadata[i].string_default);
            }
            else
            {
                _preferences.putString(key, "");
            }
            break;
        }
    }
}

// Saves the current settings to NVS, including the schema version.
void SettingsManager::saveSettings()
{
    if (!_is_begun)
        return;

    // Store the current schema version.
    _preferences.putUShort(NVS_KEY_SCHEMA_VERSION, NVS_CURRENT_SCHEMA_VERSION);
}

// Retrieves gyroscope offset values from NVS.
ImuAxisData SettingsManager::getGyroOffsets()
{
    return {
        _preferences.getFloat(KEY_MPU_GYRO_OFF_X, 0.0f),
        _preferences.getFloat(KEY_MPU_GYRO_OFF_Y, 0.0f),
        _preferences.getFloat(KEY_MPU_GYRO_OFF_Z, 0.0f)};
}

// Sets gyroscope offset values in NVS.
void SettingsManager::setGyroOffsets(const ImuAxisData &offsets)
{
    _preferences.putFloat(KEY_MPU_GYRO_OFF_X, offsets.x);
    _preferences.putFloat(KEY_MPU_GYRO_OFF_Y, offsets.y);
    _preferences.putFloat(KEY_MPU_GYRO_OFF_Z, offsets.z);
}

// Retrieves accelerometer offset values from NVS.
ImuAxisData SettingsManager::getAccelOffsets()
{
    return {
        _preferences.getFloat(KEY_MPU_ACCEL_OFF_X, 0.0f),
        _preferences.getFloat(KEY_MPU_ACCEL_OFF_Y, 0.0f),
        _preferences.getFloat(KEY_MPU_ACCEL_OFF_Z, 0.0f)};
}

// Sets accelerometer offset values in NVS.
void SettingsManager::setAccelOffsets(const ImuAxisData &offsets)
{
    _preferences.putFloat(KEY_MPU_ACCEL_OFF_X, offsets.x);
    _preferences.putFloat(KEY_MPU_ACCEL_OFF_Y, offsets.y);
    _preferences.putFloat(KEY_MPU_ACCEL_OFF_Z, offsets.z);
}

// Retrieves a float value from NVS using the internal key.
float SettingsManager::getFloat(const char *key)
{
    // Find the setting metadata to get the default float value
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0 && _settings_metadata[i].type == FLOAT)
        {
            return _preferences.getFloat(key, _settings_metadata[i].default_float_value);
        }
    }
    com_send_log(ComMessageType::LOG_WARN, "SettingsManager: getFloat(%s) -> Key not found or not a float type.", key);
    return 0.0f; // Default to 0.0f if key not found or not float
}

// Sets a float value in NVS using the internal key.
void SettingsManager::setFloat(const char *key, float value)
{
    // Find the setting metadata to ensure it's a float type
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0 && _settings_metadata[i].type == FLOAT)
        {
            _preferences.putFloat(key, value);
            com_send_log(ComMessageType::LOG_INFO, "SettingsManager: setFloat - FLOAT %s set to %.3f", key, value);
            return;
        }
    }
    com_send_log(ComMessageType::LOG_WARN, "SettingsManager: setFloat(%s, %.3f) -> Key not found or not a float type.", key, value);
}

// Lists settings belonging to a specific category to the terminal.
void SettingsManager::listSettings(CommandCategory category)
{
    int max_display_key_len = 0;
    int settings_in_category_count = 0;
    // First pass: Determine max display key length and count settings in category for formatting.
    for (int i = 0; i < _num_settings; ++i)
    {
        if (Terminal::_get_setting_category(_settings_metadata[i].display_key) == category)
        {
            int len = strlen(_settings_metadata[i].display_key);
            if (len > max_display_key_len)
            {
                max_display_key_len = len;
            }
            settings_in_category_count++;
        }
    }

    if (settings_in_category_count == 0)
        return;

    com_send_log(ComMessageType::TERMINAL_OUTPUT, "  %-*s %s", SETTING_NAME_DISPLAY_WIDTH, "Setting", "Description");
    String separator = "  ";
    for (int i = 0; i < SETTING_NAME_DISPLAY_WIDTH; ++i)
    {
        separator += "-";
    }
    separator += "--------------------------------";
    com_send_log(ComMessageType::TERMINAL_OUTPUT, separator.c_str());

    // Second pass: Print settings and their descriptions.
    for (int i = 0; i < _num_settings; i++)
    {
        if (Terminal::_get_setting_category(_settings_metadata[i].display_key) == category)
        {
            com_send_log(ComMessageType::TERMINAL_OUTPUT, "  %-*s - %s", SETTING_NAME_DISPLAY_WIDTH, _settings_metadata[i].display_key, _settings_metadata[i].description);
            vTaskDelay(pdMS_TO_TICKS(TASK_YIELD_DELAY_MS)); // Small delay to yield to other tasks.
        }
    }
    com_send_log(ComMessageType::TERMINAL_OUTPUT, separator.c_str());
}

// Dumps current values of settings belonging to a specific category to the terminal.
void SettingsManager::dumpSettings(CommandCategory category)
{
    int settings_in_category_count = 0;
    // First pass: Count settings in category.
    for (int i = 0; i < _num_settings; ++i)
    {
        if (Terminal::_get_setting_category(_settings_metadata[i].display_key) == category)
        {
            settings_in_category_count++;
        }
    }

    if (settings_in_category_count == 0)
        return;

    vTaskDelay(pdMS_TO_TICKS(TASK_YIELD_DELAY_MS)); // Small delay to yield to other tasks.
    // Iterate through all settings and dump values for the specified category.
    for (int i = 0; i < _num_settings; ++i)
    {
        if (Terminal::_get_setting_category(_settings_metadata[i].display_key) == category)
        {
            const char *display_key = _settings_metadata[i].display_key;
            const char *internal_key = _settings_metadata[i].key;
            // Print setting in "set key = value" format.
            com_send_log(ComMessageType::TERMINAL_OUTPUT, "set %s = %s", display_key, getSettingValueHumanReadable(internal_key).c_str());
            vTaskDelay(pdMS_TO_TICKS(TASK_YIELD_DELAY_MS)); // Small delay to yield to other tasks.
        }
    }
}

// Retrieves the raw string value of a setting from NVS.
String SettingsManager::getSettingValue(const char *key)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0)
        {
            switch (_settings_metadata[i].type)
            {
            case UINT8:
            {
                uint8_t val = _preferences.getUChar(key, 0);
                return String(val);
            }
            case INT32:
            {
                int32_t val = _preferences.getInt(key, 0);
                return String(val);
            }
            case FLOAT:
            {
                float val = _preferences.getFloat(key, 0.0f);
                return String(val);
            }
            case STRING:
            {
                String val = _preferences.getString(key, "");
                return val;
            }
            }
        }
    }
    com_send_log(ComMessageType::LOG_WARN, "SettingsManager: getSettingValue(%s) -> Key not found in metadata.", key);
    return "";
}

// Retrieves the human-readable string value of a setting.
String SettingsManager::getSettingValueHumanReadable(const char *key)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0)
        {
            // If a string map exists, use it to convert the value to a human-readable string.
            if (_settings_metadata[i].string_map != nullptr && _settings_metadata[i].string_map_size > 0)
            {
                int value = getSettingValue(key).toInt();
                if (value >= 0 && value < _settings_metadata[i].string_map_size)
                {
                    return _settings_metadata[i].string_map[value];
                }
            }
            // Special handling for RC channel keys: convert to 1-based index for display.
            if (strcmp(key, NVS_KEY_RC_ROLL) == 0 ||
                strcmp(key, NVS_KEY_RC_PITCH) == 0 ||
                strcmp(key, NVS_KEY_RC_THRO) == 0 ||
                strcmp(key, NVS_KEY_RC_YAW) == 0 ||
                strcmp(key, NVS_KEY_RC_ARM) == 0 ||
                strcmp(key, NVS_KEY_RC_FMODE) == 0 ||
                strcmp(key, NVS_KEY_RC_AUX1) == 0 ||
                strcmp(key, NVS_KEY_RC_AUX2) == 0 ||
                strcmp(key, NVS_KEY_RC_AUX3) == 0 ||
                strcmp(key, NVS_KEY_RC_AUX4) == 0)
            {
                if (_settings_metadata[i].type == UINT8)
                {
                    return String(getSettingValue(key).toInt() + RC_CHANNEL_DISPLAY_OFFSET);
                }
            }
            // Special handling for PID keys: scale float values to integers for display.
            if (strcmp(key, NVS_KEY_PID_R_P) == 0 ||
                strcmp(key, NVS_KEY_PID_R_I) == 0 ||
                strcmp(key, NVS_KEY_PID_R_D) == 0 ||
                strcmp(key, NVS_KEY_PID_P_P) == 0 ||
                strcmp(key, NVS_KEY_PID_P_I) == 0 ||
                strcmp(key, NVS_KEY_PID_P_D) == 0 ||
                strcmp(key, NVS_KEY_PID_Y_P) == 0 ||
                strcmp(key, NVS_KEY_PID_Y_I) == 0 ||
                strcmp(key, NVS_KEY_PID_Y_D) == 0 ||
                strcmp(key, NVS_KEY_PID_AR_P) == 0 ||
                strcmp(key, NVS_KEY_PID_AR_I) == 0 ||
                strcmp(key, NVS_KEY_PID_AP_P) == 0 ||
                strcmp(key, NVS_KEY_PID_AP_I) == 0)
            {
                if (_settings_metadata[i].type == FLOAT)
                {
                    return String((int)(getSettingValue(key).toFloat() * PidConfig::SCALE_FACTOR));
                }
            }
            return getSettingValue(key);
        }
    }
    return "";
}

// Retrieves a human-readable string of available options for a setting.
String SettingsManager::getSettingOptionsHumanReadable(const char *key)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0)
        {
            // If a string map exists, build a comma-separated string of options.
            if (_settings_metadata[i].string_map != nullptr && _settings_metadata[i].string_map_size > 0)
            {
                String options_str = "";
                for (uint8_t j = 0; j < _settings_metadata[i].string_map_size; ++j)
                {
                    options_str += _settings_metadata[i].string_map[j];
                    if (j < _settings_metadata[i].string_map_size - 1)
                    {
                        options_str += ", ";
                    }
                }
                return options_str;
            }
        }
    }
    return "";
}

// Sets the value of a setting in NVS based on a string input.
bool SettingsManager::setSettingValue(const char *key, const String &value_str)
{
    com_send_log(ComMessageType::LOG_INFO, "SettingsManager: setSettingValue(%s, %s)", key, value_str.c_str());
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0)
        {
            switch (_settings_metadata[i].type)
            {
            case UINT8:
            {
                int parsed_value = INVALID_SETTING_VALUE;
                // If a string map exists, try to parse the input string as one of the options.
                if (_settings_metadata[i].string_map != nullptr && _settings_metadata[i].string_map_size > 0)
                {
                    for (uint8_t j = 0; j < _settings_metadata[i].string_map_size; ++j)
                    {
                        if (value_str.equalsIgnoreCase(_settings_metadata[i].string_map[j]))
                        {
                            parsed_value = j;
                            break;
                        }
                    }
                }

                if (parsed_value == INVALID_SETTING_VALUE)
                {
                    parsed_value = value_str.toInt();
                }

                if (parsed_value >= 0 && parsed_value <= UINT8_MAX_VALUE)
                {
                    _preferences.putUChar(key, (uint8_t)parsed_value);
                    com_send_log(ComMessageType::LOG_INFO, "SettingsManager: setSettingValue - UINT8 %s set to %u", key, (uint8_t)parsed_value);
                    return true;
                }
                com_send_log(ComMessageType::LOG_WARN, "SettingsManager: setSettingValue - UINT8 %s failed, parsed_value %d out of range.", key, parsed_value);
                return false;
            }
            case INT32:
            {
                _preferences.putInt(key, value_str.toInt());
                com_send_log(ComMessageType::LOG_INFO, "SettingsManager: setSettingValue - INT32 %s set to %d", key, value_str.toInt());
                return true;
            }
            case FLOAT:
            {
                _preferences.putFloat(key, value_str.toFloat());
                com_send_log(ComMessageType::LOG_INFO, "SettingsManager: setSettingValue - FLOAT %s set to %.3f", key, value_str.toFloat());
                return true;
            }
            case STRING:
            {
                _preferences.putString(key, value_str);
                com_send_log(ComMessageType::LOG_INFO, "SettingsManager: setSettingValue - STRING %s set to %s", key, value_str.c_str());
                return true;
            }
            }
        }
    }
    com_send_log(ComMessageType::LOG_WARN, "SettingsManager: setSettingValue - Key %s not found in metadata.", key);
    return false;
}

// Retrieves the description of a setting.
const char *SettingsManager::getSettingDescription(const char *key)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0)
        {
            return _settings_metadata[i].description;
        }
    }
    return "Unknown Setting";
}

// Retrieves the internal key from a display key.
const char *SettingsManager::getInternalKeyFromDisplayKey(const char *display_key)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].display_key, display_key) == 0)
        {
            return _settings_metadata[i].key;
        }
    }
    return nullptr;
}