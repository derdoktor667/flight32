/**
 * @file settings_manager.cpp
 * @brief Manages persistent settings for the Flight32 firmware using NVS.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "settings_manager.h"
#include "com_manager.h"
#include "config.h"

const char *SettingsManager::GYRO_RANGE_STRINGS[] = {"250_DPS", "500_DPS", "1000_DPS", "2000_DPS"};
const uint8_t SettingsManager::NUM_GYRO_RANGES = sizeof(SettingsManager::GYRO_RANGE_STRINGS) / sizeof(SettingsManager::GYRO_RANGE_STRINGS[0]);

const char *RC_PROTOCOL_STRINGS[] = {"IBUS", "PPM"};
static constexpr uint8_t NUM_RC_PROTOCOLS = sizeof(RC_PROTOCOL_STRINGS) / sizeof(RC_PROTOCOL_STRINGS[0]);

const SettingsManager::SettingMetadata SettingsManager::_settings_metadata[] = {
    // --- System Settings ---
    {SettingsManager::KEY_SYSTEM_NAME, "system.name", "Configurable model name", SettingsManager::STRING, nullptr, 0, 0, 0.0f, SettingsManager::DEFAULT_SYSTEM_NAME},

    // --- MPU6050 Settings ---
    {SettingsManager::KEY_MPU_GYRO_RANGE, "gyro.resolution", "MPU6050 Gyroscope Range", SettingsManager::UINT8, SettingsManager::GYRO_RANGE_STRINGS, SettingsManager::NUM_GYRO_RANGES, DEFAULT_GYRO_RANGE, 0.0f, nullptr},

    // --- RC Protocol Settings ---
    {SettingsManager::KEY_RC_PROTOCOL_TYPE, "rc.protocol", "RC Receiver Protocol", SettingsManager::UINT8, RC_PROTOCOL_STRINGS, NUM_RC_PROTOCOLS, (uint8_t)DEFAULT_RC_PROTOCOL_TYPE, 0.0f, nullptr},
    {SettingsManager::KEY_RX_PIN, "rx.pin", "Generic RX Input Pin (GPIO)", SettingsManager::UINT8, nullptr, 0, DEFAULT_RX_PIN, 0.0f, nullptr},

    // --- RC Channel Mappings ---
    {SettingsManager::KEY_RC_CHANNEL_ROLL, "rc.channel.roll", "RC Roll Channel Index", SettingsManager::UINT8, nullptr, 0, 1, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_PITCH, "rc.channel.pitch", "RC Pitch Channel Index", SettingsManager::UINT8, nullptr, 0, 0, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_THRO, "rc.channel.thro", "RC Throttle Channel Index", SettingsManager::UINT8, nullptr, 0, 2, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_YAW, "rc.channel.yaw", "RC Yaw Channel Index", SettingsManager::UINT8, nullptr, 0, 3, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_ARM, "rc.channel.arm", "RC Arming Channel Index", SettingsManager::UINT8, nullptr, 0, 4, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_FMODE, "rc.channel.fmode", "RC Flight Mode Channel Index", SettingsManager::UINT8, nullptr, 0, 5, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_AUX1, "rc.channel.aux1", "RC Auxiliary Channel 1 Index", SettingsManager::UINT8, nullptr, 0, 6, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_AUX2, "rc.channel.aux2", "RC Auxiliary Channel 2 Index", SettingsManager::UINT8, nullptr, 0, 7, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_AUX3, "rc.channel.aux3", "RC Auxiliary Channel 3 Index", SettingsManager::UINT8, nullptr, 0, 8, 0.0f, nullptr},
    {SettingsManager::KEY_RC_CHANNEL_AUX4, "rc.channel.aux4", "RC Auxiliary Channel 4 Index", SettingsManager::UINT8, nullptr, 0, 9, 0.0f, nullptr},

    // --- Motor Settings ---
    {SettingsManager::KEY_MOTOR_PROTOCOL, "motor.protocol", "DShot Motor Protocol", SettingsManager::UINT8, DSHOT_PROTOCOL_STRINGS, NUM_DSHOT_PROTOCOLS, DEFAULT_MOTOR_PROTOCOL, 0.0f, nullptr},

    // --- PID Gain Settings ---
    {KEY_PID_ROLL_P, "pid.roll.p", "PID Roll Proportional Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_ROLL_P, nullptr},
    {KEY_PID_ROLL_I, "pid.roll.i", "PID Roll Integral Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_ROLL_I, nullptr},
    {KEY_PID_ROLL_D, "pid.roll.d", "PID Roll Derivative Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_ROLL_D, nullptr},
    {KEY_PID_PITCH_P, "pid.pitch.p", "PID Pitch Proportional Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_PITCH_P, nullptr},
    {KEY_PID_PITCH_I, "pid.pitch.i", "PID Pitch Integral Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_PITCH_I, nullptr},
    {KEY_PID_PITCH_D, "pid.pitch.d", "PID Pitch Derivative Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_PITCH_D, nullptr},
    {KEY_PID_YAW_P, "pid.yaw.p", "PID Yaw Proportional Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_YAW_P, nullptr},
    {KEY_PID_YAW_I, "pid.yaw.i", "PID Yaw Integral Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_YAW_I, nullptr},
    {KEY_PID_YAW_D, "pid.yaw.d", "PID Yaw Derivative Gain", SettingsManager::FLOAT, nullptr, 0, 0, DEFAULT_PID_YAW_D, nullptr}};
const int SettingsManager::_num_settings = sizeof(SettingsManager::_settings_metadata) / sizeof(SettingsManager::SettingMetadata);

SettingsManager::SettingsManager()
{
}

void SettingsManager::begin()
{

    _preferences.begin(SETTINGS_NAMESPACE, false);
    _is_begun = true;
    loadOrInitSettings();
}

void SettingsManager::loadOrInitSettings()
{
    if (!_is_begun)
        return;

    uint16_t saved_version = _preferences.getUShort(KEY_SCHEMA_VERSION, 0);


    if (saved_version != CURRENT_SCHEMA_VERSION)
    {
        com_send_log(LOG_WARN, "Settings: Schema mismatch or first run. Applying defaults.");
        factoryReset();
    }
    else
    {
        com_send_log(LOG_INFO, "Settings: Loading existing configuration.");
    }
}

void SettingsManager::factoryReset()
{
    com_send_log(LOG_INFO, "SettingsManager: factoryReset()");
    if (!_is_begun)
        return;
    _preferences.clear();
    _write_defaults();
    saveSettings();
}

void SettingsManager::_write_defaults()
{
    com_send_log(LOG_INFO, "SettingsManager: _write_defaults()");
    // Set default values for all settings based on metadata
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
                com_send_log(LOG_INFO, "Writing default for %s: %s", key, _settings_metadata[i].string_default);
                _preferences.putString(key, _settings_metadata[i].string_default);
            }
            else
            {
                com_send_log(LOG_INFO, "Writing empty default for %s", key);
                _preferences.putString(key, "");
            }
            break;
        }
    }
}

void SettingsManager::saveSettings()
{
    if (!_is_begun)
        return;

    _preferences.putUShort(KEY_SCHEMA_VERSION, CURRENT_SCHEMA_VERSION);
    // Ensure the schema version is saved.
}

void SettingsManager::listSettings()
{
    com_send_log(TERMINAL_OUTPUT, "\n--- Available Settings ---");

    // Calculate max display_key length
    int max_display_key_len = 0;
    for (int i = 0; i < _num_settings; ++i)
    {
        int len = strlen(_settings_metadata[i].display_key);
        if (len > max_display_key_len)
        {
            max_display_key_len = len;
        }
    }
    max_display_key_len += 2; // Add a small buffer

    // Print header
    com_send_log(TERMINAL_OUTPUT, "  %-*s %s", max_display_key_len, "Setting", "Description");
    String separator = "  ";
    for (int i = 0; i < max_display_key_len; ++i) {
        separator += "-";
    }
    separator += "--------------------------------------------------"; // Fixed length for description part
    com_send_log(TERMINAL_OUTPUT, separator.c_str());

    for (int i = 0; i < _num_settings; i++)
    {
        com_send_log(TERMINAL_OUTPUT, "  %-*s - %s", max_display_key_len, _settings_metadata[i].display_key, _settings_metadata[i].description);
    }
    com_send_log(TERMINAL_OUTPUT, separator.c_str());
    com_send_log(TERMINAL_OUTPUT, "--------------------------");
}

void SettingsManager::dumpSettings()
{
    com_send_log(TERMINAL_OUTPUT, "\n# Settings Dump");
    for (int i = 0; i < _num_settings; ++i)
    {
        const char *display_key = _settings_metadata[i].display_key;
        const char *internal_key = _settings_metadata[i].key;
        com_send_log(TERMINAL_OUTPUT, "set %s = %s", display_key, getSettingValueHumanReadable(internal_key).c_str());
    }
    com_send_log(TERMINAL_OUTPUT, "# End of Dump");
}

String SettingsManager::getSettingValue(const char *key)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0)
        {
            switch (_settings_metadata[i].type)
            {
            case UINT8:
                return String(_preferences.getUChar(key, 0));
            case INT32:
                return String(_preferences.getInt(key, 0));
            case FLOAT:
                return String(_preferences.getFloat(key, 0.0f));
            case STRING:
                return _preferences.getString(key, "");
            }
        }
    }
    return ""; // Setting not found
}

String SettingsManager::getSettingValueHumanReadable(const char *key)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0)
        {
            if (_settings_metadata[i].string_map != nullptr && _settings_metadata[i].string_map_size > 0)
            {
                int value = getSettingValue(key).toInt();
                if (value >= 0 && value < _settings_metadata[i].string_map_size)
                {
                    return _settings_metadata[i].string_map[value];
                }
            }
            // Special handling for RX channel keys: display as 1-based
            if (strcmp(key, SettingsManager::KEY_RC_CHANNEL_ROLL) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_PITCH) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_THRO) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_YAW) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_ARM) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_FMODE) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_AUX1) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_AUX2) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_AUX3) == 0 ||
                strcmp(key, SettingsManager::KEY_RC_CHANNEL_AUX4) == 0)
            {
                if (_settings_metadata[i].type == UINT8)
                {
                    return String(getSettingValue(key).toInt() + 1); // Convert to 1-based
                }
            }
            // Special handling for PID keys: display as scaled integers
            if (strcmp(key, KEY_PID_ROLL_P) == 0 ||
                strcmp(key, KEY_PID_ROLL_I) == 0 ||
                strcmp(key, KEY_PID_ROLL_D) == 0 ||
                strcmp(key, KEY_PID_PITCH_P) == 0 ||
                strcmp(key, KEY_PID_PITCH_I) == 0 ||
                strcmp(key, KEY_PID_PITCH_D) == 0 ||
                strcmp(key, KEY_PID_YAW_P) == 0 ||
                strcmp(key, KEY_PID_YAW_I) == 0 ||
                strcmp(key, KEY_PID_YAW_D) == 0)
            {
                if (_settings_metadata[i].type == FLOAT)
                {
                    return String((int)(getSettingValue(key).toFloat() * PID_SCALE_FACTOR));
                }
            }
            return getSettingValue(key); // Fallback to raw value if no string map or out of bounds
        }
    }
    return ""; // Setting not found
}

bool SettingsManager::setSettingValue(const char *key, const String &value_str)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].key, key) == 0)
        {
            switch (_settings_metadata[i].type)
            {
            case UINT8:
            {
                int parsed_value = INVALID_SETTING_VALUE;
                // Try to match human-readable string first
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

                // If no human-readable match, try to parse as integer
                if (parsed_value == INVALID_SETTING_VALUE)
                {
                    parsed_value = value_str.toInt();
                }

                if (parsed_value >= 0 && parsed_value <= UINT8_MAX_VALUE)
                {
                    _preferences.putUChar(key, (uint8_t)parsed_value);
                    return true;
                }
                return false; // Value out of range
            }
            case INT32:
            {
                _preferences.putInt(key, value_str.toInt());
                return true;
            }
            case FLOAT:
            {
                _preferences.putFloat(key, value_str.toFloat());
                return true;
            }
            case STRING:
            {
                _preferences.putString(key, value_str);
                return true;
            }
            }
        }
    }
    return false; // Setting not found
}

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

const char *SettingsManager::getInternalKeyFromDisplayKey(const char *display_key)
{
    for (int i = 0; i < _num_settings; ++i)
    {
        if (strcmp(_settings_metadata[i].display_key, display_key) == 0)
        {
            return _settings_metadata[i].key;
        }
    }
    return nullptr; // Not found
}