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

const SettingsManager::SettingMetadata SettingsManager::_settings_metadata[] = {
    {SettingsManager::KEY_SYSTEM_NAME, "system.name", "Configurable model name", SettingsManager::STRING, nullptr, 0, 0, 0.0f, SettingsManager::DEFAULT_SYSTEM_NAME},
    {SettingsManager::KEY_MPU_GYRO_RANGE, "gyro.resolution", "MPU6050 Gyroscope Range", SettingsManager::UINT8, SettingsManager::GYRO_RANGE_STRINGS, SettingsManager::NUM_GYRO_RANGES, DEFAULT_GYRO_RANGE, 0.0f, nullptr},
    {SettingsManager::KEY_MOTOR_PROTOCOL, "motor.protocol", "DShot Motor Protocol", SettingsManager::UINT8, DSHOT_PROTOCOL_STRINGS, NUM_DSHOT_PROTOCOLS, DEFAULT_MOTOR_PROTOCOL, 0.0f, nullptr},

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
    // Constructor can be empty
}

void SettingsManager::begin()
{
    com_send_log(LOG_INFO, "SettingsManager: begin()\n");
    _preferences.begin(SETTINGS_NAMESPACE, false);
    _is_begun = true;
    loadOrInitSettings();
}

void SettingsManager::loadOrInitSettings()
{
    if (!_is_begun)
        return;

    uint16_t saved_version = _preferences.getUShort(KEY_SCHEMA_VERSION, 0);
    com_send_log(LOG_INFO, "Settings: Saved version: %d, Current version: %d\n", saved_version, CURRENT_SCHEMA_VERSION);

    if (saved_version != CURRENT_SCHEMA_VERSION)
    {
        com_send_log(LOG_WARN, "Settings: Schema mismatch or first run. Applying defaults.\n");
        factoryReset();
    }
    else
    {
        com_send_log(LOG_INFO, "Settings: Loading existing configuration.\n");
    }
}

void SettingsManager::factoryReset()
{
    com_send_log(LOG_INFO, "SettingsManager: factoryReset()\n");
    if (!_is_begun)
        return;
    _preferences.clear();
    _write_defaults();
    saveSettings();
}

void SettingsManager::_write_defaults()
{
    com_send_log(LOG_INFO, "SettingsManager: _write_defaults()\n");
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
                com_send_log(LOG_INFO, "Writing default for %s: %s\n", key, _settings_metadata[i].string_default);
                _preferences.putString(key, _settings_metadata[i].string_default);
            }
            else
            {
                com_send_log(LOG_INFO, "Writing empty default for %s\n", key);
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
    // Save all settings based on metadata
    // The actual values are already in preferences if set via setSettingValue
    // This loop ensures all keys are written if they were modified directly
    // For now, we just ensure the schema version is saved.
}

void SettingsManager::listSettings()
{
    com_send_log(TERMINAL_OUTPUT, "\n--- Available Settings ---");
    for (int i = 0; i < _num_settings; i++)
    {
        com_send_log(TERMINAL_OUTPUT, "  %-20s - %s", _settings_metadata[i].display_key, _settings_metadata[i].description);
    }
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