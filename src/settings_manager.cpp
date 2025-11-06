#include "settings_manager.h"
#include "com_manager.h"
#include "config.h"

const char* SettingsManager::GYRO_RANGE_STRINGS[] = {"250_DPS", "500_DPS", "1000_DPS", "2000_DPS"};
const uint8_t SettingsManager::NUM_GYRO_RANGES = sizeof(SettingsManager::GYRO_RANGE_STRINGS) / sizeof(SettingsManager::GYRO_RANGE_STRINGS[0]);

const SettingsManager::SettingMetadata SettingsManager::_settings_metadata[] = {
    {SettingsManager::KEY_MPU_GYRO_RANGE, "gyro.resolution", "MPU6050 Gyroscope Range", SettingsManager::UINT8, SettingsManager::GYRO_RANGE_STRINGS, SettingsManager::NUM_GYRO_RANGES}
};
const int SettingsManager::_num_settings = sizeof(SettingsManager::_settings_metadata) / sizeof(SettingsManager::SettingMetadata);

SettingsManager::SettingsManager() {
    // Constructor can be empty
}

void SettingsManager::begin() {
    _preferences.begin(SETTINGS_NAMESPACE, false);
    _is_begun = true;
    loadOrInitSettings();
}

void SettingsManager::loadOrInitSettings() {
    if (!_is_begun) return;

    uint16_t saved_version = _preferences.getUShort(KEY_SCHEMA_VERSION, 0);

    if (saved_version != CURRENT_SCHEMA_VERSION) {
        com_send_log(LOG_WARN, "Settings: Schema mismatch or first run. Applying defaults.");
        factoryReset();
    } else {
        com_send_log(LOG_INFO, "Settings: Loading existing configuration.");
        // Load all settings based on metadata
        for (int i = 0; i < _num_settings; ++i) {
            const char* key = _settings_metadata[i].key;
            switch (_settings_metadata[i].type) {
                case UINT8: _preferences.getUChar(key, DEFAULT_GYRO_RANGE); break; // Default value is used if not found
                case INT32: _preferences.getInt(key, 0); break;
                case FLOAT: _preferences.getFloat(key, 0.0f); break;
                case STRING: _preferences.getString(key, ""); break;
            }
        }
    }
}

void SettingsManager::factoryReset() {
    if (!_is_begun) return;
    _preferences.clear();
    _write_defaults();
    saveSettings();
}

void SettingsManager::_write_defaults() {
    // Set default values for all settings based on metadata
    for (int i = 0; i < _num_settings; ++i) {
        const char* key = _settings_metadata[i].key;
        switch (_settings_metadata[i].type) {
            case UINT8: _preferences.putUChar(key, DEFAULT_GYRO_RANGE); break;
            case INT32: _preferences.putInt(key, 0); break;
            case FLOAT: _preferences.putFloat(key, 0.0f); break;
            case STRING: _preferences.putString(key, ""); break;
        }
    }
}

void SettingsManager::saveSettings() {
    if (!_is_begun) return;

    _preferences.putUShort(KEY_SCHEMA_VERSION, CURRENT_SCHEMA_VERSION);
    // Save all settings based on metadata
    // The actual values are already in preferences if set via setSettingValue
    // This loop ensures all keys are written if they were modified directly
    // For now, we just ensure the schema version is saved.
}

void SettingsManager::listSettings() {
    com_send_log(TERMINAL_OUTPUT, "\n--- Available Settings ---");
    for (int i = 0; i < _num_settings; i++) {
        com_send_log(TERMINAL_OUTPUT, "  %-20s - %s", _settings_metadata[i].display_key, _settings_metadata[i].description);
    }
    com_send_log(TERMINAL_OUTPUT, "--------------------------");
}

void SettingsManager::dumpSettings() {
    com_send_log(TERMINAL_OUTPUT, "\n# Settings Dump");
    for (int i = 0; i < _num_settings; ++i) {
        const char* display_key = _settings_metadata[i].display_key;
        const char* internal_key = _settings_metadata[i].key;
        com_send_log(TERMINAL_OUTPUT, "set %s = %s", display_key, getSettingValueHumanReadable(internal_key).c_str());
    }
    com_send_log(TERMINAL_OUTPUT, "# End of Dump");
}


String SettingsManager::getSettingValue(const char* key) {
    for (int i = 0; i < _num_settings; ++i) {
        if (strcmp(_settings_metadata[i].key, key) == 0) {
            switch (_settings_metadata[i].type) {
                case UINT8: return String(_preferences.getUChar(key, 0));
                case INT32: return String(_preferences.getInt(key, 0));
                case FLOAT: return String(_preferences.getFloat(key, 0.0f));
                case STRING: return _preferences.getString(key, "");
            }
        }
    }
    return ""; // Setting not found
}

String SettingsManager::getSettingValueHumanReadable(const char* key) {
    for (int i = 0; i < _num_settings; ++i) {
        if (strcmp(_settings_metadata[i].key, key) == 0) {
            if (_settings_metadata[i].string_map != nullptr && _settings_metadata[i].string_map_size > 0) {
                int value = getSettingValue(key).toInt();
                if (value >= 0 && value < _settings_metadata[i].string_map_size) {
                    return _settings_metadata[i].string_map[value];
                }
            }
            return getSettingValue(key); // Fallback to raw value if no string map or out of bounds
        }
    }
    return ""; // Setting not found
}

bool SettingsManager::setSettingValue(const char* key, const String& value_str) {
    for (int i = 0; i < _num_settings; ++i) {
        if (strcmp(_settings_metadata[i].key, key) == 0) {
            int value_to_store = INVALID_SETTING_VALUE; // Default to invalid

            // Check if value_str matches a human-readable string in the map
            if (_settings_metadata[i].string_map != nullptr && _settings_metadata[i].string_map_size > 0) {
                for (uint8_t j = 0; j < _settings_metadata[i].string_map_size; ++j) {
                    if (value_str.equalsIgnoreCase(_settings_metadata[i].string_map[j])) {
                        value_to_store = j; // Found a match, store its index
                        break;
                    }
                }
            }

            // If no human-readable match, try to parse as integer
            if (value_to_store == -1) {
                value_to_store = value_str.toInt();
            }

            // Validate and store the value
            switch (_settings_metadata[i].type) {
                case UINT8: 
                    if (value_to_store >= 0 && value_to_store <= UINT8_MAX_VALUE) { // Assuming UINT8 range
                        _preferences.putUChar(key, (uint8_t)value_to_store); 
                        return true;
                    }
                    break;
                case INT32: 
                    _preferences.putInt(key, value_to_store); 
                    return true;
                case FLOAT: 
                    _preferences.putFloat(key, value_str.toFloat()); 
                    return true;
                case STRING: 
                    _preferences.putString(key, value_str); 
                    return true;
            }
            return false; // Value out of range or invalid for type
        }
    }
    return false; // Setting not found
}

const char* SettingsManager::getSettingDescription(const char* key) {
    for (int i = 0; i < _num_settings; ++i) {
        if (strcmp(_settings_metadata[i].key, key) == 0) {
            return _settings_metadata[i].description;
        }
    }
    return "Unknown Setting";
}

const char* SettingsManager::getInternalKeyFromDisplayKey(const char* display_key) {
    for (int i = 0; i < _num_settings; ++i) {
        if (strcmp(_settings_metadata[i].display_key, display_key) == 0) {
            return _settings_metadata[i].key;
        }
    }
    return nullptr; // Not found
}
