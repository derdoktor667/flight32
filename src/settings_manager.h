#pragma once

#include <Preferences.h>
#include <Arduino.h>

class SettingsManager
{
public:
    SettingsManager();

    void begin();
    void loadOrInitSettings();
    void saveSettings();
    void factoryReset();
    void listSettings();
    void dumpSettings();

    // --- Getter and Setter Methods ---
    String getSettingValue(const char *key);
    bool setSettingValue(const char *key, const String &value_str);
    const char *getSettingDescription(const char *key);
    String getSettingValueHumanReadable(const char *key);
    const char *getInternalKeyFromDisplayKey(const char *display_key);

    // --- NVS Keys (max 15 chars) ---
    static constexpr const char *KEY_MPU_GYRO_RANGE = "mpu.g_range";

    // --- Gyroscope Range Mappings ---
    static const char *GYRO_RANGE_STRINGS[];
    static const uint8_t NUM_GYRO_RANGES;

private:
    Preferences _preferences;
    bool _is_begun = false;

    // --- Schema Version ---
    static constexpr uint16_t CURRENT_SCHEMA_VERSION = 1;
    static constexpr const char *KEY_SCHEMA_VERSION = "schema_ver";

    void _write_defaults();

    // --- Settings Metadata ---
    enum SettingType
    {
        UINT8,
        INT32,
        FLOAT,
        STRING
    };

    struct SettingMetadata {
        const char* key;
        const char* display_key; // New field for user-friendly display
        const char* description;
        SettingType type;
        const char* const* string_map; // Pointer to an array of strings for human-readable values
        uint8_t string_map_size;       // Size of the string map
        int default_value;             // Default integer value for the setting
    };

    static const SettingMetadata _settings_metadata[];
    static const int _num_settings;
};