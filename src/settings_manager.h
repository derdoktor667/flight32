/**
 * @file settings_manager.h
 * @brief Manages persistent settings for the Flight32 firmware using NVS.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

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
    static constexpr const char *KEY_SYSTEM_NAME = "system.name";
    static constexpr const char *DEFAULT_SYSTEM_NAME = "Flight32";
    static constexpr const char *KEY_MPU_GYRO_RANGE = "mpu.g_range";
    static constexpr const char *KEY_MOTOR_PROTOCOL = "motor.protocol";

    // --- IBUS Channel Mapping Keys ---
    static constexpr const char *KEY_IBUS_CHANNEL_ROLL = "ibus.ch.roll";
    static constexpr const char *KEY_IBUS_CHANNEL_PITCH = "ibus.ch.pitch";
    static constexpr const char *KEY_IBUS_CHANNEL_THRO = "ibus.ch.thro";
    static constexpr const char *KEY_IBUS_CHANNEL_YAW = "ibus.ch.yaw";
    static constexpr const char *KEY_IBUS_CHANNEL_ARM = "ibus.ch.arm";
    static constexpr const char *KEY_IBUS_CHANNEL_FMODE = "ibus.ch.fmode";
    static constexpr const char *KEY_IBUS_CHANNEL_AUX1 = "ibus.ch.aux1";
    static constexpr const char *KEY_IBUS_CHANNEL_AUX2 = "ibus.ch.aux2";
    static constexpr const char *KEY_IBUS_CHANNEL_AUX3 = "ibus.ch.aux3";
    static constexpr const char *KEY_IBUS_CHANNEL_AUX4 = "ibus.ch.aux4";

    // --- Gyroscope Range Mappings ---
    static const char *GYRO_RANGE_STRINGS[];
    static const uint8_t NUM_GYRO_RANGES;

private:
    Preferences _preferences;
    bool _is_begun = false;

    // --- Schema Version ---

    void _write_defaults();

    // --- Settings Metadata ---
    enum SettingType
    {
        UINT8,
        INT32,
        FLOAT,
        STRING
    };

    struct SettingMetadata
    {
        const char *key;
        const char *display_key; // New field for user-friendly display
        const char *description;
        SettingType type;
        const char *const *string_map; // Pointer to an array of strings for human-readable values
        uint8_t string_map_size;       // Size of the string map
        uint8_t default_value;             // Default integer value for the setting
        float default_float_value;     // Default float value for the setting (new)
        const char *string_default;    // Default string value
    };

    static const SettingMetadata _settings_metadata[];
    static const int _num_settings;
};