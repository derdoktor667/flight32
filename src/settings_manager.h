/**
 * @file settings_manager.h
 * @brief Manages persistent settings using ESP32's non-volatile storage (NVS).
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "imu/imu_sensor.h"
#include "config/settings_config.h"

enum class CommandCategory;

class SettingsManager
{
public:
    SettingsManager();

    void begin();
    void loadOrInitSettings();
    void saveSettings();
    void factoryReset();
    void listSettings(CommandCategory category);
    void dumpSettings(CommandCategory category);

    String getSettingValue(const char *key);
    bool setSettingValue(const char *key, const String &value_str);
    const char *getSettingDescription(const char *key);
    String getSettingValueHumanReadable(const char *key);
    String getSettingOptionsHumanReadable(const char *key);
    const char *getInternalKeyFromDisplayKey(const char *display_key);

    ImuAxisData getGyroOffsets();
    void setGyroOffsets(const ImuAxisData &offsets);
    ImuAxisData getAccelOffsets();
    void setAccelOffsets(const ImuAxisData &offsets);



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
        const char *display_key;
        const char *description;
        SettingType type;
        const char *const *string_map;
        uint8_t string_map_size;
        uint8_t default_value;
        float default_float_value;
        const char *string_default;
    };

    static const SettingMetadata _settings_metadata[];
    static const int _num_settings;

private:
    Preferences _preferences;
    bool _is_begun = false;

    void _write_defaults();
};
