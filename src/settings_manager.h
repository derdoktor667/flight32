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
#include "imu_sensor.h" // For ImuAxisData

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

    // --- Offset Getters/Setters ---
    ImuAxisData getGyroOffsets();
    void setGyroOffsets(const ImuAxisData &offsets);
    ImuAxisData getAccelOffsets();
    void setAccelOffsets(const ImuAxisData &offsets);

    // --- NVS Keys (max 15 chars) ---
    static constexpr const char *KEY_SYSTEM_NAME = "system.name";
    static constexpr const char *DEFAULT_SYSTEM_NAME = "Flight32";
    static constexpr const char *KEY_MPU_GYRO_RANGE = "mpu.g_range";
    static constexpr const char *KEY_MOTOR_PROTOCOL = "motor.protocol";
    static constexpr const char *KEY_RX_PROTOCOL = "rx.protocol";
    static constexpr const char *KEY_RC_PROTOCOL_TYPE = "rc.protocol";
    static constexpr const char *KEY_RX_PIN = "rx.pin";

    // --- RC Channel Mapping Keys ---
    static constexpr const char *KEY_RC_CHANNEL_ROLL = "rc.ch.roll";
    static constexpr const char *KEY_RC_CHANNEL_PITCH = "rc.ch.pitch";
    static constexpr const char *KEY_RC_CHANNEL_THRO = "rc.ch.thro";
    static constexpr const char *KEY_RC_CHANNEL_YAW = "rc.ch.yaw";
    static constexpr const char *KEY_RC_CHANNEL_ARM = "rc.ch.arm";
    static constexpr const char *KEY_RC_CHANNEL_FMODE = "rc.ch.fmode";
    static constexpr const char *KEY_RC_CHANNEL_AUX1 = "rc.ch.aux1";
    static constexpr const char *KEY_RC_CHANNEL_AUX2 = "rc.ch.aux2";
    static constexpr const char *KEY_RC_CHANNEL_AUX3 = "rc.ch.aux3";
    static constexpr const char *KEY_RC_CHANNEL_AUX4 = "rc.ch.aux4";

    // --- IMU Offset Keys ---
    static constexpr const char *KEY_MPU_GYRO_OFF_X = "mpu.g_off.x";
    static constexpr const char *KEY_MPU_GYRO_OFF_Y = "mpu.g_off.y";
    static constexpr const char *KEY_MPU_GYRO_OFF_Z = "mpu.g_off.z";
    static constexpr const char *KEY_MPU_ACCEL_OFF_X = "mpu.a_off.x";
    static constexpr const char *KEY_MPU_ACCEL_OFF_Y = "mpu.a_off.y";
    static constexpr const char *KEY_MPU_ACCEL_OFF_Z = "mpu.a_off.z";

    // --- Gyroscope Range Mappings ---
    static const char *GYRO_RANGE_STRINGS[];
    static const uint8_t NUM_GYRO_RANGES;

    // --- Settings Metadata (Made public for terminal access) ---
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