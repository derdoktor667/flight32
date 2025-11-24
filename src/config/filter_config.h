#pragma once



// NVS Keys for Filter Settings (max 15 characters)
inline const char *NVS_KEY_GYRO_LPF_HZ = "gyro_lpf_hz";
inline const char *NVS_KEY_NOTCH1_HZ = "notch1_hz";
inline const char *NVS_KEY_NOTCH1_Q = "notch1_q";
inline const char *NVS_KEY_NOTCH2_HZ = "notch2_hz";
inline const char *NVS_KEY_NOTCH2_Q = "notch2_q";

// Default values for Filter Settings
constexpr float DEFAULT_GYRO_LPF_HZ = 150.0f; // Gyro Low-Pass Filter Cutoff Frequency in Hz
constexpr float DEFAULT_NOTCH1_HZ = 250.0f;  // First Notch Filter Center Frequency in Hz
constexpr float DEFAULT_NOTCH1_Q = 1.0f;     // First Notch Filter Q-Factor
constexpr float DEFAULT_NOTCH2_HZ = 150.0f;    // Second Notch Filter Center Frequency in Hz (0.0f disables it)
constexpr float DEFAULT_NOTCH2_Q = 1.0f;     // Second Notch Filter Q-Factor


