/**
 * @file filter.h
 * @brief Implements a versatile second-order IIR (Biquad) filter.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <cmath>
// Implements a versatile second-order IIR (Biquad) filter.
// This class can be configured to act as a low-pass or a notch filter.
// It is based on the "Audio EQ Cookbook" by Robert Bristow-Johnson.
// The filter uses the Transposed Direct Form II structure for better
// numerical stability.
class BiquadFilter
{
public:
    BiquadFilter() = default;

    // Configures the filter as a low-pass filter.
    // sample_freq_hz: The sampling frequency in Hz.
    // cutoff_freq_hz: The cutoff frequency in Hz.
    void configureLowpass(float sample_freq_hz, float cutoff_freq_hz);

    // Configures the filter as a notch filter.
    // sample_freq_hz: The sampling frequency in Hz.
    // center_freq_hz: The center frequency of the notch in Hz.
    // q: The quality factor of the notch.
    void configureNotch(float sample_freq_hz, float center_freq_hz, float q);

    // Applies the filter to an input sample.
    // input: The input sample.
    // Returns the filtered output sample.
    float apply(float input);

    // Resets the filter's internal state to zero.
    void reset();

private:
    // Filter coefficients
    float _b0 = 1.0f, _b1 = 0.0f, _b2 = 0.0f, _a1 = 0.0f, _a2 = 0.0f;

    // Filter state (delay elements)
    float _z1 = 0.0f, _z2 = 0.0f;
};
