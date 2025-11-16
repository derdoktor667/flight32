/**
 * @file filter.cpp
 * @brief Implements a versatile second-order IIR (Biquad) filter.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "filter.h"
#include <cmath>

// Resets the filter's internal state to zero.
void BiquadFilter::reset()
{
    _z1 = 0.0f;
    _z2 = 0.0f;
}

// Configures the filter as a low-pass filter.
// Based on Robert Bristow-Johnson's "Audio EQ Cookbook".
void BiquadFilter::configureLowpass(float sample_freq_hz, float cutoff_freq_hz)
{
    // Ensure valid frequencies
    if (sample_freq_hz <= 0.0f || cutoff_freq_hz <= 0.0f || cutoff_freq_hz >= sample_freq_hz / 2.0f)
    {
        // Invalid parameters, reset filter to pass-through
        _b0 = 1.0f;
        _b1 = 0.0f;
        _b2 = 0.0f;
        _a1 = 0.0f;
        _a2 = 0.0f;
        reset();
        return;
    }

    float omega = 2.0f * M_PI * cutoff_freq_hz / sample_freq_hz;
    float sn = std::sin(omega);
    float cs = std::cos(omega);
    float alpha = sn / (2.0f * 0.707f); // Q = 0.707 for Butterworth

    float a0 = 1.0f + alpha;
    _b0 = (1.0f - cs) / (2.0f * a0);
    _b1 = (1.0f - cs) / a0;
    _b2 = (1.0f - cs) / (2.0f * a0);
    _a1 = (-2.0f * cs) / a0;
    _a2 = (1.0f - alpha) / a0;

    reset(); // Reset state after reconfiguring
}

// Configures the filter as a notch filter.
// Based on Robert Bristow-Johnson's "Audio EQ Cookbook".
void BiquadFilter::configureNotch(float sample_freq_hz, float center_freq_hz, float q)
{
    // Ensure valid frequencies and Q factor
    if (sample_freq_hz <= 0.0f || center_freq_hz <= 0.0f || center_freq_hz >= sample_freq_hz / 2.0f || q <= 0.0f)
    {
        // Invalid parameters, reset filter to pass-through
        _b0 = 1.0f;
        _b1 = 0.0f;
        _b2 = 0.0f;
        _a1 = 0.0f;
        _a2 = 0.0f;
        reset();
        return;
    }

    float omega = 2.0f * M_PI * center_freq_hz / sample_freq_hz;
    float sn = std::sin(omega);
    float cs = std::cos(omega);
    float alpha = sn / (2.0f * q);

    float a0 = 1.0f + alpha;
    _b0 = 1.0f / a0;
    _b1 = (-2.0f * cs) / a0;
    _b2 = 1.0f / a0;
    _a1 = (-2.0f * cs) / a0;
    _a2 = (1.0f - alpha) / a0;

    reset(); // Reset state after reconfiguring
}

// Applies the filter to an input sample using Transposed Direct Form II.
float BiquadFilter::apply(float input)
{
    float output = input * _b0 + _z1;
    _z1 = input * _b1 + _z2 - _a1 * output;
    _z2 = input * _b2 - _a2 * output;
    return output;
}
