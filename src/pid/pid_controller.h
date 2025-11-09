/**
 * @file pid_controller.h
 * @brief Defines the PIDController class for flight stabilization.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */
#pragma once

#include <cstdint>

// Enum for PID axes
enum class PidAxis
{
    ROLL,
    PITCH,
    YAW
};

// Struct to hold PID gains
struct PidGains
{
    float p;
    float i;
    float d;
};

class PIDController
{
public:
    PIDController(float kp, float ki, float kd);

    // Calculates the control output
    float update(float setpoint, float process_value, float dt);

    // Resets the controller's internal state
    void reset();

    // Getters and setters for gains
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setGains(float kp, float ki, float kd);

    float getKp() const;
    float getKi() const;
    float getKd() const;
    PidGains getGains() const;

private:
    float _kp; // Proportional gain
    float _ki; // Integral gain
    float _kd; // Derivative gain

    float _integral;       // Integral accumulator
    float _previous_error; // For derivative calculation
};