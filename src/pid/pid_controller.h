/**
 * @file pid_controller.h
 * @brief Defines the PIDController class for flight stabilization.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */
#pragma once

#include <cstdint>

enum class PidAxis
{
    ROLL,
    PITCH,
    YAW
};

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

    float update(float setpoint, float process_value, float dt);

    void reset();

    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setGains(float kp, float ki, float kd);

    float getKp() const;
    float getKi() const;
    float getKd() const;
    PidGains getGains() const;

private:
    float _kp;
    float _ki;
    float _kd;

    float _integral;
    float _previous_error;
};