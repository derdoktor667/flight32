#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd), _integral(0.0f), _previous_error(0.0f)
{
}

void PIDController::reset()
{
    _integral = 0.0f;
    _previous_error = 0.0f;
}

float PIDController::update(float setpoint, float process_value, float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f; // Avoid division by zero or weird behavior
    }

    float error = setpoint - process_value;

    // Proportional term
    float p_term = _kp * error;

    // Integral term
    _integral += error * dt;
    float i_term = _ki * _integral;

    // Derivative term
    float derivative = (error - _previous_error) / dt;
    float d_term = _kd * derivative;

    // Update previous error for next iteration
    _previous_error = error;

    // Calculate total output
    float output = p_term + i_term + d_term;

    return output;
}

void PIDController::setKp(float kp)
{
    _kp = kp;
}

void PIDController::setKi(float ki)
{
    _ki = ki;
}

void PIDController::setKd(float kd)
{
    _kd = kd;
}

void PIDController::setGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

PidGains PIDController::getGains() const
{
    return {_kp, _ki, _kd};
}

float PIDController::getKp() const
{
    return _kp;
}

float PIDController::getKi() const
{
    return _ki;
}

float PIDController::getKd() const
{
    return _kd;
}