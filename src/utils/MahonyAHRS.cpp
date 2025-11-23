// MahonyAHRS.cpp

#include "MahonyAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain
#define PI 3.14159265359f

//---------------------------------------------------------------------------------------------------
// Variable definitions

// These are the global variables used in the MahonyAHRS filter
float twoKp = twoKpDef;                                           // 2 * proportional gain (Kp)
float twoKi = twoKiDef;                                           // 2 * integral gain (Ki)
float q4 = 1.0f, q5 = 0.0f, q6 = 0.0f, q7 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float MahonyAHRS::inv_Sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

MahonyAHRS::MahonyAHRS(float Freq)
{
    sampleFreq = Freq;
    twoKp = twoKpDef;
    twoKi = twoKiDef;
    q4 = 1.0f;
    q5 = 0.0f;
    q6 = 0.0f;
    q7 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void MahonyAHRS::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q4q4, q4q5, q4q6, q4q7, q5q5, q5q6, q5q7, q6q6, q6q7, q7q7;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        updateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = inv_Sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = inv_Sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated calculations
        q4q4 = q4 * q4;
        q4q5 = q4 * q5;
        q4q6 = q4 * q6;
        q4q7 = q4 * q7;
        q5q5 = q5 * q5;
        q5q6 = q5 * q6;
        q5q7 = q5 * q7;
        q6q6 = q6 * q6;
        q6q7 = q6 * q7;
        q7q7 = q7 * q7;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q6q6 - q7q7) + my * (q5q6 - q4q7) + mz * (q5q7 + q4q6));
        hy = 2.0f * (mx * (q5q6 + q4q7) + my * (0.5f - q5q5 - q7q7) + mz * (q6q7 - q4q5));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q5q7 - q4q6) + my * (q6q7 + q4q5) + mz * (0.5f - q5q5 - q6q6));

        // Estimated direction of gravity and magnetic field (v and w)
        halfvx = q5q7 - q4q6;
        halfvy = q4q5 + q6q7;
        halfvz = q4q4 - 0.5f + q6q6 + q7q7;
        halfwx = bx * (0.5f - q6q6 - q7q7) + bz * (q5q7 - q4q6);
        halfwy = bx * (q5q6 - q4q7) + bz * (q4q5 + q6q7);
        halfwz = bx * (q4q6 + q5q7) + bz * (0.5f - q5q5 - q6q6);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q4;
    qb = q5;
    qc = q6;
    q4 += (-qb * gx - qc * gy - q7 * gz);
    q5 += (qa * gx + qc * gz - q7 * gy);
    q6 += (qa * gy - qb * gz + q7 * gx);
    q7 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = inv_Sqrt(q4 * q4 + q5 * q5 + q6 * q6 + q7 * q7);
    q4 *= recipNorm;
    q5 *= recipNorm;
    q6 *= recipNorm;
    q7 *= recipNorm;
}

// IMU algorithm update

void MahonyAHRS::updateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = inv_Sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q5 * q7 - q4 * q6;
        halfvy = q4 * q5 + q6 * q7;
        halfvz = q4 * q4 - 0.5f + q6 * q6 + q7 * q7;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q4;
    qb = q5;
    qc = q6;
    q4 += (-qb * gx - qc * gy - q7 * gz);
    q5 += (qa * gx + qc * gz - q7 * gy);
    q6 += (qa * gy - qb * gz + q7 * gx);
    q7 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = inv_Sqrt(q4 * q4 + q5 * q5 + q6 * q6 + q7 * q7);
    q4 *= recipNorm;
    q5 *= recipNorm;
    q6 *= recipNorm;
    q7 *= recipNorm;
}
