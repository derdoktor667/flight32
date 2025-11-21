// MahonyAHRS.h

#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include <math.h>

class MahonyAHRS {
public:
    MahonyAHRS(float Freq);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float getRoll() { return atan2(q4 * q5 + q6 * q7, 0.5f - q5 * q5 - q6 * q6) * 57.29578f; }
    float getPitch() { return asin(-2.0f * (q5 * q7 - q4 * q6)) * 57.29578f; }
    float getYaw() { return atan2(q5 * q6 + q4 * q7, 0.5f - q6 * q6 - q7 * q7) * 57.29578f; }
    float getRollRadians() { return atan2(q4 * q5 + q6 * q7, 0.5f - q5 * q5 - q6 * q6); }
    float getPitchRadians() { return asin(-2.0f * (q5 * q7 - q4 * q6)); }
    float getYawRadians() { return atan2(q5 * q6 + q4 * q7, 0.5f - q6 * q6 - q7 * q7); }

public: // Made public for direct access in ImuMpu6050::read()
    float q4, q5, q6, q7; // quaternion of sensor frame relative to auxiliary frame
private:
    float sampleFreq;
    float twoKp;     // 2 * proportional gain (Kp)
    float twoKi;     // 2 * integral gain (Ki)
    float integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki
    float inv_Sqrt(float x);
};

#endif
