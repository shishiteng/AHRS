// #ifndef MadgwickAHRS_h
// #define MadgwickAHRS_h

// //----------------------------------------------------------------------------------------------------
// // Variable declaration

// extern volatile float beta;				// algorithm gain
// extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
// extern float roll,pitch,yaw;

// //---------------------------------------------------------------------------------------------------
// // Function declarations

// void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
// void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
// void computeAngles();

#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration

class Madgwick
{
private:
    volatile float beta;  // algorithm gain
    float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
    float roll, pitch, yaw;
    char anglesComputed;
    static float invSqrt(float x);
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations

public:
    Madgwick();
    //void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void getQuaternion(float *q)
    {
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
    }
    void setQuaternion(float qw, float qx, float qy, float qz)
    {
        q0 = qw;
        q1 = qx;
        q2 = qy;
        q3 = qz;
    }
    float getRoll()
    {
        if (!anglesComputed)
            computeAngles();
        return roll * 57.29578f;
    }
    float getPitch()
    {
        if (!anglesComputed)
            computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw()
    {
        if (!anglesComputed)
            computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return roll;
    }
    float getPitchRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return pitch;
    }
    float getYawRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return yaw;
    }
};

#endif