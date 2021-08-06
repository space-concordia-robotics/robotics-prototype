#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Arduino.h"



class PidController {
public:
    // motor-dependent constants... currently arbitrary values. to be set in setup() probably
    PidController(float,float,float);

    float updatePID(volatile float currentAngle, float desiredAngle);
    void setJointVelocityTolerance(float tolerance);
    void setOutputLimits(float minVal, float maxVal, float zeroVal);
    void setGainConstants(float kp, float ki, float kd);
    float getMaxOutputValue(void);
    float getMinOutputValue(void);
    float getJointVelocityTolerance(void);
    void printPidParameters(void);
private:
    float kp, ki, kd;
    float jointVelocityTolerance;
    float maxOutputValue, minOutputValue, slowestSpeed;
    elapsedMillis dt;
    float pTerm, iTerm, dTerm;
    float error, previousError;
    float errorSum;
    float pidSum; // pid output, must be checked before assigning this value to pidOutput
};
#endif
