#include "RobotMotor.h"

int RobotMotor::numMotors = 0;

RobotMotor::RobotMotor() {
    numMotors++;
}

void RobotMotor::attachEncoder(int encA, int encB, int encRes) // :
// encoderPinA(encA), encoderPinB(encB), encoderPort(port), encoderShift(shift), encoderResolution(encRes)
{
    hasEncoder = true;
    encoderPinA = encA;
    encoderPinB = encB;
    encoderResolution = encRes;
    encoderResolutionReciprocal = 1 / (float) encRes;
}
