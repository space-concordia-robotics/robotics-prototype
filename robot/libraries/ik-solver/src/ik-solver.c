//
// Created by william on 18/05/22.
//

#include <stdio.h>
#include <math.h>

#include "ik-solver.h"

// Units are in cm TODO: Double check arm lengths
const float ARM_LENGTH_1 = 30;
const float ARM_LENGTH_2 = 40;

// TODO: Ensure that 32bit floats only are used.

float getJointAngle1For2D(float endPointX, float endPointY, float beta) {
    float ratio = ARM_LENGTH_2*sinf(M_PI - beta) / (ARM_LENGTH_1 + ARM_LENGTH_2*cosf(M_PI - beta));

    float theta = atanf(ratio);

    // TODO: Spell check this lol
    float alfa = atanf(endPointY / endPointX);

    // Handle edge cases i.e. X is negative
    return alfa - theta;
}

float getJointAngle2For2D(float endPointX, float endPointY) {
    float top = (ARM_LENGTH_1*ARM_LENGTH_1 + ARM_LENGTH_2*ARM_LENGTH_2) - (endPointX*endPointX + endPointY*endPointY);
    float bottom = 2*ARM_LENGTH_1*ARM_LENGTH_2;

    float ratio = top / bottom;

    return acosf(ratio);
}

float getJointAngularSpeedBetweenPoints(float startPointX, float startPointY, float endPointX, float endPointY,
                                        float linearSpeed, float* joint1Speed, float* joint2Speed) {
    float deltaPosX = endPointX - startPointX;
    float deltaPosY = endPointY - startPointY;

    float startJoint2Angle = getJointAngle2For2D(startPointX, startPointY);
    float startJoint1Angle = getJointAngle1For2D(startPointX, startPointY, startJoint2Angle);

    float endJoint2Angle = getJointAngle2For2D(endPointX, endPointY);
    float endJoint1Angle = getJointAngle1For2D(endPointX, endPointY, endJoint2Angle);

    float deltaJoint2Angle = endJoint2Angle - startJoint2Angle;
    float deltaJoint1Angle = endJoint1Angle - startJoint1Angle;

    float deltaTime = deltaPosX / (linearSpeed * cos(deltaPosY / deltaPosX));

    *joint2Speed = deltaJoint2Angle / deltaTime;

    *joint1Speed = deltaJoint1Angle / deltaTime;
}

int main() {
    // Unit test 1
    float endPointX = 2;
    float endPointY = 3;

    float beta = getJointAngle2For2D(endPointX, endPointY);

    float alfa = getJointAngle1For2D(endPointX, endPointY, beta);

    printf("beta: %f | alfa: %f", beta, alfa);
}