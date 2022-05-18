//
// Created by william on 18/05/22.
//

#include <stdio.h>
#include <math.h>

#include "ik-solver.h"

// Units are in cm
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

int main() {
    // Unit test 1
    float endPointX = 2;
    float endPointY = 3;

    float beta = getJointAngle2For2D(endPointX, endPointY);

    float alfa = getJointAngle1For2D(endPointX, endPointY, beta);

    printf("beta: %f | alfa: %f", beta, alfa);
}