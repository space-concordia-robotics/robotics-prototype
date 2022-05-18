//
// Created by william on 18/05/22.
//

#ifndef MCU_CONTROL_IK_SOLVER_H
#define MCU_CONTROL_IK_SOLVER_H

float getJointAngle1For2D(float endPointX, float endPointY, float beta);

float getJointAngle2For2D(float endPointX, float endPointY);

float getJointAngularSpeedBetweenPoints(float startPointX, float startPointY, float endPointX, float endPointY,
                                        float linearSpeed, float* joint1Speed, float* joint2Speed);

#endif //MCU_CONTROL_IK_SOLVER_H
