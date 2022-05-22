//
// Created by william on 18/05/22.
//

#ifndef MCU_CONTROL_IK_SOLVER_H
#define MCU_CONTROL_IK_SOLVER_H

#ifdef __cplusplus
extern "C" {
#endif
extern float getJointAngle1For2D(float endPointX, float endPointY, float beta);

extern float getJointAngle2For2D(float endPointX, float endPointY);

extern float getJointAngularSpeedBetweenPoints(float startPointX, float startPointY, float endPointX, float endPointY,
                                        float linearSpeed, float *joint1Speed, float *joint2Speed);
#ifdef __cplusplus
}
#endif

#endif //MCU_CONTROL_IK_SOLVER_H
