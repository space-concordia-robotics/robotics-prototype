#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <cmath>
#include "Servo.h"

// Servo pins

#define CF_2_SERVO 23
#define CF_1_SERVO 22
#define CB_1_SERVO 16
#define CB_2_SERVO 17

// Dc Motor pins

#define NUM_MOTORS 6

#define M6_RL_PWM 2
#define M5_ML_PWM 5
#define M4_FL_PWM 4

#define M3_RR_PWM 3
#define M2_MR_PWM 6
#define M1_FR_PWM 7

#define M6_RL_DIR 26
#define M5_ML_DIR 12
#define M4_FL_DIR 24

#define M3_RR_DIR 25
#define M2_MR_DIR 11
#define M1_FR_DIR 8

// Motor encoder pins

#define M6_RL_A 27
#define M6_RL_B 28

#define M5_ML_A 33
#define M5_ML_B 34

#define M4_FL_A 31
#define M4_FL_B 32

#define M3_RR_A 29
#define M3_RR_B 30

#define M2_MR_A 37
#define M2_MR_B 38

#define M1_FR_A 35
#define M1_FR_B 36


#endif //ROVER_CONFIG_H

