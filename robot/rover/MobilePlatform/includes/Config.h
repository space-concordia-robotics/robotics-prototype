#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include "Arduino.h"
#include "Servo.h"
#include <cstdint>
#include <SoftwareSerial.h>
#include <cmath>

#define V_SENSE_PIN 39 // for reading battery voltage

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

inline void double2bytes(uint8_t* buffer, double value){
    memcpy(buffer, (unsigned char*) (&value), sizeof(double));
}
inline void float2bytes(uint8_t* buffer, float value){
    memcpy(buffer, (unsigned char*) (&value), sizeof(float));
}
inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif //ROVER_CONFIG_H
