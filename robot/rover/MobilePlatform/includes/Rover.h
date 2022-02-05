#ifndef ROVER_H
#define ROVER_H
#include "DcMotor.h"
#include "Servo.h"

#include <cmath>

const float width = 0.83;
const float length = 1.2;

const float radius = 0.14;
const float piRad  = 0.10472;
const float wheelBase = 0.33;

#define V_SENSE_PIN 39 // for reading battery voltage

/***** MOTORS *****/

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

#define ROVER_MOVE_TIMEOUT 500

#define ACCELERATION_RATE 5

typedef struct {
    float right_linear_velocity;
    float left_linear_velocity;
    float linear_velocity;
    float rotational_velocity;
    int16_t max_output_signal;
    int16_t min_output_signal;
} RoverState;

namespace Rover {

    extern RoverState roverState;

    extern SystemState systemStatus;

    void calculateRoverVelocity();

    void moveWheel(const MotorNames &,const uint8_t &, const int8_t& );

    void moveRover(const float & ,const float &);

    void stopMotors();

    void updateWheelVelocities();

    void decelerateRover();
}
#endif