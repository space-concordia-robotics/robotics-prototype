#ifndef ROVER_H
#define ROVER_H
#include "DcMotor.h"
#include "Servo.h"
#include "APA102.h"


const float radius = 0.14;
const float piRad  = 0.10472;
const float wheelBase = 0.33;

#define V_SENSE_PIN 39 // for reading battery voltage

/***** MOTORS *****/


#define NUM_MOTORS 6

#define M6_RL_PWM 2
#define M5_ML_PWM 3
#define M4_FL_PWM 4

#define M3_RR_PWM 5
#define M2_MR_PWM 6
#define M1_FR_PWM 7


#define M6_RL_DIR 26
#define M5_ML_DIR 25
#define M4_FL_DIR 24

#define M3_RR_DIR 12
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



// CONTINUITY TEST SEPT 12 2019 TEENSY TO MOTORs
// encodres
// RR: 29,30
// MR: 31,32
// FR: 27,28
// RL: 33,34
// ML: 35,36
// FL: 37,38

/* camera servo pins */
// F/R (front/rear), S/B (side/base)
// TODO: fix this: confusing naming scheme. call them pan/tilt instead of side/base
// pin definitions were changed because of above comment
#define FS_SERVO 23//22
#define FB_SERVO 22//23
#define RS_SERVO 16
#define RB_SERVO 17

#define LED_BLINK_INTERVAL 1000
#define SENSOR_READ_INTERVAL 200
//#define THROTTLE_TIMEOUT 200
#define THROTTLE_TIMEOUT 250

#define MOTOR_CONTROL_INTERVAL 10

#define SERVO_STOP 93
#define FRONT_BASE_DEFAULT_PWM 65
#define REAR_BASE_DEFAULT_PWM 35

/* Constants for the activity light  */
#define ACTIVITY_MOSI 21
#define ACTIVITY_CLK  20
#define ACTIVITY_BLINK_PERIOD  500
#define ACTIVITY_NUM_LIGHTS 20
#define ACTIVITY_BRIGHTNESS 1
#define ACTIVITY_DEFAULT_COLOR 20, 0, 0


enum ServoNames{
    FRONT_SIDE_SERVO = 0,
    FRONT_BASE_SERVO = 1,
    REAR_SIDE_SERVO= 2,
    REAR_BASE_SERVO = 3
};
enum SteerDirection{
    LEFT = 0,
    RIGHT = 1
};
enum ThrottleDirection{
    BACKWARDS= 0,
    FORWARD= 1
};

typedef struct {
    float right_linear_velocity;
    float left_linear_velocity;
    float linear_velocity;
    float rotational_velocity;
    int16_t max_output_signal;
    int16_t min_output_signal;
    // timestamp when the last rising/falling edge of the blink cycle happened
    unsigned int timeBlinkUpdated = 0;
    bool lightOn = false;
    bool blinking = false;
    APA102 light = APA102(ACTIVITY_NUM_LIGHTS, ACTIVITY_MOSI, ACTIVITY_CLK);
    uint8_t r, g, b;
} RoverState;

namespace Rover {

    extern RoverState roverState;

    extern SystemState systemStatus;

    void calculateRoverVelocity();

    void updateDesiredMotorVelocity(const MotorNames &, const motor_direction &, const int8_t &);

    void updateAllMotorVelocities(const motor_direction &, const int8_t &);

    void attachServo(const ServoNames&, const uint8_t&);

    void writeToServo(const ServoNames&, const int16_t&);

    void moveWheel(const MotorNames &,const motor_direction&, const int8_t& );

    void openLoop();

    void closeLoop();

    void steerRover(const uint8_t& throttle_direction, const uint8_t & throttle, const uint8_t & steer_direction,const uint8_t & steering);

    void stopMotors();

    void decelerateRover();

    // These functions are to be used by the MobilePlatofrm code
    void handleActivityLight();
    void setActivityBlinking(uint8_t on);
    // These functions are used internally by the above functions
    void setActivityLight(uint8_t on);
    void setActivityColor(uint8_t r, uint8_t g, uint8_t b);
}
#endif