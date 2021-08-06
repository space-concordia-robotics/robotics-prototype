#ifndef ROVER_H
#define ROVER_H
#include "DcMotor.h"
#include "Servo.h"


const float radius = 0.14;
const float piRad  = 0.10472;
const float wheelBase = 0.33;

#define V_SENSE_PIN 39 // for reading battery voltage

/***** MOTORS *****/

// R/L (right/left), F/M/B (forward, middle, back)

/* motor driver pins (cytron) */
#define NUM_MOTORS 6

// direction pins
#define RF_DIR   2
#define RM_DIR   11
#define RB_DIR   26//12
#define LF_DIR   24
#define LM_DIR   25
#define LB_DIR   12//26

// pwm pins
#define RF_PWM   3
#define RM_PWM   4
#define RB_PWM   8//5
#define LF_PWM   6
#define LM_PWM   7
#define LB_PWM   5//8

/* encoder pins */

// right side encoders
#define RF_EA    27
#define RF_EB    28
#define RM_EA    31
#define RM_EB    32
#define RB_EA    29
#define RB_EB    30

// left side encoders
#define LF_EA    37
#define LF_EB    38
#define LM_EA    36
#define LM_EB    35
#define LB_EA    33
#define LB_EB    34

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
#define THROTTLE_TIMEOUT 200
#define MOTOR_CONTROL_INTERVAL 10

#define SERVO_STOP 93
#define FRONT_BASE_DEFAULT_PWM 65
#define REAR_BASE_DEFAULT_PWM 35

enum ServoNames{
    FRONT_SIDE_SERVO = 0,
    FRONT_BASE_SERVO = 1,
    REAR_SIDE_SERVO= 2,
    REAR_BASE_SERVO = 3
};
class Rover{
public:
    Rover();
    void roverVelocityCalculator(void);
    void stopMotors();
    void attachMotor(MotorNames,uint8_t,uint8_t,float);
    void closeAllMotorLoop();
    void enableAllMotors(uint8_t state);
    void moveWheel(uint8_t, const int16_t);
    void steerRover(int8_t,int8_t);
    void updateWheelsVelocity();
    void controlCameraMotors(ServoNames servoID, uint16_t angle);
    void openAllMotorLoop();
    void writeServo(ServoNames, int16_t);
    void setMotorPidGains(MotorNames,float,float,float);
    void setMotorsEnabled(uint8_t);
    uint8_t getMotorsEnabled() const;
    float getLinearVelocity() const;
    float getRotationalVelocity() const;
    void setThrottleTimeout(const uint8_t &);
    uint8_t getSteeringEnabled() const;
    void setSteeringEnabled(const uint8_t&);
    void attachServo(ServoNames,uint8_t);

    void setAccelerationLimiter(uint8_t state);
    uint8_t isThrottleTimeoutEnabled() const;
    DcMotor motorList[6];
    Servo servoList[4];
    float getBatteryVoltage() const;

private:
    float desiredVelocityRight;
    float desiredVelocityLeft;

    motor_direction leftMotorDirection;
    motor_direction rightMotorDirection;

    uint8_t motorCount = 0;

    float rightLinearVelocity;
    float leftLinearVelocity;
    float linearVelocity;
    float rotationalVelocity;

    float maxOutputSignal;
    float minOutputSignal;

    volatile int rotationDirection = CCW;

    uint8_t throttleTimeout;
    uint8_t areMotorsEnabled;
    uint8_t isSteering = true;
    uint8_t accelerationLimiterEnabled = false;
    uint8_t servoCount;

    inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }




};
#endif