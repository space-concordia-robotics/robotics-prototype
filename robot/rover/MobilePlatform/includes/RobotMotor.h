#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "PidController.h"

/* wheel motors */
#define PULSES_PER_REV     14
#define GEAR_RATIO         188.61
#define MAX_RPM            30


// defines motor directions, CW CLOCKWISE, CCW COUNTER_CLOCKWISE
enum motor_direction { CW = -1, CCW = 1 };
// defines whether motor control is open loop or closed loop
enum loop_state { OPEN_LOOP = 1, CLOSED_LOOP };

class RobotMotor {
public:
    // these variables are set at start and normally don't change during the main loop
    static int numMotors; // keeps track of how many motors there are
    int encoderPinA, encoderPinB;
    int output_pwm;
    int final_output_pwm;
    int prev_output_pwm;
    float acc;
    bool accLimit = false;
    String motorName;
    volatile unsigned long  dt;
    volatile unsigned long  dt2;
    volatile unsigned long prevTime;
    volatile unsigned long prevTime2;
    float gearRatio, gearRatioReciprocal; // calculating this beforehand improves speed of floating point calculations
    float encoderResolutionReciprocal; // calculating this beforehand improves speed of floating point calculations
    bool isOpenLoop = true; // decides whether to use the PID or not
    volatile int rotationDirection = CCW;
    PidController pidController; // used for speed and angle control
    // these variables change during the main loop
    volatile long encoderCount = 0; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile long aveCount = 0; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile long prevCount = 0; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile long prevVelocity = 0; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile bool movementDone; // this variable is what allows the timer interrupts to make motors turn. can be updated within said interrupts
    // setup functions
    RobotMotor();
    void attachEncoder(int encA, int encB, int encRes);
    // void attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes);
    bool hasEncoder;

protected:
    // the following variables are specific to encoders
    int encoderResolution; // ticks per revolution
    volatile float currentVelocity; // can be updated within timer interrupts
    float desiredVelocity;
    int directionModifier;
};




#endif
