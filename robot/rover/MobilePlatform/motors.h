#ifndef DCMOTOR_H
#define DCMOTOR_H
#include <Arduino.h>
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
    bool isOpenLoop = false; // decides whether to use the PID or not
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


private:
    // doesn't really make sense to have any private variables for this parent class.
    // note that virtual functions must be public in order for them to be accessible from motorArray[]
protected:
    // the following variables are specific to encoders
    int encoderResolution; // ticks per revolution
    volatile float currentVelocity; // can be updated within timer interrupts
    float desiredVelocity;
    int directionModifier;
};

int RobotMotor::numMotors = 0;

RobotMotor::RobotMotor() {
    numMotors++;
}

void RobotMotor::attachEncoder(int encA, int encB, int encRes) // :
// encoderPinA(encA), encoderPinB(encB), encoderPort(port), encoderShift(shift), encoderResolution(encRes)
{
    hasEncoder = true;
    encoderPinA = encA;
    encoderPinB = encB;
    encoderResolution = encRes;
    encoderResolutionReciprocal = 1 / (float) encRes;
}


class DcMotor: public RobotMotor {
public:
    DcMotor(int dirPin, int pwmPin, float gearRatio, String motorName);
    void setVelocity(int motorDir, float dV, volatile float currentVelocity); // currently this actually activates the dc motor and makes it turn at a set speed/direction
    int directionPin;
    int pwmPin;
    void calcCurrentVelocity(void);
    float getCurrentVelocity(void);
    float desiredVelocity;
    int desiredDirection;

protected:
    // the following variables are specific to encoders
    int encoderResolution; // ticks per revolution
    volatile float currentVelocity; // can be updated within timer interrupts
//    float desiredVelocity;
//    int directionModifier;
//    float setDesiredVelocity(float desiredVelocity);

};

DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio, String motorName):// if no encoder
        directionPin(dirPin), pwmPin(pwmPin)
{
    // variables declared in RobotMotor require the this-> operator
    this -> gearRatio = gearRatio;
    this -> gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
    this -> motorName = motorName;
}


void DcMotor::calcCurrentVelocity() {
//    Serial.print(dt);
//    Serial.print(" ");
//    Serial.print(encoderCount);
//    Serial.println(" ");

    if (dt <= 0 || encoderCount <= 0) {
        currentVelocity = 0;
    }
    else {
        currentVelocity = (float) (encoderCount * 60000000.0 * gearRatioReciprocal * encoderResolutionReciprocal / (float) (dt));
    }

    encoderCount = 0;
    dt = 0;
    //  Serial.println(dt/1000);
}

float DcMotor::getCurrentVelocity(void) {
    return currentVelocity;
}

void DcMotor::setVelocity(int motorDir, float dV, volatile float currentVelocity) {

    desiredVelocity = dV;
    desiredDirection = motorDir;
//    Serial.print(motorName);
    switch (motorDir) {
        case CW:
            digitalWrite(directionPin, HIGH);
            break;
        case CCW:
            digitalWrite(directionPin, LOW);
            break;
    }

    if (isOpenLoop) {

        output_pwm = desiredVelocity;
        analogWrite(pwmPin, output_pwm);

    }
    else if (!isOpenLoop) {
        // THIS LOOKS WRONG
        // makes sure the speed is within the limits set in the pid during setup
        if (desiredVelocity > 30) {
            desiredVelocity = 30;
        }
        else if (desiredVelocity < 0) {
            desiredVelocity = 0;
        }
        output_pwm = pidController.updatePID(currentVelocity, desiredVelocity);

//        Serial.print(output_pwm);
//        Serial.print(" ");

        analogWrite(pwmPin, fabs(output_pwm));

    }
    //this -> desiredVelocity = output_pwm;

    /* Acceleration limiter */
//    if (accLimit) {
//        final_output_pwm = output_pwm;
//        dt2 = micros() - prevTime2;
//
//        acc = ((float) output_pwm - (float) prev_output_pwm) / (float) dt2;
//
//        if (abs(acc) > 0.00051) {  // 0.00051 it the acceleration limit to go from 0 to full speed in 0.5 seconds. adjust this value for desired tuning
//            output_pwm = ((acc < 0) ? -1 : 1) * 0.000151 * dt2 + prev_output_pwm;
//        }
//        prevTime2 = micros();
//        prev_output_pwm = output_pwm;
//        analogWrite(pwmPin, output_pwm);
//    }
//    else analogWrite(pwmPin, output_pwm);

}

#endif
