#ifndef ROBOTMOTOR_H
#define ROBOTMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RobotPID.h"

enum motor_code {MOTOR1 = 1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6}; // defines 6 motors
enum motor_direction {CLOCKWISE = -1, COUNTER_CLOCKWISE = 1}; // defines motor directions // can go into ArmMotor
enum motor_speed {SPEED1 = 1, SPEED2, SPEED3, SPEED4}; // defines motor speed // can go into ArmMotor

//const int dir [16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; //quadrature encoder matrix. Corresponds to the correct direction for a specific set of prev and current encoder states

class RobotMotor {
  public:
    // these variables are set at start and normally don't change during the main loop
    static int numMotors;
    int encoderPinA, encoderPinB;
    float gearRatio;
    float gearRatioReciprocal;
    float maximumAngle, minimumAngle;
    bool hasAngleLimits;
    bool isOpenLoop; // decides whether to use the PID or not
    bool hasRamping; // decides whether to ramp the speed in open loop
    //int maxSpeed;

    RobotPID motorPID; // used for speed and angle control

    // these variables change during the main loop
    elapsedMillis sinceStart; // automatically increments every millisecond
    volatile long encoderCount; // incremented inside encoder interrupts
    volatile float currentAngle; // can be updated within timer interrupts
    float desiredAngle;
    volatile bool movementDone;

    // specifically for budge()
    int cwSpeed, ccwSpeed;
    bool budgeMovementDone;
    int rightCount, leftCount; // counters to make sure budge doesn't go too far
    bool canTurnRight = false; bool canTurnLeft = false; // bools that tell code to move or not

    RobotMotor();
    void attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes);
    void setAngleLimits(float minAngle, float maxAngle);

    virtual void budge(int budgeDir, int budgeSpeed, unsigned int budgeTime) = 0; // budges motor for short period of time

    virtual void setVelocity(int motorDir, int motorSpeed) = 0; // sets motor speed until next timer interrupt
    //void setMaxSpeed();
    bool setDesiredAngle(float angle); // need to have defined it for servos first
    //virtual float calcCurrentAngle(void) = 0; // need to have defined it for servos first

  private:
    //void update(); // not sure if this will be necessary anymore

  protected:
    // the following variables are specific to encoders
    //quadrature encoder matrix below corresponds to the correct direction for a specific set of prev and current encoder states
    //const int dir [16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    bool hasEncoder;
    uint32_t encoderPort;
    int encoderShift;
    int encoderResolution;

};

int RobotMotor::numMotors = 0; // must initialize variable outside of class

RobotMotor::RobotMotor() {
  numMotors++;
  movementDone = true;
  hasAngleLimits = false;
  isOpenLoop = true; // by default don't use PID
  hasRamping = false; // by default don't ramp the speed
}

void RobotMotor::attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes)//:
//encoderPinA(encA), encoderPinB(encB), encoderPort(port), encoderShift(shift), encoderResolution(encRes)
{
  hasEncoder = true;
  encoderPinA = encA;
  encoderPinB = encB;
  encoderPort = port;
  encoderShift = shift;
  encoderResolution = encRes;
}

void RobotMotor::setAngleLimits(float minAngle, float maxAngle) {
  minimumAngle = minAngle;
  maximumAngle = maxAngle;
  hasAngleLimits = true;
}

bool RobotMotor::setDesiredAngle(float angle) {
  if (angle > minimumAngle && angle < maximumAngle) {
    desiredAngle = angle;
    return true;
  }
  else {
    Serial.println("$E,Alert: requested angle is not within angle limits.");
    return false;
  }
}

#endif
