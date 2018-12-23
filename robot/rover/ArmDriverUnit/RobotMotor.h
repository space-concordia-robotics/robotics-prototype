#ifndef ROBOTMOTOR_H
#define ROBOTMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "PidController.h"

enum motor_code {MOTOR1 = 1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6}; // defines 6 motors
enum motor_direction {CLOCKWISE = -1, COUNTER_CLOCKWISE = 1}; // defines motor directions
enum loop_state {OPEN_LOOP = 1, CLOSED_LOOP}; // defines whether motor control is open loop or closed loop

class RobotMotor {
  public:
    // these variables are set at start and normally don't change during the main loop
    static int numMotors; // keeps track of how many motors there are
    int encoderPinA, encoderPinB;
    float gearRatio;
    float gearRatioReciprocal; // calculating this beforehand improves speed of floating point calculations
    float encoderResolutionReciprocal; // calculating this beforehand improves speed of floating point calculations
    float maximumAngle, minimumAngle; // joint angle limits, used to make sure the arm doesn't bend too far and break itself
    bool hasAngleLimits; // a wrist which wants to turn infinitely will be constrained by angle limits
    bool isOpenLoop; // decides whether to use the PID or not
    bool hasRamping; // decides whether to ramp the speed in open loop
    volatile int rotationDirection;
    //int maxSpeed;

    PidController pidController; // used for speed and angle control

    // these variables change during the main loop
    volatile long encoderCount; // incremented inside encoder interrupts, keeps track of how much the motor shaft has rotated and in which direction
    volatile float currentAngle; // can be updated within timer interrupts
    float desiredAngle;
    volatile bool movementDone; // this variable is what allows the timer interrupts to make motors turn. can be updated within said interrupts

    // setup functions
    RobotMotor();
    void attachEncoder(int encA, int encB, uint32_t port, int shift, int encRes);
    void setAngleLimits(float minAngle, float maxAngle); // sets joint limits so the arm doesn't break from trying to reach physically impossible angles
    bool hasEncoder;

    virtual void setVelocity(int motorDir, int motorSpeed) = 0; // sets motor speed and direction until next timer interrupt
    //void setMaxSpeed();
    void calcDirection(float error); // updates rotationDirection based on the angular error inputted
    bool setDesiredAngle(float angle); // need to have defined it for servos first
    float calcCurrentAngle(void);

  private:
    // doesn't really make sense to have any private variables for this parent class.
    // note that virtual functions must be public in order for them to be accessible from motorArray[]

  protected:
    // the following variables are specific to encoders
    uint32_t encoderPort; // address of the port connected to a particular encoder pin
    int encoderShift; // how many bits to shift over to find the encoder pin state
    int encoderResolution; // ticks per revolution

};

int RobotMotor::numMotors = 0; // must initialize variable outside of class

RobotMotor::RobotMotor() {
  numMotors++;
  movementDone = true; // by default the movement has finished so the motors don't need to move
  hasAngleLimits = false; // only set to true when setAngleLimits() is called
  isOpenLoop = true; // by default don't use PID
  hasRamping = false; // by default don't ramp the speed
  rotationDirection = 0; // by default invalid value
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
  if ( !hasAngleLimits || (angle > minimumAngle && angle < maximumAngle) ) {
    desiredAngle = angle;
    return true;
  }
  else {
    return false;
  }
}

void RobotMotor::calcDirection(float error) {
  if (error >= 0) rotationDirection = 1;
  else rotationDirection = -1;
}

float RobotMotor::calcCurrentAngle(void) {
  if (hasEncoder) {
    currentAngle = (float)encoderCount * 360.0 * gearRatioReciprocal * encoderResolutionReciprocal;
    return currentAngle;
  }
  else {
    Serial.println("$E,Error: motor does not have encoder");
    // use either max float value or set an actual max angle limit
    // perhaps use: return std::numeric_limits<float>::max();
    return 40404040404.0; // wants a return value, at least this value should be invalid
  }
}

#endif
