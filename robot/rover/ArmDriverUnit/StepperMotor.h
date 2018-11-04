#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RobotMotor.h"

// time interval between stepper steps
#define STEP_INTERVAL1 35
#define STEP_INTERVAL2 25
#define STEP_INTERVAL3 10
#define STEP_INTERVAL4 3

#define FULL_STEP 1
#define HALF_STEP 0.5
#define QUARTER_STEP 0.25
#define EIGHTH_STEP 0.125
#define SIXTEENTH_STEP 0.0625

const int stepIntervalArray[] = {STEP_INTERVAL1, STEP_INTERVAL2, STEP_INTERVAL3, STEP_INTERVAL4};

class StepperMotor : public RobotMotor {
  public:
    static int numStepperMotors;
    float stepResolution; // the smallest angle increment attainable by the shaft once the stepping mode is known

    StepperMotor(int enablePin, int dirPin, int stepPin, float stepRes, float stepMode, float gearRatio);
    void singleStep(int dir);
    void enablePower(void);
    void disablePower(void);

    bool calcNumSteps(float angle); // calculates how many steps to take to get to the desired position, assuming no slipping

    void setVelocity(int motorDir, int motorSpeed);

    // stuff for open loop control
    float openLoopError; // public variable for open loop control
    int openLoopSpeed; // angular speed (degrees/second)
    int numSteps; // how many steps to take for stepper to reach desired position
    volatile int stepCount; // how many steps the stepper has taken since it started moving

  private:
    int enablePin, directionPin, stepPin;
    unsigned int stepInterval;
    float fullStepResolution, steppingMode;
};

int StepperMotor::numStepperMotors = 0; // must initialize variable outside of class

StepperMotor::StepperMotor(int enablePin, int dirPin, int stepPin, float stepRes, float stepMode, float gearRatio): // if no encoder
  enablePin(enablePin), directionPin(dirPin), stepPin(stepPin), fullStepResolution(stepRes), steppingMode(stepMode)
{
  numStepperMotors++;
  // variables declared in RobotMotor require the this-> operator
  this->gearRatio = gearRatio;
  this->gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  hasEncoder = false;
  stepResolution = fullStepResolution * steppingMode;

  openLoopSpeed = 0; // no speed by default;
}

void StepperMotor::enablePower(void) {
  digitalWriteFast(enablePin, LOW);
}

void StepperMotor::disablePower(void) {
  digitalWriteFast(enablePin, HIGH);
}

void StepperMotor::singleStep(int dir) {
  switch (dir) {
    case CLOCKWISE:
      digitalWriteFast(directionPin, HIGH);
      break;
    case COUNTER_CLOCKWISE:
      digitalWriteFast(directionPin, LOW);
      break;
  }
  digitalWriteFast(stepPin, HIGH);
  digitalWriteFast(stepPin, LOW);
}

void StepperMotor::setVelocity(int motorDir, int motorSpeed) {
  ;
}

bool StepperMotor::calcNumSteps(float angle) {
  // if the error is big enough to justify movement
  // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
  if ( fabs(angle) > pidController.jointAngleTolerance) {
    numSteps = fabs(angle) * gearRatio / stepResolution; // calculate the number of steps to take
    return true;
  }
  else {
    return false;
  }
}

#endif
