#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RobotMotor.h"

class DcMotor: public RobotMotor {
  public:
    static int numDcMotors;

    // DcMotor(int pwmPin, int encA, int encB); // for sabertooth
    DcMotor(int dirPin, int pwmPin, float gearRatio); // for cytron
    /* movement helper functions */
    bool calcTurningDuration(float angle); // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
    bool calcCurrentAngle(void);
    /* movement functions */
    void stopRotation(void);
    void setVelocity(int motorDir, float motorSpeed); // currently this actually activates the dc motor and makes it turn at a set speed/direction
    void goToCommandedAngle(void);
    void budge(void);

    // stuff for open loop control
    float openLoopError; // public variable for open loop control
    int openLoopSpeed; // angular speed (degrees/second)
    float openLoopGain; // speed correction factor
    float startAngle; // used in angle esimation
    unsigned int numMillis; // how many milliseconds for dc motor to reach desired position
    elapsedMillis timeCount; // how long has the dc motor been turning for

  private:
    int directionPin; // for new driver
    int pwmPin;
};

int DcMotor::numDcMotors = 0; // must initialize variable outside of class
// for sabertooth
// DcMotor::DcMotor(int pwmPin, int encA, int encB):
// pwmPin(pwmPin), encA(encA), encB(encB)
// for new driver
DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio):// if no encoder
  directionPin(dirPin), pwmPin(pwmPin)
{
  numDcMotors++;
  // variables declared in RobotMotor require the this-> operator
  this -> gearRatio = gearRatio;
  this -> gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  this -> motorType = DC_MOTOR;
  hasEncoder = false;
  openLoopSpeed = 0; // no speed by default;
  openLoopGain = 1.0; // temp open loop control
}

void DcMotor::stopRotation(void) {
  analogWrite(pwmPin, 0); // 0 for cytron, different implementation for sabertooth
  movementDone = true;
  isBudging = false;
}

void DcMotor::setVelocity(int motorDir, float motorSpeed) {
  //static int oldDir = CLOCKWISE;
  if (!isOpenLoop) {
    motorSpeed = fabs(motorSpeed);
  }
  // makes sure the speed is within the limits set in the pid during setup
  if (motorSpeed * motorDir > pidController.getMaxOutputValue()) {
    motorSpeed = pidController.getMaxOutputValue();
  }
  if (motorSpeed * motorDir < pidController.getMinOutputValue()) {
    motorSpeed = pidController.getMinOutputValue();
  }
  //if (motorDir != oldDir) {
  switch (motorDir) {
    case CLOCKWISE:
      digitalWriteFast(directionPin, LOW);
      break;
    case COUNTER_CLOCKWISE:
      digitalWriteFast(directionPin, HIGH);
      break;
  }
  //oldDir = motorDir;
  //}
  int dutyCycle = motorSpeed * 255 / 100;
  analogWrite(pwmPin, dutyCycle);
}

void DcMotor::budge(void) {
  isBudging = true;
  movementDone = false;
  sinceBudgeCommand = 0;
  if (isOpenLoop) {
    startAngle = getImaginedAngle();
  }
  else if (!isOpenLoop) {
    startAngle = getCurrentAngle();
  }
}

void DcMotor::goToCommandedAngle(void) {
  if (isOpenLoop) {
    calcCurrentAngle();
    startAngle = getImaginedAngle();
    openLoopError = getDesiredAngle() - getImaginedAngle(); // find the angle difference
    calcDirection(openLoopError); // determine rotation direction and save the value
    // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
    if (calcTurningDuration(openLoopError)) { // returns false if the open loop error is too small
      timeCount = 0; // this elapsedMillis counts how long the motor has been turning for and is therefore reset right before it starts moving
      movementDone = false; // this flag being false lets the motor be controlled inside the timer interrupt
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
      UART_PORT.print("$S,Success: motor ");
      UART_PORT.print(1);
      UART_PORT.print(" to turn for ");
      UART_PORT.print(numMillis);
      UART_PORT.println(" milliseconds");
#endif
    }
    else {
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
      UART_PORT.println("$E,Error: requested angle is too close to current angle. Motor not changing course.");
#endif
    }
  }
  else if (!isOpenLoop) {
    // all the heavy lifting for closed loop control is done in the timer interrupt
    movementDone = false;
  }
}
bool DcMotor::calcTurningDuration(float angle) {
  // if the error is big enough to justify movement
  if (fabs(angle) > pidController.getJointAngleTolerance()) {
    // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
    numMillis = (fabs(angle) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain; // calculate how long to turn for
    return true;
  }
  else {
    return false;
  }
}

bool DcMotor::calcCurrentAngle(void) {
  if (isBudging) {
    imaginedAngle = startAngle + (float)rotationDirection * (float)sinceBudgeCommand * openLoopSpeed * gearRatioReciprocal / (1000.0 * openLoopGain);
    return true;
  }
  else if (isOpenLoop) {
    if (movementDone) {
      // imaginedAngle hasn't changed since motor hasn't moved and encoder isn't working
    }
    else if (timeCount < numMillis) {
      // if the motor is moving, calculate the angle based on how long it's been turning for
      imaginedAngle = startAngle + (desiredAngle - startAngle) * ((float)timeCount / (float)numMillis);
    }
    return true;
  }
  else if (hasEncoder) { // closed loop and has encoder
    currentAngle = (float) encoderCount * 360.0 * gearRatioReciprocal * encoderResolutionReciprocal;
    imaginedAngle = currentAngle;
    return true;
  }
  else { // closed loop and has no encoder
    return false;
  }
}

#endif
