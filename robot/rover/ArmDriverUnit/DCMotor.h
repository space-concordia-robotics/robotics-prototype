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
    void motorTimerInterrupt(void);
    /* movement helper functions */
    bool calcTurningDuration(float angle); // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
    bool calcCurrentAngle(void);
    /* movement functions */
    void stopRotation(void);
    void setVelocity(int motorDir, float motorSpeed); // currently this actually activates the dc motor and makes it turn at a set speed/direction
    void goToCommandedAngle(void);
    void goToAngle(float angle);
    void budge(int dir);

    // stuff for open loop control
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

void DcMotor::motorTimerInterrupt(void) {
  if (isBudging) {
    if (sinceBudgeCommand < BUDGE_TIMEOUT) {
      calcCurrentAngle();
      setVelocity(rotationDirection, openLoopSpeed);
    }
    else {
      isBudging = false;
      movementDone = true;
      stopRotation();
    }
  }
  // movementDone can be set elsewhere... so can numMillis, openLoopSpeed and rotationDirection (in open loop control)
  else if (isOpenLoop) { // open loop control
    if (!movementDone && timeCount <= numMillis) {
      // calculates the pwm to send to the motor and makes it move
      calcCurrentAngle();
      setVelocity(rotationDirection, openLoopSpeed);
#ifdef DEBUG_DC_TIMER
      UART_PORT.println("motor");
      UART_PORT.print(rotationDirection); UART_PORT.println(" direction");
      UART_PORT.print(timeCount); UART_PORT.print("\t / ");
      UART_PORT.print(numMillis); UART_PORT.println(" ms");
      UART_PORT.print(getSoftwareAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(getDesiredAngle()); UART_PORT.print("\t / ");
      UART_PORT.print(startAngle); UART_PORT.println(" degrees");
#endif
    }
    else {
      movementDone = true;
      stopRotation();
    }
  }
  // would be nice to have some kind of check for the above functions so the command only runs if there's been a change
  // e.g. movementDone changed or the speed or numMillis changed
  else if (!isOpenLoop) {
    if (!movementDone) {
      calcCurrentAngle();
      // determine the speed of the motor until the next interrupt
      float output = pidController.updatePID(getSoftwareAngle(), getDesiredAngle());
      if (output == 0) {
        movementDone = true;
        stopRotation();
      }
      else {
        int dir = calcDirection(output);
        // calculates the pwm to send to the motor and makes it move
        setVelocity(dir, output);
#ifdef DEBUG_DC_TIMER
        UART_PORT.println("motor");
        UART_PORT.print(rotationDirection); UART_PORT.println(" direction");
        UART_PORT.print(output); UART_PORT.println(" next output");
#endif
      }
    }
    else {
      stopRotation();
    }
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

void DcMotor::goToCommandedAngle(void) {
  if (isOpenLoop) {
    calcCurrentAngle();
    startAngle = getSoftwareAngle();
    openLoopError = getDesiredAngle() - getSoftwareAngle(); // find the angle difference
    calcDirection(openLoopError); // determine rotation direction and save the value
    // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
    if (calcTurningDuration(openLoopError)) { // returns false if the open loop error is too small
      timeCount = 0; // this elapsedMillis counts how long the motor has been turning for and is therefore reset right before it starts moving
      movementDone = false; // this flag being false lets the motor be controlled inside the timer interrupt
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#ifdef DEBUG_MAIN
      UART_PORT.print("$S,Success: motor");
      UART_PORT.print(" to turn for ");
      UART_PORT.print(numMillis);
      UART_PORT.println(" milliseconds");
#endif
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

void DcMotor::goToAngle(float angle) {
  desiredAngle = angle;
  if (isOpenLoop) {
    calcCurrentAngle();
    startAngle = getSoftwareAngle();
    openLoopError = getDesiredAngle() - getSoftwareAngle(); // find the angle difference
    calcDirection(openLoopError);
    // calculates how many steps to take to get to the desired position, assuming no slipping
    numMillis = (fabs(openLoopError) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain;
    timeCount = 0;
    movementDone = false;
#if defined(DEVEL_MODE_1) || defined(DEVEL_MODE_2)
#ifdef DEBUG_MAIN
    UART_PORT.print("$A,Alert: motor");
    UART_PORT.println(" to move back into software angle range");
#endif
#endif
  }
  else if (!isOpenLoop) {
    movementDone = false;
  }
}

void DcMotor::budge(int dir) {
  calcCurrentAngle();
  float ang = getSoftwareAngle();
  bool canMove = true;
  if (hasAngleLimits) {
    if ( ( (dir > 0) && (ang > maxJointAngle) ) || ( (dir < 0) && (ang < minJointAngle) ) ) {
      canMove = false;
    }
  }
  if (canMove) {
    calcDirection(dir);
    isBudging = true;
    movementDone = false;
    sinceBudgeCommand = 0;
    startAngle = getSoftwareAngle();
  }
}

#endif
