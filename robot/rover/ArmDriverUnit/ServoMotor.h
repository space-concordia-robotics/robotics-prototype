#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include "PinSetup.h"
#include "RobotMotor.h"

class ServoMotor: public RobotMotor {
  public:
    static int numServoMotors;

    ServoMotor(int pwmPin, float gearRatio);
    void motorTimerInterrupt(void);
    /* movement helper functions */
    bool calcTurningDuration(float angle); // guesstimates how long to turn at the preset open loop motor speed to get to the desired position
    bool calcCurrentAngle(void);
    /* movement functions */
    void stopRotation(void);
    void setVelocity(int motorDir, float motorSpeed); // currently this actually activates the servo and makes it turn at a set speed/direction
    void goToCommandedAngle(void);
    void goToAngle(float angle);
    void budge(int dir);

    // stuff for open loop control
    unsigned int numMillis; // how many milliseconds for servo to reach desired position
    elapsedMillis timeCount; // how long has the servo been turning for

  private:
    int pwmPin;
    Servo servo;
};

int ServoMotor::numServoMotors = 0; // must initialize variable outside of class

ServoMotor::ServoMotor(int pwmPin, float gearRatio):
  pwmPin(pwmPin)
{
  servo.attach(pwmPin);
  numServoMotors++;
  // variables declared in RobotMotor require the this-> operator
  this -> gearRatio = gearRatio;
  this -> gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  this -> motorType = CONTINUOUS_SERVO;
  hasEncoder = false;

  openLoopSpeed = 0; // no speed by default;
  openLoopGain = 1.0; // temp open loop control
}

void ServoMotor::motorTimerInterrupt(void) {
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
  else if (isOpenLoop) {
    // open loop control
    if (!movementDone && timeCount < numMillis) {
      calcCurrentAngle();
      setVelocity(rotationDirection, openLoopSpeed);
#ifdef DEBUG_SERVO_TIMER
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
      // really it should only do these tasks once, shouldn't repeat each interrupt the motor is done moving
      movementDone = true;
      stopRotation();
    }
  }
  else if (!isOpenLoop) {
    if (!movementDone) {
      calcCurrentAngle();
      float output = pidController.updatePID(getSoftwareAngle(), getDesiredAngle());
      if (output == 0) {
        movementDone = true;
        stopRotation();
      }
      else {
        int dir = calcDirection(output);
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

bool ServoMotor::calcTurningDuration(float angle) {
  // if the error is big enough to justify movement
  // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
  if (fabs(angle) > pidController.getJointAngleTolerance()) {
    numMillis = (fabs(angle) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain; // calculate how long to turn for
    // Serial.println(numMillis);
    return true;
  }
  else {
    return false;
  }
}

bool ServoMotor::calcCurrentAngle(void) {
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
  else if (hasEncoder) {
    currentAngle = (float) encoderCount * 360.0 * gearRatioReciprocal * encoderResolutionReciprocal;
    imaginedAngle = currentAngle;
    return true;
  }
  else {
    return false;
  }
}

void ServoMotor::stopRotation(void) {
  servo.writeMicroseconds(SERVO_STOP);
  movementDone = true;
  isBudging = false;
}

// takes a direction and offset from SERVO_STOP and sends appropriate pwm signal to servo
void ServoMotor::setVelocity(int motorDir, float motorSpeed) {
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

  // pulse time varies from 1000 to 2000, 1500 being the midpoint, so 500 is the offset from 1500
  int pulseTime = SERVO_STOP + (motorSpeed * motorDir * 500 / 100);
  if (pulseTime > 2000) {
    pulseTime = 2000;
  }
  if (pulseTime < 1000) {
    pulseTime = 1000;
  }
  servo.writeMicroseconds(pulseTime);
}

void ServoMotor::goToCommandedAngle(void) {
  if (isOpenLoop) {
    calcCurrentAngle();
    startAngle = getSoftwareAngle();
    openLoopError = getDesiredAngle() - getSoftwareAngle(); // find the angle difference
    calcDirection(openLoopError);
    if (calcTurningDuration(openLoopError)) {
      timeCount = 0;
      movementDone = false;
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
  else {
    if (!isOpenLoop) {
      movementDone = false;
    }
  }
}

void ServoMotor::goToAngle(float angle) {
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

void ServoMotor::budge(int dir) {
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
