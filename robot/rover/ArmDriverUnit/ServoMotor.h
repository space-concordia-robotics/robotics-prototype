#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include "PinSetup.h"
#include "RobotMotor.h"

class ServoMotor : public RobotMotor {
  public:
    static int numServoMotors;

    ServoMotor(int pwmPin, float gearRatio);

    bool calcTurningDuration(float angle); // guesstimates how long to turn at the preset open loop motor speed to get to the desired position

    void setVelocity(int motorDir, float motorSpeed); // currently this actually activates the servo and makes it turn at a set speed/direction
    void stopRotation(void);

    // stuff for open loop control
    float openLoopError; // public variable for open loop control
    int openLoopSpeed; // angular speed (degrees/second)
    float openLoopGain; // speed correction factor
    unsigned int numMillis; // how many milliseconds for servo to reach desired position
    elapsedMillis timeCount; // how long has the servo been turning for

  private:
    int pwmPin;
};

int ServoMotor::numServoMotors = 0; // must initialize variable outside of class

ServoMotor::ServoMotor(int pwmPin, float gearRatio):
  pwmPin(pwmPin)
{
  numServoMotors++;
  // variables declared in RobotMotor require the this-> operator
  this->gearRatio = gearRatio;
  this->gearRatioReciprocal = 1 / gearRatio; // preemptively reduce floating point calculation time
  hasEncoder = false;

  openLoopSpeed = 0; // no speed by default;
  openLoopGain = 1.0; // temp open loop control
}

void ServoMotor::stopRotation(void) {
  analogWrite(pwmPin, SERVO_STOP);
  movementDone = true;
}

// takes a direction and offset from SERVO_STOP and sends appropriate pwm signal to servo
void ServoMotor::setVelocity(int motorDir, float motorSpeed) {
  if(!isOpenLoop) motorSpeed = fabs(motorSpeed);
  // makes sure the speed is within the limits set in the pid during setup
  if (motorSpeed * motorDir > pidController.maxOutputValue) {
    motorSpeed = pidController.maxOutputValue;
  }
  if (motorSpeed * motorDir < pidController.minOutputValue) {
    motorSpeed = pidController.minOutputValue;
  }

  int dutyCycle = SERVO_STOP + motorSpeed * motorDir * 128 / 100;
  if (dutyCycle > 255) dutyCycle = 255;
  if (dutyCycle < 0) dutyCycle = 0;

  analogWrite(pwmPin, dutyCycle);
}

bool ServoMotor::calcTurningDuration(float angle) {
  // if the error is big enough to justify movement
  // here we have to multiply by the gear ratio to find the angle actually traversed by the motor shaft
  if ( fabs(angle) > pidController.jointAngleTolerance) {
    numMillis = (fabs(angle) * gearRatio / openLoopSpeed) * 1000.0 * openLoopGain; // calculate how long to turn for
    //Serial.println(numMillis);
    return true;
  }
  else {
    return false;
  }
}

#endif
