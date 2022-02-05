#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

#include "RobotMotor.h"

class DcMotor {
 public:
  // both these are positive or negative based on direction
  int currentSpeed, targetSpeed;
  float gearRatio;

  DcMotor(int dirPin, int pwmPin, float gearRatio, int dirPinForward);
  DcMotor();

  /**
   * @param newSpeed Set speed, from -255 to +255. Positive is 'forward'.
   */
  void setSpeed(int newSpeed);
  /**
   * Does the autostop, or in future budge and angle move checks.
   * Should be called frequently.
   */
  void doChecks();
  void stop();

  unsigned int millisStartedMove, millisLastUpdate;

 private:
  int directionPin;
  int pwmPin;
  /**
   * @brief Which way to set the direction pin, HIGH or LOW,
   * when moving this motor 'forwards'
   */
  int dirPinForward;
  int dirPinBackward;
  /**
   * @brief Sets the amount of time (millis) to wait after starting a move
   * after which it should autostop.
   */
  static const unsigned int timeToWaitUntilStop = 1000;
};

DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio, int dirPinForward)
    : currentSpeed(0),
      pwmPin(pwmPin),
      gearRatio(1.0),
      directionPin(dirPin),
      dirPinForward(dirPinForward) {
  dirPinBackward = dirPinForward == HIGH ? LOW : HIGH;
  stop();
}

DcMotor::DcMotor()
    : currentSpeed(0),
      pwmPin(9),  // default is to set to the M1 pins
      gearRatio(1.0),
      directionPin(7),  // default is to set to the M1 pins
      dirPinForward(HIGH) {}

void DcMotor::setSpeed(int newSpeed) {
  targetSpeed = newSpeed;
  millisStartedMove = millis();
  // Since the last command was probably more than 5ms ago, should
  // immediately do the first decrement.
  doChecks();
}

void DcMotor::doChecks() {
  if ((millis() - millisStartedMove) > timeToWaitUntilStop &&
      currentSpeed != 0) {
    stop();
  } else if (targetSpeed != currentSpeed && (millis() - millisLastUpdate) > 5) {
    // BRIEF: this increments/decrements the speed on the motor if it's been at
    // least 5ms.

    int increment = targetSpeed > currentSpeed ? 1 : -1;
    currentSpeed += increment;
    analogWrite(pwmPin, abs(currentSpeed));  // Set speed
    // Set direction based on sign of speed
    digitalWrite(directionPin,
                 currentSpeed >= 0 ? dirPinForward : dirPinBackward);

    millisLastUpdate = millis();
  }
}

void DcMotor::stop() {
  targetSpeed = 0;
  millisStartedMove = millis();
  doChecks();
}
// TODO add estop

#endif
