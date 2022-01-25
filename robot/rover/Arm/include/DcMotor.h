#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

#include "RobotMotor.h"

class DcMotor {
 public:
  int currentSpeed;
  float gearRatio;

  DcMotor(int dirPin, int pwmPin, int enablePin, float gearRatio);
  /**
   * @param newSpeed Set speed, from -255 to +255. Positive is 'forward'.
   */
  void setSpeed(int newSpeed);
  /**
   * Does the autostop, or in future budge and angle move checks.
   * Should be called frequently.
   */
  doChecks();
  stop();

  unsigned int millisStartedMove;

 private:
  int directionPin;
  int pwmPin;
  int enablePin;
  /**
   * @brief Which way to set the direction pin, HIGH or LOW,
   * when moving this motor 'forwards'
   */
  int dirPinForward;
  /**
   * @brief Sets the amount of time (millis) to wait after starting a move
   * after which it should autostop.
   */
  static const unsigned int timeToWaitUntilStop = 1000;
};

DcMotor::DcMotor(int dirPin, int pwmPin, int enablePin, float gearRatio,
                 int dirPinForward)
    : currentSpeed(0),
      gearRatio(1.0),
      directionPin(dirPin),
      pwmPin(pwmPin),
      enablePin(enablePin),
      dirPinForward(dirPinForward) {}

DcMotor::setSpeed(int newSpeed) {
  digitalWrite(enablePin, HIGH);
  // Set direction based on sign of speed
  digitalWrite(directionPin, newSpeed >= 0 ? dirPinForward : !dirPinForward);
  analogWrite(pwmPin, abs(newSpeed));  // Set speed
  millisStartedMove = millis();
  currentSpeed = newSpeed;
}

DcMotor::doChecks() {
  if (millis() - millisStartedMove && currentSpeed != 0) {
    stop();
  }
}

DcMotor::stop() {
  digitalWrite(enablePin, LOW);
  analogWrite(pwmPin, 0);
}

#endif
