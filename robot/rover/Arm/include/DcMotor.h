#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

#include "RobotMotor.h"

class DcMotor {
 public:
  int currentSpeed;
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

  unsigned int millisStartedMove;

 private:
  int directionPin;
  int pwmPin;
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

DcMotor::DcMotor(int dirPin, int pwmPin, float gearRatio, int dirPinForward)
    : currentSpeed(0),
      pwmPin(pwmPin),
      gearRatio(1.0),
      directionPin(dirPin),
      dirPinForward(dirPinForward) {}

DcMotor::DcMotor()
    : currentSpeed(0),
      pwmPin(9),  // default is to set to the M1 pins
      gearRatio(1.0),
      directionPin(7),  // default is to set to the M1 pins
      dirPinForward(HIGH) {}

void DcMotor::setSpeed(int newSpeed) {
  // Set direction based on sign of speed
  digitalWrite(directionPin, newSpeed >= 0 ? dirPinForward : !dirPinForward);
  analogWrite(pwmPin, abs(newSpeed));  // Set speed
  millisStartedMove = millis();
  currentSpeed = newSpeed;
}

void DcMotor::doChecks() {
  if (millis() - millisStartedMove && currentSpeed != 0) {
    stop();
  }
}

void DcMotor::stop() { analogWrite(pwmPin, 0); }

#endif
