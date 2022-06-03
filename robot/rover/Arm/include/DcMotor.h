#ifndef DCMOTOR_H
#define DCMOTOR_H

#include <Arduino.h>

class DcMotor {
 public:
  // both these are positive or negative based on direction
  int currentSpeed, targetSpeed;
  float gearRatio;
  unsigned int millisStartedMove, millisLastUpdate;

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
  static const unsigned int timeToWaitUntilStop = 100;
};

#endif
