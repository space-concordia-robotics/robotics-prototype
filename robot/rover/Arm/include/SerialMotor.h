#ifndef SERIALMOTOR_H
#define SERIALMOTOR_H

#include <Arduino.h>

#include "LSSServoMotor.h"

class SerialMotor {
 public:
  /**
   * Standard ctor
   */
  SerialMotor(LSSServoMotor* motor, int motorID, float gearRatio);
  /**
   * Default constructor, to allow an array of this type to be created.
   */
  SerialMotor();

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
  LSSServoMotor* theMotor;
  float gearRatio;
  int motorID;
  int currentSpeed;
  /**
   * @brief Sets the amount of time (millis) to wait after starting a move
   * after which it should autostop.
   */
  static const unsigned int timeToWaitUntilStop = 175;
};

#endif
