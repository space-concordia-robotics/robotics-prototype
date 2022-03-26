#ifndef SERIALMOTOR_H
#define SERIALMOTOR_H

#include <Arduino.h>

#include "../../internal_comms/include/LSSServoMotor.h"
#include "RobotMotor.h"

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
  static const unsigned int timeToWaitUntilStop = 1000;
};

SerialMotor::SerialMotor(LSSServoMotor* motor, int motorID, float gearRatio)
    : theMotor(motor),
      gearRatio(gearRatio),
      motorID(motorID),
      currentSpeed(0) {}

SerialMotor::SerialMotor()
    : theMotor(nullptr), gearRatio(1.0), motorID(0), currentSpeed(0) {}

void SerialMotor::setSpeed(int newSpeed) {
  // Set direction based on sign of speed
  theMotor->writeActionCommand(motorID, "WR", newSpeed);

  millisStartedMove = millis();
  currentSpeed = newSpeed;
}

void SerialMotor::doChecks() {
  if ((millis() - millisStartedMove) > timeToWaitUntilStop &&
      currentSpeed != 0) {
    stop();
  }
}

void SerialMotor::stop() {
  theMotor->writeActionCommand(motorID, "H");
  currentSpeed = 0;
}

#endif
