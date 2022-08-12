#include "include/SerialMotor.h"

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