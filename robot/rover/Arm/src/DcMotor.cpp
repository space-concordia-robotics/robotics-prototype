#include "include/DcMotor.h"

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
  if (abs(newSpeed) > 255) {
    newSpeed = 255;
  }
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
  } else if (targetSpeed != currentSpeed && (millis() - millisLastUpdate) > 2) {
    // BRIEF: this increments/decrements the speed on the motor if it's been at
    // least 2ms.

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
