#include "include/SerialMotor.h"

SerialMotor::SerialMotor(LSSServoMotor* motor, int motorID, float gearRatio)
    : theMotor(motor),
      gearRatio(gearRatio),
      motorID(motorID),
      currentSpeed(0) {
      theMotor->writeActionCommand(motorID, "SD", 100);
      
      }

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

void SerialMotor::moveDegrees(int degrees) {
  static int counter = 0;
  if (millis() - millisStartedMove > 80) {
    counter = 0;
    millisStartedMove = millis();
    
    //theMotor->writeModifiedActionCommand(motorID, "D", degrees*450, "CH", 500);
    //theMotor->writeActionCommand(motorID, "MD", degrees*100);
    
    theMotor->writeModifiedActionCommand(motorID, "MD", degrees*100, "SD", 500);
    
  } else {
    counter++;
  }  
  
  /*if (counter >= 3) {
    moveOpenedClosed(degrees);
  }*/
}

void SerialMotor::moveOpenedClosed(int degrees) {
  if (degrees > 0) {
    theMotor->writeModifiedActionCommand(motorID, "D", 1800, "CH", 400); 
  }
  if (degrees < 0) {
    theMotor->writeModifiedActionCommand(motorID, "D", -1800, "CH", 400);
  }
}

