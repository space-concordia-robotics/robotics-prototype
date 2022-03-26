//
// Created by cedric on 2020-10-10.
//

#include "include/Carousel.h"
#define NUMBER_OF_CUVETTES 8
#define DEGREES_PER_CUVETTE 360 / NUMBER_OF_CUVETTES

void Carousel::eStop() { theServo->writeActionCommand(servoID, "H"); }

void Carousel::update(unsigned long deltaMicroSeconds) {}

void Carousel::home() {}

void Carousel::goToCuvette(uint8_t cuvetteId) {}

void Carousel::moveByDegrees(float degrees) {
  theServo->writeActionCommand(servoID, "MD", (int)degrees);
}

void Carousel::nextCuvette() {
  theServo->writeActionCommand(servoID, "MD", DEGREES_PER_CUVETTE);
  currentCuvette++;
  currentCuvette %= 8;  // wrap around if needed
}

void Carousel::previousCuvette() {
  theServo->writeActionCommand(servoID, "MD", DEGREES_PER_CUVETTE);
  if (currentCuvette > 0) {
    currentCuvette--;
  } else {
    // wrap around if needed
    currentCuvette = 7;
  }
}

Carousel::Carousel(LSSServoMotor* theServo, uint8_t servoID)
    : theServo(theServo), servoID(servoID), currentCuvette(1) {}

Carousel::~Carousel() {}
