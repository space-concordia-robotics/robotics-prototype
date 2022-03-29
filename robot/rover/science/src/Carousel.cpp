//
// Created by cedric on 2020-10-10.
//

#include "include/Carousel.h"

#include "include/SciencePinSetup.h"
#define NUMBER_OF_CUVETTES 8
#define DEGREES_PER_CUVETTE 360 / NUMBER_OF_CUVETTES

void Carousel::eStop() { theServo->writeActionCommand(servoID, "H"); }

void Carousel::update(unsigned long deltaMicroSeconds) {
  if (state == State::Calibrating) {
    if (!digitalRead(CAR_POS)) {
      // if read switch, stop motor and have it hold the current position.
      theServo->writeActionCommand(servoID, "H");
      state = State::Operating;
    }
  }
}

void Carousel::home() {}

void Carousel::goToCuvette(uint8_t cuvetteId) {}

void Carousel::moveByDegrees(float degrees) {
  if (state == State::Operating) {
    theServo->writeActionCommand(servoID, "MD", (int)degrees);
  }
}

void Carousel::nextCuvette() {
  if (state == State::Operating) {
    theServo->writeActionCommand(servoID, "MD", DEGREES_PER_CUVETTE);
    currentCuvette++;
    currentCuvette %= 8;  // wrap around if needed
  }
}

void Carousel::previousCuvette() {
  if (state == State::Operating) {
    theServo->writeActionCommand(servoID, "MD", DEGREES_PER_CUVETTE * 10);
    if (currentCuvette > 0) {
      currentCuvette--;
    } else {
      // wrap around if needed
      currentCuvette = 7;
    }
  }
}

void Carousel::startCalibrating() {
  state = State::Calibrating;
  // start moving at 10deg/s
  theServo->writeActionCommand(servoID, "WD", 100);
}

Carousel::Carousel(LSSServoMotor* theServo, uint8_t servoID)
    : theServo(theServo), servoID(servoID), currentCuvette(1) {
  state = State::Uncalibrated;
}

Carousel::~Carousel() {}
