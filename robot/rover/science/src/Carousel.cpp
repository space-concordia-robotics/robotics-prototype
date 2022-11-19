//
// Created by cedric on 2020-10-10.
//

#include "include/Carousel.h"
#include "include/SciencePinSetup.h"
#include "include/HAL.h"

#include <stdlib.h>

#include <string>

#define NUMBER_OF_CUVETTES 8

// The constructor initializes this
Carousel *Carousel::instance = nullptr;

void Carousel::setup() {
  // setup callback to count the number of sw pulses 
  HAL::addLimitSwitchCallback(0, [](int state){
    if (state) {
      Carousel::instance->limitSwitchPulses++;
    }
  });
}

void Carousel::update(unsigned long deltaMicroSeconds) {
  /*
  if (state == State::Calibrating) {
    if (digitalRead(CAR_POS) == LOW) {  // pulled up
      // if switch pressed, stop motor and have it hold the current position.
      theServo->writeActionCommand(servoID, "H");
      previousPositionTenths = currentServoPosition();
      state = State::Not_Moving;
    }
  }
  */
  if (state == State::Moving_Carousel) {
    if (limitSwitchPulses >= cuvettesToMove) {
      HAL::servo(0, 142);
      state = State::Not_Moving;
      cuvettesToMove = 0;
      limitSwitchPulses = 0;
    }
  }
  /*
  if (state == State::Correcting_Move_Pos) {
    if (digitalRead(CAR_POS) == LOW) {  // switch pressed (pulled up)
      theServo->writeActionCommand(servoID, "H");
      state = State::Not_Moving;
    } else if (millis() - timeCorrectionStarted > CORRECTION_MAX) {
      theServo->writeActionCommand(servoID, "WD", -90);
      timeCorrectionStarted = millis();
      state = State::Correcting_Move_Neg;
    }
  }
  if (state == State::Correcting_Move_Neg) {
    if (digitalRead(CAR_POS) == LOW) {  // switch pressed (pulled up)
      theServo->writeActionCommand(servoID, "H");
      state = State::Not_Moving;
    } else if (millis() - timeCorrectionStarted > CORRECTION_MAX) {
      // if reached max time, give up.
      theServo->writeActionCommand(servoID, "H");
      state = State::Not_Moving;
    }
  }
  */
}

void Carousel::home() {}

void Carousel::moveNCuvettes(int cuvettesToMove) {
  /*float degreesToMove = cuvettesToMove * DEGREES_PER_CUVETTE;
  moveByDegrees(degreesToMove);*/
}

void Carousel::goToCuvette(uint8_t cuvetteId) {
  /*if (cuvetteId == currentCuvette) {
    return;
  }
  int difference = (int)cuvetteId - (int)currentCuvette;
  int amount = difference % 4;
  int direction = difference > 4 ? -1 : 1;
  moveByDegrees(amount * direction);*/
}

void Carousel::nextCuvette() {
  if (state == State::Not_Moving) {
    limitSwitchPulses = 0;
    cuvettesToMove = 1;
    HAL::servo(0, 180);
    currentCuvette++;
    currentCuvette %= 8;  // wrap around if needed
    state = State::Moving_Carousel;
  }
}

void Carousel::previousCuvette() {
  /*if (state == State::Not_Moving) {
    moveByDegrees(DEGREES_PER_CUVETTE * 10);
    if (currentCuvette > 0) {
      currentCuvette--;
    } else {
      // wrap around if needed
      currentCuvette = 7;
    }
  }*/
}

void Carousel::startCalibrating() {
  /*state = State::Calibrating;
  // start moving at 10deg/s
  theServo->writeActionCommand(servoID, "WD", 100);*/
}

Carousel::Carousel(): currentCuvette(-1) {
  Carousel::instance = this;
  HAL::servo(0, 140);
  // for now, skip calibration
  Carousel::instance->state = State::Not_Moving;
}

Carousel::~Carousel() {}
