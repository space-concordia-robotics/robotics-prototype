//
// Created by cedric on 2020-10-10.
//

#include "include/Carousel.h"
#include "include/SciencePinSetup.h"
#include "include/HAL.h"

#include <stdlib.h>

#include <string>



void Carousel::setup() {
  // These setup functions MUST be here, since the 
  // Carousel ctor is called outside of any function,
  // the add callback will be overwritten when
  // HAL's callback array is initialized.

  startCalibrating();
  
  // setup callback to count the number of sw pulses 
  HAL::addLimitSwitchCallback(0, [](int buttonState, void *user_ptr){
    Carousel *c = (Carousel*) user_ptr;
    // This checks debouncing and that the state is high
    if (buttonState && (millis() - (c->btn0LastPulse)) > DEBOUNCE_THRESHOLD) {
      c->limitSwitchPulses++;
      c->btn0LastPulse = millis();
    }
  }, this);

  // setup callback for the index limit switch
  // for calibration
  HAL::addLimitSwitchCallback(1, [](int buttonState, void *user_ptr){
    Carousel *c = (Carousel*) user_ptr;
    if (buttonState && c->state == State::Calibrating) {
      c->state = State::Not_Moving;
      c->currentCuvette = 0;
      HAL::servo(0, Carousel::stopped_speed);
    }
  }, this);
}

void Carousel::update(unsigned long deltaMicroSeconds) {
  if (state == State::Calibrating) {
    return;
  }
  
  if (state == State::Moving_Carousel) {
    if (limitSwitchPulses >= cuvettesToMove) {
      HAL::servo(0, Carousel::stopped_speed);
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
  if (cuvettesToMove == 0) {
    return;
  }
  if (state == State::Not_Moving) {
    limitSwitchPulses = 0;
    this->cuvettesToMove = abs(cuvettesToMove);
    currentCuvette += cuvettesToMove;
    currentCuvette %= NUM_CUVETTES;  // wrap around if needed
    state = State::Moving_Carousel;
    // Move servo in appropriate direction based on input
    if (cuvettesToMove > 0) {
      HAL::servo(0, Carousel::cw_speed);
    } else {
      HAL::servo(0, Carousel::ccw_speed);
    }
  }
}

void Carousel::goToCuvette(uint8_t cuvetteId) {
  if (cuvetteId == currentCuvette) {
    return;
  }
  int difference = (int)cuvetteId - (int)currentCuvette;
  moveNCuvettes(difference % NUM_CUVETTES);
}

void Carousel::nextCuvette() {
  moveNCuvettes(1);
}

void Carousel::previousCuvette() {
  moveNCuvettes(-1);
}

void Carousel::startCalibrating() {
  state = State::Calibrating;
  // start moving clockwise
  HAL::servo(0, Carousel::cw_speed);
}

int8_t Carousel::getCarouselIndex() const {
  if (state == State::Uncalibrated || state == State::Calibrating) {
    return -1;
  } else {
    return currentCuvette;
  }
}

Carousel::Carousel(): currentCuvette(-1), btn0LastPulse(0) {
}

Carousel::~Carousel() {}
