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
  checkSwitch();

  if (state == State::Calibrating) {
    return;
  } else if (state == State::Moving_Carousel) {
    if (limitSwitchPulses >= cuvettesToMove) {
      HAL::servo(0, Carousel::stopped_speed);
      state = State::Not_Moving;
      cuvettesToMove = 0;
      limitSwitchPulses = 0;
    }
  }
}

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
  if (HAL::readLimitSwitch(1)) {
    // If the switch is already pressed, no need to calibrate
    state = State::Not_Moving;
    currentCuvette = 0;
    HAL::servo(0, Carousel::stopped_speed);
  } else {
    state = State::Calibrating;
    HAL::servo(0, Carousel::cw_speed);
  }
}

int8_t Carousel::getCarouselIndex() const {
  if (state == State::Uncalibrated || state == State::Calibrating) {
    return -1;
  } else {
    return currentCuvette;
  }
}

void Carousel::checkSwitch() {
  int reading = HAL::readLimitSwitch(0);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_THRESHOLD) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    if (reading != buttonState) {
      buttonState = reading;
      // count pulse when button goes to HIGH
      if (buttonState == HIGH) {
        limitSwitchPulses++;
      }
    }
  }

  lastButtonState = reading;
}

Carousel::Carousel(): currentCuvette(-1), state(State::Uncalibrated), limitSwitchPulses(0), cuvettesToMove(0) {
  lastDebounceTime = 0;
  lastButtonState = HAL::readLimitSwitch(0);
  buttonState = lastButtonState;

}

Carousel::~Carousel() {}
