//
// Created by cedric on 2020-10-10.
//

#include "include/Carousel.h"
#include "include/SciencePinSetup.h"
#include "include/HAL.h"
#include "include/commands/ScienceCommandCenter.h"
#include <stdlib.h>

#include <string>

extern internal_comms::CommandCenter* commandCenter;
void m_sendDebug(const char* message) {
  internal_comms::Message* msg = commandCenter->createMessage(0, strlen(message), (byte*)message);
  commandCenter->sendMessage(*msg);
}



void Carousel::setup() {
  // These setup functions MUST be here. Because the 
  // Carousel ctor is called outside of any function,
  // the add callback will be overwritten when
  // HAL's callback array is initialized.

  // calibration limit switch callback
  HAL::addLimitSwitchCallback(1, [](int buttonState, void *user_ptr){
    Carousel *c = (Carousel*) user_ptr;
    if (buttonState && c->state == CarouselState::Calibrating) {
      c->state = CarouselState::Not_Moving;
      c->currentCuvette = 0;
      HAL::servo(0, Carousel::stopped_speed);
    }
  }, this);

  startCalibrating();
}

void Carousel::update(unsigned long deltaMicroSeconds) {
  checkSwitch();
  if (isAutomating()) {
    handleAutomation();
  }

  if (state == CarouselState::Calibrating) {
    return;
  } else if (state == CarouselState::Moving_Carousel) {
    if (limitSwitchPulses >= cuvettesToMove) {
      HAL::servo(0, Carousel::stopped_speed);
      state = CarouselState::Not_Moving;
      cuvettesToMove = 0;
      limitSwitchPulses = 0;
    }
  }
}

void Carousel::moveNCuvettes(int cuvettesToMove) {
  if (cuvettesToMove == 0) {
    return;
  }
  if (state == CarouselState::Not_Moving) {
    limitSwitchPulses = 0;
    this->cuvettesToMove = abs(cuvettesToMove);
    currentCuvette += cuvettesToMove;
    currentCuvette %= NUM_CUVETTES;  // wrap around if needed
    state = CarouselState::Moving_Carousel;
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
    state = CarouselState::Not_Moving;
    currentCuvette = 0;
    HAL::servo(0, Carousel::stopped_speed);
  } else {
    state = CarouselState::Calibrating;
    HAL::servo(0, Carousel::cw_speed);
  }
}


void Carousel::startAutoTesting() {
  // TODO: what to do when moving
  if (!isMoving()) {
    int startIndex = numAutomationSteps * numberAutomated;
    
    if (currentCuvette == startIndex) {
      automationState = AutomationState::WaitingForSample;
      timeStopped = millis();
    } else {
      automationState = AutomationState::Advancing;
      timeStopped = 0;
      goToCuvette(startIndex);
    }
  }  
}

CarouselState Carousel::getState() const {
  return state;
}

bool Carousel::isMoving() const {
  switch (state) {
    case CarouselState::Calibrating:
    case CarouselState::Moving_Carousel:
      return true;
    case CarouselState::Not_Moving:
    case CarouselState::Uncalibrated:
      return false;
  }
}

int8_t Carousel::getCarouselIndex() const {
  if (state == CarouselState::Uncalibrated || state == CarouselState::Calibrating) {
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


void Carousel::handleAutomation() {
  // NOTE: this is only run when the carousel is not moving
  if (isAutomating() && !isMoving()) {
    int automationStep = currentCuvette % numAutomationSteps;
    if (timeStopped == 0) {
      timeStopped = millis();
    }

    switch (automationState) {
      case AutomationState::Advancing:
        // done advancing to next cuvette
        automationState = AutomationState::WaitingForSample;
        timeStopped = millis();
        break;

      case AutomationState::WaitingForSample:
        if (millis() - timeStopped >= delayTimes[automationStep]) {
          // once done waiting, spin carousel once
          automationState = AutomationState::SpinningCarousel;
          moveNCuvettes(NUM_CUVETTES);
        }
        break;

      case AutomationState::SpinningCarousel:
        // done spinning the carousel to mix, start waiting
        timeStopped = millis();
        automationState = AutomationState::WaitingForReaction;
        break;

      case AutomationState::WaitingForReaction:
        if (millis() - timeStopped >= reactionTimes[automationStep]) {
          // Now done waiting for reaction
          if (automationStep == numAutomationSteps - 1) {
            // If on last cuvette, stop
            automationState = AutomationState::NotAutomating;
            numberAutomated++;
          } else {
            // Otherwise move to next step
            automationState = AutomationState::Advancing;
            moveNCuvettes(1);
            timeStopped = 0;
          }
        }
        break;
    }
  }
}

bool Carousel::isAutomating() {
  return automationState != AutomationState::NotAutomating;
}

Carousel::Carousel(): currentCuvette(-1), state(CarouselState::Uncalibrated), limitSwitchPulses(0), cuvettesToMove(0),
                    timeStopped(0), numberAutomated(0), automationState(AutomationState::NotAutomating){
  lastDebounceTime = 0;
  lastButtonState = HAL::readLimitSwitch(0);
  buttonState = lastButtonState;
}

Carousel::~Carousel() {}
