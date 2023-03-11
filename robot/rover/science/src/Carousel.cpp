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
      c->servo.writeActionCommand(c->servoId, "H");
    }
  }, this);

  // limit switch callback (when correcting)
  HAL::addLimitSwitchCallback(0, [](int buttonState, void *user_ptr){
    Carousel *c = (Carousel*) user_ptr;
    c->limitSwitchHit = true;
  }, this);

  startCalibrating();
}

void Carousel::update(unsigned long deltaMicroSeconds) {
  if (isAutomating()) {
    handleAutomation();
  }
  switch (state) {
    case CarouselState::Calibrating:
      return;

    case CarouselState::Moving_Carousel:
      if (queryServoStopped()) {
        if (HAL::readLimitSwitch(0)) {
          state = CarouselState::Not_Moving;
        } else {
          state = CarouselState::Correcting_cw;
          servo.writeActionCommand(servoId, "WD", calibration_speed);
          limitSwitchHit = false;
          timeStopped = millis();
        }
      }
      break;

      case CarouselState::Correcting_cw:
        if (limitSwitchHit) {
          state = CarouselState::Not_Moving;
        } else if (millis() - timeStopped > MAX_CORRECT_TIME) {
          state = CarouselState::Correcting_ccw;
          servo.writeActionCommand(servoId, "WD", -calibration_speed);
        }
        break;

      case CarouselState::Correcting_ccw:
        if (limitSwitchHit) {
          state = CarouselState::Not_Moving;
        } else if (millis() - timeStopped > MAX_CORRECT_TIME) {
          state = CarouselState::Correcting_cw;
          servo.writeActionCommand(servoId, "WD", calibration_speed);
        }
        break;
  }
}

void Carousel::moveNCuvettes(int cuvettesToMove) {
  if (cuvettesToMove != 0 && state == CarouselState::Not_Moving) {
    currentCuvette += cuvettesToMove;
    currentCuvette %= NUM_CUVETTES;  // wrap around if needed
    state = CarouselState::Moving_Carousel;
    // Move servo in appropriate direction based on input
    servo.writeActionCommand(servoId, "MD", cuvettesToMove * DEGREES_PER_CUVETTE);
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
  } else {
    state = CarouselState::Calibrating;
    servo.writeActionCommand(servoId, "WD", calibration_speed);
  }
}


CarouselState Carousel::getState() const {
  return state;
}

bool Carousel::queryServoStopped() {
  char* response = servo.writeQueryCommand(servoId, "Q");
  int code = atoi(&response[3]);
  return code == 6 || code == 1;
}

bool Carousel::isMoving() const {
  switch (state) {
    case CarouselState::Calibrating:
    case CarouselState::Moving_Carousel:
    case CarouselState::Correcting_cw:
    case CarouselState::Correcting_ccw:
      return true;
    case CarouselState::Not_Moving:
    case CarouselState::Uncalibrated:
      return false;
    default:
      return true;
  }
}

int8_t Carousel::getCarouselIndex() const {
  if (state == CarouselState::Uncalibrated || state == CarouselState::Calibrating) {
    return -1;
  } else {
    return currentCuvette;
  }
}

void Carousel::spinMix() {
  moveNCuvettes(NUM_CUVETTES);
}


void Carousel::startAutoTesting() {
  // TODO: what to do when moving
  if (!isMoving()) {
    automationState = AutomationState::SpinningCarousel;
    //moveNCuvettes(NUM_CUVETTES, Carousel::cw_spin_speed, Carousel::ccw_spin_speed);
  }
}

void Carousel::handleAutomation() {
  // NOTE: this is only run when the carousel is not moving
  if (isAutomating() && !isMoving()) {

    switch (automationState) {
      case AutomationState::SpinningCarousel:
        // done spinning the carousel to mix, advance to next test tube for camera
        automationState = AutomationState::Advancing;
        moveNCuvettes(1);
        break;

      case AutomationState::Advancing:
        // Test tube in front of camera, stop
        automationState = AutomationState::NotAutomating;
        break;

    }
  }
}

bool Carousel::isAutomating() {
  return automationState != AutomationState::NotAutomating;
}

Carousel::Carousel(int servoId): currentCuvette(-1), state(CarouselState::Uncalibrated),
                    automationState(AutomationState::NotAutomating), servoId(servoId),
                    limitSwitchHit(false), servo(&Serial5) {
}

Carousel::~Carousel() {}
