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
  // Set absolute position offset
  servo.writeActionCommand(servoId, "O", 1800);
  // Initialize to 0
  servo.writeActionCommand(servoId, "D", 0);
}

void Carousel::update(unsigned long deltaMicroSeconds) {
  if (isAutomating()) {
    handleAutomation();
  }
  switch (state) {
    case CarouselState::Moving_Carousel:
      if (queryServoStopped()) {
        state = CarouselState::Not_Moving;
      }
      break;
  }
}

void Carousel::moveNCuvettes(int cuvettesToMove) {
  if (cuvettesToMove != 0 && state == CarouselState::Not_Moving) {
    currentCuvette += cuvettesToMove;
    currentCuvette %= NUM_CUVETTES;  // wrap around if needed
    state = CarouselState::Moving_Carousel;
    // Move servo to appropriate angle based on input
    servo.writeActionCommand(servoId, "MD", cuvettesToMove * DEGREES_PER_CUVETTE);
    // Wait to prevent motor from saying it is not moving immediately
    delay(10);
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
    case CarouselState::Moving_Carousel:
      return true;
    case CarouselState::Not_Moving:
      return false;
    default:
      return true;
  }
}

int8_t Carousel::getCarouselIndex() const {
  return currentCuvette;
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

Carousel::Carousel(int servoId): currentCuvette(0), state(CarouselState::Not_Moving),
                    automationState(AutomationState::NotAutomating), servoId(servoId),
                    servo(&Serial5) {
}

Carousel::~Carousel() {}
