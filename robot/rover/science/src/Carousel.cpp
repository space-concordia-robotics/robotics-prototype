//
// Created by cedric on 2020-10-10.
//

#include "include/Carousel.h"
#include "include/SciencePinSetup.h"
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
  servo.writeActionCommand(servoId, "O", angleOffset);
  // Initialize to 0
  servo.writeActionCommand(servoId, "D", 0);
}

void Carousel::update(unsigned long deltaMicroSeconds) {
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
    virtualAngle += cuvettesToMove * DEGREES_PER_CUVETTE;

    // wrap around if needed
    currentCuvette %= NUM_CUVETTES; 
    if (currentCuvette < 0) {
      currentCuvette += NUM_CUVETTES;
    }

    state = CarouselState::Moving_Carousel;
    // Set absolute position offset in case motor connected later
    servo.writeActionCommand(servoId, "O", angleOffset);
    // Move servo
    servo.writeActionCommand(servoId, "D", virtualAngle);
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
  if (servo.writeQueryCommand(servoId, "Q", queryBuffer, sizeof(queryBuffer))) {
    int code = atoi(&queryBuffer[3]);
    return code == 6 || code == 1;
  } else {
    return false;
  }
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


Carousel::Carousel(int servoId, int32_t angleOffset): currentCuvette(0), state(CarouselState::Not_Moving),
                    servoId(servoId), servo(&Serial5), virtualAngle(0), angleOffset(angleOffset) {
}

Carousel::~Carousel() {}
