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
  // Initialize to starting point
  servo.writeActionCommand(servoId, "D", angleOffset);
}

void Carousel::update(unsigned long deltaMicroSeconds) {
  switch (state) {
    case CarouselState::Moving_Carousel:
      if (queryServoStopped()) {
        state = CarouselState::Not_Moving;
      }
      break;

    case CarouselState::Spin_Mix_Negative:
      if (queryServoStopped()) {
        servo.writeActionCommand(servoId, "MD", 3600);
        state = CarouselState::Spin_Mix_Positive;
        delay(10);
      }
      break;

    case CarouselState::Spin_Mix_Positive:
      if (queryServoStopped()) {
        delay(100);
        // move back where it's supposed to be
        servo.writeActionCommand(servoId, "D", virtualAngle);
        delay(10);

        state = CarouselState::Moving_Carousel;
      }
      break;
    
  }
}

void Carousel::moveNTestTubes(int testTubesToMove) {
  if (state == CarouselState::Not_Moving) {
    currentTestTube += testTubesToMove;

    virtualAngle = (currentTestTube * DEGREES_PER_TEST_TUBE) + angleOffset;

    // wrap around if needed
    currentTestTube %= NUM_TEST_TUBES; 
    if (currentTestTube < 0) {
      currentTestTube += NUM_TEST_TUBES;
    }

    state = CarouselState::Moving_Carousel;
    // Set absolute position offset in case motor connected later
    // Move servo to position
    servo.writeActionCommand(servoId, "D", virtualAngle);
    // Wait to prevent motor from saying it is not moving immediately
    delay(10);
  }
}

void Carousel::goToTestTube(uint8_t testTubeId) {
  int difference = (int)testTubeId - (int)currentTestTube;
  moveNTestTubes(difference % NUM_TEST_TUBES);
}

void Carousel::nextTestTube() {
  moveNTestTubes(1);
}

void Carousel::previousTestTube() {
  moveNTestTubes(-1);
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
    case CarouselState::Spin_Mix_Negative:
    case CarouselState::Spin_Mix_Positive:
      return true;
    case CarouselState::Not_Moving:
      return false;
    default:
      return true;
  }
}

int8_t Carousel::getCarouselIndex() const {
  return currentTestTube;
}

void Carousel::spinMix() {
  // the code which sets each test tube had to be redone, so this is weird
  // it moves it relative 360 in the positive direction, then back the other way
  servo.writeActionCommand(servoId, "MD", -3600);
  state = CarouselState::Spin_Mix_Negative;
  delay(10); // prevent it from showing not moving
}

void Carousel::estop() {
  servo.writeActionCommand(servoId, "L");
  state = CarouselState::Not_Moving;
}

// Does not change virtualAngle so that nextTestTube etc will behave as 
// if this was never called, ie will go to the correct absolute position.
void Carousel::setServoAngle(float angle) {
  virtualAngle = (int32_t) (angle * 10);
  // Move servo to position and ensure starts moving so it doesn't report stopped
  servo.writeActionCommand(servoId, "D", virtualAngle);
  delay(10);
}


Carousel::Carousel(int servoId, int32_t angleOffset): currentTestTube(0), state(CarouselState::Not_Moving),
                    servoId(servoId), servo(&SMART_SERVO_SERIAL), virtualAngle(0), angleOffset(angleOffset) {
}

Carousel::~Carousel() {}
