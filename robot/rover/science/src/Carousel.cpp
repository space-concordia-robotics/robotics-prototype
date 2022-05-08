//
// Created by cedric on 2020-10-10.
//

#include "include/Carousel.h"

#include <stdlib.h>

#include <string>

#include "include/SciencePinSetup.h"
#define NUMBER_OF_CUVETTES 8
#define DEGREES_PER_CUVETTE 360 / NUMBER_OF_CUVETTES

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
  if (state == State::Moving_Carousel) {
    int currentPositionTenths = currentServoPosition();
    // if at the correct position where it should be at the cuvette
    if (currentPositionTenths - previousPositionTenths >= degreesCurrentMove) {
      if (digitalRead(CAR_POS) == LOW) {  // if switch pressed (note: pulled up)
        previousPositionTenths = currentPositionTenths;
        state = Not_Moving;
      } else {
        state = Correcting_Move_Pos;
        theServo->writeActionCommand(servoID, "WD", 90);
        timeCorrectionStarted = millis();
      }
    }
  }
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

char* startOfNumber(char* servoResponse, int capacity) {
  for (int i = 0; i < capacity; i++) {
    if (isdigit(servoResponse[i])) {
      return servoResponse + i;
    }
  }
}

int Carousel::currentServoPosition() {
  char* positionResponse = theServo->writeQueryCommand(servoID, "D");
  char* strNum = startOfNumber(positionResponse, 15);
  int positionTenths = atoi(strNum);
  free(positionResponse);
  return positionTenths;
}

void Carousel::home() {}

void Carousel::moveNCuvettes(int cuvettesToMove) {
  float degreesToMove = cuvettesToMove * DEGREES_PER_CUVETTE;
  moveByDegrees(degreesToMove);
}

void Carousel::goToCuvette(uint8_t cuvetteId) {
  if (cuvetteId == currentCuvette) {
    return;
  }
  int difference = (int)cuvetteId - (int)currentCuvette;
  int amount = difference % 4;
  int direction = difference > 4 ? -1 : 1;
  moveByDegrees(amount * direction);
}

void Carousel::moveByDegrees(float degrees) {
  if (state == State::Not_Moving) {
    theServo->writeActionCommand(servoID, "MD", (int)(degrees * 10));
    degreesCurrentMove = degrees;
    state = State::Moving_Carousel;
  }
}

void Carousel::nextCuvette() {
  if (state == State::Not_Moving) {
    moveByDegrees(DEGREES_PER_CUVETTE * 10);
    currentCuvette++;
    currentCuvette %= 8;  // wrap around if needed
  }
}

void Carousel::previousCuvette() {
  if (state == State::Not_Moving) {
    moveByDegrees(DEGREES_PER_CUVETTE * 10);
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
