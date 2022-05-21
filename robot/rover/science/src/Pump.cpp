//
// Created by cedric on 2020-10-14.
//

#include "include/Pump.h"

#include "Arduino.h"
#include "include/HAL.h"

Pump::Pump(): isMoving(false), pumpTimeStarted(0), timeToPump(0) {}

void Pump::pump() {
  HAL::pump(255, 1);
  isMoving = true;
}

void Pump::backpump() {
  HAL::pump(255, 0);
  isMoving = true;
}

void Pump::stop() {
  HAL::pump(0, 0);
  isMoving = false;
}

void Pump::pumpFor(float ms) {
  timeToPump = (long)ms;
  pumpTimeStarted = millis();
  pump();
}

void Pump::backpumpFor(float ms) {
  timeToPump = (long)ms;
  pumpTimeStarted = millis();
  backpump();
}

void Pump::update(unsigned long deltaMicroSeconds) {
  if (millis() - pumpTimeStarted >= timeToPump) {
    stop();
  }
}

bool Pump::moving() { return isMoving; }
