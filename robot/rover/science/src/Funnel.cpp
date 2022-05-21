//
// Created by cedric on 2020-10-14.
//

#include "include/Funnel.h"
#include "include/HAL.h"

Funnel::Funnel() : funnelTimeStarted(0), timeToFunnel(0), isMoving(false) {}

void Funnel::update(unsigned long) {
    if (millis() - funnelTimeStarted >= timeToFunnel) {
        stop();
    }
}

void Funnel::startFunnel() {
    // TODO check if 180 is correct speed
    HAL::servo(0, 180);
    isMoving = true;
}

void Funnel::runFor(float ms) {
    timeToFunnel = (long) ms;
    funnelTimeStarted = millis();
    startFunnel();
}

void Funnel::stop() {
    // TODO check if ID 0 is correct
    HAL::servo(0, 0);
    isMoving = false;
}

bool Funnel::moving() {
    return isMoving;
}