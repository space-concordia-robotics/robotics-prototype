//
// Created by cedric on 2020-10-14.
//

#include "include/Laser.h"
#include "include/SciencePinSetup.h"

void Laser::eStop() {
    
}

void Laser::update(unsigned long deltaMicroSeconds) {

}

void Laser::turnOn() {
    digitalWrite(LASER, HIGH);
}

void Laser::turnOff() {
    digitalWrite(LASER, LOW);
}

bool Laser::isReady() {
    return false;
}

unsigned long Laser::timeLeftForWarmUp() {
    return 0;
}

Laser::~Laser() {

}
