#ifndef INTERNAL_COMMS_SERIAL_H
#define INTERNAL_COMMS_SERIAL_H

// Created by Cedric Martens on 2021-01-09
// Serial functions that are useful for the teensies

#include <Arduino.h>
#include "CommandCenter.h"

#define COMMS_BAUDRATE 57600L

namespace internal_comms {

    void startSerial(uint8_t rxPin, uint8_t txPin);
    void readCommand(CommandCenter* commandCenter);
    void send(Message& message);
    void sendEmpty();
    void endSerial();

}

#endif
