#ifndef INTERNAL_COMMS_SERIAL_H
#define INTERNAL_COMMS_SERIAL_H

// Created by Cedric Martens on 2021-01-09
// Serial functions that are useful for the teensies

#include <Arduino.h>
#include "CommandCenter.h"

namespace internal_comms {

    void startSerial(uint8_t rxPin, uint8_t txPin, long baudRate);
    void readCommand(CommandCenter* commandCenter);
    void sendCommand(Command* command);
    void endSerial();

}

#endif
