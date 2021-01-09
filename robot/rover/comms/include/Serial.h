#ifndef INTERNAL_COMMS_SERIAL_H
#define INTERNAL_COMMS_SERIAL_H

// Created by Cedric Martens on 2021-01-09
// Serial functions that are useful for the teensies

#include <Arduino.h>

namespace internal_comms {

    const int MAX_BYTE_ARRAY_LENGTH = 1024;

    void startSerial(uint8_t rxPin, uint8_t txPin, long baudRate)
    {
        pinMode(rxPin, INPUT);
        pinMode(txPin, OUTPUT); 

        Serial.begin(baudRate);
    }

    void readCommand()
    {
        uint8_t commandID = Serial.read();
        uint8_t bytesToRead = Serial.read();
        uint8_t buffer[MAX_BYTE_ARRAY_LENGTH];
        int bytesRead = Serial.readBytes(&buffer, MAX_BYTE_ARRAY_LENGTH);
    }

    void endSerial()
    {
        serial.end();
    }
}

#endif
