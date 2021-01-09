#ifndef INTERNAL_COMMS_SERIAL_H
#define INTERNAL_COMMS_SERIAL_H

// Created by Cedric Martens on 2021-01-09
// Serial functions that are useful for the teensies

#include <Arduino.h>
#include <CommandCenter.h>

namespace internal_comms {

    void startSerial(uint8_t rxPin, uint8_t txPin, long baudRate)
    {
        pinMode(rxPin, INPUT);
        pinMode(txPin, OUTPUT); 

        Serial.begin(baudRate);
    }

    void readCommand(CommandCenter* commandCenter)
    {
        Command* command = commandCenter->processCommand();
        uint8_t commandID = Serial.read();
        uint8_t bytesToRead = Serial.read();
        uint8_t* buffer = (uint8_t*) malloc(sizeof(uint8_t) * bytesRead);
        int bytesRead = Serial.readBytes(buffer, bytesRead);
        if(command->isValid)
        {
            commandCenter->executeCommand(command->commandID, command->bytesRead);
        }
        else
        {
            // Handle invalid command
        }
    }

    void endSerial()
    {
        serial.end();
    }
}

#endif
