#include "PDSCommandCenter.h"

#include <Arduino.h>
void pong();
void error();
void who();
void PDSCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength)
{
    Serial.println("inside executeCommand");
    switch(commandID)
    {
        case COMMAND_PING:
            Serial.println("PING");
            pong();
            break;
        case COMMAND_WHO:
            Serial.println("WHO");
            who();
            break;
        default:
            error();

    }
}
