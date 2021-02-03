#include "PDSCommandCenter.h"

#include <Arduino.h>
void pong();
void error();
void disableAllMotors();
void enableAllMotors();
void PDSCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength)
{
    Serial.println("inside executeCommand");
    switch(commandID)
    {
        case COMMAND_PING:
            Serial.println("PING");
            pong();
            break;
        case COMMAND_DISABLE_ALL_MOTORS:
            Serial.println("Disabling all motors");
            disableAllMotors();
            break;
        case COMMAND_ENABLE_ALL_MOTORS:
            Serial.println("Enabling all motors");
            enableAllMotors();
            break;
        default:
            error();

    }
}
