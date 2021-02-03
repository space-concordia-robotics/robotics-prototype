#include "PDSCommandCenter.h"

#include <Arduino.h>
void pong();
void error();
void disableAllMotors();
void enableAllMotors();
void motor(uint8_t, uint8_t);
void fan(uint8_t, uint8_t);
void PDSCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength)
{
    // What if command gets freed while inside function?
    Serial.print("inside executeCommand");
    if (commandID == COMMAND_PING)
    {
            Serial.println("PING");
            pong();
    }else if(commandID == COMMAND_DISABLE_ALL_MOTORS)
    {
            Serial.println("Disabling all motors");
            disableAllMotors();
    } else if (commandID == COMMAND_DISABLE_ALL_MOTORS)
    {
            Serial.println("Disabling all motors");
            disableAllMotors();
    }else if (commandID == COMMAND_ENABLE_ALL_MOTORS)
    {
            Serial.println("Enabling all motors");
            enableAllMotors();
    }else if (commandID == COMMAND_MOTOR)
    {
            if (rawArgsLength != 2) {
                Serial.println("Wrong number of arguments. It should be 2, motor pin + on/off ");
                return;
            }
            uint8_t motorPin = *rawArgs;
            uint8_t state = *(++rawArgs);
            motor(motorPin, state);
    }else if (commandID == COMMAND_FAN)
    {
            if (rawArgsLength != 2) {
                Serial.println("Wrong number of arguments. It should be 2, fan pin + speed in percent ");
                return;
            }
            uint8_t fanPin = *rawArgs;
            uint8_t fanSpeed = *(++rawArgs);
            fan(fanPin, fanSpeed);
    }else{
        error();
    } 
}
