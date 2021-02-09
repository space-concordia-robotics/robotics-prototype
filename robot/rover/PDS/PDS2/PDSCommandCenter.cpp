#include "PDSCommandCenter.h"

#include <Arduino.h>
void pong();
void error();
void disableAllMotors();
void enableAllMotors();
void motor(uint8_t, uint8_t);
void fan(uint8_t);
void PDSCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength)
{
    Serial.println("inside executeCommand");
    Serial.print("Command ID: ");
    Serial.println(commandID, DEC);
    Serial.print("Raw args: ");
    Serial.println((char*)rawArgs);
    Serial.print("Raw args length: ");
    Serial.println(rawArgsLength, DEC);
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
            if (rawArgsLength != 1) {
                Serial.println("Wrong number of arguments. It should only one, on/off");
                return;
            }
            uint8_t state = *rawArgs;
            fan(state);
    }else{
        error();
    } 
}
