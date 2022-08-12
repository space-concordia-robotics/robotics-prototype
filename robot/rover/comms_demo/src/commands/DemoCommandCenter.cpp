//
// Created by cedric on 2020-10-17.
//

#include "include/commands/DemoCommandCenter.h"

using namespace internal_comms;

void DemoCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {
    //Serial.println(String(commandID));
    Message* message = (Message*) malloc(sizeof(Message));
    uint8_t arr[3] = {1,2,3};
    switch(commandID) {
        case 0:
            message->messageID = 20;
            message->rawArgsLength = 3;
            message->rawArgs = arr; 
            this->sendMessage(*message);
            break;
        case 1:
            Serial.println("command 1");
            break;
    }
}
