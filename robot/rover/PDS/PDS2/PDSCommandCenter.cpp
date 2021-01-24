#include "PDSCommandCenter.h"

void PDSCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength)
{
    switch(commandID)
    {
        case COMMAND_PING:
            pong();
            break;
    }
}
