//
// Created by cedric on 2020-10-14.
//

#include "include/CommandCenter.h"

namespace internal_comms
{
    Command* CommandCenter::processCommand(const uint8_t* rawCommand) const
    {
        Command* com = (Command*) malloc(sizeof(Command));
        com->isValid = false;
        if(rawCommand == nullptr)
            return com;

        com->commandID = rawCommand[0];
        com->isValid = true;
        return com;
    }
}
