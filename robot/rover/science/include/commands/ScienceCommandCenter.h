//
// Created by cedric on 2020-10-17.
//

#ifndef ROVER_SCIENCECOMMANDCENTER_H
#define ROVER_SCIENCECOMMANDCENTER_H

#include "CommandCenter.h"

class ScienceCommandCenter : public CommandCenter{

    public:
        void executeCommand(const char *commandName, const char **args) override;
};


#endif //ROVER_SCIENCECOMMANDCENTER_H
