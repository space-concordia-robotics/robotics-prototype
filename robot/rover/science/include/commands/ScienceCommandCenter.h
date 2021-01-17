//
// Created by cedric on 2020-10-17.
//

#ifndef ROVER_SCIENCECOMMANDCENTER_H
#define ROVER_SCIENCECOMMANDCENTER_H

#include "../../../comms/include/CommandCenter.h"

class ScienceCommandCenter : public internal_comms::CommandCenter {
    void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) override;
};


#endif //ROVER_SCIENCECOMMANDCENTER_H
