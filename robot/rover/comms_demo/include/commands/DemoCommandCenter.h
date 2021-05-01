//
// Created by cedric on 2021-03-24.
//

#ifndef ROVER_DEMOCOMMANDCENTER_H
#define ROVER_DEMOCOMMANDCENTER_H

#include "../../../internal_comms/include/CommandCenter.h"

class DemoCommandCenter : public internal_comms::CommandCenter {
    void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) override;
};


#endif //ROVER_DEMOCOMMANDCENTER_H
