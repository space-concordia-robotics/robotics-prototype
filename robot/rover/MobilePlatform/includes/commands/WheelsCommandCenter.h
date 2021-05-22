//
// Created by Michael on 2021-01-23.
//

#ifndef WHEELS_WHEELSCOMMANDCENTER_H
#define WHEELS_WHEELSCOMMANDCENTER_H

#include "../../../internal_comms/include/CommandCenter.h"

class WheelsCommandCenter : public internal_comms::CommandCenter {
    void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) override;
};


#endif
