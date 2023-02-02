//
// Created by cedric on 2021-01-23.
//

#ifndef ARM_ARMCOMMANDCENTER_H
#define ARM_ARMCOMMANDCENTER_H

#include "../../../internal_comms/include/CommandCenter.h"
#include "../PinSetup.h"

class ArmCommandCenter : public internal_comms::CommandCenter {
  void executeCommand(const uint8_t commandID, const uint8_t* rawArgs,
                      const uint8_t rawArgsLength) override;
  const char* getIdentifier() override;
};

#endif
