//
// Created by cedric on 2021-01-23.
//

#include "include/commands/ArmCommandCenter.h"

#define COMMAND_STOP_MOTORS 2

void stopAllMotors();

void ArmCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {

  switch(commandID)
  {
    case COMMAND_STOP_MOTORS:
      stopAllMotors();
      break;
  }

}
