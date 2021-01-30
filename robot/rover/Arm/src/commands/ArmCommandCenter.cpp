//
// Created by cedric on 2021-01-23.
//

#include "include/commands/ArmCommandCenter.h"

#define COMMAND_EMERGENCY_STOP 0
#define COMMAND_REBOOT_TEENSY 1
#define COMMAND_STOP_MOTORS 2
#define COMMAND_RESET_ANGLES 3
#define COMMAND_HOME_MOTORS 4

void stopAllMotors();
void rebootTeensy();
void homeMotors();

void ArmCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {

  switch(commandID)
  {
    case COMMAND_STOP_MOTORS:
      stopAllMotors();
      break;
    case COMMAND_REBOOT_TEENSY:
      rebootTeensy();
      break;
    case COMMAND_HOME_MOTORS:
      homeMotors();
      break;
  }

}
