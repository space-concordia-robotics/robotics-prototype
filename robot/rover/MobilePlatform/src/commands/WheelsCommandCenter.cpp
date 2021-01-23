//
// Created by Michael on 2021-01-23.
//

//TODO: Fix CMake

#include "../../includes/commands/WheelsCommandCenter.h" //Change to /include once the CMake is fixed

#define COMMAND_SET_MOTORS 0
#define COMMAND_MOTORS_EMERGENCY_STOP 1
#define COMMAND_CLOSE_MOTORS_LOOP 2
#define COMMAND_OPEN_MOTORS_LOOP 3
#define COMMAND_SET_JOYSTICK 4
#define COMMAND_SET_GPS 5
#define COMMAND_SET_ENCODER 6
#define COMMAND_SET_ACCELERATION 7
#define COMMAND_GET_ROVER_STATUS 8
#define COMMAND_MOVE_ROVER 9
#define COMMAND_MOVE_WHEEL 10

#define COMMAND_STOP_MOTORS 2
#define COMMAND_RESET_ANGLES 3

void stopAllMotors();
void rebootTeensy();

void WheelsCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {

  switch(commandID)
  {
    case COMMAND_STOP_MOTORS:
      stopAllMotors();
      break;
    case COMMAND_REBOOT_TEENSY:
      rebootTeensy();
      break;
  }

}