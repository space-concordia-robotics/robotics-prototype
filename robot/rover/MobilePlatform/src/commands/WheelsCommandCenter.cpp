//
// Created by Michael on 2021-01-23.
//

//TODO: Fix CMake

#include "../../includes/commands/WheelsCommandCenter.h" //Change to /include once the CMake is fixed

/*
Command messages
*/
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

/*
Command functions
*/
void setMotors();
void stopMotors();
void closeMotorsLoop();
void openMotorsLoop();
void setJoystick();
void setGps();
void setEncoder();
void setAcceleration();
void getRoverStatus();
void moveRover();
void moveWheel();

void WheelsCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {

/*
Switch command recieved to perform specific function
*/
  switch(commandID)
  {
    case COMMAND_SET_MOTORS: // 0
      setMotors();
      break;
    case COMMAND_MOTORS_EMERGENCY_STOP: // 1
      stopMotors();
      break;
    case COMMAND_CLOSE_MOTORS_LOOP: // 2
      closeMotorsLoop();
      break;
    case COMMAND_OPEN_MOTORS_LOOP: // 3
      openMotorsLoop();
      break;
    case COMMAND_SET_JOYSTICK: // 4
      setJoystick();
      break;
    case COMMAND_SET_GPS: // 5
      setGps();
      break;
    case COMMAND_SET_ENCODER: // 6
      setEncoder();
      break;
    case COMMAND_SET_ACCELERATION: // 7
      setAcceleration();
      break;
    case COMMAND_GET_ROVER_STATUS: // 8
      getRoverStatus();
      break;
    case COMMAND_MOVE_ROVER: // 9
      moveRover();
      break;
    case COMMAND_MOVE_WHEEL: // 10
      moveWheel();
      break;
    default:
        //Function which uses the arm to perform CBT of users choice
    
  }

}