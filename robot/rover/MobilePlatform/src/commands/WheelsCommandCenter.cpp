//
// Created by Michael on 2021-01-23.
//

//TODO: Fix CMake
//TODO: Fix PinSetup.h (not sure what i need to do ngl)
//TODO: Fix path from /includes to /include (And includes which use this path) and FIX CMAKE SO IT BUILDS!
//TODO: Fix CMakeLists.txt and CMakeTemplate.txt.in from /robot/rover

#include "../../includes/commands/WheelsCommandCenter.h" //Change to /include once the CMake is fixed

/*
Command messages
*/
#define COMMAND_SET_MOTORS 0
#define COMMAND_STOP_MOTORS_EMERGENCY 1
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
void toggleMotors(bool turnMotorOn);
void stopMotors();
void closeMotorsLoop();
void openMotorsLoop();
void toggleJoystick(bool turnJoystickOn);
void toggleGps(bool turnGpsOn);
void toggleEncoder(bool turnEncOn);
void toggleAcceleration(bool turnAccelOn);
void getRoverStatus();
void moveRover(int8_t roverThrottle, int8_t roverSteering); // Throttle -49 to 49 and Steering -49 to 49
void moveWheel(uint8_t wheelNumber, int16_t wheelPWM); // Wheel number 0 to 5 and -255 to 255 

void WheelsCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {
/*
Switch command recieved to perform specific function
*/
  switch(commandID)
  {
    case COMMAND_SET_MOTORS:
      toggleMotors(*rawArgs);  // Set if ON or OFF
      break;
    case COMMAND_STOP_MOTORS_EMERGENCY:
      stopMotors();
      break;
    case COMMAND_CLOSE_MOTORS_LOOP:
      closeMotorsLoop();
      break;
    case COMMAND_OPEN_MOTORS_LOOP:
      openMotorsLoop(); 
      break;
    case COMMAND_SET_JOYSTICK:
      toggleJoystick(*rawArgs); // Set if ON or OFF
      break;
    case COMMAND_SET_GPS:
      toggleGps(*rawArgs); // Set if ON or OFF
      break;
    case COMMAND_SET_ENCODER:
      toggleEncoder(*rawArgs); // Set if ON or OFF
      break;
    case COMMAND_SET_ACCELERATION:
      toggleAcceleration(*rawArgs); // Set if ON or OFF
      break;
    case COMMAND_GET_ROVER_STATUS:
      getRoverStatus();
      break;
    case COMMAND_MOVE_ROVER:
      moveRover(*rawArgs, *(++rawArgs));
      break;
    case COMMAND_MOVE_WHEEL:
      uint8_t wheelNumber = (*rawArgs);
      // Assign 16bit PWM value from 8bit register size
      int16_t PWM = 0; // 00000000 00000000 
      PWM = *(++rawArgs) << 8; // 00000000 11111111 -> 11111111 00000000
      PWM |= *(++rawArgs); // 11111111 11111111
      moveWheel(wheelNumber, PWM);
      break;
    default:
        break; //Added so the cmake compiles
  }

}