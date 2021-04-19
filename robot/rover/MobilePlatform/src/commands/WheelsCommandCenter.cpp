//
// Created by Michael on 2021-01-23.
//

// Issue made to change includes to include, which requires changing the CMake

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

  // Create message to add to queue of messages
  Message* message = createMessage(commandID, rawArgsLength, rawArgs);

  /*
  Switch command recieved to perform specific function
  */
  switch(commandID)
  {
    case COMMAND_SET_MOTORS:
      this->enqueueSendMessage(message); // Add message to queue
      this->toggleMotors(*rawArgs);  // Toggle ON or OFF
      break;
    case COMMAND_STOP_MOTORS_EMERGENCY:
      this->enqueueSendMessage(message); // Add message to queue
      this->stopMotors();
      break;
    case COMMAND_CLOSE_MOTORS_LOOP:
      this->enqueueSendMessage(message); // Add message to queue
      this->closeMotorsLoop();
      break;
    case COMMAND_OPEN_MOTORS_LOOP:
      this->enqueueSendMessage(message); // Add message to queue
      this->openMotorsLoop(); 
      break;
    case COMMAND_SET_JOYSTICK:
      this->enqueueSendMessage(message); // Add message to queue
      this->toggleJoystick(*rawArgs); // Toggle ON or OFF
      break;
    case COMMAND_SET_GPS:
      this->enqueueSendMessage(message); // Add message to queue
      this->toggleGps(*rawArgs); // Toggle ON or OFF
      break;
    case COMMAND_SET_ENCODER:
      this->enqueueSendMessage(message); // Add message to queue
      this->toggleEncoder(*rawArgs); // Toggle ON or OFF
      break;
    case COMMAND_SET_ACCELERATION:
      this->enqueueSendMessage(message); // Add message to queue
      this->toggleAcceleration(*rawArgs); // Toggle ON or OFF
      break;
    case COMMAND_GET_ROVER_STATUS:
      this->enqueueSendMessage(message); // Add message to queue
      this->getRoverStatus();
      break;
    case COMMAND_MOVE_ROVER:
      this->enqueueSendMessage(message); // Add message to queue
      this->moveRover(*rawArgs, *(++rawArgs));
      break;
    case COMMAND_MOVE_WHEEL:
      this->enqueueSendMessage(message); // Add message to queue
      // Assign 16bit PWM value from 8bit register size
      uint8_t wheelNumber = (*rawArgs);
      int16_t PWM = 0;
      PWM = *(++rawArgs) << 8;
      PWM |= *(++rawArgs);
      this->moveWheel(wheelNumber, PWM);
      break;
    default:
        break;
  }
}