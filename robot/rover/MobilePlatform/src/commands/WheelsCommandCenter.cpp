//
// Created by Michael on 2021-01-23.
//

// Issue made to change includes to include, which requires changing the CMake

#include <cstdint>
#include "includes/commands/WheelsCommandCenter.h" //Change to /include once the CMake is fixed
// #include "../../../include/CommandCenter.h"

using namespace internal_comms;

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
#define COMMAND_GET_LINEAR_VELOCITY 11
#define COMMAND_GET_ROTATIONAL_VELOCITY 12
#define COMMAND_GET_CURRENT_VELOCITY 13
#define COMMAND_GET_DESIRED_VELOCITY 14
#define COMMAND_GET_BATTERY_VOLTAGE 15
#define COMMAND_WHEELS_PING 69

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

// Teensy to OBC value getters
void getLinearVelocity(void);
void getRotationalVelocity(void);
void getCurrentVelocity(void);
void getDesiredVelocity(void);
void getBatteryVoltage(void);
void pingWheels(void);

// this modifies the pointer
float bytes_to_float(const uint8_t* rawPointer)
{
  float f;
  byte bytes[] = {*rawPointer, *(++rawPointer), *(++rawPointer), *(++rawPointer)};
  memcpy(&f, &bytes, sizeof(f));
  return f;
}

void WheelsCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {

  // Create message to add to queue of messages
  Message* message = createMessage(commandID, rawArgsLength, (byte*)rawArgs);

  /*
  Switch command recieved to perform specific function
  */
  switch(commandID)
  {
    case COMMAND_SET_MOTORS:
      this->sendMessage(*message); // Add message to queue
      toggleMotors(*rawArgs);  // Toggle ON or OFF
      break;
    case COMMAND_STOP_MOTORS_EMERGENCY:
      this->sendMessage(*message); // Add message to queue
      stopMotors();
      break;
    case COMMAND_CLOSE_MOTORS_LOOP:
      this->sendMessage(*message); // Add message to queue
      closeMotorsLoop();
      break;
    case COMMAND_OPEN_MOTORS_LOOP:
      this->sendMessage(*message); // Add message to queue
      openMotorsLoop(); 
      break;
    case COMMAND_SET_JOYSTICK:
      this->sendMessage(*message); // Add message to queue
      toggleJoystick(*rawArgs); // Toggle ON or OFF
      break;
    case COMMAND_SET_GPS:
      this->sendMessage(*message); // Add message to queue
      toggleGps(*rawArgs); // Toggle ON or OFF
      break;
    case COMMAND_SET_ENCODER:
      this->sendMessage(*message); // Add message to queue
      toggleEncoder(*rawArgs); // Toggle ON or OFF
      break;
    case COMMAND_SET_ACCELERATION:
      this->sendMessage(*message); // Add message to queue
      toggleAcceleration(*rawArgs); // Toggle ON or OFF
      break;
    case COMMAND_GET_ROVER_STATUS:
      this->sendMessage(*message); // Add message to queue
      getRoverStatus();
      break;
    case COMMAND_MOVE_ROVER:
      this->sendMessage(*message); // Add message to queue
      moveRover(*rawArgs, *(++rawArgs));
      break;
    case COMMAND_MOVE_WHEEL:
    {
      sendMessage(*message); // Add message to queue
      // Assign 16bit PWM value from 8bit register size
      uint8_t wheelNumber = (*rawArgs);
      int16_t PWM = 0;
      PWM = *(++rawArgs) << 8;
      PWM |= *(++rawArgs);
      moveWheel(wheelNumber, PWM);
      break;
    }

    // Commands to acquire values from wheel teensy to OBC
    case COMMAND_GET_LINEAR_VELOCITY:
    {
      getLinearVelocity();
      break;
    }
    case COMMAND_GET_ROTATIONAL_VELOCITY:
    {
      getRotationalVelocity();
      break;
    }
    case COMMAND_GET_CURRENT_VELOCITY:
    {
      getCurrentVelocity();
      break;
    }
    case COMMAND_GET_DESIRED_VELOCITY:
    {
      getDesiredVelocity();
      break;
    }
    case COMMAND_GET_BATTERY_VOLTAGE:
    {
      getBatteryVoltage();
      break;
    }
    case COMMAND_WHEELS_PING:
    {
      pingWheels();
      break;
    }
    default:
        break;
  }
}