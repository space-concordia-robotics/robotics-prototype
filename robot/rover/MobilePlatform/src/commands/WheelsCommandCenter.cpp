//
// Created by Michael on 2021-01-23.
//

// Issue made to change includes to include, which requires changing the CMake

#include <cstdint>
#include "commands/WheelsCommandCenter.h"


using namespace internal_comms;

/*
   Command messages
   */

// this modifies the pointer
float bytes_to_float(const uint8_t* rawPointer)
{
    float f;
    byte bytes[] = {*rawPointer, *(++rawPointer), *(++rawPointer), *(++rawPointer)};
    memcpy(&f, &bytes, sizeof(f));
    return f;
}

void WheelsCommandCenter::executeCommand(const uint8_t cmdID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {
    // Create message to add to queue of messages
    //Message* message = createMessage(commandID, rawArgsLength, const_cast<uint8_t *>(rawArgs));
    /*
       Switch command recieved to perform specific function
       */

    int commandID = int(cmdID);

    switch(commandID)
    {
        case COMMAND_SET_MOTORS: {
                                     enableMotors(*rawArgs);
                                     break;
                                 }
        case COMMAND_STOP_MOTORS_EMERGENCY: {

                                                stopMotors();
                                                break;
                                            }
        case COMMAND_CLOSE_MOTORS_LOOP: {
                                            closeMotorsLoop();
                                            break;
                                        }
        case COMMAND_OPEN_MOTORS_LOOP: {
                                           openMotorsLoop();
                                           break;
                                       }
        case COMMAND_SET_JOYSTICK: {
                                       toggleJoystick(*rawArgs);
                                       break;
                                   }
        case COMMAND_SET_GPS: {
                                  toggleGps(*rawArgs);
                                  break;
                              }
        case COMMAND_SET_ENCODER: {
                                      toggleEncoder(*rawArgs);
                                      break;
                                  }
        case COMMAND_SET_ACCELERATION: {
                                           toggleAcceleration(*rawArgs);
                                           break;
                                       }
        case COMMAND_GET_ROVER_STATUS: {
                                           getRoverStatus();

                                           break;
                                       }
        case COMMAND_MOVE_ROVER: {
                                     const uint8_t throttle_dir = *(rawArgs++);
                                     const uint8_t throttle = *(rawArgs++);

                                     const uint8_t steering_dir = *(rawArgs++);
                                     const uint8_t steering = *(rawArgs++);


                                     moveRover(throttle_dir,throttle,steering_dir,steering);

                                     break;
                                 }
        case COMMAND_MOVE_WHEEL:
                                 {
                                     uint8_t wheelNumber = (*rawArgs++);
                                     uint8_t pwm_dir = (*rawArgs++);
                                     uint8_t pwm= (*rawArgs++);

                                      moveWheel(wheelNumber,pwm_dir,pwm);
                                     break;
                                 }

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
                                     //getMotorVelocity();
                                     break;
                                 }
        case COMMAND_GET_DESIRED_VELOCITY:
                                 {
                                     //        uint8_t wheelNumber = *(rawArgs++);
                                     //
                                     //        getMotorDesiredVelocity(wheelNumber);
                                     //        //getDesiredVelocity();
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
        case COMMAND_BLINK_TOGGLE: {
            uint8_t on = (*rawArgs++);
            handleBlink(on);
            break;
        }
        case COMMAND_BLINK_COLOR: {
            uint8_t r = (*rawArgs++);
            uint8_t g = (*rawArgs++);
            uint8_t b = (*rawArgs++);
            handleBlinkColor(r, g, b);
            break;
        }
        default:
                                 break;
    }
}


