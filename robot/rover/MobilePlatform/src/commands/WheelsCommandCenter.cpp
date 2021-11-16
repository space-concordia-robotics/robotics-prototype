
#include <cstdint>
#include "commands/WheelsCommandCenter.h"


using namespace internal_comms;

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

        case COMMAND_STOP_MOTORS_EMERGENCY: {
            stopMotors();
            break;
        }
        case COMMAND_GET_ROVER_STATUS: {
            break;
        }
        case COMMAND_MOVE_ROVER: {

            float linear;
            byte throttle_bytes[4] = { *(rawArgs++),*(rawArgs++),*(rawArgs++),*(rawArgs++) };
            memcpy(&linear,&throttle_bytes,sizeof(linear));


            float angular;
            byte steering_bytes[4] = { *(rawArgs++),*(rawArgs++),*(rawArgs++),*(rawArgs++) };
            memcpy(&angular,&steering_bytes,sizeof(angular));

            moveRover(linear,angular);

            break;
        }
        case COMMAND_MOVE_WHEEL:{
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


