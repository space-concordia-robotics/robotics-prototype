
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
            uint8_t direction = (*rawArgs++);
            uint8_t speed = (*rawArgs++);
            moveWheel(wheelNumber,direction,speed);
            break;
        }
        case COMMAND_MOVE_WHEELS:{
            for(int i = 0 ; i < 6 ; i++){
                uint8_t direction = (*rawArgs++);
                uint8_t speed = (*rawArgs++);
                moveWheel(i,direction,speed);
            }
            break;
        }
        case COMMAND_MOVE_SERVO: {
            uint8_t servoID = (*rawArgs++);
            uint8_t angle = (*rawArgs++);
            moveServo(servoID,angle);
            break;
        }
        // Most of the command to get system parameters are in 'limbo' (waiting for encoders to work, and a proper handling of these things in the OBC)

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


