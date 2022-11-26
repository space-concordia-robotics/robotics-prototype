
#include "commands/WheelsCommandCenter.h"


using namespace internal_comms;

float bytes_to_float(const uint8_t* rawPointer)
{
    float f;
    byte bytes[] = {*rawPointer, *(++rawPointer), *(++rawPointer), *(++rawPointer)};
    memcpy(&f, &bytes, sizeof(f));
    return f;
}
void blinkHere(int dur){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(dur);
    digitalWrite(LED_BUILTIN,LOW);
    delay(dur);
}
void WheelsCommandCenter::executeCommand(const uint8_t cmdID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {

    int commandID = int(cmdID);

    switch(commandID)
    {
        case COMMAND_STOP_MOTORS: {
            stopMotors();
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
            uint8_t speed = (*rawArgs);
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
            uint8_t angle = (*rawArgs);
            moveServo(servoID,angle);
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


