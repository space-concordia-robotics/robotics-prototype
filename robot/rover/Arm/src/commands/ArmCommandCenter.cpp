//
// Created by cedric on 2021-01-23.
//

#include "include/commands/ArmCommandCenter.h"

#define COMMAND_EMERGENCY_STOP 75
#define COMMAND_REBOOT_TEENSY 76
#define COMMAND_STOP_MOTORS 77
#define COMMAND_RESET_ANGLES 78
#define COMMAND_HOME_MOTORS 79
#define COMMAND_HOME 80
#define COMMAND_ARM_SPEED 81
#define COMMAND_STOP_SINGLE_MOTOR 82
#define COMMAND_GEAR_RATIO 83
#define COMMAND_OPEN_LOOP_GAIN 84
#define COMMAND_PID_CONSTANTS 85
#define COMMAND_MOTOR_SPEED 86
#define COMMAND_OPEN_LOOP_STATE 87
#define COMMAND_RESET_SINGLE_MOTOR 88
#define COMMAND_BUDGE_MOTORS 89
#define COMMAND_MOVE_MULTIPLE_MOTORS 90
#define COMMAND_PING 91
#define COMMAND_GET_MOTOR_ANGLES 92

void emergencyStop();
void rebootTeensy();
void stopAllMotors();
void resetAngles();
void homeAllMotors(uint8_t homingStyle);
void homeMotor(uint8_t motorId, uint8_t homingStyle);
void setArmSpeed(float armSpeedFactor);
void stopSingleMotor(uint8_t motorId);
void setGearRatioValue(uint8_t motorId, float gearRatio);
void setOpenLoopGain(uint8_t motorId, float gain);
void setPidConstants(uint8_t motorId, float kp, float ki, float kd);
void setMotorSpeed(uint8_t motorId, float speed);
void setOpenLoopState(uint8_t motorId, bool isOpenLoop);
void resetSingleMotor(uint8_t motorId);
void switchMotorDirection(uint8_t motorId);
void budgeMotors(const uint8_t motorActions[]);
void moveMultipleMotors(byte anglesToReach[]);
void pong();
void printMotorAngles(void);


// this modifies the pointer
float bytes_to_float(const uint8_t* rawPointer)
{
    float f;
    byte bytes[] = {*rawPointer, *(++rawPointer), *(++rawPointer), *(++rawPointer)};
    memcpy(&f, &bytes, sizeof(f));
    return f;
}

void ArmCommandCenter::executeCommand(const uint8_t cmdID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {

    int commandID = int(cmdID);

    switch(commandID)
    {
        case COMMAND_EMERGENCY_STOP: {
            emergencyStop();
            break;
                                     }
        case COMMAND_REBOOT_TEENSY: {
            rebootTeensy();
            break;
                                    }
        case COMMAND_STOP_MOTORS: {
            stopAllMotors();
            break;
                                  }
        case COMMAND_RESET_ANGLES: {
            resetAngles();
            break;
                                   }
        case COMMAND_HOME_MOTORS: {
            homeAllMotors(*rawArgs);
            break;
                                  }
        case COMMAND_HOME: {
            homeMotor(*rawArgs, *(++rawArgs));
            break;
                           }
        case COMMAND_ARM_SPEED: {
            setArmSpeed(bytes_to_float(rawArgs));
            break;
                                }
        case COMMAND_STOP_SINGLE_MOTOR:{
            stopSingleMotor(*rawArgs);
            break;
                                       }
        case COMMAND_GEAR_RATIO: {
            setGearRatioValue(*(rawArgs++), bytes_to_float(rawArgs));
            break;
                                 }
        case COMMAND_OPEN_LOOP_GAIN: {
            setOpenLoopGain(*(rawArgs++), bytes_to_float(rawArgs));
            break;
                                     }
        case COMMAND_PID_CONSTANTS: {
            setPidConstants(*(rawArgs++), bytes_to_float(rawArgs), bytes_to_float(rawArgs), bytes_to_float(rawArgs));
            break;
                                    }
        case COMMAND_OPEN_LOOP_STATE: {
            setOpenLoopState(*(rawArgs++), *(rawArgs) == 1);
            break;
                                      }
        case COMMAND_RESET_SINGLE_MOTOR: {
            resetSingleMotor(*rawArgs);
            break;
                                         }
        case COMMAND_BUDGE_MOTORS: {
            budgeMotors(rawArgs);
            break;
                                   }
        case COMMAND_MOVE_MULTIPLE_MOTORS: {
            moveMultipleMotors(rawArgs);
            break;
                                           }
        case COMMAND_PING: {
            pong();
            break;
                           }
        case COMMAND_GET_MOTOR_ANGLES: {
            printMotorAngles();
            break;
                                      }
    }
}
