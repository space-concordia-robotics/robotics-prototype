//
// Created by cedric on 2021-01-23.
//

#include "include/commands/ArmCommandCenter.h"

#define COMMAND_EMERGENCY_STOP 0
#define COMMAND_REBOOT_TEENSY 1
#define COMMAND_STOP_MOTORS 2
#define COMMAND_RESET_ANGLES 3
#define COMMAND_HOME_MOTORS 4
#define COMMAND_HOME 5
#define COMMAND_ARM_SPEED 6
#define COMMAND_STOP_SINGLE_MOTOR 7
#define COMMAND_GEAR_RATIO 8
#define COMMAND_OPEN_LOOP_GAIN 9
#define COMMAND_PID_CONSTANTS 10
#define COMMAND_MOTOR_SPEED 11
#define COMMAND_OPEN_LOOP_STATE 12
#define COMMAND_RESET_SINGLE_MOTOR 13
#define COMMAND_BUDGE_MOTORS 14
#define COMMAND_MOVE_MULTIPLE_MOTORS 15
#define COMMAND_PING 16
#define COMMAND_GET_MOTOR_ANGLES 17

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
void budgeMotors(uint8_t* motorsToMove, bool* moveCW);
void moveMultipleMotors(uint8_t* motorsToMove, float* anglesToReach);
void pong();
void printMotorAngles(void);

void ArmCommandCenter::executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) {

  switch(commandID)
  {
    case COMMAND_EMERGENCY_STOP:
      emergencyStop();
      break;
    case COMMAND_REBOOT_TEENSY:
      rebootTeensy();
      break;
    case COMMAND_STOP_MOTORS:
      stopAllMotors();
      break;
    case COMMAND_RESET_ANGLES:
      resetAngles();
      break;
    case COMMAND_HOME_MOTORS:
      homeAllMotors(*rawArgs);
      break;
    case COMMAND_HOME:
      homeMotor(*rawArgs, *(++rawArgs));
      break;
    case COMMAND_ARM_SPEED:
      setArmSpeed(1.0f);
      break;
    case COMMAND_STOP_SINGLE_MOTOR:
      stopSingleMotor(*rawArgs);
      break;
    case COMMAND_GEAR_RATIO:
      setGearRatioValue(*rawArgs, 1.0);
      break;
    case COMMAND_OPEN_LOOP_GAIN:
      setOpenLoopGain(*rawArgs, 1.0);
      break;
    case COMMAND_PID_CONSTANTS:
      uint8_t motorId = *rawArgs;
      rawArgs++;
      float pid[3] = {1.0f, 1.0f, 1.0f};
      setPidConstants(motorId, pid[0], pid[1], pid[2]);
      break;
    case COMMAND_OPEN_LOOP_STATE:
      setOpenLoopState(1, true);
      break;
    case COMMAND_RESET_SINGLE_MOTOR:
      resetSingleMotor(*rawArgs);
      break;
    case COMMAND_BUDGE_MOTORS:
      uint8_t motorsToMove[3] = {2, 4, 5};
      bool moveCW[3] = {true, false, true};
      budgeMotors(motorsToMove, moveCW);
      break;
    case COMMAND_MOVE_MULTIPLE_MOTORS:
      moveMultipleMotors(0, 0);
      break;
    case COMMAND_PING:
      pong();
      break;
    case COMMAND_GET_MOTOR_ANGLES:
      printMotorAngles();
      break;
  }

}
