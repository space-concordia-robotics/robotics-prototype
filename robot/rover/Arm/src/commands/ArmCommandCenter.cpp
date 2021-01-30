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

void emergencyStop();
void rebootTeensy();
void stopAllMotors();
void resetAngles();
void homeAllMotors();
void homeCommand();
void setArmSpeed(float armSpeedFactor);
void stopSingleMotor(int motorId);
void setGearRatioValue(int motorId, float gearRatio);
void setOpenLoopGain(int motorId, float gain);
void setPidConstants(int motorId, float kp, float ki, float kd);

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
      homeMotors();
      break;
    case COMMAND_HOME:
      homeCommand();
      break;
    case COMMAND_ARM_SPEED:
      setArmSpeed(1.0f);
      break;
    case COMMAND_STOP_SINGLE_MOTOR:
      stopSingleMotor(1);
      break;
    case COMMAND_GEAR_RATIO:
      setGearRatioValue(1, 1.0);
      break;
    case COMMAND_OPEN_LOOP_GAIN:
      setOpenLoopGain(1, 1.0);
      break;
    case COMMAND_PID_CONSTANTS:
      setPidConstants(1, 1.0, 1.0, 1.0);
      break;
  }

}
