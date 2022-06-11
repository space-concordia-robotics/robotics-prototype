//
// Created by Marc on 2021-9-25.
//

#include "include/commands/ArmCommandCenter.h"

#include "Arduino.h"
#include "include/PinSetup.h"

#define COMMAND_PING 75
#define COMMAND_MOVE_BY 76
// #define SEND_MOTOR_ANGLES 77
#define SET_MOTOR_SPEEDS 78
#define COMMAND_DEBUG_TEST 79
#define COMMAND_GET_POWER 80

// Commands that expect a pointer are assumed to provide a pointer
// to 6 values.

void invalidCommand(const uint8_t cmdID, const uint8_t* rawArgs,
                    const uint8_t rawArgsLength);
void pong();
void sendMotorAngles();
void moveMotorsBy(float* angles, uint16_t numAngles);
void debug_test();

/**
 * @brief Sets motor speeds.
 * @param angles pointer to NUM_MOTOR floats.
 */
void setMotorSpeeds(float* angles);

float bytes_to_float(const uint8_t* rawPointer) {
  float f;
  // I'm not going down the rabbit hole to figure out why the bytes in the float
  // are in the wrong order, but interpreting the bytes flipped like this works.
  byte bytes[] = {*(rawPointer + 3), *(rawPointer + 2), *(rawPointer + 1),
                  *(rawPointer)};
  memcpy(&f, &bytes, sizeof(f));
  return f;
}

void ArmCommandCenter::executeCommand(const uint8_t cmdID,
                                      const uint8_t* rawArgs,
                                      const uint8_t rawArgsLength) {
  int commandID = int(cmdID);

  switch (commandID) {
    case COMMAND_PING: {
      pong();
      break;
    }
    case SET_MOTOR_SPEEDS: {
      uint16_t numAngles = rawArgsLength / sizeof(float);
      if (numAngles == NUM_MOTORS) {
        float* angles = (float*)malloc(sizeof(*angles) * numAngles);

        for (int i = 0; i < numAngles; i++) {
          angles[i] = bytes_to_float(rawArgs + i * sizeof(float));
        }

        setMotorSpeeds(angles);
        free(angles);
        break;
      } else {
        invalidCommand(cmdID, rawArgs, rawArgsLength);
        break;
      }
    }
    case COMMAND_DEBUG_TEST:
      debug_test();
      break;
    case COMMAND_GET_POWER:
      getServoPower();
      break;
    default: {
      invalidCommand(cmdID, rawArgs, rawArgsLength);
    }
  }
}
