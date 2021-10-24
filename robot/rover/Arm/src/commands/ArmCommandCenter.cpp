//
// Created by Marc on 2021-9-25.
//

#include "include/commands/ArmCommandCenter.h"

#include "Arduino.h"

#define COMMAND_PING 75
#define COMMAND_MOVE_BY 76
#define SEND_MOTOR_ANGLES 77

// Commands that expect a pointer are assumed to provide a pointer
// to 6 values.

void invalidCommand();
void pong();
void sendMotorAngles();
void moveMotorsBy(float* angles, uint16_t numAngles);

// this modifies the pointer
float bytes_to_float(const uint8_t* rawPointer) {
  float f;
  byte bytes[] = {*rawPointer, *(++rawPointer), *(++rawPointer),
                  *(++rawPointer)};
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
    case COMMAND_MOVE_BY: {
      uint16_t numAngles = rawArgsLength / 4;
      if (numAngles == 6) {
        float* angles = (float*)malloc(sizeof(*angles) * numAngles);

        for (int i = 0; i < numAngles; i++) {
          angles[i] = bytes_to_float(rawArgs + i * sizeof(float));
        }

        moveMotorsBy(angles, numAngles);
        free(angles);
        break;
      } else {
        invalidCommand();
        /*Serial.write("wrong number of angles: ");
        Serial.write(String(numAngles).c_str());
        Serial.write("\n");*/
        break;
      }
    }
    case SEND_MOTOR_ANGLES: {
      sendMotorAngles();
      break;
    }
    default: {
      invalidCommand();

      /*Serial.write("invalid command id ");
      char buffer[8];
      snprintf(buffer, 8, "%d\n", commandID);
      Serial.write(buffer);*/
    }
  }
}
