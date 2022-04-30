//
// Created by cedric on 2020-10-17.
//

#include "include/commands/ScienceCommandCenter.h"

#include "../../include/SciencePinSetup.h"

#define COMMAND_MOVE_DEGREES 41
#define COMMAND_NEXT_CUVETTE 42
#define COMMAND_PREVIOUS_CUVETTE 43
#define COMMAND_MOVE_N_CUVETTES 44
#define SET_LASER 36

float bytes_to_float(const uint8_t* rawPointer) {
  float f;
  // I'm not going down the rabbit hole to figure out why the bytes in the float
  // are in the wrong order, but interpreting the bytes flipped like this works.
  byte bytes[] = {*(rawPointer + 3), *(rawPointer + 2), *(rawPointer + 1),
                  *(rawPointer)};
  memcpy(&f, &bytes, sizeof(f));
  return f;
}

// These declare method, in raman.ino, that are needed by the command center
void carousel_move_degrees(float degrees);
void carousel_next_cuvette();
void carousel_previous_cuvette();
void set_laser(int state);

void ScienceCommandCenter::executeCommand(const uint8_t commandID,
                                          const uint8_t* rawArgs,
                                          const uint8_t rawArgsLength) {
  switch (commandID) {
    case COMMAND_MOVE_DEGREES:
      if (rawArgsLength == 4) {
        float degrees = bytes_to_float(rawArgs);
        carousel_move_degrees(degrees);
      }
      break;
    case COMMAND_NEXT_CUVETTE:
      carousel_next_cuvette();
      break;
    case COMMAND_PREVIOUS_CUVETTE:
      carousel_previous_cuvette();
      break;
    case SET_LASER:
      set_laser(rawArgs[0]);
      break;
    default:
      Serial.printf("command id %d args length %d", commandID, rawArgsLength);
      digitalWrite(LED, !digitalRead(LED));
  }
}
