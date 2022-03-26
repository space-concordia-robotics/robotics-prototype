//
// Created by cedric on 2020-10-17.
//

#include "include/commands/ScienceCommandCenter.h"

#define COMMAND_MOVE_DEGREES 41

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

void ScienceCommandCenter::executeCommand(const uint8_t commandID,
                                          const uint8_t* rawArgs,
                                          const uint8_t rawArgsLength) {
  switch (commandID) {
    case COMMAND_MOVE_DEGREES:
      if (rawArgsLength == 4) {
        float degrees = bytes_to_float(rawArgs);
        carousel_move_degrees(degrees);
      }
  }
}
