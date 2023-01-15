//
// Created by cedric on 2020-10-17.
//

#include "include/commands/ScienceCommandCenter.h"
#include "../../../internal_comms/include/CommandCenter.h"
#include "../../include/SciencePinSetup.h"
#include "include/HAL.h"

#define COMMAND_ESTOP 25

#define COMMAND_GET_CAROUSEL_INDEX 40
#define COMMAND_NEXT_TEST_TUBE 42
#define COMMAND_GO_TO_TEST_TUBE 43
#define COMMAND_START_CALIBRATING 44

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
void carousel_next_test_tube();
void carousel_go_to_test_tube(uint8_t index);
void carousel_calibrate();
int8_t carousel_get_carousel_index();

void ScienceCommandCenter::executeCommand(const uint8_t commandID,
                                          const uint8_t* rawArgs,
                                          const uint8_t rawArgsLength) {
  switch (commandID) {
    case COMMAND_ESTOP:
      HAL::estop();
      break;
    case COMMAND_NEXT_TEST_TUBE:
      carousel_next_test_tube();
      break;
    case COMMAND_GO_TO_TEST_TUBE:
      if (rawArgsLength == 1) {
        carousel_go_to_test_tube(rawArgs[0]);
      } else {
        digitalWrite(LED, !digitalRead(LED));  
      }
      break;
    case COMMAND_START_CALIBRATING:
      carousel_calibrate();
      break;
    case COMMAND_GET_CAROUSEL_INDEX:
      {
        uint8_t index = carousel_get_carousel_index();
        internal_comms::Message* returnMsg = createMessage(41, 1, &index);
        sendMessage(*returnMsg);
        break;
      }
    default:
      //Serial.printf("command id %d args length %d", commandID, rawArgsLength);
      digitalWrite(LED, !digitalRead(LED));
  }
}
