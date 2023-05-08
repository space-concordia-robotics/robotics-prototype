//
// Created by cedric on 2020-10-17.
//

#include "include/commands/ScienceCommandCenter.h"
#include "../../../internal_comms/include/CommandCenter.h"
#include "../../include/SciencePinSetup.h"

#define COMMAND_GET_STATUS 39
#define COMMAND_PREVIOUS_TEST_TUBE 41
#define COMMAND_NEXT_TEST_TUBE 42
#define COMMAND_GO_TO_TEST_TUBE 43
#define COMMAND_SPIN_MIX 45

#define COMMAND_ESTOP 25
#define COMMAND_SET_SERVO_ANGLE 26


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
int8_t  carousel_get_carousel_index();
bool carousel_get_moving();

void carousel_previous_test_tube();
void carousel_next_test_tube();
void carousel_go_to_test_tube(uint8_t index);
void carousel_spin_mix();

void carousel_estop();


void ScienceCommandCenter::executeCommand(const uint8_t commandID,
                                          const uint8_t* rawArgs,
                                          const uint8_t rawArgsLength) {
  switch (commandID) {
    case COMMAND_GET_STATUS:
      {
        int8_t msg[2] = {carousel_get_carousel_index(), carousel_get_moving()};
        internal_comms::Message* returnMsg = createMessage(40, 2, (byte*)msg);
        sendMessage(*returnMsg);
        break;
      }

    case COMMAND_PREVIOUS_TEST_TUBE:
      carousel_previous_test_tube();
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

    case COMMAND_SPIN_MIX:
      carousel_spin_mix();
      break;

    case COMMAND_ESTOP:
      carousel_estop();
      break;

    case COMMAND_SET_SERVO_ANGLE:
      {
        break;
      }

    default:
      digitalWrite(LED, !digitalRead(LED));
  }
}
