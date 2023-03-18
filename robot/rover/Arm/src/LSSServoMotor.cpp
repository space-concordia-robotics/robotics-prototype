#include "../include/LSSServoMotor.h"

LSSServoMotor::LSSServoMotor(HardwareSerial* serialPort) {
  ServosSerialBus = serialPort;

  ServosSerialBus->begin(115200);
}

/**
 * Action command
 *
 * @param servoId
 * @param actionCommand
 * @param actionValue
 */
void LSSServoMotor::writeActionCommand(unsigned int servoId,
                                       const char* actionCommand,
                                       int actionValue) {
  ServosSerialBus->write('#');
  ServosSerialBus->print(servoId, DEC);
  ServosSerialBus->write(actionCommand);
  ServosSerialBus->print(actionValue, DEC);
  ServosSerialBus->write('\r');

  // This delay is very important in order to avoid spamming the smart servos,
  // it won't work otherwise
  delay(1);
}
/**
 * Modified action command
 *
 * @param servoId
 * @param actionCommand
 * @param actionValue
 * @param modifier
 * @param modifierValue
 */
void LSSServoMotor::writeModifiedActionCommand(unsigned int servoId,
                                       const char* actionCommand,
                                       int actionValue, const char* modifier, int modifierValue) {
  ServosSerialBus->write('#');
  ServosSerialBus->print(servoId, DEC);
  ServosSerialBus->write(actionCommand);
  ServosSerialBus->print(actionValue, DEC);
  ServosSerialBus->write(modifier);
  ServosSerialBus->print(modifierValue, DEC);
  ServosSerialBus->write('\r');

  // This delay is very important in order to avoid spamming the smart servos,
  // it won't work otherwise
  delay(1);
}
/**
 * Action command
 *
 * @param servoId
 * @param actionCommand
 * @param actionValue
 */
void LSSServoMotor::writeActionCommand(unsigned int servoId,
                                       const char* actionCommand) {
  ServosSerialBus->write('#');
  ServosSerialBus->print(servoId, DEC);
  ServosSerialBus->write(actionCommand);
  ServosSerialBus->write('\r');

  // This delay is very important in order to avoid spamming the smart servos,
  // it won't work otherwise
  delay(1);
}

/**
 * Query command
 *
 * @param servoId
 * @param queryCommand
 * @return response to query command from servo NOTE: you have to free this.
 */
char* LSSServoMotor::writeQueryCommand(unsigned int servoId,
                                       const char* queryCommand) {
  ServosSerialBus->write('#');
  ServosSerialBus->print(servoId, DEC);
  ServosSerialBus->write(queryCommand);
  ServosSerialBus->write('\r');

  // This delay is very important in order to avoid spamming the smart servos,
  // it won't work otherwise
  delay(1);

  char* buff = (char*)malloc(15);

  ServosSerialBus->readBytesUntil('\r', buff, 15);

  return buff;
}
