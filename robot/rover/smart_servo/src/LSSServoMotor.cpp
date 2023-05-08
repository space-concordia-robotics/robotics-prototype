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
 * Action command with a 32bit argument
 *
 * @param servoId
 * @param actionCommand
 * @param actionValue
 */
void LSSServoMotor::writeActionCommand(unsigned int servoId,
                                       const char* actionCommand,
                                       int32_t actionValue) {
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

/**
 * Query command
 *
 * @param servoId
 * @param queryCommand
 * @param buffer Buffer to place response in. Null is added after 
 * @param buflen Length of buffer, including space for null terminator
 * @return 1 on success, 0 on fail
 */
int LSSServoMotor::writeQueryCommand(unsigned int servoId, const char* queryCommand, 
                                       char* buffer, size_t buflen) {
  ServosSerialBus->write('#');
  ServosSerialBus->print(servoId, DEC);
  ServosSerialBus->write(queryCommand);
  ServosSerialBus->write('\r');

  // This delay is very important in order to avoid spamming the smart servos,
  // it won't work otherwise
  delay(1);

  size_t numberCharactersRead = ServosSerialBus->readBytesUntil('\r', buffer, buflen - 1);
  if (numberCharactersRead < buflen) {
    buffer[numberCharactersRead] = 0; // null terminate
  }

  return numberCharactersRead != 0;
}