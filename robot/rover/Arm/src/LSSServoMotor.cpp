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
 * @param servoId ID of the servo (remember, this is stored in the servo)
 * @param queryCommand Command to send over, must be a query
 * @param buf Buffer to store response
 * @param len Length of buf
 * @return response to query command from servo; the same buffer that was provided.
 */
char* LSSServoMotor::writeQueryCommand(unsigned int servoId,
                                       const char* queryCommand, char* buf, int len) {
  ServosSerialBus->write('#');
  ServosSerialBus->print(servoId, DEC);
  ServosSerialBus->write(queryCommand);
  ServosSerialBus->write('\r');

  // This delay is very important in order to avoid spamming the smart servos,
  // it won't work otherwise
  delay(1);

  ServosSerialBus->readBytesUntil('\r', buf, len);

  return buf;
}