/*
 * Written by William Wells October 01, 2021
 *
 * This is a simple library for interfacing with the communication protocol of
 * the two Lynxmotion Smart Servos used in the gripper of the arm. This was
 * designed to be compatible with the current arm code as of now and any code in
 * the future. As well as being compatible with different microcontrollers.
 */

#ifndef LSSSERVOMOTOR_H
#define LSSSERVOMOTOR_H

#include <Arduino.h>

class LSSServoMotor {
 public:
  LSSServoMotor(HardwareSerial* serialPort);
  void writeActionCommand(unsigned int servoId, const char* actionCommand,
                          int actionValue);
  void writeActionCommand(unsigned int servoId, const char* actionCommand);
  char* writeQueryCommand(unsigned int servoId, const char* queryCommand);

 private:
  HardwareSerial* ServosSerialBus;
};

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
 * @param servoId
 * @param queryCommand
 * @return response to query command from servo
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

#endif
