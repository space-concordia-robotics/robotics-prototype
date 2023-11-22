/*
 * Written by William Wells October 01, 2021
 * Modified by Marc Scattolin on May 29, 2022
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
  void writeModifiedActionCommand(unsigned int servoId,
                                       const char* actionCommand,
                                       int actionValue, const char* modifier, int modifierValue);
  char* writeQueryCommand(unsigned int servoId, const char* queryCommand);

 private:
  HardwareSerial* ServosSerialBus;
};

#endif
