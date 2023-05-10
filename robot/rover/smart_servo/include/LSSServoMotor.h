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
  // Write action command to servo
  void writeActionCommand(unsigned int servoId, const char* actionCommand,
                          int actionValue);
  // Write action command to servo, with argument that is 32 bits wide
  void writeActionCommand(unsigned int servoId, const char* actionCommand,
                          int32_t actionValue);
  // Write action command to servo without argument
  void writeActionCommand(unsigned int servoId, const char* actionCommand);
  // Write query command to servo, sends back response. NOTE: you must free this response
  char* writeQueryCommand(unsigned int servoId, const char* queryCommand);
  // Write query command, and place response in given buffer.
  // Returns 1 on success, 0 on fail 
  int writeQueryCommand(unsigned int servoId, const char* queryCommand, char* buffer, size_t buflen);

 private:
  HardwareSerial* ServosSerialBus;
};

#endif
