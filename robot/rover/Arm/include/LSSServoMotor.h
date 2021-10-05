/*
 * Written by William Wells October 01, 2021
 *
 * This is a simple library for interfacing with the communication protocol of the two Lynxmotion Smart Servos used
 * in the gripper of the arm. This was designed to be compatible with the current arm code as of now and any code in
 * the future. As well as being compatible with different microcontrollers.
 */

#ifndef LSSSERVOMOTOR_H
#define LSSSERVOMOTOR_H

#include <Arduino.h>

class LSSServoMotor {
    public:
        LSSServoMotor(HardwareSerial* serialPort);
        void writeServoCommand(unsigned int servoId, char* actionCommand, int actionValue);
        char* writeServoCommand(unsigned int servoId, char* queryCommand);

    private:
        HardwareSerial* ServosSerialBus;
};

LSSServoMotor::LSSServoMotor(HardwareSerial* serialPort) {
    this -> ServosSerialBus = serialPort;

    this -> ServosSerialBus -> begin(115200);
}

/**
 * Action command
 *
 * @param servoId
 * @param actionCommand
 * @param actionValue
 */
void LSSServoMotor::writeServoCommand(unsigned int servoId, char* actionCommand, int actionValue) {
    this -> ServosSerialBus -> write('#');
    this -> ServosSerialBus -> print(servoId, DEC);
    this -> ServosSerialBus -> write(actionCommand);
    this -> ServosSerialBus -> print(actionValue, DEC);
    this -> ServosSerialBus -> write('\r');
}

/**
 * Query command
 *
 * @param servoId
 * @param queryCommand
 * @return response to query command from servo
 */
char* LSSServoMotor::writeServoCommand(unsigned int servoId, char* queryCommand) {
    this -> ServosSerialBus -> write('#');
    this -> ServosSerialBus -> print(servoId, DEC);
    this -> ServosSerialBus -> write(queryCommand);
    this -> ServosSerialBus -> write('\r');

    char* buff;

    this -> ServosSerialBus -> readBytesUntil('\r', buff, 5);

    return buff;
}

#endif
