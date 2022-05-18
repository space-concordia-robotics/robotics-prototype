
#include "Arduino.h"
#define LED 13
#include <HardwareSerial.h>
#include <Servo.h>

#include "CommandCenter.h"
#include "include/DcMotor.h"
#include "include/Encoder_Data.h"
#include "include/SerialMotor.h"
#include "include/commands/ArmCommandCenter.h"

#define NUM_MOTORS 6
#define NUM_DC_MOTORS 4
#define NUM_SMART_SERVOS 2

internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

EncoderData encoderData[15];
DcMotor motors[NUM_DC_MOTORS];
SerialMotor serialMotors[NUM_SMART_SERVOS];
LSSServoMotor servoController(&Serial5);

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);

  pinSetup();

  motors[0] = DcMotor(M1_DIR_PIN, M1_STEP_PIN, 1.0, LOW);
  motors[1] = DcMotor(M2_DIR_PIN, M2_STEP_PIN, 1.0, LOW);
  motors[2] = DcMotor(M3_DIR_PIN, M3_STEP_PIN, 1.0, LOW);
  motors[3] = DcMotor(M4_DIR_PIN, M4_STEP_PIN, 1.0, LOW);
  serialMotors[0] = SerialMotor(&servoController, 5, 1.0);
  serialMotors[1] = SerialMotor(&servoController, 6, 1.0);

  // tx2 sends to 25,     output 26
  // TX teensy, RX teensy, enable pin, transmit pin
  commandCenter->startSerial(1, 0, 25, 26);

  digitalWrite(LED, LOW);
}


/**
 * Implements all the Arm commands.
 */

void invalidCommand(const uint8_t cmdID, const uint8_t* rawArgs,
                    const uint8_t rawArgsLength) {
  digitalWrite(LED, !digitalRead(LED));
}

void invalidCommand() {
  digitalWrite(LED, !digitalRead(LED));
}

void pong() {
  digitalWrite(LED, !digitalRead(LED));
}

void moveMotorsBy(float* angles, uint16_t numAngles) {
  // Right now, this only prints out back the received angles
  byte* data = new byte[128];
  int r = snprintf(data, 128, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", angles[0],
                   angles[1], angles[2], angles[3], angles[4], angles[5]);
  if (r < 0 || r > 128) {
    invalidCommand();
  }
  internal_comms::Message* message = commandCenter->createMessage(0, r, data);
  commandCenter->sendMessage(*message);
}

void setMotorSpeeds(float* angles) {
  for (int i = 0; i < NUM_DC_MOTORS; i++) {
    int angle = (int)(angles[i]);
    motors[i].setSpeed(angle);
  }
  for (int i = 0; i < NUM_SMART_SERVOS; i++) {
    int angle = (int)(angles[NUM_DC_MOTORS + i]);
    serialMotors[i].setSpeed(angle);
  }
}

void sendMotorAngles() {
  float softwareAngles[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    softwareAngles[i] = encoderData[i].angle;
  }

  byte* data = new byte[24];

  for (int i = 0; i < NUM_MOTORS; i++) {
    byte* inByte = (byte*)(&softwareAngles[i]);
    data[4 * i] = inByte[0];
    data[4 * i + 1] = inByte[1];
    data[4 * i + 2] = inByte[2];
    data[4 * i + 3] = inByte[3];
  }

  internal_comms::Message* message = commandCenter->createMessage(2, 24, data);
  commandCenter->sendMessage(*message);
}

void doMotorChecks() {
  // Do motor checks (ex stop after certain time of no messages)
  for (int i = 0; i < NUM_DC_MOTORS; i++) {
    motors[i].doChecks();
  }
  for (int i = 0; i < NUM_SMART_SERVOS; i++) {
    serialMotors[i].doChecks();
  }
}

void delayChecks(unsigned long delayMillis) {
  unsigned long startTime = millis();
  while (millis() - startTime < delayMillis) {
    doMotorChecks();
    delay(1);
  }
}

void loop() {
  // Read and send messages
  if (Serial1.available() > 0) {
    commandCenter->readCommand();
    doMotorChecks();
  }
  doMotorChecks();
  commandCenter->sendMessage();
  doMotorChecks();
}