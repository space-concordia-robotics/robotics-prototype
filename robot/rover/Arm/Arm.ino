#include <HardwareSerial.h>
#include <Servo.h>

#include "Arduino.h"
#include "CommandCenter.h"
#include "include/ArmDebug.h"
#include "include/DcMotor.h"
#include "include/SerialMotor.h"
#include "include/commands/ArmCommandCenter.h"

#define NUM_MOTORS 6
#define NUM_DC_MOTORS 4
#define NUM_SMART_SERVOS 2
#define BASE_SERVO_ID 5
#define GRIP_SERVO_ID 6

internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

DcMotor motors[NUM_DC_MOTORS];
SerialMotor serialMotors[NUM_SMART_SERVOS];
LSSServoMotor servoController(&Serial5);

void setup() {
#ifndef DEBUG
  Serial.begin(9600);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);

  pinSetup();

  motors[0] = DcMotor(M1_DIR_PIN, M1_PWM_PIN, 1.0, LOW);
  motors[1] = DcMotor(M2_DIR_PIN, M2_PWM_PIN, 1.0, LOW);
  motors[2] = DcMotor(M3_DIR_PIN, M3_PWM_PIN, 1.0, LOW);
  motors[3] = DcMotor(M4_DIR_PIN, M4_PWM_PIN, 1.0, LOW);
  serialMotors[0] = SerialMotor(&servoController, BASE_SERVO_ID, 1.0);
  serialMotors[1] = SerialMotor(&servoController, GRIP_SERVO_ID, 1.0);

  // tx2 sends to 25,     output 26
  // TX teensy, RX teensy, enable pin, transmit pin
  commandCenter->startSerial(1, 0, 25, 26);

  digitalWrite(LED_BUILTIN, LOW);
}

/**
 * Implements all the Arm commands.
 */

void invalidCommand(const uint8_t cmdID, const uint8_t* rawArgs,
                    const uint8_t rawArgsLength) {
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // NOTE: this code is commented out since sending debug strings
  // can overwhelm the bus if the teensy is seeing many invalid commands,
  // which has happened in the past.

  /*char buf[64];
  snprintf(buf, sizeof(buf), "Invalid command id %d arg length %d", cmdID,
           rawArgsLength);
  internal_comms::Message* message =
      commandCenter->createMessage(0, strlen(buf), buf);
  commandCenter->sendMessage(*message);*/
}

void pong() {
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  internal_comms::Message* message =
      commandCenter->createMessage(1, 0, nullptr);
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

void getServoPower() {
  const char* velocity = serialMotors[1].writeQueryCommand(GRIP_SERVO_ID, "QV");
  const char* current = serialMotors[1].writeQueryCommand(GRIP_SERVO_ID, "QC");
  const char* power = velocity + current;
  internal_comms::Message* message =
      commandCenter->createMessage(80, strlen(power), power);
  commandCenter->sendMessage(*message);

  free(velocity);
  free(current);
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

void debug_test() {
  const char* strMessage = "This is a debug string";
  internal_comms::Message* message =
      commandCenter->createMessage(0, strlen(strMessage), strMessage);
  commandCenter->sendMessage(*message);
}

void loop() {
  // Read and send messages
  if (Serial.available() > 0) {
    commandCenter->readCommand();
  }
  doMotorChecks();
  commandCenter->sendMessage();
}