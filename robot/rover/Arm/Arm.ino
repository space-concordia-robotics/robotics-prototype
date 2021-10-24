
#include "Arduino.h"
#define LED 13
#include <Servo.h>

#include "CommandCenter.h"
#include "include/commands/ArmCommandCenter.h"

internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  commandCenter->startSerial(-1, -1, 24,
                             -1);  // not using transmitenable with usb
}

/**
 * Implements all the Arm commands.
 */

void invalidCommand() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }
  char* allocedMessage = strdup("Invalid Arm command");
  internal_comms::Message* message =
      commandCenter->createMessage(0, strlen(allocedMessage), allocedMessage);
  commandCenter->sendMessage(*message);
}

void pong() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);

  internal_comms::Message* message =
      commandCenter->createMessage(1, 0, nullptr);
  commandCenter->sendMessage(*message);
}

void moveMotorsBy(float* angles, uint16_t numAngles) {
  char buffer[8];
  snprintf(buffer, 8, "%d", numAngles);
  Serial.write(buffer);
  Serial.write("Angles received");

  for (int i = 0; i < numAngles; i++) {
    sprintf(buffer, "%f\n", angles[i]);
    Serial.write(buffer);
  }
}
#define NUM_MOTORS 6
void sendMotorAngles() {
  float softwareAngles[NUM_MOTORS] = {-10000000000, 2.0, 3.0, 4.0, 5.0, 6.0};

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
  //

  /*
    float angles[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    byte* data = malloc(sizeof(angles) * 6);
    data[0] = 0x0a;
    data[1] = 0xF3;
    // memcpy(data, angles, sizeof(angles) * 6);
    internal_comms::Message* message =
        commandCenter->createMessage(2, sizeof(angles) * 6, data);
    commandCenter->sendMessage(*message);*/
}

void loop() {
  if (Serial.available() > 0) {
    commandCenter->readCommand();
  }
  commandCenter->sendMessage();
}