
#include "Arduino.h"
#define LED 13
#include <Servo.h>

#include "CommandCenter.h"
#include "include/commands/ArmCommandCenter.h"

#define ENCODER_SERIAL Serial1
#define SERIAL_EVENT serialEvent1
#define ARM_PREAMBLE 0xA5
#define ARM_PACKET_LENGTH 6  // contains the preamble and CRC

internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

byte encoderTemp[ARM_PACKET_LENGTH];
byte encoderData[ARM_PACKET_LENGTH];
// 0 bytes received means waiting on next preamble
// 1 means received preamble, 2 means received 1 preamble
// and 1 data byte, etc.
volatile byte encoderBytesReceived = 0;
volatile bool newEncoderData = false;

void setup() {
  for (int i = 0; i < ARM_PACKET_LENGTH; i++) {
    encoderTemp[i] = 2;
    encoderData[i] = 2;
  }

  ENCODER_SERIAL.begin(9600);
  Serial.begin(9600);
  while (!Serial) {
    // wait for serial monitor
  }
  Serial.println("Hello!");

  pinMode(LED_BUILTIN, OUTPUT);

  while (true) {
    Serial.println("Hello world!");
    delay(50);

    if (newEncoderData) {
      Serial.print("Encoder data: ");
      for (int i = 0; i < ARM_PACKET_LENGTH; i++) {
        Serial.print(encoderData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      newEncoderData = false;
    } else {
      // debug only
      /*Serial.print("Encoder temp: ");
      for (int i = 0; i < ARM_PACKET_LENGTH; i++) {
        Serial.print(encoderTemp[i], HEX);
      }
      Serial.print(" Bytes received: ");
      Serial.print(encoderBytesReceived);
      Serial.println();*/
    }
  }

  /*
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  commandCenter->startSerial(-1, -1, 24,
                             -1);  // not using transmitenable with usb
                             */
}

void SERIAL_EVENT() {
  byte read = ENCODER_SERIAL.read();
  // check if waiting for preamble
  if (encoderBytesReceived == 0) {
    if (read == ARM_PREAMBLE) {
      encoderTemp[0] = read;
      encoderBytesReceived++;
    } else {
      // do nothing, wait on valid preamble
    }
  } else {
    // here, not waiting on preamble (receiving some middle byte)
    encoderTemp[encoderBytesReceived] = read;
    encoderBytesReceived++;
    if (encoderBytesReceived == ARM_PACKET_LENGTH) {
      // if here, just received last byte of packet
      // copy temp to shared memory
      for (int i = 0; i < ARM_PACKET_LENGTH; i++) {
        encoderData[i] = encoderTemp[i];
      }
      // signal new data packet
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      newEncoderData = true;
      // reset to waiting on preamble
      encoderBytesReceived = 0;
    }
  }
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