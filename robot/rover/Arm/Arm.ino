
#include "Arduino.h"
#define LED 13
#include <Servo.h>

#include "CommandCenter.h"
#include "include/Encoder_Data.h"
#include "include/commands/ArmCommandCenter.h"

#define ENCODER_SERIAL Serial1
#define SERIAL_EVENT serialEvent1
#define ARM_PREAMBLE 0xA5
#define ARM_PACKET_LENGTH 6  // contains the preamble and CRC

internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

EncoderData encoderData[15];

void setup() {
  pinMode(LED, OUTPUT);

  // ENCODER_SERIAL.begin(9600);
  // TODO: For testing, with no data
  for (int i = 0; i < 6; i++) {
    EncoderData datum = EncoderData();
    datum.angle = 1.5 + i;
    encoderData[i] = datum;
  }

  /*Serial.begin(9600);
  while (!Serial) {
    // wait for serial monitor
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }*/

  commandCenter->startSerial(-1, -1, 24,
                             -1);  // not using transmitenable with usb
}

// 0 bytes received means waiting on next preamble
// 1 means received preamble, 2 means received 1 preamble and 1 data byte, etc.
volatile byte encoderBytesReceived = 0;
volatile bool newEncoderData = false;
byte encoderTemp[ARM_PACKET_LENGTH];
volatile byte tRead = 255;  // for debug only
volatile int passedEvent = 0;

void SERIAL_EVENT() {
  byte read = ENCODER_SERIAL.read();
  tRead = read;
  passedEvent = 1;
  // check if waiting for preamble
  if (encoderBytesReceived == 0) {
    if (read == ARM_PREAMBLE) {
      encoderTemp[0] = read;
      encoderBytesReceived++;
    } else {
      digitalWrite(LED, HIGH);
      // do nothing, wait on valid preamble
    }
  } else {
    // here, not waiting on preamble (receiving some middle byte)
    encoderTemp[encoderBytesReceived] = read;
    encoderBytesReceived++;

    // Do the address check once 2 received address.
    if (encoderBytesReceived == 2) {
      unsigned char address = EncoderData::getAddress(encoderTemp[1]);
      if (address > 15) {
        // If address invalid, reset to waiting on preamble.
        encoderBytesReceived = 0;
        digitalWrite(LED, LOW);
      }
    }

    if (encoderBytesReceived == ARM_PACKET_LENGTH) {
      // if here, just received last byte of packet
      // copy temp to shared memory with the address as the index in the array
      unsigned char address = EncoderData::getAddress(encoderTemp[1]);
      if (address <= 15) {  // check that the address arrived intact
        EncoderData thisData;
        thisData.setData(encoderTemp);
        encoderData[address] = thisData;

        // signal new data packet
        newEncoderData = true;
      }
      // reset to waiting on preamble
      encoderBytesReceived = 0;
      digitalWrite(LED, LOW);
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
  byte* data = new byte[128];
  int r = snprintf(data, 128, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", angles[0],
                   angles[1], angles[2], angles[3], angles[4], angles[5]);
  if (r < 0 || r > 128) {
    invalidCommand();
  }
  internal_comms::Message* message = commandCenter->createMessage(0, r, data);
  commandCenter->sendMessage(*message);
}

#define NUM_MOTORS 6
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

  /*if (newEncoderData) {
    for (int i = 0; i <= 15; i++) {
      EncoderData enc = encoderData[i];
      Serial.print("Encoder ");
      Serial.print(i);
      Serial.print(" data: ");
      Serial.print(enc.angle);
      Serial.print(" ");
      Serial.print(enc.temperatureData, HEX);
      Serial.print(" ");
      Serial.print(enc.CRC, HEX);
      Serial.println();
    }
    Serial.print("\n\n");
    newEncoderData = false;
  } else if (passedEvent) {
    Serial.print("last read: ");
    Serial.print(tRead, HEX);
    Serial.print("\r\n");
    passedEvent = 0;
  }*/
}