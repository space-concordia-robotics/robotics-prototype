
#include "Arduino.h"
#define LED 13
#include <Servo.h>

// The includes from pinsetup come from Arm commandcenter
#include "CommandCenter.h"
#include "include/DcMotor.h"
#include "include/Encoder_Data.h"
#include "include/commands/ArmCommandCenter.h"

#define ENCODER_SERIAL Serial1
#define SERIAL_EVENT serialEvent1
#define ARM_PREAMBLE 0xA5
#define ARM_PACKET_LENGTH 6  // contains the preamble and CRC
#define NUM_MOTORS 6

internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

EncoderData encoderData[15];
DcMotor motors[NUM_MOTORS];

void setup() {
  pinMode(LED, OUTPUT);

  // ENCODER_SERIAL.begin(9600);
  // TODO: For testing, with no data
  for (int i = 0; i < 6; i++) {
    EncoderData datum = EncoderData();
    datum.angle = 1.5 + i;
    encoderData[i] = datum;
  }
  // motors[0] = DcMotor(M1_DIR_PIN, M1_STEP_PIN, 1.0, HIGH);
  motors[0] = DcMotor(33, 2, 1.0, HIGH);  // TODO for test only
  motors[1] = DcMotor(M2_DIR_PIN, M2_STEP_PIN, 1.0, HIGH);
  motors[2] = DcMotor(M3_DIR_PIN, M3_STEP_PIN, 1.0, HIGH);
  motors[3] = DcMotor(M4_DIR_PIN, M4_STEP_PIN, 1.0, HIGH);
  // Will have to be changed to allow for the UART motors
  motors[4] = DcMotor(M4_DIR_PIN, M4_STEP_PIN, 1.0, HIGH);
  motors[5] = DcMotor(M4_DIR_PIN, M4_STEP_PIN, 1.0, HIGH);

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
  /*for (int i = 0; i < 3; i++) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  }*/
  char* allocedMessage = strdup("Invalid Arm command");
  internal_comms::Message* message =
      commandCenter->createMessage(0, strlen(allocedMessage), allocedMessage);
  commandCenter->sendMessage(*message);
}

void pong() {
  internal_comms::Message* message =
      commandCenter->createMessage(1, 0, nullptr);
  commandCenter->sendMessage(*message);
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
  for (int i = 0; i < NUM_MOTORS; i++) {
    int angle = (int)(angles[i]);
    motors[i].setSpeed(angles[i]);
  }

  byte* data = new byte[128];
  int r = snprintf(data, 128, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", angles[0],
                   angles[1], angles[2], angles[3], angles[4], angles[5]);
  if (r < 0 || r > 128) {
    invalidCommand();
  }
  internal_comms::Message* message = commandCenter->createMessage(0, r, data);
  commandCenter->sendMessage(*message);
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

void loop() {
  // Read and send messages
  if (Serial.available() > 0) {
    commandCenter->readCommand();
  }
  commandCenter->sendMessage();

  // Do motor checks (ex stop after certain time of no messages)
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].doChecks();
  }
}