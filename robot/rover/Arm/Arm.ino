
#include "Arduino.h"
#include "MagneticEncoder.h"
#include "DcMotor.h"
#include "SerialMotor.h"
#include "commands/ArmCommandCenter.h"

#include <IntervalTimer.h>
#include <HardwareSerial.h>
#include <Servo.h>

#define ENCODER_1_ADDRESS      (0x09)
#define ENCODER_2_ADDRESS      (0x0A)
#define ENCODER_3_ADDRESS      (0x0B)

#define MAGNETIC_ENCODER_IRQ_RATE 1000
#define ENCODER_SEND_RATE         100

#define SERIAL_EVENT serialEven t1
#define ARM_PREAMBLE 0xA5
#define ARM_PACKET_LENGTH 6  // contains the preamble and CRC
#define NUM_MOTORS 6
#define NUM_DC_MOTORS 4
#define NUM_SMART_SERVOS 2
internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

DcMotor motors[NUM_DC_MOTORS];
SerialMotor serialMotors[NUM_SMART_SERVOS];
LSSServoMotor servoController(&Serial5);

TLE5012MagneticEncoder magneticEncoder1(ENCODER_1_ADDRESS);
TLE5012MagneticEncoder magneticEncoder2(ENCODER_2_ADDRESS);
TLE5012MagneticEncoder magneticEncoder3(ENCODER_3_ADDRESS);


IntervalTimer magneticEncoderTimer1;
uint32_t encodersLastSent = millis();

void initMagneticEncoder();

void magneticEncoder1IRQ();
void magneticEncoder2IRQ();
void magneticEncoder3IRQ();

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);

  // ENCODER_SERIAL.begin(9600);
  // TODO: For testing, with no data
  /*for (int i = 0; i < 6; i++) {
    EncoderData datum = EncoderData();
    datum.angle = 1.5 + i;
    encoderData[i] = datum;
  }*/
  pinSetup();
  initMagneticEncoder();

  motors[0] = DcMotor(M1_DIR_PIN, M1_STEP_PIN, 1.0, LOW);
  motors[1] = DcMotor(M2_DIR_PIN, M2_STEP_PIN, 1.0, LOW);
  motors[2] = DcMotor(M3_DIR_PIN, M3_STEP_PIN, 1.0, LOW);
  motors[3] = DcMotor(M4_DIR_PIN, M4_STEP_PIN, 1.0, LOW);
  serialMotors[0] = SerialMotor(&servoController, 5, 1.0);
  serialMotors[1] = SerialMotor(&servoController, 6, 1.0);

  magneticEncoderTimer1.begin(magneticEncoder1IRQ,MAGNETIC_ENCODER_IRQ_RATE);
//  magneticEncoderTimer2.begin(magneticEncoder2IRQ,MAGNETIC_ENCODER_IRQ_RATE);
//  magneticEncoderTimer3.begin(magneticEncoder3IRQ,MAGNETIC_ENCODER_IRQ_RATE);


    /*Serial.begin(9600);
    while (!Serial) {
      // wait for serial monitor
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
    }*/
  // tx2 sends to 25,     output 26
  // TX teensy, RX teensy, enable pin, transmit pin
  commandCenter->startSerial(1, 0, 25, 26);
  /*commandCenter->startSerial(-1, -1, 24,
                             -1);*/  // not using transmitenable with usb
  digitalWrite(LED_BUILTIN, LOW);
}

// 0 bytes received means waiting on next preamble
// 1 means received preamble, 2 means received 1 preamble and 1 data byte, etc.
volatile byte tRead = 255;  // for debug only
volatile int passedEvent = 0;

void initMagneticEncoder(){
    // Decimation filter to maximum
    magneticEncoder1.SPIWrite16(REG_MOD1,0xC000);

}
void magneticEncoder1IRQ() {

    uint16_t digitalAngle;

    magneticEncoder1.SPIRead16(REG_ANGLE_VAL,&digitalAngle,1);

    if(magneticEncoder1.status == SUCCESS) {
        // Note that the first bit is a status indicator, we don't need it for the actual angle.
        float angle = (float) (digitalAngle & 0x7FFF) * 360.f / powf(2, 17);
        magneticEncoder1.angle._float = angle;
    }
}

/**
 * Implements all the Arm commands.
 */

void invalidCommand(const uint8_t cmdID, const uint8_t* rawArgs,
                    const uint8_t rawArgsLength) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  /*char* buffer = (char*)malloc(256);
  snprintf(buffer, 256, "Invalid command, ID %d, args length %d", cmdID,
           rawArgsLength);

  internal_comms::Message* message =
      commandCenter->createMessage(0, strlen(buffer), buffer);
  commandCenter->sendMessage(*message);*/
}

void invalidCommand() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  /*char* allocedMessage = strdup("Invalid Arm command");
  internal_comms::Message* message =
      commandCenter->createMessage(0, strlen(allocedMessage), allocedMessage);
  commandCenter->sendMessage(*message);*/
}

void pong() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  /*internal_comms::Message* message =
      commandCenter->createMessage(1, 0, nullptr);
  commandCenter->sendMessage(*message);*/
}

void moveMotorsBy(float* angles, uint16_t numAngles) {
  // Right now, this only prints out back the received angles
  byte* data = new byte[128];
  int r = snprintf((char*)data, 128, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", angles[0],
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
void sendEncoderData(const TLE5012MagneticEncoder& encoder){
    if(encoder.status == SUCCESS){
        auto msg = commandCenter->createMessage(2,4,encoder.angle._bytes);
        commandCenter->sendMessage(*msg);
    }
    else{
        commandCenter->sendDebug(encoder.status_msg_buffer);
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

  if( (millis() - encodersLastSent) > ENCODER_SEND_RATE){
      encodersLastSent = millis();
      sendEncoderData(magneticEncoder1);
  }

}