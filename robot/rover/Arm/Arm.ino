#include <HardwareSerial.h>
#include <Servo.h>

#include "Arduino.h"
#include "CommandCenter.h"
#include "include/DcMotor.h"
#include "include/SerialMotor.h"
#include "MagneticEncoder.h"
#include "include/commands/ArmCommandCenter.h"

#define NUM_MOTORS 6
#define NUM_DC_MOTORS 4
#define NUM_SMART_SERVOS 2

#define ENCODER_1_ADDRESS      (0x90)
#define ENCODER_2_ADDRESS      (0xA0)
#define ENCODER_3_ADDRESS      (0xC0)

#define ENCODER_SAMPLE_RATE    1000

internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

DcMotor motors[NUM_DC_MOTORS];
SerialMotor serialMotors[NUM_SMART_SERVOS];
LSSServoMotor servoController(&Serial5);

Encoder encoder1,encoder2,encoder3;
Encoder* encoders[] = {&encoder1};

uint32_t encodersLastSample = millis();

void initMagneticEncoder();
void encoderSample(Encoder& encoder);

void setup() {
#ifndef DEBUG
  Serial.begin(9600);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);

  pinSetup();
  initMagneticEncoder();

  motors[0] = DcMotor(M1_DIR_PIN, M1_PWM_PIN, 1.0, LOW);
  motors[1] = DcMotor(M2_DIR_PIN, M2_PWM_PIN, 1.0, LOW);
  motors[2] = DcMotor(M3_DIR_PIN, M3_PWM_PIN, 1.0, LOW);
  motors[3] = DcMotor(M4_DIR_PIN, M4_PWM_PIN, 1.0, LOW);
  serialMotors[0] = SerialMotor(&servoController, 5, 1.0);
  serialMotors[1] = SerialMotor(&servoController, 6, 1.0);

  // tx2 sends to 25,     output 26
  // TX teensy, RX teensy, enable pin, transmit pin
  commandCenter->startSerial(1, 0, 25, 26);

  digitalWrite(LED_BUILTIN, LOW);
}

void initMagneticEncoder(){

    SPI.begin();
    pinMode(CS_PIN,OUTPUT);

    encoder1.address = ENCODER_1_ADDRESS;
    encoder2.address = ENCODER_2_ADDRESS;
    encoder3.address = ENCODER_3_ADDRESS;

    for(auto& e : encoders) {
        // Decimation filter to maximum
        //EncoderWrite16(e.address,REG_MOD1, 0xC000);
        // Activation Status (Set monitoring flags)
        //EncoderWrite16(e.address,REG_MOD1, 0x02FF);

    }
}

void encoderSample(Encoder& encoder) {
    encoder.status =  EncoderRead16(encoder.address,REG_ANGLE_VAL, &encoder.angle._raw, 1);
    //updateState(encoder);
}

/**
 * Implements all the Arm commands.
 */

void invalidCommand(const uint8_t cmdID, const uint8_t* rawArgs,
                    const uint8_t rawArgsLength) {
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
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
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
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

void sendEncoderData(Encoder& encoder){

    if(encoder.status == OK){
        encoder.angle._float = (float)(encoder.angle._raw & 0x7FFF) / (powf(2,15)) * 360.f;

        byte data_msg[5] = {encoder.address,0,0,0,0};
        memcpy(&data_msg[1],encoder.angle._bytes,sizeof(float));
        auto msg = commandCenter->createMessage(2,sizeof(data_msg),data_msg);
        commandCenter->sendMessage(*msg);
    }
    else{
        Serial.print(encoder.status);
        // commandCenter->sendDebug(encoder.status_msg);
    }

}

void loop() {
  // Read and send messages
  if (Serial.available() > 0) {
    commandCenter->readCommand();
  }
  doMotorChecks();

  if( (millis() - encodersLastSample) > ENCODER_SAMPLE_RATE){

    encodersLastSample = millis();
    for(auto e : encoders){
      encoderSample(*e);
      sendEncoderData(*e);
    }
  }

  commandCenter->sendMessage();
}
