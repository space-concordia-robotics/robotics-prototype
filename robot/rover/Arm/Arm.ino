

#include <HardwareSerial.h>
#include <Servo.h>

#include "Arduino.h"
#include "CommandCenter.h"
#include "include/ArmDebug.h"
#include "include/DcMotor.h"
#include "include/SerialMotor.h"
#include "MagneticEncoder.h"
#include "include/commands/ArmCommandCenter.h"

#define NUM_MOTORS 6
#define NUM_DC_MOTORS 4
#define NUM_SMART_SERVOS 2

#define ENCODER_1_ADDRESS      (0x09)
#define ENCODER_2_ADDRESS      (0x0A)
#define ENCODER_3_ADDRESS      (0x0B)

#define MAGNETIC_ENCODER_IRQ_RATE 1000
#define ENCODER_SEND_RATE         100

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

  magneticEncoderTimer1.begin(magneticEncoder1IRQ,MAGNETIC_ENCODER_IRQ_RATE);
//  magneticEncoderTimer2.begin(magneticEncoder2IRQ,MAGNETIC_ENCODER_IRQ_RATE);
//  magneticEncoderTimer3.begin(magneticEncoder3IRQ,MAGNETIC_ENCODER_IRQ_RATE);

  // tx2 sends to 25,     output 26
  // TX teensy, RX teensy, enable pin, transmit pin
  commandCenter->startSerial(1, 0, 25, 26);

  digitalWrite(LED_BUILTIN, LOW);
}

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


}

void invalidCommand() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

}

void pong() {

}

void moveMotorsBy(float* angles, uint16_t numAngles) {

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
  commandCenter->sendDebug(strMessage);
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
  if (Serial.available() > 0) {
    commandCenter->readCommand();
  }
  doMotorChecks();
  commandCenter->sendMessage();
  doMotorChecks();

  if( (millis() - encodersLastSent) > ENCODER_SEND_RATE){
      encodersLastSent = millis();
      sendEncoderData(magneticEncoder1);
  }

}