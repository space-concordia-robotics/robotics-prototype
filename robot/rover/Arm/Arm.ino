

#include <HardwareSerial.h>
#include <Servo.h>
#include "Arduino.h"
#include "CommandCenter.h"
#include "TeensyTimerTool.h"
#include "include/ArmDebug.h"
#include "include/DcMotor.h"
#include "include/SerialMotor.h"
#include "MagneticEncoder.h"
#include "include/commands/ArmCommandCenter.h"

using namespace TeensyTimerTool;
// motor 1 - 4
// motor 3 - 1
// motor 2 - 2

#define NUM_MOTORS 6
#define NUM_DC_MOTORS 4
#define NUM_SMART_SERVOS 2

#define ENCODER_1_ADDRESS      (0x90)
#define ENCODER_2_ADDRESS      (0xA0)
#define ENCODER_3_ADDRESS      (0xB0)

#define MAGNETIC_ENCODER_IRQ_RATE 250'000
#define ENCODER_SEND_RATE         100

internal_comms::CommandCenter* commandCenter = new ArmCommandCenter();

DcMotor motors[NUM_DC_MOTORS];
SerialMotor serialMotors[NUM_SMART_SERVOS];
LSSServoMotor servoController(&Serial5);

TLE5012MagneticEncoder encoder1(ENCODER_1_ADDRESS);
TLE5012MagneticEncoder encoder2(ENCODER_2_ADDRESS);
TLE5012MagneticEncoder encoder3(ENCODER_3_ADDRESS);

IntervalTimer encoder1Timer;

uint32_t encodersLastSent = millis();

void initMagneticEncoder();

void encoder1IRQ();
void encoder2IRQ();
void encoder3IRQ();

void setup() {

  Serial.begin(9600);


  pinSetup();
  initMagneticEncoder();

  motors[0] = DcMotor(M1_DIR_PIN, M1_PWM_PIN, 1.0, LOW);
  motors[1] = DcMotor(M2_DIR_PIN, M2_PWM_PIN, 1.0, LOW);
  motors[2] = DcMotor(M3_DIR_PIN, M3_PWM_PIN, 1.0, LOW);
  motors[3] = DcMotor(M4_DIR_PIN, M4_PWM_PIN, 1.0, LOW);
  serialMotors[0] = SerialMotor(&servoController, 5, 1.0);
  serialMotors[1] = SerialMotor(&servoController, 6, 1.0);

  encoder1Timer.begin(encoder1IRQ,1'000'000);
  //  Serial.print(static_cast<char>(err));
//  magneticEncoderTimer2.begin(magneticEncoder2IRQ,MAGNETIC_ENCODER_IRQ_RATE);
//  magneticEncoderTimer3.begin(magneticEncoder3IRQ,MAGNETIC_ENCODER_IRQ_RATE);

  // tx2 sends to 25,     output 26
  // TX teensy, RX teensy, enable pin, transmit pin
  commandCenter->startSerial(1, 0, 25, 26);

}

void initMagneticEncoder(){
    // Decimation filter to maximum
    //magneticEncoder1.SPIWrite16(REG_MOD1,0xC000);
    //magneticEncoder1.SPIWrite16(0x01,0x0001);
}
void encoder1IRQ() {

    uint16_t raw_angle;
    encoder1.status =  encoder1.SPIRead16(REG_ANGLE_VAL, &raw_angle, 1);
    //checkError(encoder1.status, const_cast<char **>(&encoder1.status_msg_buffer));
}

/**
 * Implements all the Arm commands.
 */

void invalidCommand() {
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // NOTE: this code is commented out since       sending debug strings
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
  for (auto & motor : motors) {
    motor.doChecks();
  }
  for (auto & serialMotor : serialMotors) {
    serialMotor.doChecks();
  }
}

void debug_test() {
  const char* strMessage = "This is a debug string";
  commandCenter->sendDebug(strMessage);
}
void sendEncoderData(const TLE5012MagneticEncoder& encoder){
    //Serial.write(0x01);

    if(true){

        float angle = (float)(encoder.raw_angle & 0x7FFF) / (powf(2,15)) * 360.f;
        Serial.println(angle);
        byte angle_bytes[4];
        memcpy(angle_bytes, (unsigned char*) (&angle), sizeof(float));

        //auto msg = commandCenter->createMessage(2,4,angle_bytes);
        //commandCenter->sendMessage(*msg);
    }
    else{
        Serial.println(*encoder.status_msg_buffer);
        //commandCenter->sendDebug(encoder.status_msg_buffer);
    }
}
void loop() {
  // Read and send messages
  if (Serial.available() > 0) {
    commandCenter->readCommand();
  }
  //doMotorChecks();
    //doMotorChecks();

    if( (millis() - encodersLastSent) > ENCODER_SEND_RATE){
        encodersLastSent = millis();
//        /sendEncoderData(encoder1);
    }
    commandCenter->sendMessage();

}