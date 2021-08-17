//
// Edited by Michael on 2021-03-20.
//

// USB : Debug, UART : Production
#define USB

#include <cmath>
#include <cstdint>
#include "Navigation.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#include "ArduinoBlue.h"
#include "Rover.h"
#include "commands/WheelsCommandCenter.h"
/* Global variables*/
uint32_t sinceFeedbackPrint = millis(); // timer for sending motor speeds and battery measurements
uint32_t  sinceLedToggle = millis(); // timer for heartbeat
uint32_t  sinceSensorRead = millis(); // timer for reading battery, gps and imu data
uint32_t  sinceMC = millis(); // timer for reading battery, gps and imu data


// Pins for Serial
const uint8_t RX_TEENSY_3_6_PIN = 0;
const uint8_t TX_TEENSY_3_6_PIN = 1;
const uint8_t ENABLE_PIN = 15; // TEMPORARY BEFORE FLOW CONTROL IMPLEMENTED
const uint8_t TRANSMIT_PIN = 14;


// To read commands from wheelscommandcenter
internal_comms::CommandCenter* commandCenter = new WheelsCommandCenter();
//Rover* rover = new Rover();

// Initialization Serial
void initSerialCommunications(void);

// Wheel command methods from WheelsCommandCenter.cpp
// https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=963483371
void stopMotors(void);
void attachMotors();
void attachServos();
void attachEncoders();
void initPidControllers();
void writeServoDefaultValues();
void initPins();
// Messages to send back to OBC from wheel Teensy
// https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=963483371
volatile uint32_t InterruptHandler::LEFT_BACK_MOTOR_PREV_DT=0;
volatile uint32_t InterruptHandler::LEFT_BACK_MOTOR_ENCODER_COUNT=0;
volatile uint32_t InterruptHandler::LEFT_BACK_MOTOR_DT=0;

volatile uint32_t InterruptHandler::LEFT_MIDDLE_MOTOR_PREV_DT=0;
volatile uint32_t InterruptHandler::LEFT_MIDDLE_MOTOR_DT=0;
volatile uint32_t InterruptHandler::LEFT_MIDDLE_MOTOR_ENCODER_COUNT=0;


volatile uint32_t InterruptHandler::LEFT_FRONT_MOTOR_PREV_DT=0;
volatile uint32_t InterruptHandler::LEFT_FRONT_MOTOR_ENCODER_COUNT=0;
volatile uint32_t InterruptHandler::LEFT_FRONT_MOTOR_DT=0;


volatile uint32_t InterruptHandler::RIGHT_BACK_MOTOR_PREV_DT=0;
volatile uint32_t InterruptHandler::RIGHT_BACK_MOTOR_ENCODER_COUNT=0;
volatile uint32_t InterruptHandler::RIGHT_BACK_MOTOR_DT=0;


volatile uint32_t InterruptHandler::RIGHT_FRONT_MOTOR_PREV_DT=0;
volatile uint32_t InterruptHandler::RIGHT_FRONT_MOTOR_ENCODER_COUNT=0;
volatile uint32_t InterruptHandler::RIGHT_FRONT_MOTOR_DT=0;


volatile uint32_t InterruptHandler::RIGHT_MIDDLE_MOTOR_PREV_DT=0;
volatile uint32_t InterruptHandler::RIGHT_MIDDLE_MOTOR_ENCODER_COUNT=0;
volatile uint32_t InterruptHandler::RIGHT_MIDDLE_MOTOR_DT=0;


// Initial teensy setup
void setup() {
    pinMode(LED_BUILTIN,OUTPUT);
    initSerialCommunications();
    pinMode(V_SENSE_PIN, INPUT);
    // Initialize setup for pins from PinSetup.h
  //initPins();
  attachMotors();
  attachEncoders();
  initPidControllers();

    pinMode(ENABLE_PIN,OUTPUT);
  digitalWrite(ENABLE_PIN,HIGH);
  // Initialize servo motors

  //attachServos();
  //writeServoDefaultValues();
  // Handle navigation commands each time a new command is received
  // Looped and called as new commands are received 
  //initNav(Cmds);

  Rover::openLoop();
  //rover->openAllMotorLoop();
}

// Running the wheels
void loop() {

    digitalWrite(LED_BUILTIN,HIGH);

    delay(1000);
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
  // Acquire method based on command sent from serial
    //Serial.write(Serial.available());
    //delay(1000);
    //delay(100);

    if (Serial.available()) {
    //  Serial.write(Serial.read());
    /*
    1. Pointer to select the method (WheelsCommandCenter.cpp) to run based on the command
    2. Read command performs executeCommand()
    3. If the serial is not enabled (such as being used by the arm), then the command skips this
    */
        commandCenter->readCommand();
    }
    /*

  if (sinceSensorRead-millis() > SENSOR_READ_INTERVAL) {

    commandCenter->executeCommand(COMMAND_GET_BATTERY_VOLTAGE, nullptr,0);
    //navHandler(Cmds);
    sinceSensorRead = 0;
  }

  if (sinceLedToggle-millis() > LED_BLINK_INTERVAL) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
    sinceLedToggle = 0;
  }
    */
  /*if (sinceThrottle-millis() > THROTTLE_TIMEOUT) {
    stopMotors();
  }*/
    /*
  if (sinceMC-millis() > MOTOR_CONTROL_INTERVAL) {
    // Loop through motors to get and set their velocity
    //rover->updateWheelsVelocity();
    sinceMC = 0;
  }
    */
  /*
  Check if the message is available to be sent
  If available, send the message to be read and pop it out of the message queue
  If the message is unavailable, the message isn't removed from the queue
q  */
  commandCenter->sendMessage();

}


void attachMotors(){

    Motor::attachMotor(FRONT_RIGHT,M1_FR_DIR,M1_FR_PWM,GEAR_RATIO);
    Motor::attachMotor(MIDDLE_RIGHT,M2_MR_DIR,M2_MR_PWM,GEAR_RATIO);
    Motor::attachMotor(REAR_RIGHT,M3_RR_DIR,M3_RR_PWM,GEAR_RATIO);

    Motor::attachMotor(FRONT_LEFT,M4_FL_DIR,M4_FL_PWM,GEAR_RATIO);
    Motor::attachMotor(MIDDLE_LEFT,M5_ML_DIR,M5_ML_PWM,GEAR_RATIO);
    Motor::attachMotor(REAR_LEFT,M6_RL_DIR,M6_RL_PWM,GEAR_RATIO);


}
void initPidControllers(){
    Motor::initPidController(FRONT_RIGHT,14.1,0.282,40.625);
    Motor::initPidController(MIDDLE_RIGHT,14.1,0.282,40.625);
    Motor::initPidController(REAR_RIGHT,14.1,0.282,40.625);
    Motor::initPidController(FRONT_LEFT,14.1,0.282,40.625);
    Motor::initPidController(MIDDLE_LEFT,14.1,0.282,40.625);
    Motor::initPidController(REAR_LEFT,14.1,0.282,40.625);
}
void attachEncoders(){

    Motor::attachEncoder(FRONT_RIGHT,M1_FR_A,M1_FR_B,PULSES_PER_REV,InterruptHandler::RightFrontMotorInterruptHandler);
    Motor::attachEncoder(MIDDLE_RIGHT,M2_MR_A,M2_MR_B,PULSES_PER_REV,InterruptHandler::RightMiddleMotorInterruptHandler);
    Motor::attachEncoder(REAR_RIGHT,M3_RR_A,M3_RR_B,PULSES_PER_REV,InterruptHandler::RightBackMotorInterruptHandler);

    Motor::attachEncoder(FRONT_LEFT,M4_FL_A,M4_FL_B,PULSES_PER_REV,InterruptHandler::LeftFrontMotorInterruptHandler);
    Motor::attachEncoder(MIDDLE_LEFT,M5_ML_A,M5_ML_B,PULSES_PER_REV,InterruptHandler::LeftMiddleMotorInterruptHandler);
    Motor::attachEncoder(REAR_LEFT,M6_RL_A,M6_RL_B,PULSES_PER_REV,InterruptHandler::LeftBackMotorInterruptHandler);
}
void attachServos(){
    Rover::attachServo(FRONT_BASE_SERVO,FB_SERVO);
    Rover::attachServo(FRONT_SIDE_SERVO,FS_SERVO);
    Rover::attachServo(REAR_SIDE_SERVO,RS_SERVO);
    Rover::attachServo(REAR_BASE_SERVO,RB_SERVO);
}
void writeServoDefaultValues(){
    Rover::writeToServo(FRONT_BASE_SERVO,FRONT_BASE_DEFAULT_PWM);
    Rover::writeToServo(FRONT_SIDE_SERVO,SERVO_STOP);
    Rover::writeToServo(REAR_BASE_SERVO,REAR_BASE_DEFAULT_PWM);
    Rover::writeToServo(REAR_SIDE_SERVO,SERVO_STOP);

}
void initSerialCommunications(void) {
  // Create serial connection with teensy pins 0 and 1
  commandCenter->startSerial(TX_TEENSY_3_6_PIN, RX_TEENSY_3_6_PIN, ENABLE_PIN, TRANSMIT_PIN);

  // initialize serial communications at 115200 bps:
    //Serial.begin(SERIAL_BAUD); // switched from 9600 as suggested to conform with the given gps library
 // Serial.setTimeout(SERIAL_TIMEOUT);

}


void WheelsCommandCenter::enableMotors(uint8_t turnMotorOn) {
    //rover->enableAllMotors( (bool) turnMotorOn);
}

void WheelsCommandCenter::stopMotors() {
    //stopMotors();
}

void WheelsCommandCenter::closeMotorsLoop() {
    //rover->closeAllMotorLoop();
    Rover::closeLoop();
}

void WheelsCommandCenter::openMotorsLoop() {
    //rover->openAllMotorLoop();
    Rover::openLoop();
}

void WheelsCommandCenter::toggleJoystick(uint8_t turnJoystickOn) {

}

void WheelsCommandCenter::toggleGps(uint8_t turnGpsOn) {

}

void WheelsCommandCenter::toggleEncoder(uint8_t turnEncOn) {

}

void WheelsCommandCenter::toggleAcceleration(uint8_t turnAccelOn) {

}

void WheelsCommandCenter::getRoverStatus() {

}

void WheelsCommandCenter::moveRover(int8_t roverThrottle, int8_t roverSteering) {
    Rover::steerRover(roverThrottle,roverSteering);
}

void WheelsCommandCenter::moveWheel(uint8_t wheelNumber, int16_t wheelPWM) {

    Rover::moveWheel((MotorNames)wheelNumber,wheelPWM);

}

void WheelsCommandCenter::getLinearVelocity(void) {

}

void WheelsCommandCenter::getRotationalVelocity(void) {

}

void WheelsCommandCenter::getCurrentVelocity(void) {

}

void WheelsCommandCenter::getDesiredVelocity() {

}


void WheelsCommandCenter::pingWheels(void) {
    // CommandID set to 69 for ping
    internal_comms::Message* message = commandCenter->createMessage(COMMAND_WHEELS_PING, 0, nullptr);
    commandCenter->sendMessage(*message);
}

void WheelsCommandCenter::getBatteryVoltage() {
    //convert to 3.3V reference from analog values (3.3/1023=0.003225806)
    auto vsense = (float)analogRead(V_SENSE_PIN) * 0.003225806;

    float vbatt = vsense * 6.0;
    auto* buffer = (uint8_t *) malloc(4);


    float2bytes(buffer,vbatt);


    internal_comms::Message* message = commandCenter->createMessage(
            COMMAND_GET_BATTERY_VOLTAGE, sizeof(buffer), buffer);
    commandCenter->sendMessage(*message);
    free(buffer);
}
