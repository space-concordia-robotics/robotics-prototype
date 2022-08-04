//
// Edited by Michael on 2021-03-20.
//

// USB : Debug, UART : Production

#include "APA102.h"
#include <cmath>
#include <cstdint>
#include "Navigation.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#include "ArduinoBlue.h"
#include "Rover.h"
#include "commands/WheelsCommandCenter.h"
//
#define DEBUG

#ifndef DEBUG // in ../internal_comms/src/CommandCenter.cpp
#define Serial Serial1
#endif

// Pins for Serial
const uint8_t TX_TEENSY_3_6_PIN = 1;
const uint8_t RX_TEENSY_3_6_PIN = 0;
const uint8_t ENABLE_PIN = 15;
const uint8_t TRANSMIT_PIN = 14;
const uint8_t LIGHT_MOSI = 21;
const uint8_t LIGHT_CLOCK = 20;


// To read commands from wheelscommandcenter
internal_comms::CommandCenter* commandCenter = new WheelsCommandCenter();

// Wheel command methods from WheelsCommandCenter.cpp
// https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=963483371
void attachMotors();
void attachServos();
void attachEncoders();
void initPidControllers();
void writeServoDefaultValues();

// Messages to send back to OBC from wheel Teensy
// https://docs.google.com/spreadsheets/d/1bE3h0ZCqPAUhW6Gn6G0fKEoOPdopGTZnmmWK1VuVurI/edit#gid=963483371

void pollBlink();

void blink(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN,LOW);
    delay(500);
}
APA102 light(20, 21, 20);

void setup() {
    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(V_SENSE_PIN, INPUT);
    pinMode(LIGHT_CLOCK, OUTPUT);
    pinMode(LIGHT_MOSI, OUTPUT);

    blink();

    commandCenter->startSerial(TX_TEENSY_3_6_PIN, RX_TEENSY_3_6_PIN, ENABLE_PIN, TRANSMIT_PIN);

    attachMotors();
    attachEncoders();
    initPidControllers();

    attachServos();
    writeServoDefaultValues();

    Rover::openLoop();
    Rover::systemStatus.is_throttle_timeout_enabled = true;

    Rover::systemStatus.has_moved = false;
    Rover::systemStatus.is_passive_rover_feedback_enabled = false;
}

void loop() {
    if(Serial.available() > 0) {
        commandCenter->readCommand();
    }
    commandCenter->sendMessage();
    pollBlink();

    if(Rover::systemStatus.is_passive_rover_feedback_enabled){
        Rover::calculateRoverVelocity();
        commandCenter->executeCommand(COMMAND_GET_BATTERY_VOLTAGE, nullptr,0);

    }
//  if (sinceSensorRead-millis() > SENSOR_READ_INTERVAL) {
//
//      Rover::calculateRoverVelocity();
//      commandCenter->executeCommand(COMMAND_GET_BATTERY_VOLTAGE, nullptr,0);
//
//      commandCenter->executeCommand(COMMAND_GET_LINEAR_VELOCITY, nullptr,0);
//      commandCenter->executeCommand(COMMAND_GET_ROTATIONAL_VELOCITY, nullptr,0);
//
//
//      //      navHandler(Cmds);
//    sinceSensorRead = 0;
//  }
//
    if (Rover::systemStatus.is_throttle_timeout_enabled &&
    ( (millis() - Rover::systemStatus.last_throttle) > THROTTLE_TIMEOUT)) {
        Rover::decelerateRover();
        //Rover::stopMotors();
    }

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


void WheelsCommandCenter::enableMotors(uint8_t turnMotorOn) {
    //rover->enableAllMotors( (bool) turnMotorOn);
}

void WheelsCommandCenter::stopMotors() {
    Rover::stopMotors();
    Rover::systemStatus.last_throttle = millis();
}

void WheelsCommandCenter::closeMotorsLoop() {
    Rover::closeLoop();
}

void WheelsCommandCenter::openMotorsLoop() {
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

void WheelsCommandCenter::moveRover(const uint8_t & throttle_dir,const uint8_t & throttle, const uint8_t& steering_dir,const uint8_t& steering) {

    Rover::steerRover(throttle_dir,throttle,steering_dir,steering);
}

void WheelsCommandCenter::moveWheel(const uint8_t& wheelNumber,const uint8_t& direction,const uint8_t& velocity) {

    Rover::moveWheel((MotorNames)wheelNumber,(motor_direction)direction,velocity);

}
void WheelsCommandCenter::getLinearVelocity(void) {
    const float linear_velocity = Rover::roverState.linear_velocity;
    uint8_t buffer[4];
    float2bytes(buffer,linear_velocity);
    internal_comms::Message* message = commandCenter->createMessage(
            COMMAND_GET_BATTERY_VOLTAGE, sizeof(buffer), buffer);
    commandCenter->sendMessage(*message);
}

void WheelsCommandCenter::getRotationalVelocity(void) {
    const float rotational_velocity = Rover::roverState.rotational_velocity;
    uint8_t buffer[4];
    float2bytes(buffer,rotational_velocity);
    internal_comms::Message* message = commandCenter->createMessage(
            COMMAND_GET_BATTERY_VOLTAGE, sizeof(buffer), buffer);
    commandCenter->sendMessage(*message);
}

void WheelsCommandCenter::getMotorVelocity(const uint8_t& wheelNumber) {

}

void WheelsCommandCenter::getMotorDesiredVelocity(const uint8_t& wheelNumber) {

}

void WheelsCommandCenter::pingWheels(void) {
    internal_comms::Message* message = commandCenter->createMessage(1, 0, nullptr);
    commandCenter->sendMessage(*message);
}

// timestamp when the last rising/falling edge of the blink cycle happened
unsigned int timeBlinkUpdated = 0;
bool lightOn = false;
bool blinking = false;

void pollBlink() {
    if (blinking && (millis() - timeBlinkUpdated) > 500) {
        if (lightOn) {
            light.setAll(0, 0, 0, 0);
        } else {
            light.setAll(20, 0, 0, 1);
        }
        light.send();
        lightOn = !lightOn;
        timeBlinkUpdated = millis();
    }
}

void WheelsCommandCenter::handleBlink(uint8_t on) {
    if (on) {
        light.setAll(20, 0, 0, 1);
        timeBlinkUpdated = millis();
    } else {
        light.setAll(0, 0, 0, 0);
    }
    lightOn = (bool)on;
    blinking = lightOn;
    light.send();
}

void WheelsCommandCenter::getBatteryVoltage() {

    //convert to 3.3V reference from analog values (3.3/1023=0.003225806)
    auto vsense = (float)analogRead(V_SENSE_PIN) * 0.003225806;

    float vbatt = vsense * 6.0;
    auto* buffer = (uint8_t *) malloc(4);
    //uint8_t* buffer;

    float2bytes(buffer,vbatt);


    internal_comms::Message* message = commandCenter->createMessage(
            COMMAND_GET_BATTERY_VOLTAGE, sizeof(buffer), buffer);
    commandCenter->sendMessage(*message);
    free(buffer);
}
