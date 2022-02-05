#include <cstdint>
#include <SoftwareSerial.h>
#include <Servo.h>
#include "Rover.h"
#include "commands/WheelsCommandCenter.h"

//#define DEBUG

#ifndef DEBUG // in ../internal_comms/src/CommandCenter.cpp
#define Serial Serial1
#endif

// Pins for Serial
const uint8_t TX_TEENSY_3_6_PIN = 1;
const uint8_t RX_TEENSY_3_6_PIN = 0;
const uint8_t ENABLE_PIN = 15;
const uint8_t TRANSMIT_PIN = 14;

internal_comms::CommandCenter* commandCenter = new WheelsCommandCenter();

void attachMotors();
void attachEncoders();
void attachServos();

void blink(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
}

void setup() {

    pinMode(LED_BUILTIN,OUTPUT);
    pinMode(V_SENSE_PIN, INPUT);

    blink();

    commandCenter->startSerial(TX_TEENSY_3_6_PIN, RX_TEENSY_3_6_PIN, ENABLE_PIN, TRANSMIT_PIN);


    attachMotors();
    attachEncoders();
    attachServos();

    // Here different parameters of how the system should behave can be set
    Rover::systemStatus.is_throttle_timeout_enabled = true;
    Rover::systemStatus.is_passive_rover_feedback_enabled = false;
}

void loop() {
    if(Serial.available() > 0) {
        commandCenter->readCommand();
    }
    // Sort of un-used at the moment, but this can periodically transmit messages to the OBC which can define the status
    // of the rover.

    if(Rover::systemStatus.is_passive_rover_feedback_enabled){
        Rover::calculateRoverVelocity();
        commandCenter->executeCommand(COMMAND_GET_BATTERY_VOLTAGE, nullptr,0);

    }
    // The rover will update its own current velocity according to a time interval which is measured from when it
    // last moved (in moveRover() )
    if( (millis() - Rover::systemStatus.last_velocity_adjustment) > ACCELERATION_RATE){
        Rover::updateWheelVelocities();
    }
    // If the rover has not been commanded to move which a time interval, it must  be stopped (security risk).
    if (Rover::systemStatus.is_throttle_timeout_enabled &&
    ( (millis() - Rover::systemStatus.last_move) > ROVER_MOVE_TIMEOUT)) {
        Rover::decelerateRover();
    }

    // In case there are any messages queued in the transmit buffer, they should be sent.
    commandCenter->sendMessage();
}

void attachServos(){
    Rover::attachServo(CENTER_FRONT_1_SERVO,CF_1_SERVO);
    Rover::attachServo( CENTER_FRONT_2_SERVO,CF_2_SERVO);
    Rover::attachServo(CENTER_BACK_1_SERVO,CB_1_SERVO);
    Rover::attachServo(CENTER_BACK_2_SERVO,CB_2_SERVO);
}

void attachMotors(){
    Motor::attachMotor(FRONT_RIGHT,M1_FR_DIR,M1_FR_PWM);
    Motor::attachMotor(MIDDLE_RIGHT,M2_MR_DIR,M2_MR_PWM);
    Motor::attachMotor(REAR_RIGHT,M3_RR_DIR,M3_RR_PWM);

    Motor::attachMotor(FRONT_LEFT,M4_FL_DIR,M4_FL_PWM);
    Motor::attachMotor(MIDDLE_LEFT,M5_ML_DIR,M5_ML_PWM);
    Motor::attachMotor(REAR_LEFT,M6_RL_DIR,M6_RL_PWM);
}
// These are not currently operational
void attachEncoders(){
    Motor::attachEncoder(FRONT_RIGHT,M1_FR_A,M1_FR_B,PULSES_PER_REV,InterruptHandler::RightFrontMotorInterruptHandler);
    Motor::attachEncoder(MIDDLE_RIGHT,M2_MR_A,M2_MR_B,PULSES_PER_REV,InterruptHandler::RightMiddleMotorInterruptHandler);
    Motor::attachEncoder(REAR_RIGHT,M3_RR_A,M3_RR_B,PULSES_PER_REV,InterruptHandler::RightBackMotorInterruptHandler);
    Motor::attachEncoder(FRONT_LEFT,M4_FL_A,M4_FL_B,PULSES_PER_REV,InterruptHandler::LeftFrontMotorInterruptHandler);
    Motor::attachEncoder(MIDDLE_LEFT,M5_ML_A,M5_ML_B,PULSES_PER_REV,InterruptHandler::LeftMiddleMotorInterruptHandler);
    Motor::attachEncoder(REAR_LEFT,M6_RL_A,M6_RL_B,PULSES_PER_REV,InterruptHandler::LeftBackMotorInterruptHandler);
}

void WheelsCommandCenter::stopMotors() {
    Rover::stopMotors();
}

void WheelsCommandCenter::moveRover(const float & linear_y,const float & omega_z) {

    Rover::moveRover(linear_y,omega_z);
}

void WheelsCommandCenter::moveServo(const uint8_t & servoID, const uint8_t & angle) {
    Rover::moveServo((ServoNames)servoID,angle);
}
void WheelsCommandCenter::moveWheel(const uint8_t& wheelNumber,const uint8_t& direction,const uint8_t& velocity) {

    Rover::moveWheel((MotorNames)wheelNumber,direction,velocity);

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

void WheelsCommandCenter::pingWheels(void) {
    internal_comms::Message* message = commandCenter->createMessage(1, 0, nullptr);
    commandCenter->sendMessage(*message);
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
