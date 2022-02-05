#include "DcMotor.h"
namespace Motor {
    DcMotorState motorList[6];

    void attachEncoder(const MotorNames& motorID, const uint8_t& encoderPinA, const uint8_t& encoderPinB, const float& encoder_res,
                       void (*handler)(void)) {

        //->encoderResolution= encoder_res;
        //motorList[motorID].encoder_resolution_reciprocal = 1 / encoder_res;

        pinMode(encoderPinA, INPUT_PULLUP);
        pinMode(encoderPinB, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(encoderPinA), handler, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPinB), handler, CHANGE);
    }

    void attachMotor(const MotorNames &motorID, const uint8_t& dirPin,const uint8_t& pwmPin) {
        motorList[motorID] = {};
        motorList[motorID].dir_pin = dirPin;
        motorList[motorID].pwm_pin = pwmPin;
        motorList[motorID].id = motorID;

        motorList[motorID].desired_velocity = 0;
        motorList[motorID].current_velocity = 0;

        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    void stop(const MotorNames &motorID){
        analogWrite(motorList[motorID].pwm_pin,0);
    }
    void updateDesiredMotorVelocity(const MotorNames &motorID, const uint8_t &desired_direction,
                                    const uint8_t &desired_velocity) {
        auto &motor = motorList[motorID];


        // Prevent a new high speed in the other direction. To overcome this, first decelerate to 0 and then switch direction.
        if( (desired_direction != motor.desired_direction) && ( motor.current_velocity > 60) ){
            motor.desired_velocity = 0;
        }
        else {
            motor.desired_direction = desired_direction;
            motor.desired_velocity = desired_velocity;
        }

        applyDesiredMotorVelocity(motorID);
    }
    void applyDesiredMotorVelocity(const MotorNames &motorID) {

        auto &motor = motorList[motorID];

    }
}

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

void InterruptHandler::RightFrontMotorInterruptHandler(){
    RIGHT_FRONT_MOTOR_ENCODER_COUNT++;
    RIGHT_FRONT_MOTOR_DT = micros() - RIGHT_FRONT_MOTOR_PREV_DT;
    RIGHT_FRONT_MOTOR_PREV_DT = micros();
}
void InterruptHandler::RightMiddleMotorInterruptHandler(){
    RIGHT_MIDDLE_MOTOR_ENCODER_COUNT++;
    RIGHT_MIDDLE_MOTOR_DT = micros() - RIGHT_MIDDLE_MOTOR_PREV_DT;
    RIGHT_MIDDLE_MOTOR_PREV_DT = micros();
}
 void InterruptHandler::RightBackMotorInterruptHandler(){
    RIGHT_BACK_MOTOR_ENCODER_COUNT++;
    RIGHT_BACK_MOTOR_DT = micros() - RIGHT_BACK_MOTOR_PREV_DT;
    RIGHT_BACK_MOTOR_PREV_DT = micros();
}
 void InterruptHandler::LeftFrontMotorInterruptHandler(){
    LEFT_FRONT_MOTOR_ENCODER_COUNT++;
    LEFT_FRONT_MOTOR_DT = micros() - LEFT_FRONT_MOTOR_PREV_DT;
    LEFT_FRONT_MOTOR_PREV_DT = micros();
}
 void InterruptHandler::LeftMiddleMotorInterruptHandler(){
    LEFT_MIDDLE_MOTOR_ENCODER_COUNT++;
    LEFT_MIDDLE_MOTOR_DT = micros() - LEFT_MIDDLE_MOTOR_PREV_DT;
    LEFT_MIDDLE_MOTOR_PREV_DT = micros();
}
void InterruptHandler::LeftBackMotorInterruptHandler(){
    LEFT_BACK_MOTOR_ENCODER_COUNT++;
    LEFT_BACK_MOTOR_DT = micros() - LEFT_BACK_MOTOR_PREV_DT;
    LEFT_BACK_MOTOR_PREV_DT = micros();
}
uint32_t InterruptHandler::getEncoderCount(MotorNames name){
    switch (name) {
        case FRONT_RIGHT:
            return RIGHT_FRONT_MOTOR_ENCODER_COUNT;
        case MIDDLE_RIGHT:
            return RIGHT_MIDDLE_MOTOR_ENCODER_COUNT;
        case REAR_RIGHT:
            return RIGHT_BACK_MOTOR_ENCODER_COUNT;
        case FRONT_LEFT:
            return LEFT_FRONT_MOTOR_ENCODER_COUNT;
        case MIDDLE_LEFT:
            return LEFT_MIDDLE_MOTOR_ENCODER_COUNT;
        case REAR_LEFT:
            return LEFT_BACK_MOTOR_ENCODER_COUNT;
}
}
uint32_t InterruptHandler::getMotorDt(MotorNames name){
    switch (name) {
        case FRONT_RIGHT:
            return RIGHT_FRONT_MOTOR_DT;
        case MIDDLE_RIGHT:
            return RIGHT_MIDDLE_MOTOR_DT;
        case REAR_RIGHT:
            return RIGHT_BACK_MOTOR_DT;
        case FRONT_LEFT:
            return LEFT_FRONT_MOTOR_DT;
        case MIDDLE_LEFT:
            return LEFT_MIDDLE_MOTOR_DT;
        case REAR_LEFT:
            return LEFT_BACK_MOTOR_DT;

    }
}
void InterruptHandler::reset(MotorNames name) {
    switch (name) {
        case FRONT_RIGHT: {
            RIGHT_FRONT_MOTOR_DT = 0;
            RIGHT_FRONT_MOTOR_ENCODER_COUNT = 0;
            RIGHT_FRONT_MOTOR_PREV_DT = 0;
            break;
        }
        case MIDDLE_RIGHT: {
            RIGHT_MIDDLE_MOTOR_DT = 0;
            RIGHT_MIDDLE_MOTOR_ENCODER_COUNT = 0;
            RIGHT_MIDDLE_MOTOR_PREV_DT = 0;
            break;
        }
        case REAR_RIGHT: {
            RIGHT_BACK_MOTOR_DT = 0;
            RIGHT_BACK_MOTOR_ENCODER_COUNT = 0;
            RIGHT_BACK_MOTOR_PREV_DT=0;
        } break;

        case FRONT_LEFT: {
            LEFT_FRONT_MOTOR_DT = 0;
            LEFT_FRONT_MOTOR_ENCODER_COUNT = 0;
            LEFT_FRONT_MOTOR_PREV_DT = 0;
            break;
        }
        case MIDDLE_LEFT:{
            LEFT_MIDDLE_MOTOR_ENCODER_COUNT = 0;
            LEFT_MIDDLE_MOTOR_DT = 0;
            LEFT_MIDDLE_MOTOR_PREV_DT = 0;
            break;
        }

        case REAR_LEFT: {
            LEFT_BACK_MOTOR_ENCODER_COUNT = 0;
            LEFT_BACK_MOTOR_DT = 0;
            LEFT_BACK_MOTOR_PREV_DT = 0;
            break;
        }
    }

}