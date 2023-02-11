#include "DcMotor.h"

namespace Motor {
    DcMotorState motorList[6];

    void attachMotor(const MotorNames &motorID, const uint8_t& dirPin,const uint8_t& pwmPin) {
        motorList[motorID] = {};
        motorList[motorID].dir_pin = dirPin;
        motorList[motorID].pwm_pin = pwmPin;
        motorList[motorID].id = motorID;

        motorList[motorID].desired_velocity = 0;
        motorList[motorID].current_velocity = 0;

        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        
        analogWriteFrequency(pwmPin, 18000);
        if (pwmPin == 6) {
            analogWriteFrequency(30, 200);
            analogWriteResolution(16);
        }
    }

    void stop(const MotorNames &motorID){
        if (motorID == MIDDLE_RIGHT) {
            analogWrite(motorList[motorID].pwm_pin, 19661);
        } else {
            analogWrite(motorList[motorID].pwm_pin, 0);
        }
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

        digitalWrite(motor.dir_pin, motor.desired_direction);

        if (motor.id == MIDDLE_RIGHT) {

            int newPWM = motor.desired_velocity;
            if (motor.desired_direction == 1) {
                newPWM = 19661 - newPWM*7;
            } else {
                newPWM = 19661 + newPWM*7;
            }

            analogWrite(30, newPWM);
        } 

        analogWrite(motor.pwm_pin, motor.current_velocity*256);
    }
}
