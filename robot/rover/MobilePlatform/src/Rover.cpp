#include "Rover.h"

namespace Rover {

    Servo servoList[6];

    SystemState systemStatus;
    RoverState roverState;

    void moveWheel(const MotorNames &motorID,const motor_direction& direction,const int8_t& wheelPWM) {
        //const auto &direction = (wheelPWM < 0) ? CW : CCW;

        //this->isSteering = 0; // From Globals.h

        Motor::updateDesiredMotorVelocity(motorID, direction, wheelPWM);
        //Motor::applyDesiredMotorVelocity(motorID);
    }
    void stopMotors(){
        for(auto& motor : Motor::motorList ){
            Serial.write(motor.pwm_pin);
            analogWrite(motor.pwm_pin,0);
        }
    }


    void steerRover(const uint8_t& throttle_direction, const uint8_t & throttle, const uint8_t & steer_direction,const uint8_t & steering) {
        //TODO : Figure out how the hell this multiplier works. WHY IS THE MAX LOWER THAN THE MIN LOL.
        //  WHY 49 ???
        float multiplier = mapFloat(abs(steering), 0, MAX_INPUT_VALUE, 0, 1);



        //float multiplier = 0.1;
        if(steering == 0) {
            multiplier = 1;
        }
        //TODO : Figure out why the hell we need to map this
        //float leadingSideAbs = mapFloat(abs(throttle), 0, MAX_INPUT_VALUE, 0, roverState.max_output_signal);
        float leadingSideAbs = throttle;

        float trailingSideAbs = leadingSideAbs * multiplier;

//        int8_t dir = 1;
//        if (throttle >= 0) dir = CCW;
//        else if (throttle < 0) dir = CW;

        uint8_t desiredVelocityRight;

        uint8_t desiredVelocityLeft;

        motor_direction leftMotorDirection;
        motor_direction rightMotorDirection;

        Serial.write(steering);

        if(steer_direction == LEFT){
            desiredVelocityRight = (uint8_t)( leadingSideAbs );
            desiredVelocityLeft = (uint8_t)( trailingSideAbs );

        }
        else{
            desiredVelocityRight = (uint8_t)(trailingSideAbs );
            desiredVelocityLeft = (uint8_t)(leadingSideAbs);

        }
        if(throttle_direction == FORWARD){
            leftMotorDirection = CCW;
            rightMotorDirection = CW;
        }
        else{
            leftMotorDirection = CW;
            rightMotorDirection = CCW;
        }

        Motor::updateDesiredMotorVelocity(FRONT_RIGHT, rightMotorDirection, desiredVelocityRight);
        Motor::updateDesiredMotorVelocity(MIDDLE_RIGHT, rightMotorDirection,desiredVelocityRight);
        Motor::updateDesiredMotorVelocity(REAR_RIGHT, rightMotorDirection,desiredVelocityRight);
        Motor::updateDesiredMotorVelocity(FRONT_LEFT, leftMotorDirection, desiredVelocityLeft);
        Motor::updateDesiredMotorVelocity(MIDDLE_LEFT, leftMotorDirection,desiredVelocityLeft);
        Motor::updateDesiredMotorVelocity(REAR_LEFT, leftMotorDirection, desiredVelocityLeft);

        //this->sinceThrottle = 0;

    }
    void calculateRoverVelocity() {

        using namespace Motor;

        roverState.right_linear_velocity =
                (float) (motorList[FRONT_RIGHT].desired_direction * motorList[FRONT_RIGHT].current_velocity +
                         motorList[MIDDLE_RIGHT].desired_direction * motorList[MIDDLE_RIGHT].current_velocity +
                         motorList[REAR_RIGHT].desired_direction * motorList[REAR_RIGHT].current_velocity) * radius *
                piRad;

        roverState.left_linear_velocity =
                (float) (motorList[FRONT_LEFT].desired_direction * motorList[FRONT_LEFT].current_velocity +
                         motorList[MIDDLE_LEFT].desired_direction * motorList[MIDDLE_LEFT].current_velocity +
                         motorList[REAR_LEFT].desired_direction * motorList[REAR_LEFT].current_velocity) * radius *
                piRad;

        roverState.linear_velocity = (roverState.right_linear_velocity - roverState.left_linear_velocity) / 6;
        roverState.rotational_velocity = (roverState.left_linear_velocity + roverState.right_linear_velocity) / wheelBase;
    }

    void writeToServo(const ServoNames& servo_id, const int16_t& value) {
        servoList[servo_id].write(value);
    }

    void attachServo(const ServoNames& servoID,const uint8_t& pin) {

        servoList[servoID] = Servo();
        servoList[servoID].attach(pin);
    }

    void closeLoop() {

        roverState.max_output_signal = MAX_RPM_VALUE;
        roverState.min_output_signal = 0;

        if (systemStatus.are_motors_enabled) {
            stopMotors();
        }
        systemStatus.is_open_loop = false;

    }

    void openLoop() {

        roverState.max_output_signal = MAX_PWM_VALUE;
        roverState.min_output_signal = 0;

        if (systemStatus.are_motors_enabled) {
            stopMotors();
        }
        systemStatus.is_open_loop = true;
    }


}


/*//! !FB, @FS, #RB, $RS
void Rover::controlCameraMotors(ServoNames servoID, uint16_t angle) {
    servoList[servoID].write(angle);
}*/
