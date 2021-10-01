#include "Rover.h"

namespace Rover {

    Servo servoList[6];

    SystemState systemStatus;
    RoverState roverState;

    void moveWheel(const MotorNames &motorID,const motor_direction& direction,const int8_t& wheelPWM) {

        Motor::updateDesiredMotorVelocity(motorID, direction, wheelPWM);
    }
    void stopMotors(){
        for(auto& motor : Motor::motorList ){
            analogWrite(motor.pwm_pin,0);
        }
    }

    void steerRover(const uint8_t& throttle_direction, const uint8_t & throttle, const uint8_t & steer_direction,const uint8_t & steering) {
        //TODO : Figure out how the hell this multiplier works. WHY IS THE MAX LOWER THAN THE MIN LOL.
        //  WHY 49 ???

        float multiplier = mapFloat(abs(steering), 0, 255, 0, 1);
        if(steering == 0) {
            multiplier = 1;
        }
        //TODO : Figure out why the hell we need to map this
        //float leadingSideAbs = mapFloat(abs(throttle), 0, MAX_INPUT_VALUE, 0, roverState.max_output_signal);
        float leadingSideAbs = throttle;

        float trailingSideAbs = leadingSideAbs * multiplier;


        uint8_t desiredVelocityRight;
        uint8_t desiredVelocityLeft;

        motor_direction leftMotorDirection;
        motor_direction rightMotorDirection;

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
        int current_motor = 0;


        while(current_motor <= 5) {
            Motor::updateDesiredMotorVelocity((MotorNames) current_motor, rightMotorDirection,desiredVelocityRight);
            //delay(25);

            Motor::updateDesiredMotorVelocity((MotorNames) (5 - current_motor), leftMotorDirection,desiredVelocityLeft);

            current_motor++;
        }
        systemStatus.last_throttle = millis();
    }
    void decelerateRover(){
        //if(systemStatus.is_deccelerating){

            for(auto& motor : Motor::motorList){

                if(motor.desired_velocity <= 50){
                    break;
                }
                uint8_t new_velocity = motor.desired_velocity - 1;

                if(motor.desired_velocity < 10) {
                    motor.desired_velocity = 0;
//                    systemStatus.is_deccelerating = false;
//                    break;
                }
                Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,new_velocity);

                delay(2);

            }
      //  }

//            auto motor = Motor::motorList[i];
//            Serial.write(motor.id);
//            if(motor.desired_velocity >= 0){
//                uint8_t new_velocity = motor.desired_velocity - 10;
//                Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,new_velocity);
//            }
            //
//            uint8_t new_velocity = motor.desired_velocity - 10;
//
//            Motor::updateDesiredMotorVelocity((MotorNames)i,motor.desired_direction,new_velocity);

//            if(motor.desired_velocity >= 0){
//                uint8_t new_velocity = motor.desired_velocity - 10;
//                Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,new_velocity);
//            }

        //
        /*
        for(auto& motor : Motor::motorList){
            Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,0);
//            if(motor.desired_velocity >= 0){
//                uint8_t new_velocity = motor.desired_velocity - 10;
//                Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,new_velocity);
//            }
        }*/
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
        for(auto& motor : Motor::motorList){
            motor.is_open_loop = true;
        }
        systemStatus.is_open_loop = true;
    }


}
