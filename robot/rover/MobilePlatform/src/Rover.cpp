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
           if(motor.desired_velocity <= 0){
                continue;
            };
            Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,0);
        }
    }
    void updateWheelVelocities(){
        for(auto& motor : Motor::motorList ){

          if(motor.current_velocity < motor.desired_velocity){
                motor.current_velocity += 1;
          }
          else if(motor.current_velocity > motor.desired_velocity){
                motor.current_velocity -= 1;
          }
          else return;

          Motor::applyDesiredMotorVelocity(motor.id);
          systemStatus.last_velocity_adjustment = millis();
         }
    }
    /*
     * 0 255 0 0 : THRUST FORWARD
     * 1 255 0 0 : THRUST BACKWARD
     *
     * (0-1) 0 0 (0-255) : POINT TURN RIGHT
     * (0-1) 0 1 (0-255) : POINT TURN LEFT

     * 0 128 0 128 : FORWARD AND TURNING RIGHT
     * 0 128 1 128 : FORWARD AND TURNING LEFT
     *
     * 1 128 0 128 : BACKWARDS AND TURNING RIGHT
     * 0 128 0 128 : BACKWARDS AND TURNING LEFT
     */
    void moveRover(const uint8_t& throttle_direction, const uint8_t & throttle, const uint8_t & steer_direction,const uint8_t & steering){
        uint8_t right_motor_velocity;
        uint8_t left_motor_velocity;

        motor_direction right_motor_direction;
        motor_direction left_motor_direction;

        if(steering >= 250){
            right_motor_velocity = steering;
            left_motor_velocity = steering;

            right_motor_velocity = (motor_direction)steer_direction;
            left_motor_direction = (motor_direction)steer_direction;
        }
        else if(steering < 5){
            right_motor_velocity = throttle;
            left_motor_velocity = throttle;

            left_motor_direction = (motor_direction)throttle_direction;
            right_motor_direction = (motor_direction)(~throttle_direction);
        }
        else{
            float multiplier = mapFloat(abs(steering), 5, 250, 0, 1);

            auto trailing_motor_velocity = (uint8_t )(throttle * multiplier);
            left_motor_direction = (motor_direction)throttle_direction;
            right_motor_direction = (motor_direction)(~throttle_direction);

            if(steer_direction == RIGHT){
                left_motor_velocity = throttle ;
                right_motor_velocity = trailing_motor_velocity;
            }
            else{
                right_motor_velocity = throttle;
                left_motor_velocity= trailing_motor_velocity;
            }
        }

        int current_motor = 0;

        while(current_motor <= 5) {
            Motor::updateDesiredMotorVelocity((MotorNames) current_motor, right_motor_direction,right_motor_velocity);
            //delay(25);

            Motor::updateDesiredMotorVelocity((MotorNames) (5 - current_motor), left_motor_direction,left_motor_velocity);

            current_motor++;
        }
        systemStatus.last_move = millis();
    }

    void steerRover(const uint8_t& throttle_direction, const uint8_t & throttle, const uint8_t & steer_direction,const uint8_t & steering) {
        float multiplier = mapFloat(abs(steering), 0, 255, 0, 1);
        if(steering == 0) {
            multiplier = 1;
        }
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
        systemStatus.last_move = millis();
    }
    void decelerateRover(){

            for(auto& motor : Motor::motorList){

                if(motor.desired_velocity <= 0){
                    continue;
                }
                uint8_t new_velocity = motor.desired_velocity - 1;

                if(new_velocity < 5) {
                    new_velocity= 0;
                }
                Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,new_velocity);
                delay(2);
            }
      }

    void calculateRoverVelocity() {

        using namespace Motor;

        roverState.right_linear_velocity =
                (float) (motorList[FRONT_RIGHT].desired_direction * motorList[FRONT_RIGHT].actual_velocity +
                         motorList[MIDDLE_RIGHT].desired_direction * motorList[MIDDLE_RIGHT].actual_velocity +
                         motorList[REAR_RIGHT].desired_direction * motorList[REAR_RIGHT].actual_velocity) * radius *
                piRad;

        roverState.left_linear_velocity =
                (float) (motorList[FRONT_LEFT].desired_direction * motorList[FRONT_LEFT].actual_velocity +
                         motorList[MIDDLE_LEFT].desired_direction * motorList[MIDDLE_LEFT].actual_velocity +
                         motorList[REAR_LEFT].desired_direction * motorList[REAR_LEFT].actual_velocity) * radius *
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
