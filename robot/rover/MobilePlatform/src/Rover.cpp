#include "Rover.h"

#include <cmath>

namespace Rover {

    Servo servoList[6];

    SystemState systemStatus;
    RoverState roverState;

    void moveWheel(const MotorNames &motorID,uint8_t direction, uint8_t speed) {

        systemStatus.last_move = millis();
        Motor::updateDesiredMotorVelocity(motorID,direction,speed);
//        if(speed < 0){
//            Motor::updateDesiredMotorVelocity(motorID,0,abs(speed));
//        }
//        else{
//            Motor::updateDesiredMotorVelocity(motorID,1,abs(speed));
//        }

    }
    void stopMotors(){
        for(auto& motor : Motor::motorList ){
           if(motor.current_velocity > 0){
               Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,0);
           }
        }
    }
    void updateWheelVelocities(){
        systemStatus.last_velocity_adjustment = millis();

        for(auto& motor : Motor::motorList ){

          if(motor.current_velocity < motor.desired_velocity){
                motor.current_velocity += 1;
          }
          else if(motor.current_velocity > motor.desired_velocity){
                motor.current_velocity -= 1;
          }
          Motor::applyDesiredMotorVelocity(motor.id);
         }

    }

    FASTRUN void moveRover(const float & linear_y,const float& omega_z ){

        float slip_track = 1.2f;

        float right_wheels_velocity = linear_y - ( omega_z * slip_track * 0.5f);
        float left_wheels_velocity = linear_y + ( omega_z * slip_track * 0.5f);

        if(right_wheels_velocity >= 1.0){
            right_wheels_velocity = 1.0;
        }
        if(left_wheels_velocity >= 1.0){
            left_wheels_velocity = 1.0;
        }
        float r = (slip_track / 2.0f) * std::fabs(  (right_wheels_velocity +  left_wheels_velocity)/(right_wheels_velocity - left_wheels_velocity));

        float r_prime = slip_track/2.0f;

        //Serial.write((uint8_t)right_wheels_velocity);
        uint8_t right_motor_direction;
        uint8_t  left_motor_direction;

        //Point turn;
        if(r_prime >= r){
            if(linear_y == 0 && omega_z < 0){
                right_motor_direction = 1;
                left_motor_direction = 1;
            }
            else if(linear_y == 0 && omega_z > 0){
                right_motor_direction = 0;
                left_motor_direction = 0;
            }
            else {
                right_motor_direction = std::signbit(linear_y);
                left_motor_direction = std::signbit(linear_y);
            }

            }
        //Normal turn
        else{
            left_motor_direction = !std::signbit(linear_y);
            right_motor_direction = std::signbit(linear_y);
        }
        int current_motor = 0;

        auto right_wheels_pwm_velocity = (uint8_t ) mapFloat(std::fabs(right_wheels_velocity),0,1.0,0,255);
        auto left_wheels_pwm_velocity = (uint8_t ) mapFloat(std::fabs(left_wheels_velocity),0,1.0,0,255);


        while(current_motor <= 5) {
            Motor::updateDesiredMotorVelocity((MotorNames) current_motor, right_motor_direction,(uint8_t )right_wheels_pwm_velocity);

            Motor::updateDesiredMotorVelocity((MotorNames) (5 - current_motor), left_motor_direction,(uint8_t )left_wheels_pwm_velocity);

            current_motor++;
        }
        systemStatus.last_move = millis();
    }


    void decelerateRover(){
        for(auto& motor : Motor::motorList){
            motor.desired_velocity = 0;
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

}
