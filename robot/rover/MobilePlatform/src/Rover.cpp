#include "Rover.h"

namespace Rover {

    Servo servos[4];
    SystemState systemStatus;
    RoverState roverState;

    void moveWheel(const MotorNames &motorID,const uint8_t& direction,const uint8_t& speed) {

        systemStatus.last_move = millis();
        Motor::updateDesiredMotorVelocity(motorID,direction,speed);
    }
    void stopMotors(){
        for(auto& motor : Motor::motorList ){
           if(motor.current_velocity > 0){
               Motor::updateDesiredMotorVelocity(motor.id,motor.desired_direction,0);
           }
        }
    }
    // Update current velocity of the wheel, based on whether it has reached the target velocity. This effect
    // is how acceleration is done.
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
    // This is a fixed value that represents the diameter of the point turn
    FASTRUN void moveRover(const float & linear_y,const float& omega_z ){

        float slip_track = 1.2f;

        // This can be derived from the equation in the paper.
        float right_wheels_velocity = linear_y - ( omega_z * slip_track * 0.5f);
        float left_wheels_velocity = linear_y + ( omega_z * slip_track * 0.5f);

        if(right_wheels_velocity >= 1.0){
            right_wheels_velocity = 1.0;
        }
        if(left_wheels_velocity >= 1.0){
            left_wheels_velocity = 1.0;
        }
        // Taken from the paper
        float r = (slip_track / 2.0f) * std::fabs(  (right_wheels_velocity +  left_wheels_velocity)/(right_wheels_velocity - left_wheels_velocity));

        float r_prime = slip_track/2.0f;

        uint8_t right_motor_direction;
        uint8_t left_motor_direction;

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
                // Since the velocity is calculated as a float, we can look at the sign bit (IEEE 754 representation) to figure out the sign of the number
                // Also, this case is only when point turning, or when both wheels go in the same direction.
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

        // Go through each motor and update the speed, but actually symmetrically alternate from the far corners which motor is selected,
        // which makes for easier for the rover to start moving

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

    void attachServo(const ServoNames& servoID,const uint8_t& pin) {
        servos[servoID].attach(pin);
        moveServo(servoID,0);

    }

    // Angle is given by 0-180, if the value is > 180 than it is clipped to 180
    void moveServo(const ServoNames& servo_id, const uint8_t & value) {
        servos[servo_id].write(value);
    }
}
