#include "Rover.h"

namespace Rover {

    Servo servoList[6];

    SystemState systemStatus;
    RoverState roverState;

    void moveWheel(const MotorNames &motorID, const int16_t& wheelPWM) {
        const auto &direction = (wheelPWM < 0) ? CW : CCW;

        //this->isSteering = 0; // From Globals.h

        Motor::updateDesiredMotorVelocity(motorID, direction, abs(wheelPWM));
        Motor::applyDesiredMotorVelocity(motorID);
    }
    void steerRover(const int8_t& throttle, const int8_t& steering) {

        float multiplier = mapFloat(abs(steering), 0, MAX_INPUT_VALUE, 1, -1);
        float leadingSideAbs = mapFloat(abs(throttle), 0, MAX_INPUT_VALUE, 0, roverState.max_output_signal);
        float trailingSideAbs = leadingSideAbs * multiplier;

        int8_t dir = 1;
        if (throttle >= 0) dir = CCW;
        else if (throttle < 0) dir = CW;

        int16_t desiredVelocityRight;
        int16_t desiredVelocityLeft;

        motor_direction leftMotorDirection;
        motor_direction rightMotorDirection;

        if (steering < 0) { // turning left
            desiredVelocityRight = static_cast<int16_t>( leadingSideAbs * dir);
            desiredVelocityLeft = static_cast<int16_t>( trailingSideAbs * dir);
        } else { // turning right
            desiredVelocityRight = static_cast<int16_t>(trailingSideAbs * dir);
            desiredVelocityLeft = static_cast<int16_t>(leadingSideAbs * dir);
        }

        if (desiredVelocityLeft > 0)
            leftMotorDirection = CCW;
        else
            leftMotorDirection = CW;

        if (desiredVelocityRight < 0)
            rightMotorDirection = CCW;
        else
            rightMotorDirection = CW;

        Motor::updateDesiredMotorVelocity(FRONT_RIGHT, rightMotorDirection, abs(desiredVelocityRight));
        Motor::updateDesiredMotorVelocity(MIDDLE_RIGHT, rightMotorDirection, abs(desiredVelocityRight));
        Motor::updateDesiredMotorVelocity(REAR_RIGHT, rightMotorDirection, abs(desiredVelocityRight));
        Motor::updateDesiredMotorVelocity(FRONT_LEFT, leftMotorDirection, abs(desiredVelocityLeft));
        Motor::updateDesiredMotorVelocity(MIDDLE_LEFT, leftMotorDirection, abs(desiredVelocityLeft));
        Motor::updateDesiredMotorVelocity(REAR_LEFT, leftMotorDirection, abs(desiredVelocityLeft));

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

    void stopMotors() {
        //steerRover(roverState, 0, 0);
    }

    void closeLoop() {

        roverState.max_output_signal = MAX_RPM_VALUE;
        roverState.min_output_signal = MIN_RPM_VALUE;

        if (systemStatus.are_motors_enabled) {
            stopMotors();
        }
        systemStatus.is_open_loop = false;

    }

    void openLoop() {

        roverState.max_output_signal = MAX_PWM_VALUE;
        roverState.min_output_signal = MIN_PWM_VALUE;

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
