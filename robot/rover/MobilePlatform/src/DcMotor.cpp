#include "DcMotor.h"
namespace Motor {
    DcMotorState motorList[6];

    void attachEncoder(const MotorNames& motorID, const uint8_t& encoderPinA, const uint8_t& encoderPinB, const float& encoder_res,
                       void (*handler)(void)) {

        //->encoderResolution= encoder_res;
        motorList[motorID].encoder_resolution_reciprocal = 1 / encoder_res;


        pinMode(encoderPinA, INPUT_PULLUP);
        pinMode(encoderPinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(encoderPinA), handler, CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoderPinB), handler, CHANGE);
    }

    void attachMotor(const MotorNames &motorID, const uint8_t& dirPin,const uint8_t& pwmPin, const float& gearRatio) {
        //motorList[motorID] = {};
        motorList[motorID].dir_pin = dirPin;
        motorList[motorID].pwm_pin = pwmPin;

        motorList[motorID].gear_ratio = gearRatio;
        motorList[motorID].gear_ratio_reciprocal = 1 / gearRatio;

        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    void initPidController(const MotorNames &motorID, const float& kp, const float& ki, const float& kd) {

        motorList[motorID].pid_controller.kd = kd;
        motorList[motorID].pid_controller.kp = kp;
        motorList[motorID].pid_controller.ki = ki;

    }
    void stop(const MotorNames &motorID){
        analogWrite(motorList[motorID].pwm_pin,0);
    }
    void updateDesiredMotorVelocity(const MotorNames &motorID, const motor_direction &desired_direction,
                                    const int16_t &desired_velocity) {
        motorList[motorID].desired_velocity = desired_velocity;
        motorList[motorID].desired_direction = desired_direction;

        applyDesiredMotorVelocity(motorID);
    }

    void applyDesiredMotorVelocity(const MotorNames &motorID) {

        calculateCurrentVelocity(motorID);

        auto &motor = motorList[motorID];

        digitalWrite(motor.dir_pin,motor.desired_direction);

        if (motor.is_open_loop) {

            int16_t output_pwm = motor.desired_velocity;
            analogWrite(motor.pwm_pin, output_pwm);


        } else if (!motor.is_open_loop) {
            // THIS LOOKS WRONG
            // makes sure the speed is within the limits set in the pid during setup
            if (motor.desired_velocity > 30) {
                motor.desired_velocity = 30;
            } else if (motor.desired_velocity < 0) {
                motor.desired_velocity = 0;
            }
            int16_t output_pwm = updatePID(motor.pid_controller, motor.current_velocity, motor.desired_velocity);

//        Serial.print(output_pwm);
//        Serial.print(" ");

            analogWrite(motor.pwm_pin, abs(output_pwm));

        }
        //this -> desiredVelocity = output_pwm;

        /* Acceleration limiter */
//    if (accLimit) {
//        final_output_pwm = output_pwm;
//        dt2 = micros() - prevTime2;
//
//        acc = ((float) output_pwm - (float) prev_output_pwm) / (float) dt2;
//
//        if (abs(acc) > 0.00051) {  // 0.00051 it the acceleration limit to go from 0 to full speed in 0.5 seconds. adjust this value for desired tuning
//            output_pwm = ((acc < 0) ? -1 : 1) * 0.000151 * dt2 + prev_output_pwm;
//        }
//        prevTime2 = micros();
//        prev_output_pwm = output_pwm;
//        analogWrite(pwmPin, output_pwm);
//    }
//    else analogWrite(pwmPin, output_pwm);

//
    }


    void calculateCurrentVelocity(const MotorNames &motorID) {
        auto &motor = motorList[motorID];
        uint32_t dt = InterruptHandler::getMotorDt(motorID);
        uint32_t encoderCount = InterruptHandler::getEncoderCount(motorID);

        if (dt <= 0 || encoderCount <= 0) {
            motor.current_velocity = 0;
        } else {
            //TODO : figure out the whole float weirdness. like can't i just cast to a int16_t ? do i really need a float
            float calculated_velocity = (float) (encoderCount * 60000000.0 * motor.gear_ratio_reciprocal *
                                                 motor.encoder_resolution_reciprocal / (float) (dt));
            motor.current_velocity = calculated_velocity;
            //motor.current_velocity = map(calculated_velocity,0,MAX);

        }
        InterruptHandler::reset(motorID);

    }
}