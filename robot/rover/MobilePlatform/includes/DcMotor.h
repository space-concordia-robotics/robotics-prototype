#ifndef DCMOTOR_H
#define DCMOTOR_H
#include <cstdio>
#include "PidController.h"
#define PULSES_PER_REV     14
#define GEAR_RATIO         188.61
#define MAX_RPM            30


#define MAX_INPUT_VALUE 49  // maximum speed signal from controller
#define MIN_INPUT_VALUE -MAX_INPUT_VALUE // minimum speed signal from controller
#define MAX_PWM_VALUE 255
#define MIN_PWM_VALUE 0
#define MAX_RPM_VALUE MAX_RPM
#define MIN_RPM_VALUE 0

enum MotorNames{
    FRONT_RIGHT=0,
    MIDDLE_RIGHT=1,
    REAR_RIGHT=2,
    FRONT_LEFT=3,
    MIDDLE_LEFT=4,
    REAR_LEFT=5
};

enum motor_direction {
    CW = HIGH,
    CCW = LOW,
};

class InterruptHandler{
public:
    static volatile u_int32_t LEFT_FRONT_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t LEFT_FRONT_MOTOR_DT;
    static volatile u_int32_t LEFT_FRONT_MOTOR_PREV_DT;

    static volatile u_int32_t LEFT_MIDDLE_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t LEFT_MIDDLE_MOTOR_DT;
    static volatile u_int32_t LEFT_MIDDLE_MOTOR_PREV_DT;

    static volatile u_int32_t LEFT_BACK_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t LEFT_BACK_MOTOR_DT;
    static volatile u_int32_t LEFT_BACK_MOTOR_PREV_DT;

    static volatile u_int32_t RIGHT_FRONT_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t RIGHT_FRONT_MOTOR_DT;
    static volatile u_int32_t RIGHT_FRONT_MOTOR_PREV_DT;

    static volatile u_int32_t RIGHT_MIDDLE_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t RIGHT_MIDDLE_MOTOR_DT;
    static volatile u_int32_t RIGHT_MIDDLE_MOTOR_PREV_DT;

    static volatile u_int32_t RIGHT_BACK_MOTOR_ENCODER_COUNT;
    static volatile u_int32_t RIGHT_BACK_MOTOR_DT;
    static volatile u_int32_t RIGHT_BACK_MOTOR_PREV_DT;

    static void RightFrontMotorInterruptHandler(){
        RIGHT_FRONT_MOTOR_ENCODER_COUNT++;
        RIGHT_FRONT_MOTOR_DT = micros() - RIGHT_FRONT_MOTOR_PREV_DT;
        RIGHT_FRONT_MOTOR_PREV_DT = micros();
    }
    static void RightMiddleMotorInterruptHandler(){
        RIGHT_MIDDLE_MOTOR_ENCODER_COUNT++;
        RIGHT_MIDDLE_MOTOR_DT = micros() - RIGHT_MIDDLE_MOTOR_PREV_DT;
        RIGHT_MIDDLE_MOTOR_PREV_DT = micros();
    }
    static void RightBackMotorInterruptHandler(){
        RIGHT_BACK_MOTOR_ENCODER_COUNT++;
        RIGHT_BACK_MOTOR_DT = micros() - RIGHT_BACK_MOTOR_PREV_DT;
        RIGHT_BACK_MOTOR_PREV_DT = micros();
    }
    static void LeftFrontMotorInterruptHandler(){
        LEFT_FRONT_MOTOR_ENCODER_COUNT++;
        LEFT_FRONT_MOTOR_DT = micros() - LEFT_FRONT_MOTOR_PREV_DT;
        LEFT_FRONT_MOTOR_PREV_DT = micros();
    }
    static void LeftMiddleMotorInterruptHandler(){
        LEFT_MIDDLE_MOTOR_ENCODER_COUNT++;
        LEFT_MIDDLE_MOTOR_DT = micros() - LEFT_MIDDLE_MOTOR_PREV_DT;
        LEFT_MIDDLE_MOTOR_PREV_DT = micros();
    }
    static void LeftBackMotorInterruptHandler(){
        LEFT_BACK_MOTOR_ENCODER_COUNT++;
        LEFT_BACK_MOTOR_DT = micros() - LEFT_BACK_MOTOR_PREV_DT;
        LEFT_BACK_MOTOR_PREV_DT = micros();
    }

    static uint32_t getEncoderCount(MotorNames name){
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
    static uint32_t getMotorDt(MotorNames name){
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
    static void reset(MotorNames name) {
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
};

typedef struct DcMotorState{
    MotorNames id;

    uint8_t desired_velocity;
    float current_velocity;

    motor_direction desired_direction;

    float gear_ratio;
    float gear_ratio_reciprocal;
    float encoder_resolution_reciprocal;

    int8_t max_output_signal;
    int8_t min_output_signal;

    uint8_t dir_pin;
    uint8_t pwm_pin;

    bool is_open_loop;

    pidControllerState pid_controller;
} DcMotorState;

typedef struct{
    float kp;
    float ki;
    float kd;
} PidController;


typedef struct {
    bool is_throttle_timeout_enabled;
    bool are_motors_enabled;
    bool is_open_loop;
} SystemState;

namespace Motor {

    extern DcMotorState motorList[6];

    void attachMotor(const MotorNames &, const uint8_t&, const uint8_t&, const float&);

    void attachEncoder(const MotorNames &, const uint8_t&, const uint8_t&, const float&, void (*handler)(void));

    void calculateCurrentVelocity(const MotorNames &);

    void updateDesiredMotorVelocity(const MotorNames &, const motor_direction &, const uint8_t  &);

    void applyDesiredMotorVelocity(const MotorNames &);

    void initPidController(const MotorNames &, const float&, const float&, const float&);

    void stop(const MotorNames &);
}
inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif