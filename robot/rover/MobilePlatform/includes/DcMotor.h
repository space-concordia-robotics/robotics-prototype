#ifndef DCMOTOR_H
#define DCMOTOR_H
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

    static void RightFrontMotorInterruptHandler();
    static void RightMiddleMotorInterruptHandler();
    static void RightBackMotorInterruptHandler();
    static void LeftFrontMotorInterruptHandler();
    static void LeftMiddleMotorInterruptHandler();
    static void LeftBackMotorInterruptHandler();

    static uint32_t getEncoderCount(MotorNames name);
    static uint32_t getMotorDt(MotorNames name);
    static void reset(MotorNames name);

};

typedef struct DcMotorState{
    MotorNames id;

    uint8_t desired_velocity;
    uint8_t current_velocity;

    float actual_velocity;
    uint8_t max_pwm_value;

    uint8_t desired_direction;

    float gear_ratio;
    float gear_ratio_reciprocal;
    float encoder_resolution_reciprocal;


    uint8_t dir_pin;
    uint8_t pwm_pin;

    bool is_open_loop;
    bool has_reached_target_velocity;

    pidControllerState pid_controller;
} DcMotorState;

typedef struct{
    float kp;
    float ki;
    float kd;
} PidController;


typedef struct {
    bool is_throttle_timeout_enabled;
    bool is_passive_rover_feedback_enabled;

    bool are_motors_enabled;
    bool is_open_loop;

    uint32_t last_velocity_adjustment;
    uint32_t last_move;

} SystemState;

namespace Motor {

    extern DcMotorState motorList[6];

    void attachMotor(const MotorNames &, const uint8_t&, const uint8_t&, const float&);

    void attachEncoder(const MotorNames &, const uint8_t&, const uint8_t&, const float&, void (*handler)(void));

    void calculateMotorVelocity(const MotorNames &);

    void updateDesiredMotorVelocity(const MotorNames &, const uint8_t &, const uint8_t  &);

    void applyDesiredMotorVelocity(const MotorNames &);

    void initPidController(const MotorNames &, const float&, const float&, const float&);

    void stop(const MotorNames &);
}
inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif