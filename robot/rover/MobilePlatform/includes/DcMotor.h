#ifndef DCMOTOR_H
#define DCMOTOR_H

#define PULSES_PER_REV     14
#define GEAR_RATIO         188.61
#define MAX_RPM            30

#include "Config.h"

enum MotorNames{
    FRONT_RIGHT=0,
    MIDDLE_RIGHT=1,
    REAR_RIGHT=2,
    FRONT_LEFT=3,
    MIDDLE_LEFT=4,
    REAR_LEFT=5
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
    uint8_t desired_direction;

    uint8_t dir_pin;
    uint8_t pwm_pin;

} DcMotorState;


typedef struct {

    bool is_throttle_timeout_enabled;
    bool is_passive_rover_feedback_enabled;
    bool is_gps_enabled;

    uint32_t last_velocity_adjustment;
    uint32_t last_move;

} SystemState;

namespace Motor {

    extern DcMotorState motorList[6];

    void attachMotor(const MotorNames &, const uint8_t&, const uint8_t&);

    void attachEncoder(const MotorNames &, const uint8_t&, const uint8_t&, const float&, void (*handler)(void));

    void updateDesiredMotorVelocity(const MotorNames &, const uint8_t &, const uint8_t  &);

    void applyDesiredMotorVelocity(const MotorNames &);

    void stop(const MotorNames &);
}

#endif