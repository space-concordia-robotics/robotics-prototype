#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "Config.h"

enum MotorNames{
    FRONT_RIGHT=0,
    MIDDLE_RIGHT=1,
    REAR_RIGHT=2,
    FRONT_LEFT=3,
    MIDDLE_LEFT=4,
    REAR_LEFT=5
};

typedef struct DcMotorState{
    MotorNames id;
    uint8_t desired_velocity;
    uint8_t current_velocity;
    uint8_t desired_direction;
    uint8_t dir_pin;
    uint8_t pwm_pin;
} DcMotorState;


typedef struct {
    bool is_throttle_timeout_enabled;
    uint32_t last_velocity_adjustment;
    uint32_t last_move;
} SystemState;

namespace Motor {

    extern DcMotorState motorList[6];

    void attachMotor(const MotorNames &, const uint8_t&, const uint8_t&);

    void updateDesiredMotorVelocity(const MotorNames &, const uint8_t &, const uint8_t  &);

    void applyDesiredMotorVelocity(const MotorNames &);

    void stop(const MotorNames &);
}

#endif
