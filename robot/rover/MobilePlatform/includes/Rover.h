#ifndef ROVER_H
#define ROVER_H

#include "DcMotor.h"

/***** MOTORS *****/

#define NUM_MOTORS 6
#define ROVER_MOVE_TIMEOUT 250
#define ACCELERATION_RATE 5


enum ServoNames{
    CENTER_FRONT_1_SERVO = 0,
    CENTER_FRONT_2_SERVO= 1,
    CENTER_BACK_1_SERVO = 2,
    CENTER_BACK_2_SERVO = 3
};
extern Servo servos[4];

typedef struct {
    float right_linear_velocity;
    float left_linear_velocity;
    float linear_velocity;
    float rotational_velocity;
    int16_t max_output_signal;
    int16_t min_output_signal;
} RoverState;


namespace Rover {

    extern RoverState roverState;

    extern SystemState systemStatus;

    void attachServo(const ServoNames&, const uint8_t&);

    void moveWheel(const MotorNames &,const uint8_t& ,const uint8_t&);

    void moveRover(const float & ,const float &);

    void moveServo(const ServoNames&, const uint8_t&);

    void stopMotors();

    void updateWheelVelocities();

    void decelerateRover();
}
#endif
