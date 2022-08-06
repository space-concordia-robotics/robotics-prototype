#ifndef ROVER_H
#define ROVER_H
#include "DcMotor.h"
#include "APA102.h"


/***** MOTORS *****/

#define NUM_MOTORS 6
#define ROVER_MOVE_TIMEOUT 250
#define ACCELERATION_RATE 5


/* Constants for the activity light  */
#define ACTIVITY_MOSI 21
#define ACTIVITY_CLK  20
#define ACTIVITY_BLINK_PERIOD  500
#define ACTIVITY_NUM_LIGHTS 20
#define ACTIVITY_BRIGHTNESS 31
#define ACTIVITY_DEFAULT_COLOR 255, 0, 0


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
    // timestamp when the last rising/falling edge of the blink cycle happened
    unsigned int timeBlinkUpdated = 0;
    bool lightOn = false;
    bool blinking = false;
    APA102 light = APA102(ACTIVITY_NUM_LIGHTS, ACTIVITY_MOSI, ACTIVITY_CLK);
    uint8_t r, g, b;
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

    // These functions are to be used by the MobilePlatofrm code
    void handleActivityLight();
    void setActivityBlinking(uint8_t on);
    // These functions are used internally by the above functions
    void setActivityLight(uint8_t on);
    void setActivityColor(uint8_t r, uint8_t g, uint8_t b);
}
#endif
