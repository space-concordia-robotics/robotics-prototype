//
// Created by Michael on 2021-01-23.
//

#ifndef WHEELS_WHEELSCOMMANDCENTER_H
#define WHEELS_WHEELSCOMMANDCENTER_H

#include "../../../internal_comms/include/CommandCenter.h"

#define COMMAND_SET_MOTORS 0
#define COMMAND_STOP_MOTORS_EMERGENCY 1
#define COMMAND_CLOSE_MOTORS_LOOP 2
#define COMMAND_OPEN_MOTORS_LOOP 3
#define COMMAND_SET_JOYSTICK 4
#define COMMAND_SET_GPS 5
#define COMMAND_SET_ENCODER 6
#define COMMAND_SET_ACCELERATION 7
#define COMMAND_GET_ROVER_STATUS 8
#define COMMAND_MOVE_ROVER 9
#define COMMAND_MOVE_WHEEL 10
#define COMMAND_GET_LINEAR_VELOCITY 11
#define COMMAND_GET_ROTATIONAL_VELOCITY 12
#define COMMAND_GET_CURRENT_VELOCITY 13
#define COMMAND_GET_DESIRED_VELOCITY 14
#define COMMAND_GET_BATTERY_VOLTAGE 15
#define COMMAND_WHEELS_PING 16

class WheelsCommandCenter : public internal_comms::CommandCenter {

    void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) override;

public:
    void enableMotors(uint8_t turnMotorOn);
    void stopMotors();
    void closeMotorsLoop();
    void openMotorsLoop();
    void toggleJoystick(uint8_t turnJoystickOn);
    void toggleGps(uint8_t turnGpsOn);
    void toggleEncoder(uint8_t turnEncOn);
    void toggleAcceleration(uint8_t turnAccelOn);
    void getRoverStatus();
    void moveRover(int8_t roverThrottle, int8_t roverSteering); // Throttle -49 to 49 and Steering -49 to 49
    void moveWheel(uint8_t wheelNumber, int16_t wheelPWM); // Wheel number 0 to 5 and -255 to 255

// Teensy to OBC value getters
    void getLinearVelocity(void);
    void getRotationalVelocity(void);
    void getCurrentVelocity(void);
    void getDesiredVelocity(void);
    void getBatteryVoltage(void);
    void pingWheels(void);

    inline void float2bytes(uint8_t* buffer, float value){
        memcpy(buffer, (unsigned char*) (&value), sizeof(float));
    }
};


#endif
