#ifndef WHEELS_WHEELSCOMMANDCENTER_H
#define WHEELS_WHEELSCOMMANDCENTER_H

#include "CommandCenter.h"

#define COMMAND_STOP_MOTORS_EMERGENCY 0
#define COMMAND_GET_ROVER_STATUS 2
#define COMMAND_SEND_GPS 3
#define COMMAND_MOVE_SERVO 7
#define COMMAND_MOVE_ROVER 8
#define COMMAND_MOVE_WHEEL 9
#define COMMAND_MOVE_WHEELS 10

#define COMMAND_GET_LINEAR_VELOCITY 11
#define COMMAND_GET_ROTATIONAL_VELOCITY 12
#define COMMAND_GET_CURRENT_VELOCITY 13
#define COMMAND_GET_DESIRED_VELOCITY 14
#define COMMAND_GET_BATTERY_VOLTAGE 15
#define COMMAND_WHEELS_PING 16
#define COMMAND_BLINK_TOGGLE 17
#define COMMAND_BLINK_COLOR 18


class WheelsCommandCenter : public internal_comms::CommandCenter {

    void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) override;

public:
    void stopMotors();
    void moveRover(const float &,const float& );
    void moveServo(const uint8_t&, const uint8_t&);
    void moveWheel(const uint8_t& ,const uint8_t& ,const uint8_t&);

    void getLinearVelocity(void);
    void getRotationalVelocity(void);
    void getBatteryVoltage(void);
    void pingWheels(void);
    
    void handleBlink(uint8_t on);
    void handleBlinkColor(uint8_t r, uint8_t g, uint8_t b);

};


#endif
