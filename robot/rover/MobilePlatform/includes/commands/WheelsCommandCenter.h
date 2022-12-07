#ifndef WHEELS_WHEELSCOMMANDCENTER_H
#define WHEELS_WHEELSCOMMANDCENTER_H

#include "CommandCenter.h"

#define COMMAND_WHEELS_PING 0
#define COMMAND_MOVE_SERVO 1
#define COMMAND_MOVE_ROVER 2
#define COMMAND_MOVE_WHEEL 3
#define COMMAND_MOVE_WHEELS 4
#define COMMAND_STOP_MOTORS 5

#define COMMAND_GET_BATTERY_VOLTAGE 6

#define COMMAND_BLINK_TOGGLE 17
#define COMMAND_BLINK_COLOR 18

        class WheelsCommandCenter : public internal_comms::CommandCenter {

    void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) override;

public:
    void pingWheels();
    void moveServo(const uint8_t&, const uint8_t&);
    void moveRover(const float &,const float& );
    void moveWheel(const uint8_t& ,const uint8_t& ,const uint8_t&);
    void stopMotors();

    void getBatteryVoltage();


    void handleBlink(uint8_t on);
    void handleBlinkColor(uint8_t r, uint8_t g, uint8_t b);

};


#endif
