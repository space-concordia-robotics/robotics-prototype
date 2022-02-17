//
// Created by Michael on 2021-01-23.
//

#ifndef WHEELS_WHEELSCOMMANDCENTER_H
#define WHEELS_WHEELSCOMMANDCENTER_H

#include "../../../internal_comms/include/CommandCenter.h"


#define COMMAND_STOP_MOTORS_EMERGENCY 0

#define COMMAND_GET_ROVER_STATUS 2
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

class WheelsCommandCenter : public internal_comms::CommandCenter {

    void executeCommand(const uint8_t commandID, const uint8_t* rawArgs, const uint8_t rawArgsLength) override;

public:
    void stopMotors();
    void moveRover(const float &,const float& );
    void moveWheel(const uint8_t& ,uint8_t ,uint8_t);

    void getLinearVelocity(void);
    void getRotationalVelocity(void);
    void getBatteryVoltage(void);
    void pingWheels(void);

    inline void float2bytes(uint8_t* buffer, float value){
        memcpy(buffer, (unsigned char*) (&value), sizeof(float));
    }
};


#endif
