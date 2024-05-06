//
// Created by nik on 06/02/24.
//

#ifndef REV_WHEELS_CONTROLLER_H
#define REV_WHEELS_CONTROLLER_H

#include "command_prefixes.h"
#include "can_controller.h"
#define NOMINAL_VOLTAGE 12


//typedef struct status{
//    uint8_t temperature;
//    float rpm;
//} status;

//typedef struct faults{
//    bool gate_driver;
//} faults;

typedef struct Device{
    bool isRunning = false;
    uint8_t id;
} device;



class RevMotorController{

    // inline static std::map<uint8_t, Device*> s_Devices;
    
    
    
    // inline static <s_Devices;

public:
    inline static Device s_Devices[6];

    static void requestStatusFrame();

    static void registerDevice(uint8_t);

    static void setDeviceId(uint8_t id,uint8_t);

    void initializeDefaultSettings();

    static void voltagePercentControl(uint8_t deviceID,float percent);
    static void stopMotor(uint8_t device_id);
    static void velocityControl(uint8_t deviceID, float velocity);
    static void startMotor(uint64_t mask);
    void printStatus();
};

#endif