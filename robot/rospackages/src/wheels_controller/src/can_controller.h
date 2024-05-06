//
// Created by nik on 08/02/24.
//

#ifndef CAN_CONTROLLER_H
#define CAN_CONTROLLER_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <unistd.h>
#include <cstdint>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <termios.h>
#include <net/if.h>
#include <iostream>

#define STATUS_BUFFER_SIZE 1000

enum status{
    SUCCESS = 0,
    CAN_ERROR
};

class CANController {

    inline static int s_Socket = 0;

    inline static char* s_StatusBuffer = nullptr;
public:

    static uint8_t configureCAN(const char* fd_path);

    static uint8_t sendFrame(struct can_frame& frame);

    static uint8_t readFrame(struct can_frame& frame);

    static uint8_t sendBlockingFrame(struct can_frame& frame);

    static inline void printStatus(){
        printf("%s \n", s_StatusBuffer);
    }
};


#endif 
