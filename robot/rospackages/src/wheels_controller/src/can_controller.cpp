//
// Created by nik on 08/02/24.
//

#include "can_controller.h"

uint8_t CANController::configureCAN(const char* fd_path){
    /*
        Open can socket
    */
    struct sockaddr_can addr{};
    s_Socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    s_StatusBuffer = new char[1000];
    if(s_Socket == -1){
        sprintf(s_StatusBuffer,"Socket error : %s (%i)\n", strerror(errno),errno);
        return CAN_ERROR;
    }
    struct ifreq ifr{};
    strcpy(ifr.ifr_name,fd_path);
    if(ioctl(s_Socket, SIOCGIFINDEX, &ifr) == -1){
        sprintf(s_StatusBuffer,"Iocl error : %s (%i)\n", strerror(errno),errno);
        return CAN_ERROR;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(s_Socket, (struct sockaddr *)&addr, sizeof(addr)) == -1){
        sprintf(s_StatusBuffer,"Bind error : %s (%i)\n", strerror(errno),errno);
        return CAN_ERROR;
    }
    sprintf(s_StatusBuffer,"CAN configuration on %s successful\n",fd_path);
    return SUCCESS;
}
/*
    Someone fuking document this (it wont fucking be me)
*/
 uint8_t CANController::sendBlockingFrame(struct can_frame& frame){
    while (true){
        auto status = sendFrame(frame);
        if (errno != 105) break;
        errno = 0;
        usleep(100);
    }
    int errno_0 = errno;
    errno = 0;
    if(errno_0 !=0)
        std::cout << strerror(errno_0) << "\n";
    return errno_0 ? CAN_ERROR : SUCCESS;  
}

/*
    Send CAN frame 
*/
uint8_t CANController::sendFrame(struct can_frame& frame){
    if (write(s_Socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        sprintf(s_StatusBuffer,"Error with send frame, did not write all data \n");
        return CAN_ERROR;
    }
    // sprintf(s_StatusBuffer,"Write complete\n");
    return SUCCESS;
}

uint8_t CANController::readFrame(struct can_frame& frame){
    auto nbytes = read(s_Socket, &frame, sizeof(struct can_frame));

    if (nbytes == -1) {
        sprintf(s_StatusBuffer,"read : error %i\n",errno);
        return CAN_ERROR;
    }
    /*
     * ssize_t cast to avoid compiler warning.
     */
    if (nbytes < (ssize_t)sizeof(struct can_frame)) {
        sprintf(s_StatusBuffer,"read: incomplete CAN frame\n");
        return CAN_ERROR;
    }
    return SUCCESS;
}