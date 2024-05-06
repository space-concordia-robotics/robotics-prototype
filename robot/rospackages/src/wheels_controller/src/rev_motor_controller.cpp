//
// Created by nik on 06/02/24.
//
#include "rev_motor_controller.h"
#include <linux/can/raw.h>
#include <linux/can/bcm.h>
#include <linux/can/netlink.h>
#include <linux/can/j1939.h>

void RevMotorController::setDeviceId(uint8_t deviceID, uint8_t newId){

    struct can_frame frame{};

    frame.can_dlc = 5;
    frame.can_id = (COMMAND_PREFIX_SET_DEVCE_ID << 8) | deviceID;

    uint8_t buf[5] = {newId,0,0,0,1};
    memcpy(frame.data,buf,sizeof(buf));

    CANController::sendFrame(frame);
}

void RevMotorController::requestStatusFrame(){
    struct can_frame frame{};
    /*
     * Periodic status messages will be sent over the CAN bus after sending this command once.
     */
    
    frame.can_id = 0x000502C0;
    frame.can_dlc = 1;
    frame.can_id |= CAN_EFF_FLAG;
    frame.data[0] = 1;

    CANController::sendFrame(frame);

}

void RevMotorController::voltagePercentControl(uint8_t deviceId, float percent){
    struct can_frame frame{};
    /*
     *
     */
    bool isRunning = s_Devices[deviceId].isRunning;
    if(isRunning){
        uint64_t buf_data = (1ULL << deviceId);

        frame.can_id = COMMAND_PREFIX_MAINTAIN_SPEED;
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 8;
        memcpy(frame.data,&buf_data,sizeof(buf_data));
    }
    else{
        s_Devices[deviceId].isRunning = true;

        // For a start move command, the id field needs to be added with 0x80. This command is only issued once per move.
        frame.can_id = (COMMAND_PREFIX_MOVE_MOTORS << 8) | (deviceId + 0x80);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 8;
        memcpy(frame.data,&percent, sizeof(float));
    }
    CANController::sendFrame(frame);
}

void RevMotorController::stopMotor(uint8_t deviceID){

    struct can_frame frame{};
    frame.can_id = (COMMAND_PREFIX_MOVE_MOTORS << 8) | (deviceID);
    frame.can_dlc = 8;

    CANController::sendFrame(frame);
}

void RevMotorController::velocityControl(uint8_t deviceId, float velocity){

    // if(abs(velocity) < 20){
    //     return;
    // }
    
    struct can_frame frame{};
    
    frame.can_id = (COMMAND_PREFIX_VELOCITY_CONTROL << 8) | (deviceId+0x80);
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    memcpy(frame.data,&velocity,sizeof(float));
    
    CANController::sendBlockingFrame(frame);

    // std::cout << "Running  " << (int)deviceId << " on " << velocity << "\n";
    // uint64_t buf_data = (1ULL << deviceId);
    // frame.can_id = COMMAND_PREFIX_MAINTAIN_VELOCITY;
    // frame.can_id |= CAN_EFF_FLAG;
    // frame.can_dlc = 8;
    // memcpy(frame.data,&buf_data,sizeof(buf_data));
    
    // CANController::sendBlockingFrame(frame);
    
    // std::cout << "Starting run on " << (int)deviceId << "\n";
    // s_Devices.at(deviceId)->isRunning = true;
    // s_Devices[deviceId].isRunning = true;

}
void RevMotorController::startMotor(uint64_t mask){
    struct can_frame frame{};
    
    // uint64_t buf_data = (1ULL << deviceId);
    frame.can_id = COMMAND_PREFIX_MAINTAIN_VELOCITY;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    memcpy(frame.data,&mask,sizeof(mask));
    
    CANController::sendBlockingFrame(frame);
}

void RevMotorController::registerDevice(uint8_t deviceId) {
    // s_Devices.emplace(deviceId, new Device());
    s_Devices[deviceId].id = deviceId;
}
//void RevMotorController::printStatus() {
//
//    snprintf (s_StatusBuffer, STATUS_BUFFER_SIZE - strlen(s_StatusBuffer),
//              "Motor %d status : \n RPM : %f \n Temperature : %i \n Position : %f \n\n",
//             m_deviceID,m_Status.rpm,m_Status.temperature,m_Position);
//}