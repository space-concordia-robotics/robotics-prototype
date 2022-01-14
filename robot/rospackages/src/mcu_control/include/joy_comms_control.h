//
// Created by william on 2022-01-11.
//

#ifndef ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H
#define ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H

namespace ros { class NodeHandle; }

class JoyCommsControl {
public:
    JoyCommsControl(ros::NodeHandle *nh, ros::NodeHandle *nh_param);
    void getControllerMappings(ros::NodeHandle *nh_param);
    struct Implement;

private:
    Implement* pImplement;
};

#endif //ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H
