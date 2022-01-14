//
// Created by william on 2022-01-11.
//

#ifndef ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H
#define ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H

namespace ros { class NodeHandle; }

class JoyCommsControl {
public:
    JoyCommsControl(ros::NodeHandle *nh, ros::NodeHandle *nh_param);
    void MapButtonNamesToIds();
    int getButtonIdFromName(std::string button_name);
    void getControllerMappings(ros::NodeHandle *nh_param);
    struct Implement;

private:
    // These enums are defined for PS4 controllers
    enum buttons {
        BUTTON_CROSS = 0,
        BUTTON_CIRCLE = 1,
        BUTTON_SQUARE = 2,
        BUTTON_TRIANGLE = 3,
        BUTTON_L1 = 4,
        BUTTON_R1 = 5,
        BUTTON_SHARE = 6,
        BUTTON_OPTION = 7,
        BUTTON_HOME = 8,
        BUTTON_L3 = 9,
        BUTTON_R3 = 10,
    };

    const char* button_names[11] = {"BUTTON_CROSS",
                                "BUTTON_CIRCLE",
                                "BUTTON_SQUARE",
                                "BUTTON_TRIANGLE",
                                "BUTTON_L1",
                                "BUTTON_R1",
                                "BUTTON_SHARE",
                                "BUTTON_OPTION",
                                "BUTTON_HOME",
                                "BUTTON_L3",
                                "BUTTON_R3",};

    std::map<const char*, int> button_name_to_id_map;

    enum axes {
        JOY_LEFT_X = 0,
        JOY_LEFT_Y = 1,
        TRIGGER_L2 = 2,
        JOY_RIGHT_X = 3,
        JOY_RIGHT_Y = 4,
        TRIGGER_R2 = 5,
    };

    Implement* pImplement;
};

#endif //ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H
