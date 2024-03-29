//
// Created by william on 2022-01-11.
//

#ifndef ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H
#define ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H

#include "rclcpp/rclcpp.hpp"

namespace ros { class NodeHandle; }

class JoyCommsControl : public rclcpp::Node {
public:
    JoyCommsControl();
    void MapButtonNamesToIds();
    int getButtonIdFromName(std::string button_name);
    int getAxisIdFromName(std::string button_name);
    bool isButton(std::string control_name);
    void getControllerMappings();
    void getCommandTopics();
    void publish_command_with_rate();
    struct Implement;

private:
    const int QUEUE_LOOP_RATE = 120;

    enum controller_type {
        PLAYSTATION = 1,
        XBOX = 2,
	    PLAYSTATION_UBUNTU_18 = 3
    };

    controller_type controller_type;

    const std::string share_name = "BUTTON_SHARE";
    const std::string option_name = "BUTTON_OPTION";
    const std::string xbox_button_names[17] = {"BUTTON_CROSS",
                                "BUTTON_CIRCLE",
                                "BUTTON_TRIANGLE",
                                "BUTTON_SQUARE",
                                "BUTTON_L1",
                                "BUTTON_R1",
                                "BUTTON_L2",
                                "BUTTON_R2",
                                share_name,
                                option_name,
                                "BUTTON_HOME",
                                "BUTTON_L3",
                                "BUTTON_R3",
                                "BUTTON_DPAD_UP",
                                "BUTTON_DPAD_DOWN",
                                "BUTTON_DPAD_LEFT",
                                "BUTTON_DPAD_RIGHT"};

    std::string ps_button_names[15] = {"BUTTON_CROSS",
                                "BUTTON_CIRCLE",
                                "BUTTON_SQUARE",
                                "BUTTON_TRIANGLE",
                                share_name,
                                "BUTTON_HOME",
                                option_name,
                                "BUTTON_L3",
                                "BUTTON_R3",
                                "BUTTON_L1",
                                "BUTTON_R1",
                                "BUTTON_DPAD_UP",
                                "BUTTON_DPAD_DOWN",
                                "BUTTON_DPAD_LEFT",
                                "BUTTON_DPAD_RIGHT"};


    std::string ps_u18_button_names[13] = {"BUTTON_SQUARE",
                                "BUTTON_CROSS",
                                "BUTTON_CIRCLE",
                                "BUTTON_TRIANGLE",
                                "BUTTON_L1",
                                "BUTTON_R1",
                                "BUTTON_L2",
                                "BUTTON_R2",
                                share_name,
                                option_name,
                                "BUTTON_L3",
                                "BUTTON_R3",
    				"BUTTON_HOME"};


    std::map<std::string, int> button_name_to_id_map;


    const std::string trigger_l2_name = "TRIGGER_L2";
    const std::string trigger_r2_name = "TRIGGER_R2";
    std::string xbox_axis_names[6] = {"JOY_LEFT_X",
                                "JOY_LEFT_Y",
                                trigger_l2_name,
                                "JOY_RIGHT_X",
                                "JOY_RIGHT_Y",
                                trigger_r2_name};



    std::string ps_axis_names[8] = {"JOY_LEFT_X",
                                "JOY_LEFT_Y",
                                "JOY_RIGHT_X",
                                "JOY_RIGHT_Y",
                                trigger_l2_name,
                                trigger_r2_name};

    std::string ps_u18_axis_names[8] = {"JOY_LEFT_X",
                                "JOY_LEFT_Y",
                                "JOY_RIGHT_X",
                                trigger_l2_name,
                                trigger_r2_name,
                                "JOY_RIGHT_Y",
                                "DPAD_X",
                                "DPAD_Y"};


    std::map<std::string, int> axis_name_to_id_map;

    Implement* pImplement;
};

#endif //ROBOTICS_PROTOTYPE_JOY_COMMS_CONTROL_H
