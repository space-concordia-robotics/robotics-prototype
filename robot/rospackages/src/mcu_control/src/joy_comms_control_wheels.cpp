//
// Created by william on 2021-12-26.
//

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"

#include <map>
#include <string>
#include <vector>
#include <iostream>

#include "joy_comms_control.h"

struct JoyCommsControl::Implement {
    struct ButtonMappings;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    void sendCommsCommand(const sensor_msgs::Joy::ConstPtr &joy_msg);

    void publish_command(std_msgs::String command);

    ros::Subscriber joy_sub;

    int enable_button;

    std_msgs::String stop_command;

    bool sent_disable_msg;

    std::string command_topic;

    ros::Publisher comms_pub;

    std::vector<ButtonMappings> controller_mappings;

    std::vector<ButtonMappings> alt_controller_mappings;
};

struct JoyCommsControl::Implement::ButtonMappings {
    int buttonId;
    std_msgs::String command_string;
    std_msgs::String alt_command_string;
    int toggleAltButton;
    int command_publish_rate;
};

void JoyCommsControl::MapButtonNamesToIds() {
    for (int currentButtonId = BUTTON_CROSS; currentButtonId<=BUTTON_DPAD_RIGHT; ++currentButtonId) {
        button_name_to_id_map.insert(
                std::pair<std::string, int>(button_names[currentButtonId], currentButtonId));
    }
}

int JoyCommsControl::getButtonIdFromName(std::string button_name) {
    return button_name_to_id_map[button_name];
}

void JoyCommsControl::getControllerMappings(ros::NodeHandle *nh_param) {
    // Get controller mappings from ROS params
    XmlRpc::XmlRpcValue mappingsXML;
    nh_param->getParam("controller_mappings", mappingsXML);

    Implement::ButtonMappings currentButtonMap;
    std::string button_name, command, toggle_alt_button, alt_command;
    XmlRpc::XmlRpcValue mappingObject;
    if (mappingsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < mappingsXML.size(); ++i) {
            mappingObject = mappingsXML[i];

            button_name = static_cast<std::string>(mappingObject[0]).c_str();

            currentButtonMap.buttonId = getButtonIdFromName(button_name);

            command = static_cast<std::string>(mappingObject[1]).c_str();
            currentButtonMap.command_string.data = command;

            toggle_alt_button = static_cast<std::string>(mappingObject[2]).c_str();
            currentButtonMap.toggleAltButton = getButtonIdFromName(toggle_alt_button);

            alt_command = static_cast<std::string>(mappingObject[3]).c_str();
            currentButtonMap.alt_command_string.data = alt_command;

            pImplement->controller_mappings.push_back(currentButtonMap);
        }
    }
}

JoyCommsControl::JoyCommsControl(ros::NodeHandle *nh, ros::NodeHandle *nh_param) {
    pImplement = new Implement;

    MapButtonNamesToIds();

    nh_param->param<int>("command_topic", pImplement->enable_button, 0);

    nh_param->param<int>("enable_button", pImplement->enable_button, 0);

    std::string stop_command;
    nh_param->param<std::string>("stop_command", stop_command, "stop");
    pImplement->stop_command.data = stop_command;

    nh_param->param<std::string>("command_topic", pImplement->command_topic, "stop");

    getControllerMappings(nh_param);

    pImplement->sent_disable_msg = false;

    pImplement->comms_pub = nh->advertise<std_msgs::String>(pImplement->command_topic, 5, true);

    std::string sub_topic;
    nh_param->param<std::string>("sub_topic", sub_topic, "/joy_wheels");

    pImplement->joy_sub = nh->subscribe<sensor_msgs::Joy>(sub_topic, 1, &JoyCommsControl::Implement::joyCallback,
                                                          pImplement);
}

void JoyCommsControl::Implement::sendCommsCommand(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    std::vector<int> buttons = joy_msg->buttons;
    for (int i = 0; i<buttons.size(); ++i) {
        if (buttons[i] && i != enable_button) {
            for (ButtonMappings controller_mapping : controller_mappings) {
                if (controller_mapping.buttonId == i) {
                    if (!buttons[controller_mapping.toggleAltButton]) {
                        publish_command(controller_mapping.command_string);
                    } else {
                        // publish alt if toggle button is being held
                        publish_command(controller_mapping.alt_command_string);
                    }
                }
            }

        }
    }
}

void JoyCommsControl::Implement::publish_command(std_msgs::String command) {
    comms_pub.publish(command);
    sent_disable_msg = false;
}

void JoyCommsControl::Implement::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    if (joy_msg->buttons.size() > enable_button && joy_msg->buttons[enable_button]) {
        sendCommsCommand(joy_msg);
    } else {
        if (!sent_disable_msg) {
            comms_pub.publish(stop_command);
            sent_disable_msg = true;
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "joy_comms_control_node");

    ros::NodeHandle nh(""), nh_param("~");
    JoyCommsControl joy_control(&nh, &nh_param);

    ros::spin();
}