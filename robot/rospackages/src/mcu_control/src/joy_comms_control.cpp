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
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    void sendCommsCommand(const sensor_msgs::Joy::ConstPtr &joy_msg);

    void publish_command(std_msgs::String command);

    ros::Subscriber joy_sub;

    int enable_button;

    std_msgs::String stop_command;

    bool sent_disable_msg;

    std::string command_topic;

    ros::Publisher comms_pub;

    std::map<int, std_msgs::String> button_command_mappings;
};

void JoyCommsControl::getControllerMappings(ros::NodeHandle *nh_param) {
    // Get controller mappings from ROS params
    XmlRpc::XmlRpcValue mappingsXML;
    nh_param->getParam("/controller_mappings", mappingsXML);

    int buttonId;
    std::string command;
    std_msgs::String commandMessage;
    XmlRpc::XmlRpcValue mappingObject;
    if (mappingsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < mappingsXML.size(); ++i) {
            mappingObject = mappingsXML[i];

            buttonId = mappingObject[0];

            command = static_cast<std::string>(mappingObject[1]).c_str();
            commandMessage.data = command;

            command = static_cast<std::string>(mappingObject[1]).c_str();

            pImplement->button_command_mappings.insert(std::pair<int, std_msgs::String>(buttonId, commandMessage));
        }
    }
}

JoyCommsControl::JoyCommsControl(ros::NodeHandle *nh, ros::NodeHandle *nh_param) {
    pImplement = new Implement;

    nh_param->param<int>("/command_topic", pImplement->enable_button, 0);

    nh_param->param<int>("/enable_button", pImplement->enable_button, 0);

    std::string stop_command;
    nh_param->param<std::string>("/stop_command", stop_command, "stop");
    pImplement->stop_command.data = stop_command;

    nh_param->param<std::string>("/command_topic", pImplement->command_topic, "stop");

    getControllerMappings(nh_param);

    pImplement->sent_disable_msg = false;

    pImplement->comms_pub = nh->advertise<std_msgs::String>(pImplement->command_topic, 1, true);

    pImplement->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &JoyCommsControl::Implement::joyCallback,
                                                          pImplement);
}

void JoyCommsControl::Implement::sendCommsCommand(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    std::vector<int> buttons = joy_msg->buttons;
    for (int i = 0; i<buttons.size(); ++i) {
        if (buttons[i] && i != enable_button) {
            publish_command(button_command_mappings[i]);
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