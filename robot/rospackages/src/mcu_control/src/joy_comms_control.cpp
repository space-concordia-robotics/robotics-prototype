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
#include <boost/thread/thread.hpp>

#include "joy_comms_control.h"

struct JoyCommsControl::Implement {
    struct ButtonMappings;

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    void addToCommandQueue(const sensor_msgs::Joy::ConstPtr &joy_msg);

    void publish_command(std_msgs::String command);

    void manageQueue();

    ros::Subscriber joy_sub;

    int enable_button;

    std_msgs::String stop_command;

    bool sent_disable_msg;

    std::string command_topic;

    ros::Publisher comms_pub;

    std_msgs::String commands[11];
    int rates[11] = {-1};
    int clicked[11];
};

struct JoyCommsControl::Implement::ButtonMappings {
    int buttonId;
    std_msgs::String command_string;
    int command_publish_rate;
};

void JoyCommsControl::MapButtonNamesToIds() {
    for (int currentButtonId = BUTTON_CROSS; currentButtonId<=BUTTON_R3; ++currentButtonId) {
        button_name_to_id_map.insert(
                std::pair<const char*, int>(button_names[currentButtonId] , currentButtonId));
    }
}

int JoyCommsControl::getButtonIdFromName(std::string button_name) {
    return button_name_to_id_map[button_name.c_str()];
}

void JoyCommsControl::getControllerMappings(ros::NodeHandle *nh_param) {
    // Get controller mappings from ROS params
    XmlRpc::XmlRpcValue mappingsXML;
    nh_param->getParam("/controller_mappings", mappingsXML);

    Implement::ButtonMappings currentButtonMap;
    std::string button_name, command;
    XmlRpc::XmlRpcValue mappingObject;
    if (mappingsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < mappingsXML.size(); ++i) {
            mappingObject = mappingsXML[i];

            button_name = static_cast<std::string>(mappingObject[0]).c_str();
            int buttonId = getButtonIdFromName(button_name);

            command = static_cast<std::string>(mappingObject[1]).c_str();
            pImplement->commands[buttonId].data = command;

            pImplement->rates[buttonId] = mappingObject[2];
        }
    }
}

JoyCommsControl::JoyCommsControl(ros::NodeHandle *nh, ros::NodeHandle *nh_param) {
    MapButtonNamesToIds();

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

void JoyCommsControl::Implement::addToCommandQueue(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    std::vector<int> buttons = joy_msg->buttons;
    for (int i = 0; i < buttons.size(); ++i)
    {
        clicked[i] = buttons[i];
    }
}

void JoyCommsControl::publish_command_with_rate() {
    for (int i = 0; i < sizeof(pImplement->clicked); ++i)
    {
        // TODO in last condition rate is used to check existance of command. consider changing it
        if (i != pImplement->enable_button && pImplement->clicked[i] == 1 && pImplement->rates[i] > 0)
        {
            //TODO this rate is not unique to each button when more than one is pressed
            ros::Rate loop_rate(pImplement->rates[i]);
            pImplement->publish_command(pImplement->commands[i]);
            loop_rate.sleep();
        }
    }
}

void JoyCommsControl::Implement::publish_command(std_msgs::String command) {
    comms_pub.publish(command);
    sent_disable_msg = false;
}

void JoyCommsControl::Implement::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    if (joy_msg->buttons.size() > enable_button && joy_msg->buttons[enable_button]) {
        addToCommandQueue(joy_msg);
    } else {
        //clear clicked array since enable button is not clicked
        //consider moving it to "not publish" when enable is not clicked
        for(int i =0; i< sizeof(clicked);i++){
            clicked[i] = 0;
        }

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

    while (ros::ok())
    {
        joy_control.publish_command_with_rate();
        ros::spinOnce();
    }
}