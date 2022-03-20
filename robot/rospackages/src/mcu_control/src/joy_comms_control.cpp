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

    std_msgs::String stop_commands[4];

    bool sent_disable_msg;

    std::string command_topics[4];

    ros::Publisher comms_pubs[4];

    std_msgs::String button_commands[4][11];
    int button_rates[4][11] = {-1};
    int buttons_clicked[11];

    std_msgs::String axis_commands[4][8];
    int axis_rates[4][8] = {-1};
    int axis_ranges[4][8] = {-1};
    int axes_moved[8];
    float axes_values[8];
    float axes_percentage[8];

    int number_of_mappings = 0;
    int current_mappings_index = 0;
};

struct JoyCommsControl::Implement::ButtonMappings {
    int buttonId;
    std_msgs::String command_string;
    int command_publish_rate;
};

void JoyCommsControl::MapButtonNamesToIds() {
    for (int i = BUTTON_CROSS; i<=BUTTON_R3; ++i) {
        button_name_to_id_map.insert(std::pair<std::string, int>(button_names[i], i));
    }
    for (int i = JOY_LEFT_X; i<=DPAD_Y; ++i) {
        axis_name_to_id_map.insert(std::pair<std::string, int>(axis_names[i], i));
    }
}

int JoyCommsControl::getButtonIdFromName(std::string button_name) {
    return button_name_to_id_map[button_name];
}

int JoyCommsControl::getAxisIdFromName(std::string button_name) {
    return axis_name_to_id_map[button_name];
}

bool JoyCommsControl::isButton(std::string control_name) {
    return button_name_to_id_map.count(control_name);
}

void JoyCommsControl::getControllerMappings(ros::NodeHandle *nh_param) {
    // Get controller mappings from ROS params
    XmlRpc::XmlRpcValue mappingsXML;
    nh_param->getParam("/controller_mappings", mappingsXML);

    Implement::ButtonMappings currentButtonMap;
    std::string button_name, command;
    int buttonId;
    XmlRpc::XmlRpcValue mappingObject;
    if (mappingsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < mappingsXML.size(); ++i) {
            for (int j = 0; j < mappingsXML[i].size(); ++j){

                mappingObject = mappingsXML[i][j];

                button_name = static_cast<std::string>(mappingObject[0]).c_str();
                command = static_cast<std::string>(mappingObject[1]).c_str();
                //check if control pressed is a button or an axes by the name of the control and compare to the maps
                //if button add to button comands else add to axes commands
                if(isButton(button_name)){
                    buttonId = getButtonIdFromName(button_name);

                    pImplement->button_commands[i][buttonId].data = command;

                    pImplement->button_rates[i][buttonId] = mappingObject[2];
                }else{
                    buttonId = getAxisIdFromName(button_name);

                    pImplement->axis_commands[i][buttonId].data = command;

                    pImplement->axis_rates[i][buttonId] = mappingObject[2];

                    pImplement->axis_ranges[i][buttonId] = mappingObject[3];
                }
            }
            pImplement->number_of_mappings = i+1;
        }
        pImplement->current_mappings_index = 0;
    }else{
        std::cout << "mappingsXML type is not XmlRpc::XmlRpcValue::TypeArray. Make sure the paramater controller_mappings is there" << std::endl;
    }
}


void JoyCommsControl::getCommandTopics(ros::NodeHandle *nh, ros::NodeHandle *nh_param) {
    XmlRpc::XmlRpcValue topicsXML;
    XmlRpc::XmlRpcValue stopCommandsXML;

    nh_param->getParam("/command_topics", topicsXML);
    nh_param->getParam("/stop_commands", stopCommandsXML);

    if (topicsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < topicsXML.size(); ++i) {
            pImplement->command_topics[i] = static_cast<std::string>(topicsXML[i]).c_str();
            pImplement->comms_pubs[i] = nh->advertise<std_msgs::String>(pImplement->command_topics[i], 1, true);
        }
    }
    if (stopCommandsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < stopCommandsXML.size(); ++i) {
            pImplement->stop_commands[i].data = static_cast<std::string>(stopCommandsXML[i]).c_str();
        }
    }
}

JoyCommsControl::JoyCommsControl(ros::NodeHandle *nh, ros::NodeHandle *nh_param) {
    MapButtonNamesToIds();

    pImplement = new Implement;

    nh_param->param<int>("/enable_button", pImplement->enable_button, 0);

    getControllerMappings(nh_param);

    pImplement->sent_disable_msg = false;

    getCommandTopics(nh, nh_param);

    pImplement->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &JoyCommsControl::Implement::joyCallback, pImplement);
}

void JoyCommsControl::Implement::addToCommandQueue(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    std::vector<int> buttons = joy_msg->buttons;
    for (int i = 0; i < buttons.size(); ++i)
    {
        buttons_clicked[i] = buttons[i];
    }
    std::vector<float> axes = joy_msg->axes;
    for (int i = 0; i < axes.size(); ++i)
    {
        float value = axes[i];
        //triggers have a different behavior
        if(i == TRIGGER_L2 || i == TRIGGER_R2){
            //todo when joy starts it publishes triggers as 0 and hence will be registered as being pressed until it is pressed for the first time
            axes_moved[i] = value != 1 ? 1 : 0;
            //this is a fix because triggers have value 0 when they are half pressed
            axes_percentage[i] = (value - 1)/-2;
        }else{
            axes_moved[i] = value < 0 ? -1 : (value > 0 ? 1 : 0);
            axes_percentage[i] = value;
        }
        axes_values[i] = value;
    }
}

void JoyCommsControl::publish_command_with_rate() {
    //TODO take rate from config file for each button
    ros::Rate loop_rate(10);
    //loop through clicked buttons
    //todo use length of buttons_clicked instead of 11
    for (int i = 0; i < 11; ++i)
    {
        // TODO in last condition rate is used to check existance of command. consider changing it
        if (i != pImplement->enable_button && pImplement->buttons_clicked[i] == 1 && pImplement->button_rates[pImplement->current_mappings_index][i] > 0)
        {
            pImplement->publish_command(pImplement->button_commands[pImplement->current_mappings_index][i]);
        }
    }

    //loop through moved axes
    //todo use length of axes_moved instead of 8
    for (int i = 0; i < 8; ++i)
    {
        // TODO in last condition rate is used to check existance of command. consider changing it
        if (pImplement->axes_moved[i] != 0 && pImplement->axis_rates[pImplement->current_mappings_index][i] > 0)
        {
            std::string commandAsString = pImplement->axis_commands[pImplement->current_mappings_index][i].data;
            std::string newCommandAsString = commandAsString;
            //replace the values in the command if % is present
            int index = commandAsString.find('%');
            while(index != std::string::npos){
                newCommandAsString = commandAsString.substr(0, index);
                if(commandAsString[index+1] == 'b'){//the axis is treated as a button. pass 1 or -1
                    newCommandAsString.append(std::to_string(pImplement->axes_moved[i]));
                    newCommandAsString.append(commandAsString.substr(index+2, commandAsString.length()));
                }else{//the axis is treated as an axes. pass percentage multiplied by range
                    int range = pImplement->axis_ranges[pImplement->current_mappings_index][i];
                    newCommandAsString.append(std::to_string(pImplement->axes_percentage[i] * range));
                    newCommandAsString.append(commandAsString.substr(index+2, commandAsString.length()));
                }

                commandAsString = newCommandAsString;
                index = commandAsString.find('%');
            }
            std_msgs::String command;
            command.data = newCommandAsString;
            pImplement->publish_command(command);
        }
    }
    loop_rate.sleep();
}

void JoyCommsControl::Implement::publish_command(std_msgs::String command) {
    comms_pubs[current_mappings_index].publish(command);
    sent_disable_msg = false;
}

void JoyCommsControl::Implement::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    if (joy_msg->buttons.size() > enable_button && joy_msg->buttons[enable_button]) {
        if (joy_msg->axes[DPAD_X] != 0 || joy_msg->axes[DPAD_Y] != 0) {
            int new_mapping_index;
            if (joy_msg->axes[DPAD_X] == 1)
            {
                new_mapping_index = 0;
                
            } else if (joy_msg->axes[DPAD_Y] == 1)
            {
                new_mapping_index = 1;
                
            } else if (joy_msg->axes[DPAD_X] == -1)
            {
                new_mapping_index = 2;

            } else if (joy_msg->axes[DPAD_Y] == -1)
            {
                new_mapping_index = 3;
            }
            if (number_of_mappings > new_mapping_index)
            {
                if (current_mappings_index == new_mapping_index)
                {
                    std::cout << "Mapping already active. Nothing changed" << std::endl;
                }else{
                    comms_pubs[current_mappings_index].publish(stop_commands[current_mappings_index]);
                    current_mappings_index = new_mapping_index;
                    std::cout << "Mapping changed. Will publish on topic: " << command_topics[current_mappings_index] << std::endl;
                }
                
            }else{
                std::cout << "No mapping provided. Nothing changed" << std::endl;
            }
        }else{
            addToCommandQueue(joy_msg);
        }
    } else {
        //clear buttons_clicked array since enable button is not clicked
        //consider moving it to "not publish" when enable is not clicked
        //todo use length of buttons_clicked instead of 11
        for(int i =0; i< 11 ;i++){
            buttons_clicked[i] = 0;
        }
        for(int i =0; i< 8 ;i++){
            axes_moved[i] = 0;
        }

        if (!sent_disable_msg) {
            comms_pubs[current_mappings_index].publish(stop_commands[current_mappings_index]);
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