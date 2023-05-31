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
#include <algorithm>
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

    std::vector<std_msgs::String>  stop_commands;

    bool sent_disable_msg;

    std::vector<std::string>  command_topics;

    std::vector<ros::Publisher>  comms_pubs;

    int next_layout_button;
    int previous_layout_button;
    int trigger_l2;
    int trigger_r2;

    int numberOfButtons;
    std::vector<std::vector<std_msgs::String>> button_commands;
    std::vector<std::vector<int>> button_rates;
    std::vector<int> buttons_clicked;

    int numberOfAxes;
    std::vector<std::vector<std_msgs::String>> axis_commands;
    std::vector<std::vector<int>> axis_rates;
    std::vector<std::vector<int>> axis_ranges;
    std::vector<int>  axes_moved;
    std::vector<float> axes_values;
    std::vector<float> axes_percentage;

    int number_of_mappings = 0;
    int current_mappings_index = 0;

    bool change_layout_button_held = false;

    float motor_max = 250.0;
};

struct JoyCommsControl::Implement::ButtonMappings {
    int buttonId;
    std_msgs::String command_string;
    int command_publish_rate;
};

void JoyCommsControl::MapButtonNamesToIds() {
    if (controller_type == PLAYSTATION)
    {    
        for (int i = 0; i < pImplement->numberOfButtons; ++i) {
            button_name_to_id_map.insert(std::pair<std::string, int>(ps_button_names[i], i));
        }
        for (int i = 0; i< pImplement->numberOfAxes; ++i) {
            axis_name_to_id_map.insert(std::pair<std::string, int>(ps_axis_names[i], i));
        }
    }else if(controller_type == XBOX){    
        for (int i = 0; i < pImplement->numberOfButtons; ++i) {
            button_name_to_id_map.insert(std::pair<std::string, int>(xbox_button_names[i], i));
        }
        for (int i = 0; i< pImplement->numberOfAxes; ++i) {
            axis_name_to_id_map.insert(std::pair<std::string, int>(xbox_axis_names[i], i));
        }
    }else if(controller_type == PLAYSTATION_UBUNTU_18){
        for (int i = 0; i < pImplement->numberOfButtons; ++i) {
            button_name_to_id_map.insert(std::pair<std::string, int>(ps_u18_button_names[i], i));
        }
        for (int i = 0; i< pImplement->numberOfAxes; ++i) {
            axis_name_to_id_map.insert(std::pair<std::string, int>(ps_u18_axis_names[i], i));
        }
    }else{
        std::cout << "error in MapButtonNamesToIds: controller type unknown" << std::endl;
        exit(1);
    }
}

void JoyComsControl::mapControlModeToColor() {
    int command_topic_num = 0;
    for (std::string command_topic : command_topics) {
        control_mode_to_color.insert(std::pair<std::string, std::truple<int, int, int>>(command_topic, light_colors[command_topic_num]);
	command_topic_num++;
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
    nh_param->getParam("controller_mappings", mappingsXML);

    Implement::ButtonMappings currentButtonMap;
    std::string button_name, command;
    int buttonId;
    XmlRpc::XmlRpcValue mappingObject;
    if (mappingsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        //for each layout
        for (int i = 0; i < mappingsXML.size(); ++i) {
            //set up the command vectors for each mapping
            //buttons
            std::vector<std_msgs::String> buttons;
            std::vector<int> buttonRates;
            for (size_t i = 0; i < pImplement->numberOfButtons; i++)
            {
                std_msgs::String command;
                buttons.push_back(command);

                buttonRates.push_back(-1);                
            }
            pImplement->button_commands.push_back(buttons);
            pImplement->button_rates.push_back(buttonRates);
            //axes
            std::vector<std_msgs::String> axes;
            std::vector<int> axesRates;
            std::vector<int> axesRanges;
            for (size_t i = 0; i < pImplement->numberOfAxes; i++)
            {
                std_msgs::String command;
                axes.push_back(command);

                axesRates.push_back(-1);
                axesRanges.push_back(-1);
            }
            pImplement->axis_commands.push_back(axes);
            pImplement->axis_rates.push_back(axesRates);
            pImplement->axis_ranges.push_back(axesRanges);

            //for each button
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

    nh_param->getParam("command_topics", topicsXML);
    nh_param->getParam("stop_commands", stopCommandsXML);

    if (topicsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < topicsXML.size(); ++i) {
            pImplement->command_topics.push_back(static_cast<std::string>(topicsXML[i]).c_str());
            pImplement->comms_pubs.push_back(nh->advertise<std_msgs::String>(pImplement->command_topics[i], 1, true));

	    if (pImplement->command_topics[i] == "/rover_command") {
	        plight_indicator_pub = &pImplement->comms_pubs.at(i);
	    }
        }
    }
    if (stopCommandsXML.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < stopCommandsXML.size(); ++i) {
            std_msgs::String stopCommand;
            stopCommand.data = static_cast<std::string>(stopCommandsXML[i]).c_str();
            pImplement->stop_commands.push_back(stopCommand);
        }
    }
}

JoyCommsControl::JoyCommsControl(ros::NodeHandle *nh, ros::NodeHandle *nh_param) {

    pImplement = new Implement;

    nh_param->param<int>("controller_type", controller_type, 1);

    if (controller_type == PLAYSTATION)
    {
        std::cout << "controller is PLAYSTATION controller" << std::endl;
        pImplement->numberOfButtons = sizeof(ps_button_names)/sizeof(ps_button_names[0]);
        pImplement->numberOfAxes = sizeof(ps_axis_names)/sizeof(ps_axis_names[0]);
    }else if (controller_type == PLAYSTATION_UBUNTU_18)
    {
        std::cout << "controller is PLAYSTATION controller on Ubuntu 18.04" << std::endl;
        pImplement->numberOfButtons = sizeof(ps_u18_button_names)/sizeof(ps_u18_button_names[0]);
        pImplement->numberOfAxes = sizeof(ps_u18_axis_names)/sizeof(ps_u18_axis_names[0]);
    }else if(controller_type == XBOX){
        std::cout << "controller is XBOX controller" << std::endl;
        pImplement->numberOfButtons = sizeof(xbox_button_names)/sizeof(xbox_button_names[0]);
        pImplement->numberOfAxes = sizeof(xbox_axis_names)/sizeof(xbox_axis_names[0]);
    }else{
        std::cout << "error in JoyCommsControl constructor: controller type unknown" << std::endl;
        exit(1);
    }
    
    pImplement->buttons_clicked.resize(pImplement->numberOfButtons, 0);
    pImplement->axes_moved.resize(pImplement->numberOfAxes, 0);
    pImplement->axes_values.resize(pImplement->numberOfAxes);
    pImplement->axes_percentage.resize(pImplement->numberOfAxes);
    
    MapButtonNamesToIds();
    nh_param->param<int>("enable_button", pImplement->enable_button, 0);
    std::string next, previous;
    nh_param->param<std::string>("next_layout_button", next, option_name);
    nh_param->param<std::string>("previous_layout_button", previous, share_name);

    pImplement->next_layout_button = button_name_to_id_map[next];
    pImplement->previous_layout_button = button_name_to_id_map[previous];

    pImplement->trigger_l2 = axis_name_to_id_map[trigger_l2_name];
    pImplement->trigger_r2 = axis_name_to_id_map[trigger_r2_name];

    getControllerMappings(nh_param);

    pImplement->sent_disable_msg = false;

    getCommandTopics(nh, nh_param);

    pImplement->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &JoyCommsControl::Implement::joyCallback, pImplement);
    
    mapControlModeToColor();
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
        if(i == trigger_l2 || i == trigger_r2){
            //when joy starts it publishes triggers as 0 and hence will be registered as being pressed until it is pressed for the first time
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
    
    //define a vector of commands to be filled and published
    std::vector<std_msgs::String> commands;

    //loop through clicked buttons
    for (int i = 0; i < pImplement->numberOfButtons; ++i)
    {
        // TODO in last condition rate is used to check existance of command. consider changing it
        if (i != pImplement->enable_button && pImplement->buttons_clicked[i] == 1 && pImplement->button_rates[pImplement->current_mappings_index][i] > 0)
        {
            commands.push_back(pImplement->button_commands[pImplement->current_mappings_index][i]);
        }
    }

    //loop through moved axes
    for (int i = 0; i < pImplement->numberOfAxes; ++i)
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
                    float value = pImplement->axes_percentage[i] * range;
                    
                    // handle -
                    if (newCommandAsString.back() == '-') {
                        value *= -1;
                        newCommandAsString.pop_back();
                    }

                    newCommandAsString.append(std::to_string(value));
                    newCommandAsString.append(commandAsString.substr(index+2, commandAsString.length()));
                }

                commandAsString = newCommandAsString;
                index = commandAsString.find('%');
            }
            std_msgs::String command;
            command.data = newCommandAsString;
            commands.push_back(command);
            // std::cout << "adding command: " << command << " axis index " << i << " axis range " << pImplement->axis_ranges[pImplement->current_mappings_index][i] << "\n";
        }
    }

    if(commands.size() > 0) {
        //special case for arm_command and rover_commmand command
        if(pImplement->command_topics[pImplement->current_mappings_index] == "/arm_command"
           || pImplement->command_topics[pImplement->current_mappings_index] == "/rover_command") {
            //check if the command is "set_motor_speeds" or "rover_command" which have the special case
            std::string command_name = commands[0].data.substr(0,commands[0].data.find(' '));
            if(command_name == "set_motor_speeds" || command_name == "move_wheels") {
                float motors [6];
                std::fill(motors, motors+6, 0);

                //parse the motor values into seperate motor variables
                for(int i = 0; i < commands.size(); ++i){
                    std::string commandAsString = commands[i].data;
                    // Only do this if it is the appropriate command
                    if (commandAsString.rfind(command_name, 0) == 0) {
                        for(int j = 0; j < 6; ++j) {
                            commandAsString = commandAsString.substr(commandAsString.find(' ') + 1, commandAsString.length());
                            motors[j] += std::stof(commandAsString.substr(0, commandAsString.find(' ')));
                        }
                    }
                }

                // clamp value
                for (int i = 0; i < 6; i++) {
                    motors[i] = std::min(motors[i], pImplement->motor_max);
                    motors[i] = std::max(motors[i], -(pImplement->motor_max));
                }

                //rebuild the command with the values of all motors
                std::string combinedCommandAsString = command_name;
                for(int i =0; i < 6; ++i){
                    combinedCommandAsString.append(" ").append(std::to_string(motors[i]));
                }

                //create the combined command
                std_msgs::String command;
                command.data = combinedCommandAsString;
                
                //publish the combined command
                pImplement->publish_command(command);
            }
        }else{
            //publish normally for topics other than arm_command
            for(int i = 0; i < commands.size(); ++i){
                pImplement->publish_command(commands[i]);
            }
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
        if ((joy_msg->buttons[next_layout_button] || joy_msg->buttons[previous_layout_button])
                                                                && !change_layout_button_held) {
            change_layout_button_held = true;

            int new_mapping_index;
            if (joy_msg->buttons[next_layout_button])
            {
                new_mapping_index = (current_mappings_index + 1) % number_of_mappings;
            }
            if (joy_msg->buttons[previous_layout_button])
            {
                new_mapping_index = std::abs(current_mappings_index - 1) % number_of_mappings;
            }
            if (current_mappings_index == new_mapping_index)
            {
                std::cout << "Mapping already active. Nothing changed" << std::endl;
            }else{
                comms_pubs[current_mappings_index].publish(stop_commands[current_mappings_index]);
                current_mappings_index = new_mapping_index;
                std::cout << "Mapping changed. Will publish on topic: " << command_topics[current_mappings_index] << std::endl;

		std::truple<int, int, int> mode_color = control_mode_to_color[command_topics[current_mappings_index]];

                plight_indicator_pub->publisher(std::format(LIGHT_CONTROL_COMMAND, control_mode_to_color[0], control_mode_to_color[1], control_mode_to_color[2]));
            }
        }else{
            change_layout_button_held = false;
            addToCommandQueue(joy_msg);
        }
    } else {
        change_layout_button_held = false;

        //clear buttons_clicked array since enable button is not clicked
        //consider moving it to "not publish" when enable is not clicked
        for(int i =0; i< numberOfButtons ;i++){
            buttons_clicked[i] = 0;
        }
        for(int i =0; i< numberOfAxes ;i++){
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

    std::cerr<< "WARNING: After running the joy_node from the joy package, press the R2 and L2 triggers once before using the controller." << std::endl;
    while (ros::ok())
    {
        joy_control.publish_command_with_rate();
        ros::spinOnce();
    }
}
