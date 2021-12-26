//
// Created by william on 2021-12-26.
//

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <map>
#include <string>


namespace joy_comms_control {
    struct JoyCommsControl::ModuleControls {
        std::string module_command_topic;

        /*
         * example entry in map:
         * button: 1 | command: "budge_motors 1 0 0 0 0 0"
         */
        std::map<int, std::string> button_command_mappings;
    }

    struct JoyCommsControl::Implement {
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void sendCommsCommand(const sensor_msgs::Joy::ConstPtr& joy_msg);

        ros::Subscriber joy_sub;
        ros::Publisher comms_pubs[];

        int enable_button;

        ModuleControls modules[];

        std::string stop_command;

        bool sent_disable_msg;
    };

    JoyCommsControl::JoyCommsControl(ros::NodeHandle* nh, ros::NodeHandle* nh_param) {
        nh_param->param<int>("command_topic", Implement->enable_button, 0);

        Implement = new Implement;

        nh_param->param<int>("modules", Implement->ModuleControls, 0);

        int number_of_modules = Implement->modules.length(); // Might need to be tested

        // Create module command publishers
        Implement->comms_pubs = new ros::Publisher[numberOfModules];
        int current_index = 0;
        for (int & comms_pub : Implement->comms_pubs) {
            comms_pub = nh->advertise<geometry_msgs::Twist>(Implement->modules[currentIndex]->module_command_topic, 1, true);
            current_index++;
        }

        Implement->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, Implement);

        nh_param->param<int>("enable_button", Implement->enable_button, 0);

        nh_param->param<int>("stop_command", Implement->stop_command, 0);

        Implement->sent_disable_msg = false;
    }

    void JoyCommsControl::Implement::sendCommsCommand(const sensor_msgs::Joy::ConstPtr& joy_msg) {


        cmd_vel_pub.publish(cmd_vel_msg);
        sent_disable_msg = false;
    }

    void JoyCommsControl::Implement::publish_command(const std::string topic, const std::string command) {


        cmd_vel_pub.publish(cmd_vel_msg);
        sent_disable_msg = false;
    }

    void JoyCommsControl::Implement::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        if (joy_msg->buttons.size() > enable_button && joy_msg->buttons[enable_button]) {
            sendCommsCommand(joy_msg);
        } else {
            if (!sent_disable_msg) {
                for (int & comms_pub : Implement->comms_pubs) {
                    comms_pub.publish(stop_command);
                }
                sent_disable_msg = true;
            }
        }
    }
}