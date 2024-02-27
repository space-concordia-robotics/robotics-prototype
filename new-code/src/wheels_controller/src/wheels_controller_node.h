
#ifndef WHEELS_CONTROLLER_NODE_H
#define WHEELS_CONTROLLER_NODE_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rev_motor_controller.h"
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
using namespace std::chrono;

#define DEVICE_1_ID 1
#define DEVICE_2_ID 2
#define DEVICE_3_ID 3
#define DEVICE_4_ID 4
#define DEVICE_5_ID 5
#define DEVICE_6_ID 6

using namespace std::literals::chrono_literals;

class WheelsControllerNode : public rclcpp::Node{
public:
    WheelsControllerNode();
    void TwistMessageCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
    void JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

private :

    void accelerateTwist();
    void accelerateValue();

    //rclcpp::callback_group::CallbackGroup::SharedPtr update_group;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msg_callback;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_msg_publisher;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msg_callback;

    // geometry_msgs::msg::Twist twist_msg;

    void pollControllersCallback();
    rclcpp::TimerBase::SharedPtr timer;

    float old_linear_y;
    float old_angular_z;
    
    float linear_y;
    float angular_z;
    float linear_acceleration_rate = 0.5;
    float angular_acceleration_rate = 0.75;
    float max_speed = 0.5;
    float max_angular_speed = 1;

//    rclcpp::Publisher<geometry_msgs::msg::Twist_>::SharedPtr twist_message_publisher;
};

#endif