
#ifndef WHEELS_CONTROLLER_NODE_H
#define WHEELS_CONTROLLER_NODE_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
// #include "arm_controller/msg/arm_motor_values.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// #include <JetsonGPIO.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <unistd.h>
#include <cstdint>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <iostream>
#include <fcntl.h>
#define SET_MOTOR_SPEED 0x4E
#define MAX_MOTOR_SPEED 1024.f

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn callbackReturn;

class ArmControllerNode : public rclcpp_lifecycle::LifecycleNode{
    public:
        ArmControllerNode();
        ~ArmControllerNode();
        callbackReturn on_configure(const rclcpp_lifecycle::State &);
        callbackReturn on_activate(const rclcpp_lifecycle::State & state);
        callbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
        callbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        callbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

        void JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
        void ArmMessageCallback(const std_msgs::msg::Float32MultiArray::SharedPtr arm_val_msgs);

    private :
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msg_callback;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_vals_msg_callback;
        rclcpp::TimerBase::SharedPtr timer_;

        int fd;
        // The mapping seems to change randomly between reboots. Stores the
        // inferred type of the controller.
        // Type 0 is where L2 and R2 are at axes[2] and axes[5],
        // Type 1 is where L2 and R2 are at axes[4] and axes[5]
        int controller_type = -1;
};

#endif
