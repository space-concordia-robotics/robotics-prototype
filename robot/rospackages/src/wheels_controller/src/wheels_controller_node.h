
#ifndef WHEELS_CONTROLLER_NODE_H
#define WHEELS_CONTROLLER_NODE_H
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rev_motor_controller.h"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <builtin_interfaces/msg/time.hpp>
// #include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <chrono>
using namespace std::chrono;

#define DEVICE_1_ID 1
#define DEVICE_2_ID 2
#define DEVICE_3_ID 3
#define DEVICE_4_ID 4
#define DEVICE_5_ID 5
#define DEVICE_6_ID 6

using namespace std::literals::chrono_literals;
typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn callbackReturn;

class WheelsControllerNode : public rclcpp_lifecycle::LifecycleNode{
  public:
    WheelsControllerNode();
    callbackReturn on_configure(const rclcpp_lifecycle::State &);
    callbackReturn on_activate(const rclcpp_lifecycle::State & state);
    callbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
    callbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    callbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

    void TwistMessageCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
    void JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  private :

    void AccelerateTwist(geometry_msgs::msg::Twist);
    float AccelerateValue(float current, float desired, float rate, float dt);
    
    void publishOdom();
    void publishStop();

    // void Zed2OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    //rclcpp::callback_group::CallbackGroup::SharedPtr update_group;
    rclcpp::TimerBase::SharedPtr timer_;
		// rclcpp::Publisher::SharedPtr pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_msg_callback;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_msg_publisher;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sil_publisher;
    
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_msg_callback;

    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr zed2_odom_callback;
    

    void pollControllersCallback();
    
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr odom_timer;
    
    // nav_msgs::msg::Odometry current_odom;


    float old_linear_y;
    float old_angular_z;
    
    float linear_y;
    float angular_z;
    float max_speed = 0.5;
    std::chrono::time_point<std::chrono::system_clock> start;
    float sil_mode=0;
    float max_angular_speed = 1;
    bool is_manual_control = false;
    bool reached_goal = false;
    
    std::string color;

//    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;

  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
    navigation_goal_status_sub_;

  // The mapping seems to change randomly between reboots. Stores the
  // inferred type of the controller.
  // Type 0 is where L2 and R2 are at axes[2] and axes[5],
  // Type 1 is where L2 and R2 are at axes[4] and axes[5],
  // Type 2 is the flight joystick
  int controller_type = -1;

   //navigation_goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>;
};

#endif
