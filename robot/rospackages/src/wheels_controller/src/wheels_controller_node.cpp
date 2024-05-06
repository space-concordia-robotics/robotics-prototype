#include "wheels_controller_node.h"

WheelsControllerNode::WheelsControllerNode(): Node("wheels_controller") {
    this->declare_parameter("can_path", "can0");
    this->declare_parameter("multiplier", 500);
    
    this->declare_parameter("linear_acceleration_rate", 0.25);
    this->declare_parameter("angular_acceleration_rate", 0.25);
    this->declare_parameter("initial_ramp_factor", 3);

    // this->declare_parameter("turning_rate", 2000);
    // float linear_acceleration_rate = 0.5

    start = std::chrono::system_clock::now();


    if(CANController::configureCAN("can0") != SUCCESS){
        RCLCPP_ERROR(this->get_logger(),"Error accessing CAN interface \n");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(),"Initialized node : %s\n",this->get_name());

    /*)
     * This command will inform a motor controller to start transmitting periodic status frames.
     */
    // RevMotorController::requestStatusFrame();
    
    // update_group = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    
    timer = this->create_wall_timer( 50ms, std::bind(&WheelsControllerNode::pollControllersCallback, this));
    
    // odom_timer = this->create_wall_timer( 50ms, std::bind(&WheelsControllerNode::OdomCallback, this));
    
    joy_msg_callback = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&WheelsControllerNode::JoyMessageCallback, this, std::placeholders::_1)
            );

    twist_msg_callback = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&WheelsControllerNode::TwistMessageCallback, this, std::placeholders::_1)
            );
    twist_msg_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);     
    
    // odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);     

    // zed2_odom_callback = this->create_subscription<nav_msgs::msg::Odometry>(
            // "zed/zed_node/odom", 10, std::bind(&WheelsControllerNode::Zed2OdomCallback, this, std::placeholders::_1));
    
   
    sil_publisher = this->create_publisher<std_msgs::msg::String>("SIL_Color", 10);

     navigation_goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status",
    rclcpp::SystemDefaultsQoS(),
    [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        int status = msg->status_list.back().status;
        if(status == 2){
            this->is_manual_control = false;
        }
        else if(status == 4){
            this->reached_goal = true;
        }
        else{
            this->is_manual_control = true;     
        }

    });

}
// void WheelsControllerNode::publishOdom(){
//     odom_publisher->publish(current_odom); 
// }

// void WheelsControllerNode::Zed2OdomCallback(const nav_msgs::msg::Odometry::SharedPtr zed2_odom_msg){
//     auto pose = zed2_odom_msg->pose;
//     auto header = zed2_odom_msg->header;
    
//     geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist{};
//     twist.linear.x = this->linear_y;
//     twist.angular.z = this->angular_z;

//     header.frame_id = "odom";

//     auto odom_msg = nav_msgs::msg::Odometry{};
//     odom_msg.pose = pose;
//     odom_msg.header = header;
//     odom_msg.twist.twist = twist;
//     odom_msg.child_frame_id = "base_link";

//     odom_publisher->publish(odom_msg);
    
// }

void WheelsControllerNode::JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){


    if(joy_msg->buttons[0] ==1){
        color = "#0000FF";
    }
    if(joy_msg->buttons[1] ==1){
        color = "#FF0000";
    }
    if(joy_msg->buttons[3] ==1){
        color = "#00FF00";
    }
    
    // Only move if holding down R1 only (that is, L1 has to be unpressed and R1 pressed)
     if( ! ( joy_msg->buttons[9] == 1 && joy_msg->buttons[10] == 0 ) ){
        return;
    }
    float linear_y_axes_val = joy_msg->axes[1];
    float angular_z_axes_val = joy_msg->axes[2];

    geometry_msgs::msg::Twist twist_msg = geometry_msgs::msg::Twist{};
    twist_msg.linear.x = linear_y_axes_val;
    twist_msg.angular.z = angular_z_axes_val;

    twist_msg_publisher->publish(twist_msg);

    
}   

void WheelsControllerNode::pollControllersCallback(){
    
    /*
        Rover not moving
    */
    // if( this->reached_goal ){
    //     auto msg = std_msgs::msg::String{};
    //     msg.data = "#00FF000";
    //     sil_publisher->publish(msg);
    //     return;
    // }

    if( abs(this->linear_y) < 1e-3 && abs(this->angular_z) < 1e-3){
        auto msg = std_msgs::msg::String{};
        msg.data = "#0000000";
        sil_publisher->publish(msg);
        return;
    }
    /*
        Rover is in motion. Determine is manually controlled or autonomous.
    */
    else{
         /*
        Set SIL Color to RED for manual control
        */
        auto msg = std_msgs::msg::String{};
        // if(this->is_manual_control){
        //     msg.data = "#FF00000";
        // }
        // else{	
        //     msg.data = "#0000FF";
        // }
        msg.data = color;
        sil_publisher->publish(msg);
    }
    
    float slip_track = 1.2f;
    //this->angular_z *= -1.f;
    // This can be derived from the equation in the paper (to be fucking linked).
    float right_wheels_velocity = this->linear_y - ( -this->angular_z * slip_track * 0.5f);
    float left_wheels_velocity = this->linear_y + ( -this->angular_z * slip_track * 0.5f);

    float right_wheels_vel_rpm = right_wheels_velocity * this->get_parameter("multiplier").as_int();
    float left_wheels_vel_rpm = left_wheels_velocity * this->get_parameter("multiplier").as_int();
    
    /*
        Motors 3 and 6 are currently brushed, hence they will not run at the same speed as the brushless motors.
    */
    RevMotorController::velocityControl(1,right_wheels_vel_rpm);    
    RevMotorController::velocityControl(2,right_wheels_vel_rpm);
    RevMotorController::velocityControl(3,right_wheels_vel_rpm*0.5);
    
    RevMotorController::velocityControl(4,left_wheels_vel_rpm);
    RevMotorController::velocityControl(5,left_wheels_vel_rpm);
    RevMotorController::velocityControl(6,left_wheels_vel_rpm*0.5);
    
    uint64_t mask = 0x7E;
    RevMotorController::startMotor(mask);
}

void WheelsControllerNode::AccelerateTwist(geometry_msgs::msg::Twist twist_msg){
    static float last_speed_change_ms;
    static float last_linear_speed = 0.0f;
    static float last_angular_speed = 0.0f;
    static float expire_rate = 300.f;

    bool is_expired = false;
    
    float twist_linear = twist_msg.linear.x;
    float twist_angular = twist_msg.angular.z;

    std::chrono::milliseconds time_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    
    if(last_speed_change_ms != 0.f){
        if( (time_ms.count() - last_speed_change_ms) > expire_rate){
            std::cout << "Command expired\n";
            is_expired = true;
        }
        else {
            is_expired = false;
        }
    }
    if (last_speed_change_ms < 1e-7 || is_expired) {
        last_linear_speed = 0;
        last_angular_speed = 0;
        last_speed_change_ms = time_ms.count ()- (expire_rate / this->get_parameter("initial_ramp_factor").as_int());
    }
    float delta =  time_ms.count() - last_speed_change_ms;
    

    this->linear_y = AccelerateValue(last_linear_speed, twist_linear, this->get_parameter("linear_acceleration_rate").as_double(), delta);
    this->angular_z = AccelerateValue(last_angular_speed, twist_angular, this->get_parameter("angular_acceleration_rate").as_double(), delta);
    
    last_linear_speed = this->linear_y;
    last_angular_speed = this->angular_z;

    last_speed_change_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();
}

float WheelsControllerNode::AccelerateValue(float current, float desired, float rate, float dt) {
    /*
    Accelerates the current speed to a desired speed at a certain rate while
    considering a certain time difference. Ex : Current Speed 0.3 m/s, desired speed 0.5 m/s,
    if the rate is 0.1 m/s^2 and dt is 100 milliseconds then the new speed should be 0.31 m/s.
    */
    if( (desired-current) < 1e-6 )
        return desired;

    if(desired < 1e-6)
        return 0;

    if(desired < current)
        rate = -rate;

    float new_value = current + rate * dt /1000;

    if(abs(new_value) > abs(desired)){
        new_value = desired;
    }
    std::cout << new_value << "\n";
    return new_value;
}

void WheelsControllerNode::TwistMessageCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
    this->linear_y = twist_msg->linear.x;
    this->angular_z = twist_msg->angular.z;    

    // auto header = std_msgs::msg::Header{};

    // auto stamp = builtin_interfaces::msg::Time{};
    
    // auto now = std::chrono::system_clock::now();
    // auto time_since_start = std::chrono::duration_cast<std::chrono::nanoseconds>(now-start);

    // stamp.nanosec = time_since_start.count();
    // stamp.sec = time_since_start.count() / 1e9;
    
    // header.frame_id = "odom";
    // header.stamp = stamp;

    // auto point = geometry_msgs::msg::Point{};
    // point.x = 0.03;
    // point.y = -0.03;
    // point.z = -0.619;

    // auto quat = geometry_msgs::msg::Quaternion{};
    // quat.x = 0.006;
    // quat.y = 0.006;
    // quat.z = 0.0002;
    // quat.w = 0.99999;
    
    // auto pose = geometry_msgs::msg::Pose{};
    // pose.position = point;
    // pose.orientation = quat;
    

    
    // auto odom_msg = nav_msgs::msg::Odometry{};
    // auto twist_with_cov = geometry_msgs::msg::TwistWithCovariance{};
    
    // auto pose_with_cov = geometry_msgs::msg::PoseWithCovariance{};
    
    
    //   std::array<double,36> cov_matrix= {
    //     1e-5,0,0,0,0,0,
    //     0,1e-5,0,0,0,0,
    //     0,0,1e11,0,0,0,
    //     0,0,0,1e11,0,0,
    //     0,0,0,0,1e11,0,
    //     0,0,0,0,0,0.001

    // };

    
    // pose_with_cov.covariance = cov_matrix;
    // pose_with_cov.pose = pose;
    
    // twist_with_cov.twist = *twist_msg;
    
    // odom_msg.header = header;
    // odom_msg.child_frame_id = "base_link";
    // odom_msg.twist = twist_with_cov;
    // odom_msg.pose = pose_with_cov;
    
    // this->odom_publisher->publish(odom_msg);

    // // AccelerateTwist(*msg);

    // std::cout  << "Linear  : " << linear_y << "\n";
    // std::cout << "Angular : " << angular_z  << "\n";
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor exec;
    
    auto controller_node = std::make_shared<WheelsControllerNode>();
    exec.add_node(controller_node);
    exec.spin();

    // rclcpp::spin(std::make_shared<WheelsControllerNode>());
    rclcpp::shutdown();
    return 0;
}
