#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "absenc_interface/msg/encoder_values.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/string.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"


#define NO_ERROR            0
#define ERR_SERIAL_FAILURE  1
#define ERR_SLAVE_INVALID   2
#define ERR_NO_RESPONSE     3
#define ERR_FRAME_CORRUPTED 4
typedef struct {
    int error; 
    int cause; 
    int line; 
} ABSENC_Error_t;

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn callbackReturn;

// Gets string corresponding to error code
const char* strAbsencErr(int err);

typedef struct {
    uint8_t slvnum; 
    uint16_t status; 
    double angval; 
    double angspd; 
} ABSENC_Meas_t; 

#define no_error (ABSENC_Error_t{0, 0, __LINE__})

extern void ABSENC_ReportError(ABSENC_Error_t err); 
extern void ABSENC_PrintMeas(const ABSENC_Meas_t * meas); 

class AbsencDriver {
public:    
    static ABSENC_Error_t OpenPort(const char* path, uint16_t baud_rate, int& s_fd);
    static ABSENC_Error_t PollSlave(int slvnum, ABSENC_Meas_t * meas, int s_fd);
    static ABSENC_Error_t ClosePort(int s_fd);
};

class Absenc: public rclcpp_lifecycle::LifecycleNode{
public:
    Absenc();
    ~Absenc();
    callbackReturn on_configure(const rclcpp_lifecycle::State &);
    callbackReturn on_activate(const rclcpp_lifecycle::State & state);
    callbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
    callbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    callbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

private:
    void absEncPollingCallback();
    void cadValuesCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void joyValuesCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void ikValuesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void controlEndEffector(float spin, float close);

    int s_fd = -1;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_cad_mouse;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_joy;
    rclcpp::Publisher<absenc_interface::msg::EncoderValues>::SharedPtr angles_publisher;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_controller_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_publisher;

    // Both these store 4 angles to be ready if a 4th encoder is added
    std::array<float, 4> ik_angles = {0.0, 0.0, 0.0, 0.0};
    std::array<float, 4> abs_angles = {0.0, 0.0, 0.0, 0.0};
    // Controls if motor sign is aligned with encoder direction
    std::array<int, 4> motor_signs = {1, -1, -1, -1};
    // Stores value for controlling base motor
    float base_motor_input;
    // Stores values for controlling claw
    float claw_spin, claw_close;
    // Inhibit movement of the arm (for 3d mouse control scheme)
    bool inhibitArmMovement = false;
};