#include "arm_controller_node.h"
#include <byteswap.h>
#include <string.h>

ArmControllerNode::ArmControllerNode(): LifecycleNode("arm_controller_node") {}

callbackReturn ArmControllerNode::on_configure(const rclcpp_lifecycle::State &){
    this->declare_parameter("local_mode", false);
    bool local_mode = this->get_parameter("local_mode").as_bool();
    RCLCPP_INFO(this->get_logger(),"entered on_configured");



    if (!local_mode){
        fd = open("/dev/ttyTHS0",O_RDWR);
        if(fd < 0){
            int errno0 = errno;
            RCLCPP_ERROR(this->get_logger(),"Error opening file : %i. Message: %s\n",errno0, strerror(errno));
            errno = 0;
            rclcpp::shutdown();
            return callbackReturn::FAILURE;
        }   
    }

    RCLCPP_INFO(this->get_logger(),"Initialized node : %s\n",this->get_name());

    struct termios ttycfg; 
  
    // tcgetattr(fd, &tty);
    ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
    ttycfg.c_lflag = 0; 
    ttycfg.c_iflag = 0; 
    ttycfg.c_oflag = 0; 
    ttycfg.c_line = 0; 
    ttycfg.c_cc[VTIME] = 1; // 100ms timeout
    ttycfg.c_cc[VMIN] = 0; // Return anything read so far
    cfsetispeed(&ttycfg,B57600);
    cfsetospeed(&ttycfg,B57600);

    if(!local_mode){
        tcsetattr(fd, TCSANOW, &ttycfg);   
    }

//  GPIO::setmode(GPIO::BOARD);

//  GPIO::setup(29, GPIO::OUT, GPIO::HIGH);
//  GPIO::setup(31, GPIO::OUT, GPIO::LOW);
//  GPIO::setup(33, GPIO::OUT, GPIO::LOW);

    joy_msg_callback = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ArmControllerNode::JoyMessageCallback, this, std::placeholders::_1)
    );
    
    arm_vals_msg_callback = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "arm_values", 10, std::bind(&ArmControllerNode::ArmMessageCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    return callbackReturn::SUCCESS;
}

callbackReturn ArmControllerNode::on_activate(const rclcpp_lifecycle::State & state){
    LifecycleNode::on_activate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    return callbackReturn::SUCCESS;
}

callbackReturn ArmControllerNode::on_deactivate(const rclcpp_lifecycle::State & state){
    LifecycleNode::on_deactivate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return callbackReturn::SUCCESS;
}

callbackReturn ArmControllerNode::on_cleanup(const rclcpp_lifecycle::State &){

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return callbackReturn::SUCCESS;
}

callbackReturn ArmControllerNode::on_shutdown(const rclcpp_lifecycle::State & state){

    RCUTILS_LOG_INFO_NAMED(
        get_name(),
        "on shutdown is called from state %s.",
        state.label().c_str()
    );

    rclcpp::shutdown();
    return callbackReturn::SUCCESS;
}


ArmControllerNode::~ArmControllerNode() {
    if (!this->get_parameter("local_mode").as_bool()){
        if (fd >= 0) {
            close(fd);
        }
    }
}

void ArmControllerNode::ArmMessageCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    uint8_t out_buf[1 + 1 + sizeof(float)*6 + 1] ={};
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float)*6;

    float speeds [6]= {msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]};
    for(int i = 0 ; i < 6 ; i++){
        float speed = speeds[i] * MAX_MOTOR_SPEED;
        memcpy(&out_buf[ (i*sizeof(float)) +2],&speed,sizeof(float));
    }
    out_buf[26] = 0x0A;

    if (!this->get_parameter("local_mode").as_bool()){   
        int status = write(fd,out_buf,sizeof(out_buf));
        if(status == -1){
            // int errno0 = errno;
            // RCLCPP_WARN(this->get_logger(),"Write error : %s (%i)\n",strerr(errno0),errno0);
            errno = 0;
        }
    }
}

void ArmControllerNode::JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
    if (controller_type == -1) {
        // Infer controller type. Assume that L2 and R2 are not pressed on startup,
        // and so will be at values 1.0.
        if (joy_msg->axes[2] == 1.0 && joy_msg->axes[5] == 1.0) {
            RCLCPP_INFO(this->get_logger(), "Controller type 0");
            controller_type = 0;
        } else if (joy_msg->axes[4] == 1.0 && joy_msg->axes[5] == 1.0) {
            RCLCPP_INFO(this->get_logger(), "Controller type 1");
            controller_type = 1;
        } else {
            return;
        }
    }

    // Must hold L1 and R1 to get arm control
    if (controller_type == 0) {
        if(joy_msg->buttons[4] == 0 || joy_msg->buttons[5] == 0){
            return;
        }
    } else {
        // Or when L1 and R1 get fuckiing reammped?
        if (( joy_msg->buttons[9] == 0 || joy_msg->buttons[10] == 0 ) ){
            return;
        }
    }


    // LEFT HORIZ : axes[0]
    // LEFT VERT : axes[1]
    // if(joy_msg->buttons[4] == 1){
    //     joy_msg->axes[]
    // }
    // RIGHT HORIZ : axes[3]
    // RIGHT VERT : axes[4]
    
    // L2 : axes [2]
    // R2 : axes [5]


    // joy_msg->axes[2] = (joy_msg->axes[2] - 1.f)/2.f;
    
    // joy_msg->axes[5] = (joy_msg->axes[5] - 1.f)/2.f;

    /*
        Hold R2 or L2 to control the turning the base rotation
        
    */
    float rotation;

    if (controller_type == 0) {
        rotation = (joy_msg->axes[2] - joy_msg->axes[5]) / 2;
    } else {
        if (joy_msg->axes[4] < 1) {
            rotation = -(joy_msg->axes[4] - 1.f) / 2.f;
        } else if (joy_msg->axes[5] < 1) {
            rotation = (joy_msg->axes[5] - 1.f) / 2.f;
        }
    }

    //Map so L2 and R2 control base, left x, left y, right x then control the rest of the 12v motors
    float speeds[6] = {rotation, joy_msg->axes[0],joy_msg->axes[1],joy_msg->axes[3],0,0};
    
    if (controller_type == 1) {
        // Invert certain axes in this case
        speeds[2] = -speeds[2];
        speeds[3] = -speeds[3];
    }

    if (controller_type == 0) {
        // circle-square controls the first smart servo
        speeds[4] = joy_msg->buttons[1] - joy_msg->buttons[3];
        // triangle-cross controls the second smart servo
        speeds[5] = joy_msg->buttons[2] - joy_msg->buttons[0];
    } else {
        // circle-square controls the first smart servo (Other fukcing mapping?)
        speeds[4] = joy_msg->buttons[1] - joy_msg->buttons[2];
        // triangle-cross controls the second smart servo
        speeds[5] = joy_msg->buttons[3] - joy_msg->buttons[0];
    }


    // std::cout << "Motor speeds (from -1 1) " << speeds[0] << " "  << speeds[1] << " "  << speeds[2] << " "  << " "  << speeds[3] << " "  << speeds[4] << " "  << speeds[5] << std::endl;
    // // RIGHT BUMPER
    // if(joy_msg->buttons[7] == 1){
    //     speeds[2] = speeds[5] * -1.f;
    // }
    
    
    uint8_t out_buf[1 + 1 + sizeof(float)*6 + 1] ={};
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float)*6;

    for(int i = 0 ; i < 6 ; i++){
        float speed = speeds[i] * MAX_MOTOR_SPEED;
        memcpy(&out_buf[ (i*sizeof(float)) +2],&speed,sizeof(float));
        
    }
    out_buf[26] = 0x0A;

    /* THIS FUCKING LINE CAUSES THE LINUX KERNEL TO CRASH WHEN USED (DMA ERROR???!!?!)
    // tcflush(fd, TCIOFLUSH); 
    */
   if (!this->get_parameter("local_mode").as_bool()){
        int status = write(fd,out_buf,sizeof(out_buf));
        if(status == -1){
            std::cout << "Error : " << errno << " \n"; 
        }
   }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<ArmControllerNode> arm_controller_node = std::make_shared<ArmControllerNode>();
    exe.add_node(arm_controller_node->get_node_base_interface());

    exe.spin();
    rclcpp::shutdown();
    return 0;
}
