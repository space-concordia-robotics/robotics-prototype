#include "arm_controller_node.h"
#include <byteswap.h>
#include <string.h>


ArmControllerNode::ArmControllerNode(): Node("arm_controller_node") {
    fd = open("/dev/ttyTHS0",O_RDWR);
    if(fd < 0){
        int errno0 = errno;
        RCLCPP_ERROR(this->get_logger(),"Error opening file : %i. Message: %s\n",errno0, strerror(errno));
        errno = 0;
        rclcpp::shutdown();
        return;
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
    tcsetattr(fd, TCSANOW, &ttycfg);

//  GPIO::setmode(GPIO::BOARD);

//  GPIO::setup(29, GPIO::OUT, GPIO::HIGH);
//  GPIO::setup(31, GPIO::OUT, GPIO::LOW);
//  GPIO::setup(33, GPIO::OUT, GPIO::LOW);

    
    // this->create_wall_timer( 500ms, std::bind(&WheelsControllerNode::pollControllersCallback, this));

    joy_msg_callback = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ArmControllerNode::JoyMessageCallback, this, std::placeholders::_1)
            );
    
    arm_vals_msg_callback = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "arm_values", 10, std::bind(&ArmControllerNode::ArmMessageCallback, this, std::placeholders::_1)
            );
    
}

ArmControllerNode::~ArmControllerNode() {
    if (fd >= 0) {
        close(fd);
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
        
    int status = write(fd,out_buf,sizeof(out_buf));
    if(status == -1){
        int errno0 = errno;
        // RCLCPP_WARN(this->get_logger(),"Write error : %s (%i)\n",strerr(errno0),errno0);
        errno = 0;
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


    std::cout << "Motor speeds (from -1 1) " << speeds[0] << " "  << speeds[1] << " "  << speeds[2] << " "  << " "  << speeds[3] << " "  << speeds[4] << " "  << speeds[5] << std::endl;
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
    int status = write(fd,out_buf,sizeof(out_buf));
    if(status == -1){
        std::cout << "Error : " << errno << " \n"; 
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControllerNode>());
    rclcpp::shutdown();
    return 0;
}
