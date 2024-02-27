#include "arm_controller_node.h"
#include <byteswap.h>

ArmControllerNode::ArmControllerNode() : Node("arm_controller_node")
{

    // ARM = [0,0,1]
    fd = open("/dev/ttyTHS0", O_RDWR);
    // fd = open("/dev/ttyACM0",O_RDWR);
    if (fd < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error opening file %i\n", errno);
    }

    struct termios ttycfg;

    // tcgetattr(fd, &tty);
    ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
    ttycfg.c_lflag = 0;
    ttycfg.c_iflag = 0;
    ttycfg.c_oflag = 0;
    ttycfg.c_line = 0;
    ttycfg.c_cc[VTIME] = 1; // 100ms timeout
    ttycfg.c_cc[VMIN] = 0;  // Return anything read so far
    cfsetispeed(&ttycfg, B57600);
    cfsetospeed(&ttycfg, B57600);
    tcsetattr(fd, TCSANOW, &ttycfg);

    GPIO::setmode(GPIO::BOARD);

    GPIO::setup(29, GPIO::OUT, GPIO::HIGH);
    GPIO::setup(31, GPIO::OUT, GPIO::LOW);
    GPIO::setup(33, GPIO::OUT, GPIO::LOW);

    // this->create_wall_timer( 500ms, std::bind(&WheelsControllerNode::pollControllersCallback, this));

    joy_msg_callback = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ArmControllerNode::JoyMessageCallback, this, std::placeholders::_1));

    arm_vals_msg_callback = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "arm_values", 10, std::bind(&ArmControllerNode::ArmMessageCallback, this, std::placeholders::_1));
}

// The message sent over serial has the following format:  1 byte for command, 1 byte for size(n) of data to be read, n bytes of data, 1 stop byte
void ArmControllerNode::ArmMessageCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    uint8_t out_buf[1 + 1 + sizeof(float) * 6 + 1] = {}; // buffer size = 27
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float) * 6;

    float speeds[6] = {msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]};
    for (int i = 0; i < 6; i++)
    {
        // float swapped = bswap_32(speeds[i]);
        memcpy(&out_buf[(i * sizeof(float)) + 2], &speeds[i], sizeof(float));
    }
    out_buf[26] = 0x0A;

    int status = write(fd, out_buf, sizeof(out_buf));

    // TODO: check status before printing success message
    std::cout << "Wrote message " << std::endl;
}

void ArmControllerNode::JoyMessageCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{

    // if(joy_msg->buttons[5] == 0){
    //     return;
    // }
    // LEFT HORIZ : axes[0]
    // LEFT VERT : axes[1]
    // if(joy_msg->buttons[4] == 1){
    //     joy_msg->axes[]
    // }
    // RIGHT HORIZ : axes[3]
    // RIGHT VERT : axes[4]

    // L2 : axes [2]
    // R2 : axes [5]

    joy_msg->axes[2] = (joy_msg->axes[2] - 1.f) / 2.f;

    joy_msg->axes[5] = (joy_msg->axes[5] - 1.f) / 2.f;

    // RIGHT VERT : axes[4]
    float speeds[6] = {joy_msg->axes[0], joy_msg->axes[1], joy_msg->axes[2], joy_msg->axes[3], joy_msg->axes[4], joy_msg->axes[5]};
    // RIGHT BUMPER
    if (joy_msg->buttons[7] == 1)
    {
        speeds[2] = speeds[5] * -1.f;
    }

    uint8_t out_buf[1 + 1 + sizeof(float) * 6 + 1] = {};
    out_buf[0] = SET_MOTOR_SPEED;
    out_buf[1] = sizeof(float) * 6;

    for (int i = 0; i < 6; i++)
    {
        float speed = speeds[i] * 250.f;
        //        std::cout << speed << "\n";
        memcpy(&out_buf[(i * sizeof(float)) + 2], &speed, sizeof(float));
    }
    out_buf[26] = 0x0A;

    tcflush(fd, TCIOFLUSH);
    int status = write(fd, out_buf, sizeof(out_buf));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControllerNode>());
    rclcpp::shutdown();
    return 0;
}
