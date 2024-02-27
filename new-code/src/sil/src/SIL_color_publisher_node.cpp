#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <stdio.h>       
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>


class Configuration : public rclcpp::Node
{
public:
  Configuration()
  : Node("Configuration_Node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("SIL_Color", 10);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "SIL_Color", 10, std::bind(&Configuration::topic_callback, this, std::placeholders::_1));

    //Opens LED strip communication
    fd = open("/dev/ttyUSB0", O_RDWR);

    if(fd == -1){
        std::cerr << "Error opening file " << errno << std::endl;
        exit(1);
    }

    //Set baud rate to 115200
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    tcsetattr(fd, TCSANOW, &options);

  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int fd;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    //Gives time for LED strip color to be visible
    int nbytes = write(fd, msg->data.c_str(), sizeof(msg->data.c_str()));

    if(nbytes == -1){
        std::cerr << "Error writing into file " << errno << std::endl;
        exit(1);
    }

    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Configuration>());
  rclcpp::shutdown();
  return 0;
}
