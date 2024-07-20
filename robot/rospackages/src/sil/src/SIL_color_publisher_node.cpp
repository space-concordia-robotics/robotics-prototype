#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <stdio.h>       
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <chrono>
#include <thread>
typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn callbackReturn;

//needs to add subscriber coming from the wheels_controller_node.cpp
//from class Configuration here in Configuration
class Configuration : public rclcpp_lifecycle::LifecycleNode{
  public: 
    explicit Configuration(const std::string& node_name, bool intra_process_comms = false)
    : rclcpp_lifecycle::LifecycleNode(node_name, 
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)){}

    callbackReturn on_configure(const rclcpp_lifecycle::State &){
      this->declare_parameter("sil_path", "/dev/ttyUSB1");

      pub_ = this->create_publisher<std_msgs::msg::String>("SIL_Color", 10);

      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "SIL_Color", 10, std::bind(&Configuration::topic_callback, this, std::placeholders::_1));

      //Opens LED strip communication
      fd = open(this->get_parameter("sil_path").as_string().c_str(), O_RDWR);

      if(fd == -1){
          RCLCPP_ERROR(this->get_logger(),"Error opening file (%i)\n",errno);
          rclcpp::shutdown();
      }
      //Set baud rate to 115200
      struct termios options;
      tcgetattr(fd, &options);
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);
      tcsetattr(fd, TCSANOW, &options);

      RCLCPP_INFO(get_logger(), "on_configure() is called.");
      return callbackReturn::SUCCESS;
    }

    callbackReturn on_activate(const rclcpp_lifecycle::State & state){
      LifecycleNode::on_activate(state);

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

      return callbackReturn::SUCCESS;
    }

    callbackReturn on_deactivate(const rclcpp_lifecycle::State & state){
      LifecycleNode::on_deactivate(state);

      RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

      return callbackReturn::SUCCESS;
    }

    callbackReturn on_cleanup(const rclcpp_lifecycle::State &){
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
      return callbackReturn::SUCCESS;
    }

    callbackReturn on_shutdown(const rclcpp_lifecycle::State & state){
      pub_.reset();

      RCUTILS_LOG_INFO_NAMED(
        get_name(),
        "on shutdown is called from state %s.",
        state.label().c_str()
      );

      rclcpp::shutdown();
      return callbackReturn::SUCCESS;
    }

    
  private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    int fd;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
      //Gives time for LED strip color to be visible
      int nbytes = write(fd, msg->data.c_str(), sizeof(msg->data.c_str()));

      if(nbytes == -1){
          RCLCPP_ERROR(this->get_logger(),"writing to file %i\n",errno);
          exit(1);
      }
    }
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<Configuration> sil_node = std::make_shared<Configuration>("sil_pub");

  exe.add_node(sil_node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
