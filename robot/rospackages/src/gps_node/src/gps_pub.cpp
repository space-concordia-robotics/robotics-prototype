#include "rclcpp/rclcpp.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

class RoverLocalizationPublisher : public rclcpp::Node
{
public:
    RoverLocalizationPublisher() : Node("rover_localization")
    {
        publisher_ = this->create_publisher<geographic_msgs::msg::GeoPoint>("rover_coordinates", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&RoverLocalizationPublisher::publishCoordinates, this));
        RCLCPP_INFO(this->get_logger(), "Rover localization publisher has been started");
    }
private:
   void publishCoordinates()
   {
        geographic_msgs::msg::GeoPoint msg{};
        msg.latitude = 45.5017;
        msg.longitude = -73.5673;
        msg.altitude = 30.0;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published GeoPoint: [%f, %f, %f]", msg.latitude, msg.longitude, msg.altitude);
    }

   rclcpp::Publisher<geographic_msgs::msg::GeoPoint>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverLocalizationPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}