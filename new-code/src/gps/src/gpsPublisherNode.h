#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "gps2.cpp"

using sensor_msgs::msg::NavSatFix;

struct gpsData
{
	float latitude;
	float longitude;
	float altitude;
};

class gpsPublisherNode : public rclcpp::Node
{
 public:
	explicit gpsPublisherNode(const std::string& gpsTopic = "/gps_data");

 private:
	void publishGpsData();
	gpsData extractGpsData();
	const float MULTIPLYING_FACTOR = 1e-7;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<NavSatFix>::SharedPtr publisher_;
	SAM_M8Q_GPS gps_;

};
