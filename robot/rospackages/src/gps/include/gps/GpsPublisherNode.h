#pragma once

#include <string>

#include "controller/GPSController.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"


using sensor_msgs::msg::NavSatFix;

struct GpsData
{
	float latitude;
	float longitude;
	float altitude;
};

class GpsPublisherNode : public rclcpp::Node
{
 public:
	explicit GpsPublisherNode(const std::string& gpsTopic = "/gps_data",
		const std::string& devicePath = "/dev/i2c-1",
		uint8_t devaddr = 0x42);

 private:
	void publishGpsData();

	GpsData extractGpsData();

	const float MULTIPLYING_FACTOR = 1e-7;
	GPSController gpsController;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<NavSatFix>::SharedPtr publisher_;
};
