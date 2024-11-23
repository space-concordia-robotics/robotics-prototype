#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using sensor_msgs::msg::NavSatFix;

struct SAM_M8Q_GPS
{
	int openPort(const std::string& str)
	{
		return 0;
	}

	/**
	 * Placeholder to compile
	 */
};

struct GpsData
{
	float latitude;
	float longitude;
	float altitude;
};

class GpsPublisherNode : public rclcpp::Node
{
public:
	explicit GpsPublisherNode(const std::string& gpsTopic = "/gps_data");

private:
	void publishGpsData();
	GpsData extractGpsData();
	const float MULTIPLYING_FACTOR = 1e-7;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<NavSatFix>::SharedPtr publisher_;
	SAM_M8Q_GPS gps_;
};
