#pragma once

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/transition.hpp"


#include "gps2.cpp"

using sensor_msgs::msg::NavSatFix;
typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn callbackReturn;


struct gpsData
{
	float latitude;
	float longitude;
	float altitude;
};

class gpsPublisherNode : public rclcpp_lifecycle::LifecycleNode {
	public:
		explicit gpsPublisherNode();
		callbackReturn on_configure(const rclcpp_lifecycle::State &);
        callbackReturn on_activate(const rclcpp_lifecycle::State & state);
        callbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
        callbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        callbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

 	private:
		void publishGpsData();
		gpsData extractGpsData();
		const float MULTIPLYING_FACTOR = 1e-7;

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<NavSatFix>::SharedPtr publisher_;
		SAM_M8Q_GPS gps_;

};
