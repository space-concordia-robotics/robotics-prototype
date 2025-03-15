#include "gps/GpsPublisherNode.h"

GpsPublisherNode::GpsPublisherNode(const std::string& gpsTopic, const std::string& devicePath, uint8_t devaddr)
	: Node("gps_publisher"),
	  gpsController(devicePath, devaddr)
{
	RCLCPP_INFO(this->get_logger(), "Initialized GPSController successfully");

	publisher_ = this->create_publisher<NavSatFix>(gpsTopic, 10);
	RCLCPP_INFO(this->get_logger(), "Started publishing GPS data to %s", gpsTopic.c_str());

	timer_ = this->create_wall_timer(
		std::chrono::milliseconds(500),
		std::bind(&GpsPublisherNode::publishGpsData, this)
	);
}


void GpsPublisherNode::publishGpsData()
{
	auto gps_data_message = sensor_msgs::msg::NavSatFix();
	gps_data_message.header = std_msgs::msg::Header();

	GpsData gps_data = extractGpsData();

	gps_data_message.latitude = gps_data.latitude;
	gps_data_message.longitude = gps_data.longitude;
	gps_data_message.altitude = gps_data.altitude;

	publisher_->publish(gps_data_message);
}

GpsData GpsPublisherNode::extractGpsData()
{
	char res[256];
	int32_t latitude{}, longitude{}, height{};

	// TODO: change function called here and remove multiplying factor stuff (will be done in gps code)
	GpsData gps_data{};
	gps_data.latitude = float(latitude) * MULTIPLYING_FACTOR;
	gps_data.longitude = float(longitude) * MULTIPLYING_FACTOR;
	gps_data.altitude = float(height) * MULTIPLYING_FACTOR;
	return gps_data;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GpsPublisherNode>());
	rclcpp::shutdown();
	return 0;
}
