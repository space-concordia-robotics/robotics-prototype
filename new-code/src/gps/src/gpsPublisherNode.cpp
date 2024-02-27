#include "gpsPublisherNode.h"

gpsPublisherNode::gpsPublisherNode(const std::string& gpsTopic) : Node("gpsPublisherNode")
{
	publisher_ = this->create_publisher<NavSatFix>(gpsTopic, 10);
	timer_ =
		this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&gpsPublisherNode::publishGpsData, this));

}

void gpsPublisherNode::publishGpsData()
{
	auto message = sensor_msgs::msg::NavSatFix();

	message.header = std_msgs::msg::Header();
	gpsData data = extractGpsData();
	message.latitude = data.latitude;
	message.longitude = data.longitude;
	message.altitude = data.altitude;

	publisher_->publish(message);
}

gpsData gpsPublisherNode::extractGpsData()
{
	char res[256];
	int32_t latitude, longitude, height;
	gps_.pollNAV_PVT(res, latitude, longitude, height);
	gpsData data{};
	data.latitude = float(latitude) * MULTIPLYING_FACTOR;
	data.longitude = float(longitude) * MULTIPLYING_FACTOR;
	data.altitude = float(height) * MULTIPLYING_FACTOR;
	return data;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<gpsPublisherNode>());
	rclcpp::shutdown();
	return 0;
}