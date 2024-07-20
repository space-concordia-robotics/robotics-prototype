#include "./gpsPublisherNode.h"

gpsPublisherNode::gpsPublisherNode() : LifecycleNode("gpsPublisherNode"){}

callbackReturn gpsPublisherNode::on_configure(const rclcpp_lifecycle::State &){
	const std::string& gpsTopic = "/gps_data";
	publisher_ = this->create_publisher<NavSatFix>(gpsTopic, 10);
	RCLCPP_INFO(this->get_logger(), "Started publishing GPS data to %s", gpsTopic.c_str());
	timer_ =
		this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&gpsPublisherNode::publishGpsData, this));
	return callbackReturn::SUCCESS;

}

callbackReturn gpsPublisherNode::on_activate(const rclcpp_lifecycle::State & state){
    LifecycleNode::on_activate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    return callbackReturn::SUCCESS;
}

callbackReturn gpsPublisherNode::on_deactivate(const rclcpp_lifecycle::State & state){
    LifecycleNode::on_deactivate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return callbackReturn::SUCCESS;
}

callbackReturn gpsPublisherNode::on_cleanup(const rclcpp_lifecycle::State &){
	publisher_.reset();
	timer_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return callbackReturn::SUCCESS;
}

callbackReturn gpsPublisherNode::on_shutdown(const rclcpp_lifecycle::State & state){
	publisher_.reset();
	timer_.reset();

    RCUTILS_LOG_INFO_NAMED(
        get_name(),
        "on shutdown is called from state %s.",
        state.label().c_str()
    );
	
	rclcpp::shutdown();
    return callbackReturn::SUCCESS;
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
	rclcpp::executors::SingleThreadedExecutor exe;

	std::shared_ptr<gpsPublisherNode> gps_node = std::make_shared<gpsPublisherNode>();
	exe.add_node(gps_node->get_node_base_interface());

	exe.spin();
	rclcpp::shutdown();
	return 0;
}