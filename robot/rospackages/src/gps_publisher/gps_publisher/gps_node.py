#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


class GpsPublisherNode(Node):
    def __init__(self):
        super().__init__("gps_publisher_node")
        self.publisher = self.create_publisher(NavSatFix, "gps_data", 10)
        self.timer = self.create_timer(1.0, self.publish_gps_data)

    def publish_gps_data(self):
        gps_msg = NavSatFix()
        gps_msg.header = Header()

        # Place holder gps values
        gps_msg.latitude = 37.7749
        gps_msg.longitude = -122.4194
        gps_msg.altitude = 0.0
        gps_msg.position_covariance = [0.0] * 9
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.publisher.publish(gps_msg)
        self.get_logger().info("Published GPS data")


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
