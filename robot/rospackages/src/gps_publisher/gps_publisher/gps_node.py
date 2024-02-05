#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


class GpsPublisherNode(Node):
    def __init__(self, gps_pub_topic="/gps_data"):
        super().__init__("gps_publisher_node")
        self.publisher = self.create_publisher(NavSatFix, gps_pub_topic, 10)
        self.timer = self.create_timer(1.0, self.publish_gps_data)

    def publish_gps_data(self, latitude, longitude, altitude=0.0):
        gps_msg = NavSatFix()
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps"
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = altitude  # currently unused

        self.publisher.publish(gps_msg)
        self.get_logger().info(
            f"Published GPS data: Latitude={gps_msg.latitude}, Longitude={gps_msg.longitude}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
