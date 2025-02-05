import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from zed_processor import ZEDProcessor


class StratigraphyNode(Node):
    def __init__(self):
        super().__init__("stratigraphy_node")
        self.publisher_ = self.create_publisher(Float32MultiArray, "/stratigraphy_profile", 10)

        try:
            self.processor = ZEDProcessor()
        except RuntimeError as e:
            self.get_logger().error(str(e))
            return

        self.timer = self.create_timer(1.0, self.process_frame)

    def process_frame(self):
        depth_data = self.processor.capture_depth()
        if depth_data is not None:
            profile = self.processor.extract_stratigraphy(depth_data)

            msg = Float32MultiArray()
            msg.data = profile
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published stratigraphy profile: {profile}")

    def shutdown(self):
        self.processor.close()


def main():
    rclpy.init()
    node = StratigraphyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down stratigraphy node")
    finally:
        node.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
