import rclpy
from rclpy.node import Node

def do_stuff(node):
    # node.declare_parameter("my_str", rclpy.Parameter.Type.STRING)
    param = node.get_parameter("my_str").value
    node.get_logger().warn("NODE: " + param)

    while rclpy.ok():
        pass

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("testt", allow_undeclared_parameters=True)
    do_stuff(node)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
