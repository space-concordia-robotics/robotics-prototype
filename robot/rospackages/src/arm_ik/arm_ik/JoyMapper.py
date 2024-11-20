#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_msgs.msg import String
import time  

class JoyMapper(Node):
  
    def __init__(self):
        node_name = 'joymap_node'
        super().__init__(node_name)
        self.get_logger().info('Initialized "'+node_name+'" node for functionality')

        # Subscribe to the /joy topic
        map_topic = '/joy'
        
        # QoS settings
        qos_profile = QoSProfile(depth=10)

        self.map_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)
        self.get_logger().info('Created joy map subscriber for topic "'+map_topic)

        # Publisher for wheels status
        self.wheels_publisher = self.create_publisher(String, '/wheels', 10)
        self.get_logger().info('Created wheels publisher for topic "/wheels"')
        
        # Publisher for arm values
        self.arm_values_publisher = self.create_publisher(String, '/arm_values', 10)
        self.get_logger().info('Created arm values publisher for topic "/arm_values"')
        
        # Publisher for arm endpoint
        self.arm_endpoint_publisher = self.create_publisher(String, '/arm_endpoint', 10)
        self.get_logger().info('Created arm endpoint publisher for topic "/arm_endpoint"')
        
        # Publisher for arm axes
        self.arm_axes_publisher = self.create_publisher(String, '/arm_axes', 10)
        self.get_logger().info('Created arm axes publisher for topic "/arm_axes"')


    def joy_callback(self, report:Joy):

        buttons = list(report.buttons)
        axes = list(report.axes)

        joy_data_str = f"Buttons: {buttons}, Axes: {axes}"

        # Output delay
        time.sleep(1)

        # Log the string of joystick data
        # self.get_logger().info(f"Published joystick data: {joy_data_str}")

        if buttons[9] == 1 and any(buttons[i] == 1 for i in range(4)): 
    
            selected_mode = next(i for i in range(4) if buttons[i] == 1)
            
            print(f"Mode switched to {selected_mode}")

            # Publish data based on the selected mode
            if selected_mode == 0:
                print("Performing action for option 0.")
            elif selected_mode == 1:
                print("Performing action for option 1.")
            elif selected_mode == 2:
                print("Performing action for option 2.")
            elif selected_mode == 3:
                print("Performing action for option 3.")

    def destroy_node(self):
        self.get_logger().info("Destroying joymap_node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    joy_mapper_node = JoyMapper()

    try:
        rclpy.spin(joy_mapper_node)
    except KeyboardInterrupt:
        joy_mapper_node.get_logger().info("Node shutting down due to keyboard interrupt.")
    finally:
        if rclpy.ok():
            joy_mapper_node.destroy_node()
            rclpy.shutdown()
