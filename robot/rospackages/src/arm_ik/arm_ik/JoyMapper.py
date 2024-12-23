#!/usr/bin/env python3

from ament_index_python import get_package_share_directory
import rclpy
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import yaml
import os

class JoyMapper(Node):
    
  
    def __init__(self):
        node_name = 'joymap_node'
        super().__init__(node_name)
        self.get_logger().info('Initialized "'+node_name+'" node for functionality')

        # Define the path to the motors configuration file
        package_root = get_package_share_directory('arm_ik')
        motors_file = os.path.join(package_root, 'config', 'motors-config.yaml')

        try:
            with open(motors_file, 'r') as file:
                self.config = yaml.safe_load(file)
                self.get_logger().info('Successfully loaded motors configuration.')
        except FileNotFoundError:
            self.get_logger().error(f"Configuration file not found: {motors_file}")
            self.config = None
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML configuration: {e}")
            self.config = None

        # Extract configurations
        if self.config:
            self.motion_range = self.config.get('joy_mapper_config', {}).get('ros__parameters', {}).get('sensitivity', {})
            self.arm_motors = self.config.get('joy_mapper_config', {}).get('ros__parameters', {}).get('arm_motors', {})
            self.wheels_motors = self.config.get('joy_mapper_config', {}).get('ros__parameters', {}).get('wheels_motors', {})
            self.arm_axes = self.config.get('joy_mapper_config', {}).get('ros__parameters', {}).get('arm_axes', {})
            self.arm_endpoints = self.config.get('joy_mapper_config', {}).get('ros__parameters', {}).get('arm_endpoints', {})

        self.mode = 'wheels'
        self.mode_switch_button = 9 

        map_topic = '/joy'
        
        qos_profile = QoSProfile(depth=10)

        self.map_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)
        self.get_logger().info('Created joy map subscriber for topic "'+map_topic)

        self.wheels_publisher = self.create_publisher(Twist, '/wheels', qos_profile)
        self.arm_values_publisher = self.create_publisher(Float32MultiArray, '/arm_values', qos_profile)
        self.arm_endpoint_publisher = self.create_publisher(Float32MultiArray, '/arm_endpoint', qos_profile)
        self.arm_axes_publisher = self.create_publisher(Float32MultiArray, '/arm_axes', qos_profile)


    def joy_callback(self, report: Joy):

        buttons = list(report.buttons)
        axes = list(report.axes)

        # Log the string of joystick data
        # joy_data_str = f"Buttons: {buttons}, Axes: {axes}"
        # self.get_logger().info(f"Published joystick data: {joy_data_str}")

        if buttons[self.mode_switch_button]:
            self.switch_mode()

        if self.mode == 'arm_forward_kinematics':
            self.arm_forward_kinematics(buttons, axes)
        elif self.mode == 'arm_inverse_kinematics':
            self.arm_inverse_kinematics(buttons, axes)
        elif self.mode == 'wheels':
            self.wheels_control(axes)

        
    def switch_mode(self):
        """Switch between modes."""
        modes = ['wheels', 'arm_forward_kinematics', 'arm_inverse_kinematics']
        current_index = modes.index(self.mode)
        self.mode = modes[(current_index + 1) % len(modes)]
        self.get_logger().info(f"Switched mode to: {self.mode}")

    def wheels_control(self, axes):
        wheels_motors = self.wheels_motors
        wheels_movement_range = self.motion_range
        twist_msg = Twist()
        twist_msg.linear.y = axes[wheels_motors["linear_y"]['index']] * wheels_movement_range # Set forward/backward speed
        twist_msg.angular.z = axes[wheels_motors["angular_z"]['index']] * wheels_movement_range # Set turning speed
        self.wheels_publisher.publish(twist_msg)
        self.get_logger().info(f'Published wheels Twist message: {twist_msg}')


    def arm_forward_kinematics(self, buttons, axes):

        arm_motors = self.arm_motors
        sensitivity = self.motion_range
        arm_movement_range = sensitivity * (-1 if buttons[arm_motors['arm_scale_0']['negative']] else 
                                    1 if buttons[arm_motors['arm_scale_0']['positive']] else 0)
        gripper_movement_range = sensitivity * (-1 if buttons[arm_motors['gripper_scale_4']['negative']] else 
                                    1 if buttons[arm_motors['gripper_scale_4']['positive']] else 0)
        
        # Create a list of arm values for 6 motors
        arm_values = [
        arm_movement_range,  # arm_value_0
        axes[arm_motors['arm_value_1']['index']] * (arm_movement_range),  # arm_value_1
        axes[arm_motors['arm_value_2']['index']] * (arm_movement_range),  # arm_value_2
        axes[arm_motors['arm_value_3']['index']] * (arm_movement_range),  # arm_value_3
        gripper_movement_range,  # arm_value_4
        gripper_movement_range * (-1 if buttons[arm_motors['gripper_open_close_5']['arm_open_close'][0]] else 
            1 if buttons[arm_motors['gripper_open_close_5']['arm_open_close'][1]] else 0)  # arm_value_5
        ]

        # Publish the arm values
        arm_values_msg = Float32MultiArray(data=arm_values)
        self.arm_values_publisher.publish(arm_values_msg)

        self.get_logger().info(f'Published forward_kin values: {arm_values}')

    def arm_inverse_kinematics(self, buttons, axes):
        arm_endpoints = self.arm_endpoints
        arm_axes = self.arm_axes
        sensitivity = self.motion_range

        endpoints_values = [
            axes[arm_endpoints['x']['index']] * sensitivity,  # X-axis
            axes[arm_endpoints['y']['index']] * sensitivity * (-1 if buttons[5] else 
            1 if buttons[7] else 0),  # Y-axis
            axes[arm_endpoints['pitch']['index']] * sensitivity  # Pitch
        ]

        axes_values = [
            axes[arm_axes['gripper_spin']['index']] * sensitivity,  # Gripper spin
            (-1 if buttons[arm_axes['gripper_open_close']['arm_open_close'][0]] else 
            1 if buttons[arm_axes['gripper_open_close']['arm_open_close'][1]] else 0) * sensitivity,  # Gripper open/close
            axes[arm_axes['base_motor']['index']] * sensitivity  # Base motor
        ]
        
        # Publish the arm values
        arm_endpoints_msg = Float32MultiArray(data=endpoints_values)
        self.arm_endpoint_publisher.publish(arm_endpoints_msg)

        arm_axes_msg = Float32MultiArray(data=axes_values)
        self.arm_axes_publisher.publish(arm_axes_msg)
        
        self.get_logger().info(f"Published IK endpoints: {endpoints_values}")
        self.get_logger().info(f"Published IK axes: {axes_values}")


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


