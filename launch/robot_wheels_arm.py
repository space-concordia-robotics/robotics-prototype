import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='wheels_controller',
            executable='wheels_controller_node',
            name='wheels_controller_node',
            output='screen',
            parameters=[{'multiplier': 3000}]
        ),
       Node(
            package='arm_controller',
            executable='arm_controller_node',
            name='arm_controller',
            output='screen'
        ),
    ])
