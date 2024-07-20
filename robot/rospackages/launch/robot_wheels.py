import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'astro_arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('arm_ik'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Node(
        #     package='arm_ik',
        #     executable='CadMouseJoyNode',
        #     name='cad_mouse_joy_node',
        #     output='screen',
        #     parameters=[
        #         # More deadzone on yaw (pivot)
        #         {'deadzones': [20, 20, 20, 20, 20, 200]}
        #     ]
        # ),
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
            parameters=[
                {'multiplier': 2000},
                {'local_mode': False}
            ]
        ),
    ])
