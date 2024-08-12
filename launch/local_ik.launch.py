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
        #DeclareLaunchArgument(
        #    'use_sim_time',
        #    default_value='false',
        #    description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='arm_ik',
            executable='IKNode',
            name='ik_node',
            output='screen',
            parameters=[
                {'joint_lengths': [1.354, 1.333, 1.250]},
                {'joint_angle_mins': [-180.0, -90.0, -170.0, -150.0]},
                {'joint_angle_maxes': [180, 90.0, 170.0, 150.0]},
                {'sensitivity': 2.0},
                {'mode': '2D'}
            ]
                        ),
        Node(
            package='arm_ik',
            executable='CadMouseJoyNode',
            name='cad_mouse_joy_node',
            output='screen'),
    ])
