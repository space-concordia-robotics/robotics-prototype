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
        Node(
            package='absenc_interface',
            executable='absenc_node',
            name='absenc_node',
            output='screen'
        ),
        # Node(
        #     package='arm_ik',
        #     executable='AbsencControlSystem',
        #     name='absenc_control_system',
        #     output='screen'
        # ),
        Node(
            package='arm_ik',
            executable='IKNode',
            name='ik_node',
            output='screen',
            parameters=[
                {'joint_lengths': [1.354, 1.333, 1.250]},
                {'joint_angle_mins': [-180.0, -60.0, -111.0, -101.0]},
                {'joint_angle_maxes': [180, 66.0, 115.0, 106.0]},
                {'sensitivity': 1.5},
                {'mode': '2D'}
            ]
                        ),
        Node(
            package='arm_ik',
            executable='CadMouseJoyNode',
            name='cad_mouse_joy_node',
            output='screen'),
    ])
