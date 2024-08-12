import os
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import Shutdown


def generate_launch_description():
    localMode = False

    wheels_controller_node = LifecycleNode(
        package='wheels_controller',
        executable='wheels_controller_node',
        name='wheels_controller',
        output='screen',
        parameters=[
            {'multiplier': 2000},
            {'local_mode': localMode}
        ],
        namespace='/',
    )
    
    arm_ik_node = LifecycleNode(
        package='arm_ik',
        executable='IKNode',
        name='ik_node',
        output='screen',
        parameters=[
            {'joint_lengths': [1.354, 1.333, 1.250]},
            {'joint_angle_mins': [-180.0, -80.0, -111.0, -101.0]},
            {'joint_angle_maxes': [180, 80.0, 115.0, 106.0]},
            {'sensitivity': 1.0},
            {'mode': '2D'},
            {'solution': 1},
            # "joint" sets final joint angle, while "vertical" sets the
            # angle of the gripper relative to vertical while keeping the end effector
            # position constant.
            {'angle_set': 'vertical'},
            {'local_mode': False}
        ],
        namespace='/',
    )

    arm_controller_node = LifecycleNode(
        package='arm_controller',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
        namespace='/',
        parameters=[
            {'local_mode': localMode}
        ]
    )

    absenc_interface_node = LifecycleNode(
        package='absenc_interface',
        executable='absenc_node',
        name='absenc_node',
        output='screen',
        namespace='/',
        parameters=[
            {'local_mode': localMode}
        ]
    )

    wheel_sc = LifecycleNode(
        package='service_client',
        executable='service_client',
        name='wheel_sc',
        output='screen',
        parameters=[
            {"node": 'wheels_controller'},
        ],
        namespace='/',
    )

    absenc_sc = LifecycleNode(
        package='service_client',
        executable='service_client',
        name='absenc_sc',
        output='screen',
        parameters=[
            {"node": 'absenc_node'},
        ],
        namespace='/',
    )

    arm_sc = LifecycleNode(
        package='service_client',
        executable='service_client',
        name='arm_sc',
        output='screen',
        parameters=[
            {"node": 'arm_controller'},
        ],
        namespace='/',
    )

    aik_sc = LifecycleNode(
        package='service_client',
        executable='service_client',
        name='aik_sc',
        output='screen',
        parameters=[
            {"node": 'ik_node'},
        ],
        namespace='/',
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'astro_arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('arm_ik'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    return LaunchDescription([
        # TODO: create a separate package that is abstract enough to avoid having to create a service_client file for every node concerned
        
        # runs a script that enables automatic transitions to inactive
        # Node(package='lifecycle', executable='lifecycle_service_client', output='screen', on_exit=Shutdown()),
        wheels_controller_node,
        arm_ik_node,
        arm_controller_node,
        absenc_interface_node,

        wheel_sc,
        absenc_sc,
        aik_sc,
        arm_sc,

        LifecycleNode(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            namespace='/'
        ),
    ])
