import os
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    wheels_controller_driver = LifecycleNode(
        package='wheels_controller',
        executable='wheels_controller_node',
        name='wheels_controller_node',
        output='screen',
        parameters=[
            {'multiplier': 2000},
            {'local_mode': False}
        ],
        namespace='/',
    )


    aik_sc = LifecycleNode(
        package='service_client',
        executable='service_client',
        name='aik_sc',
        output='screen',
        parameters=[
            {"node": 'wheels_controller_node'},
        ],
        namespace='/',
    )


    return LaunchDescription([
        wheels_controller_driver,
        aik_sc,
        LifecycleNode(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            namespace="/"
        ),
    ])
