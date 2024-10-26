import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='beep_autonomy').find('beep_autonomy')
    bringup_dir = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    zed2_dir=launch_ros.substitutions.FindPackageShare(package='zed_wrapper').find('zed_wrapper')
    ouster_dir=launch_ros.substitutions.FindPackageShare(package='ouster_ros').find('ouster_ros')
    slam_dir=launch_ros.substitutions.FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    default_model_path = os.path.join(pkg_share, 'src/description/rover_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/default_view.rviz')
    default_params_file=os.path.join(pkg_share, 'config/nav2_params_no_map.yaml')
    world_path=os.path.join(pkg_share, 'world/sonoma.world')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/video-ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}],
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                    {'use_sim_time':LaunchConfiguration('use_sim_time')}]
    )

    slam_launch=launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [slam_dir,'/launch/online_async_launch.py']),
            launch_arguments={'use_sim_time':LaunchConfiguration('use_sim_time'), 
                            'slam_params_file': os.path.join(pkg_share, 'config/slam.yaml')}.items()
    )

    zed_launch=launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [zed2_dir,'/launch/zed_camera.launch.py']),
            launch_arguments={'publish_urdf': 'false', 'use_sim_time':LaunchConfiguration('use_sim_time'), 
            'camera_model': 'zed2', 'publish_tf': 'false', 'publish_map_tf': 'false',
            'xacro_path': default_model_path}.items()
    )

    lidar_launch=launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [ouster_dir,'/launch/driver.launch.py']),
            launch_arguments={'params_file':os.path.join(pkg_share, 'config/ouster_driver_params.yaml'),
                                'viz':'false'}.items()
    )

    # This node maps the PointCloud2 from the lidar to a LaserScan
    # The ouster driver does this automatically, but it seems to take the 
    # top slice of the lidar (so it misses obstacles near the ground)
    scan_publisher = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        remappings=[
            ("/cloud_in", "/ouster/points"),
        ]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                    description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        robot_localization_node,

        joint_state_publisher_node,
        robot_state_publisher_node,

        slam_launch,
        lidar_launch,
        zed_launch,
        scan_publisher
    ])