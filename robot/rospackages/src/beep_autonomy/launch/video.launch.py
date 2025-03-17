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
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    configured_params = default_params_file

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    use_respawn = LaunchConfiguration('use_respawn')
    params_file = LaunchConfiguration('params_file')
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}
    
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']


    wheels_controller_node = launch_ros.actions.LifecycleNode(
        package='wheels_controller',
        executable='wheels_controller_node',
        name='wheels_controller',
        output='screen',
        parameters=[
            {'multiplier': 2000},
            {'local_mode': False}
        ],
        namespace='/',
    )
    wheel_sc = launch_ros.actions.LifecycleNode(
        package='service_client',
        executable='service_client',
        name='wheel_sc',
        output='screen',
        parameters=[
            {"node": 'wheels_controller'},
        ],
        namespace='/',
    )


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
        name='robot_state_publisher',
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

    controller_node=launch_ros.actions.Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )

    smoother_node=launch_ros.actions.Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    planner_node=launch_ros.actions.Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    behaviours_node=launch_ros.actions.Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    bt_navigator_node=launch_ros.actions.Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    waypoint_follower_node=launch_ros.actions.Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    velocity_smoother_node=launch_ros.actions.Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
    )
    
    lifecycle_manager_node=launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )



    return launch.LaunchDescription([                
        launch.actions.SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        launch.actions.DeclareLaunchArgument(name='namespace',default_value='',
                                            description='Top-level namespace'),
        launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='params_file',default_value=default_params_file,
                                            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        launch.actions.DeclareLaunchArgument(name='autostart', default_value='true',
                                            description='Automatically startup the nav2 stack'),
        launch.actions.DeclareLaunchArgument(name='use_respawn', default_value='False',
                                            description='Whether to respawn if a node crashes. Applied when composition is disabled.'),
        launch.actions.DeclareLaunchArgument(name='log_level', default_value='info',
                                            description='log level'),
        launch.actions.DeclareLaunchArgument(name='use_rviz', default_value='true',
                                            description='use rviz'),


        # Launch nodes related to mapping - sensors (lidar and zed2) and SLAM 
        slam_launch,
        lidar_launch,
        zed_launch,
        scan_publisher,

        # Bridge node for wheel control
        wheels_controller_node,
        wheel_sc,

        # Nav2 nodes
        robot_localization_node, # like odom_node in the sim

        robot_state_publisher_node,
        joint_state_publisher_node,

        controller_node,
        smoother_node,
        planner_node,
        behaviours_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_node
    ])
