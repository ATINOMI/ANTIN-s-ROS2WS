"""两套入口共用资源查找与预检；建图和导航进程集合独立。"""
import os
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from .bundle import load_bundle


def preflight(context, mapping, bringup_share):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    ros_context = Context()
    rclpy.init(args=[], context=ros_context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('fastlivo_launch_preflight', context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    try:
        until = time.monotonic() + 0.7
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.05)
        topics = ['/cmd_vel', '/mini_nav/cmd_vel_raw', '/mini_nav/task_active']
        if not mapping:
            topics += ['/map', '/mini_nav/localization_valid']
        if LaunchConfiguration('use_simulator').perform(context).lower() == 'true':
            topics += ['/clock']
        conflicts = [topic for topic in topics if node.count_publishers(topic)]
        if conflicts:
            raise RuntimeError('Existing runtime must be stopped first: ' + ', '.join(conflicts))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros_context)
    if mapping and LaunchConfiguration('teleop').perform(context).lower() == 'true' and LaunchConfiguration('external_control').perform(context).lower() == 'true':
        raise RuntimeError('Choose PS5 or external control, not both')
    sim_share = get_package_share_directory('fastlivo_sim')
    package_share = get_package_share_directory('mini_nav_fastlivo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    use_sim = {'use_sim_time': True}
    common = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'gazebo.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_simulator')),
            launch_arguments={'gui': LaunchConfiguration('gui'), 'frontend': 'true',
                'mapping': 'true' if mapping else 'false',
                'x_pose': LaunchConfiguration('x_pose'), 'y_pose': LaunchConfiguration('y_pose'), 'yaw': LaunchConfiguration('yaw'),
                'map_dir': LaunchConfiguration('output_dir') if mapping else './unused'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[use_sim, {'robot_description': Path(tb3, 'urdf', 'turtlebot3_waffle.urdf').read_text()}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
            arguments=['--z', '0.30', '--frame-id', 'base_link', '--child-frame-id', 'livo_lidar'],
            parameters=[use_sim])
    ]
    if mapping:
        common += [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('mini_nav_teleop'), 'launch', 'ps5_teleop.launch.py')),
                condition=IfCondition(LaunchConfiguration('teleop'))),
            Node(package='mini_nav_nodes', executable='velocity_guard_node', name='velocity_guard',
                 condition=IfCondition(LaunchConfiguration('external_control')), parameters=[use_sim]),
            Node(package='mini_nav_fastlivo', executable='height_mapper', output='screen',
                 parameters=[os.path.join(package_share, 'config', 'mapping.yaml'),
                             {'output_dir': LaunchConfiguration('output_dir')}])
        ]
    else:
        root, _, _, _ = load_bundle(LaunchConfiguration('map_bundle').perform(context))
        common += [
            Node(package='mini_nav_fastlivo', executable='prior_localizer', output='screen',
                 parameters=[use_sim, {'map_bundle': str(root)}]),
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[use_sim, {'yaml_filename': str(root / 'navigation.yaml')}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                 parameters=[use_sim, {'autostart': True, 'node_names': ['map_server']}]),
            Node(package='mini_nav_nodes', executable='costmap_publisher_node', name='costmap_publisher', output='screen',
                 parameters=[os.path.join(bringup_share, 'config', 'planning_costmap.yaml'), use_sim,
                             {'map_topic': '/map', 'map_file': '', 'enable_topic_goals': False,
                              'fuse_local_obstacles': True, 'collision_cloud_topic': '/fastlivo/collision_cloud'}]),
            Node(package='mini_nav_nodes', executable='local_costmap_node', name='local_costmap', output='screen',
                 parameters=[os.path.join(bringup_share, 'config', 'local_costmap.yaml'), use_sim,
                             {'collision_cloud_topic': '/fastlivo/collision_cloud'}]),
            Node(package='mini_nav_nodes', executable='path_follower_node', name='path_follower', output='screen',
                 parameters=[os.path.join(bringup_share, 'config', 'path_follower.yaml'), use_sim,
                             {'action_mode': True, 'require_localization_quality': True, 'cmd_vel_topic': '/mini_nav/cmd_vel_raw'}]),
            Node(package='mini_nav_nodes', executable='navigation_manager_node', name='navigation_manager', output='screen',
                 parameters=[os.path.join(bringup_share, 'config', 'navigation_manager.yaml'), use_sim]),
            Node(package='mini_nav_nodes', executable='velocity_guard_node', name='velocity_guard', output='screen',
                 parameters=[use_sim, {'command_timeout': 0.35}])
        ]
    config = 'fastlivo_mapping.rviz' if mapping else 'fastlivo_navigation.rviz'
    common.append(Node(package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(bringup_share, 'rviz', config)], parameters=[use_sim],
        additional_env={'QT_QPA_PLATFORM': os.environ.get('QT_QPA_PLATFORM', 'xcb')}))
    return common


def description(mapping, bringup_share):
    args = [
        DeclareLaunchArgument('ros_domain_id', default_value='219'),
        DeclareLaunchArgument('gz_partition', default_value='mini_nav_fastlivo_deploy'),
        DeclareLaunchArgument('use_simulator', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true')
    ]
    if mapping:
        args.append(DeclareLaunchArgument('teleop', default_value='true', description='Start PS5 driver and its guard'))
        args.append(DeclareLaunchArgument('external_control', default_value='false', description='Start guard for a separate raw/lease source; teleop must be false'))
        args.append(DeclareLaunchArgument('output_dir', default_value=os.environ.get('FASTLIVO_MAP_DIR', './maps/fastlivo2')))
    else:
        args.append(DeclareLaunchArgument('map_bundle', description='Saved and hash-verified map bundle directory'))
    return LaunchDescription(args + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        OpaqueFunction(function=preflight, kwargs={'mapping': mapping, 'bringup_share': bringup_share})
    ])
