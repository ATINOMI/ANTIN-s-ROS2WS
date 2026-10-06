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
from launch_ros.parameter_descriptions import ParameterValue
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
                             {'output_dir': LaunchConfiguration('output_dir'),
                              'record_session': ParameterValue(LaunchConfiguration('record_session'), value_type=bool)}])
        ]
    else:
        root, _, _, _ = load_bundle(LaunchConfiguration('map_bundle').perform(context))
        soft_radius = float(LaunchConfiguration('inflation_radius').perform(context))
        scaling = float(LaunchConfiguration('cost_scaling_factor').perform(context))
        collision_topic = '/fastlivo/collision_cloud' if LaunchConfiguration(
            'use_3d_collision_cloud').perform(context).lower() == 'true' else ''
        if not 0.26 <= soft_radius <= 1.0:
            raise ValueError('inflation_radius must cover the 0.26 m vehicle safety circle')
        if not 0 < scaling <= 100.0:
            raise ValueError('cost_scaling_factor must be positive and bounded by 100')
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
                              'fuse_local_obstacles': True, 'collision_cloud_topic': collision_topic,
                              'planning.inflation_radius': soft_radius, 'planning.cost_scaling_factor': scaling}]),
            Node(package='mini_nav_nodes', executable='local_costmap_node', name='local_costmap', output='screen',
                 parameters=[os.path.join(bringup_share, 'config', 'local_costmap.yaml'), use_sim,
                             {'collision_cloud_topic': collision_topic,
                              'local_costmap.inflation_radius': soft_radius, 'local_costmap.cost_scaling_factor': scaling}]),
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
        args.append(DeclareLaunchArgument('record_session', default_value='true', description='Record full local scans for offline final-map generation'))
        args.append(DeclareLaunchArgument('teleop', default_value='true', description='Start PS5 driver and its guard'))
        args.append(DeclareLaunchArgument('external_control', default_value='false', description='Start guard for a separate raw/lease source; teleop must be false'))
        args.append(DeclareLaunchArgument('output_dir', default_value=os.environ.get('FASTLIVO_MAP_DIR', './maps/fastlivo2')))
    else:
        args.append(DeclareLaunchArgument('map_bundle', description='Saved and hash-verified map bundle directory'))
        args.append(DeclareLaunchArgument('use_3d_collision_cloud', default_value='true',
            description='Add projected 3D collision points to costmaps; false keeps the original 2D scan inputs'))
        args.append(DeclareLaunchArgument('inflation_radius', default_value='0.32',
            description='Soft cost outer radius in metres; hard vehicle clearance remains 0.26'))
        args.append(DeclareLaunchArgument('cost_scaling_factor', default_value='16.0',
            description='Soft cost decay in 1/metre; original profile used 10.0'))
    return LaunchDescription(args + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        OpaqueFunction(function=preflight, kwargs={'mapping': mapping, 'bringup_share': bringup_share})
    ])
