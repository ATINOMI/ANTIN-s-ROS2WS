"""SCURM fixed-prior FAST-LIO2 localization with mini_nav's own navigation."""
import hashlib
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('scurm_sim'))
    mini = Path(get_package_share_directory('mini_nav_bringup'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    maps = share / 'maps'
    manifest = json.loads((maps / 'localization_manifest.json').read_text())
    for filename, expected in manifest['files'].items():
        if hashlib.sha256((maps / filename).read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Prior bundle changed: {filename}')
    sim = {'use_sim_time': True}
    ui_env = {'QT_QPA_PLATFORM': os.environ.get('QT_QPA_PLATFORM', 'xcb')}
    nodes = [
        DeclareLaunchArgument('ros_domain_id', default_value='227',
                              description='ROS domain for FAST-LIO2 navigation.'),
        DeclareLaunchArgument('gz_partition', default_value='scurm_mini_nav',
                              description='Gazebo transport partition for this instance.'),
        DeclareLaunchArgument('rmw_implementation', default_value='rmw_cyclonedds_cpp',
                              description='ROS middleware used by all child nodes.'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open navigation RViz.'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw_implementation')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(tb3 / 'models')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', '-v2',
                            str(share / 'models/tb3_scurm.world')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v2'], name='scurm_navigation_gazebo',
                       condition=IfCondition(LaunchConfiguration('gui')), additional_env=ui_env),
        Node(package='ros_gz_sim', executable='create', name='scurm_spawn', arguments=[
            '-name', 'waffle', '-file', str(share / 'models/waffle_scurm.sdf'),
            '-x', '-2.0', '-y', '-0.5', '-Y', '0.0', '-z', '0.01'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scurm_bridge',
             parameters=[sim, {'config_file': str(share / 'config/bridge.yaml')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': (share / 'models/waffle_scurm.urdf').read_text()}]),
        Node(package='scurm_sim', executable='mini_nav_localizer', parameters=[sim, {
            'prior_map_path': str(maps / 'local_prior.pcd')}], output='screen'),
        Node(package='scurm_sim', executable='localization_backend', parameters=[sim, {
            'prior_map_path': str(maps / 'local_prior.pcd'),
            'frontend_config': str(share / 'config/fastlio.yaml')}], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[sim, {'yaml_filename': str(maps / 'local_map.yaml')}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle_manager',
             parameters=[sim, {'autostart': True, 'node_names': ['map_server']}], output='screen'),
        Node(package='mini_nav_nodes', executable='costmap_publisher_node', name='costmap_publisher',
             parameters=[str(mini / 'config/planning_costmap.yaml'), sim, {
                 'map_topic': '/map', 'map_file': '', 'enable_topic_goals': False,
                 'fuse_local_obstacles': True}], output='screen'),
        Node(package='mini_nav_nodes', executable='local_costmap_node', name='local_costmap',
             parameters=[str(mini / 'config/local_costmap.yaml'), sim], output='screen'),
        Node(package='mini_nav_nodes', executable='path_follower_node', name='path_follower',
             parameters=[str(mini / 'config/path_follower.yaml'), sim, {'action_mode': True,
                 'controller.goal_position_hysteresis': .03,
                 'require_localization_quality': True, 'cmd_vel_topic': '/mini_nav/cmd_vel_raw'}], output='screen'),
        Node(package='mini_nav_nodes', executable='navigation_manager_node', name='navigation_manager',
             parameters=[str(mini / 'config/navigation_manager.yaml'), sim, {'enabled': True}], output='screen'),
        Node(package='mini_nav_nodes', executable='velocity_guard_node', name='velocity_guard',
             parameters=[sim, {'command_timeout': .35}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='scurm_mini_nav_rviz', parameters=[sim],
             arguments=['-d', str(share / 'config/mini_nav_fastlio.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), additional_env=ui_env),
    ]
    return LaunchDescription(nodes)
