import glob
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('scurm_sim'))
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    config = str(share / 'config/nav2.yaml')
    map_path = LaunchConfiguration('map').perform(context)
    prior = LaunchConfiguration('prior_map').perform(context)
    for filename in [map_path, prior]:
        if not Path(filename).is_file():
            raise RuntimeError(f'Map file missing: {filename}')
    sim = {'use_sim_time': True}
    # PCL on this host requires Ubuntu libusb ahead of the MVS camera SDK copy.
    usb = glob.glob('/usr/lib/*-linux-gnu/libusb-1.0.so.0')
    pcl_env = {'LD_PRELOAD': usb[0]} if usb else {}
    ui_env = {'QT_QPA_PLATFORM': os.environ.get('QT_QPA_PLATFORM', 'xcb')}
    actions = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', '-v2',
                            str(share / 'models/tb3_scurm.world')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v2'], name='scurm_gazebo_gui',
                       condition=IfCondition(LaunchConfiguration('gui')), additional_env=ui_env),
        Node(package='ros_gz_sim', executable='create', arguments=[
            '-name', 'waffle', '-file', str(share / 'models/waffle_scurm.sdf'),
            '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
            '-Y', LaunchConfiguration('yaw'), '-z', '0.01'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scurm_bridge',
             parameters=[sim, {'config_file': str(share / 'config/bridge.yaml')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': (share / 'models/waffle_scurm.urdf').read_text()}]),
        Node(package='scurm_sim', executable='adapter', parameters=[sim], output='screen'),
        Node(package='fast_lio', executable='fastlio_mapping', parameters=[str(share / 'config/fastlio.yaml'),
             {'prior_map_path': prior}], remappings=[('/Odometry', '/scurm/imu_odometry')],
             additional_env=pcl_env, output='screen'),
        TimerAction(period=5.0, actions=[
            Node(package='icp_relocalization', executable='icp_node', parameters=[sim, {
                'map_path': prior, 'map_frame_id': 'map', 'pcl_type': 'gazebo',
                'rotate_input_x_180': False, 'initial_x': -0.0314, 'initial_y': 0.0015,
                'initial_z': 0.088, 'initial_a': 0.006, 'map_voxel_leaf_size': 0.08,
                'cloud_voxel_leaf_size': 0.08, 'solver_max_iter': 60,
                'max_correspondence_distance': 0.3, 'fitness_score_thre': 0.015,
                'RANSAC_outlier_rejection_threshold': 0.3, 'converged_count_thre': 5}],
                remappings=[('/pointcloud2', '/scurm/icp_cloud'), ('initialpose', '/scurm/initial_imu')],
                additional_env=pcl_env, output='screen')]),
        Node(package='terrain_analysis', executable='terrainAnalysis', name='terrain_analysis',
             parameters=[sim, {'map_frame': 'odom', 'scanVoxelSize': 0.05, 'decayTime': 0.5,
                 'noDecayDis': 0.0, 'clearingDis': 0.0, 'useSorting': True, 'quantileZ': 0.1,
                 'considerDrop': False, 'limitGroundLift': False, 'clearDyObs': False,
                 'noDataObstacle': False, 'minBlockPointNum': 1, 'vehicleHeight': 0.5,
                 'minRelZ': -0.5, 'maxRelZ': 0.8, 'disRatioZ': 0.1}],
             remappings=[('/registered_scan', '/cloud_registered')], additional_env=pcl_env),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[sim, {'yaml_filename': map_path}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', parameters=[sim, {'autostart': True, 'node_names': ['map_server']}]),
    ]
    nodes = []
    for name, package in [
        ('controller_server', 'nav2_controller'), ('planner_server', 'nav2_planner'),
        ('smoother_server', 'nav2_smoother'), ('behavior_server', 'nav2_behaviors'),
        ('bt_navigator', 'nav2_bt_navigator'), ('velocity_smoother', 'nav2_velocity_smoother')]:
        remappings = []
        if name in ['controller_server', 'behavior_server']:
            remappings = [('cmd_vel', '/scurm/cmd_vel_nav')]
        if name == 'velocity_smoother':
            remappings = [('cmd_vel', '/scurm/cmd_vel_nav'), ('cmd_vel_smoothed', '/scurm/cmd_vel_smoothed')]
        nodes.append(Node(package=package, executable=name, name=name, parameters=[config],
                          remappings=remappings, additional_env=pcl_env, output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', parameters=[sim, {'autostart': True,
        'node_names': ['controller_server', 'planner_server', 'smoother_server',
                       'behavior_server', 'bt_navigator', 'velocity_smoother']}], output='screen'))
    actions.append(TimerAction(period=10.0, actions=nodes))
    actions.append(Node(package='rviz2', executable='rviz2', name='scurm_rviz',
        parameters=[sim], arguments=['-d', str(share / 'config/navigation.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')), additional_env=ui_env))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('scurm_sim'))
    mini_share = Path(get_package_share_directory('mini_nav_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=str(mini_share / 'maps/tb3_learning.yaml')),
        DeclareLaunchArgument('prior_map', default_value=str(share / 'maps/prior.pcd')),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=start),
    ])
