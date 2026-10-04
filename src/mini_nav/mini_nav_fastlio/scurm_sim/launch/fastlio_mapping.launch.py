"""SCURM FAST-LIO2 mapping using Gazebo 3D LiDAR and IMU only."""
import glob
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('scurm_sim'))
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    sim = {'use_sim_time': True}
    usb = glob.glob('/usr/lib/*-linux-gnu/libusb-1.0.so.0')
    frontend_env = {'LD_PRELOAD': usb[0]} if usb else {}
    ui_env = {'QT_QPA_PLATFORM': os.environ.get('QT_QPA_PLATFORM', 'xcb')}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('controls', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', '-v2',
                            str(share / 'models/tb3_scurm.world')], output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v2'], name='scurm_gazebo_gui',
                       condition=IfCondition(LaunchConfiguration('gui')), additional_env=ui_env),
        Node(package='ros_gz_sim', executable='create', name='scurm_spawn', arguments=[
            '-name', 'waffle', '-file', str(share / 'models/waffle_scurm.sdf'),
            '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
            '-Y', LaunchConfiguration('yaw'), '-z', '0.01'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scurm_bridge',
             parameters=[sim, {'config_file': str(share / 'config/bridge.yaml')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': (share / 'models/waffle_scurm.urdf').read_text()}]),
        Node(package='scurm_sim', executable='adapter', parameters=[sim, {'mapping_mode': True}],
             output='screen'),
        Node(package='fast_lio', executable='fastlio_mapping', name='scurm_fastlio2',
             parameters=[str(share / 'config/fastlio.yaml'), {
                 'locate_in_prior_map': False, 'publish.ikd_tree_en': True,
                 'publish.map_en': False}],
             remappings=[('/Odometry', '/scurm/imu_odometry')],
             additional_env=frontend_env, output='screen'),
        Node(package='rviz2', executable='rviz2', name='scurm_fastlio_rviz',
             parameters=[sim], arguments=['-d', str(share / 'config/fastlio_mapping.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), additional_env=ui_env),
        Node(package='scurm_sim', executable='mapping_controls', parameters=[sim],
             condition=IfCondition(LaunchConfiguration('controls')), output='screen'),
    ])
