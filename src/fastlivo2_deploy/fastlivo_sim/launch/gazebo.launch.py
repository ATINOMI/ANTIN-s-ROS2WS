import os
import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fastlivo_sim')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    world = os.path.join(share, 'models', 'tb3_livo.world')
    # MVS SDK's legacy libusb shadows Ubuntu's PCL dependency on this host.
    # Restrict the override to the FAST-LIVO process, preserving the user's environment.
    usb = glob.glob('/usr/lib/*-linux-gnu/libusb-1.0.so.0')
    frontend_env = {}
    if usb and '/opt/MVS' in os.environ.get('LD_LIBRARY_PATH', ''):
        frontend_env['LD_PRELOAD'] = ':'.join([usb[0], os.environ.get('LD_PRELOAD', '')]).rstrip(':')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('frontend', default_value='true'),
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument('map_dir', default_value=os.environ.get('FASTLIVO_MAP_DIR', os.path.join(os.environ.get('ROS_LOG_DIR', '.'), 'maps'))),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', '-v2', world],
                       output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v2'], name='gazebo_gui',
            output='screen',
            additional_env={'QT_QPA_PLATFORM': os.environ.get('QT_QPA_PLATFORM', 'xcb')},
            condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='ros_gz_sim', executable='create', name='spawn_waffle',
             arguments=['-name', 'waffle', '-file', os.path.join(share, 'models', 'waffle_livo.sdf'),
                        '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
                        '-Y', LaunchConfiguration('yaw'), '-z', '0.01'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='livo_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                          'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_image', executable='image_bridge', name='livo_image_bridge',
             arguments=['/camera/image_raw'], parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='fastlivo_sim', executable='map_store',
             parameters=[{'use_sim_time': True, 'output_dir': LaunchConfiguration('map_dir')}],
             output='screen', condition=IfCondition(LaunchConfiguration('mapping'))),
        Node(package='fastlivo_sim', executable='map_store', name='fastlivo_color_map_store',
             parameters=[{'use_sim_time': True, 'colored': True,
                          'output_dir': LaunchConfiguration('map_dir')}],
             output='screen', condition=IfCondition(LaunchConfiguration('mapping'))),
        TimerAction(period=5.0, actions=[
            Node(package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
                 parameters=[os.path.join(share, 'config', 'fastlivo.yaml')], output='screen',
                 additional_env=frontend_env,
                 condition=IfCondition(LaunchConfiguration('frontend')))]),
    ])
