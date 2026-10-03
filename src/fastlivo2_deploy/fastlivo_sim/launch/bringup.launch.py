import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fastlivo_sim')
    rviz_env = {'QT_QPA_PLATFORM': os.environ.get('QT_QPA_PLATFORM', 'xcb')}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open the localization RViz view'),
        DeclareLaunchArgument('teleop', default_value='true', description='Prepare USB PS5 control, initially disabled'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'config', 'localization.rviz')),
        DeclareLaunchArgument('frontend', default_value='true'),
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument('map_dir', default_value=os.environ.get(
            'FASTLIVO_MAP_DIR', os.path.join(os.environ.get('ROS_LOG_DIR', '.'), 'maps'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('mini_nav_teleop'), 'launch', 'ps5_teleop.launch.py')),
            condition=IfCondition(LaunchConfiguration('teleop'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'gazebo.launch.py')),
            launch_arguments={name: LaunchConfiguration(name)
                              for name in ['gui', 'frontend', 'mapping', 'map_dir']}.items()),
        Node(package='rviz2', executable='rviz2', name='fastlivo_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': True}],
             additional_env=rviz_env,
             output='screen', condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
