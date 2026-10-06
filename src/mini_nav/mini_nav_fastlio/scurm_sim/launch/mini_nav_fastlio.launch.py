"""Compatibility entry for the FAST-LIO2 navigation launch."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('scurm_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value=os.environ.get('ROS_DOMAIN_ID', '227')),
        DeclareLaunchArgument('gz_partition', default_value=os.environ.get('GZ_PARTITION', 'scurm_mini_nav')),
        DeclareLaunchArgument('rmw_implementation',
                              default_value=os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch/fastlio2_navigation.launch.py')), launch_arguments={
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'gz_partition': LaunchConfiguration('gz_partition'),
                'rmw_implementation': LaunchConfiguration('rmw_implementation'),
            }.items()),
    ])
