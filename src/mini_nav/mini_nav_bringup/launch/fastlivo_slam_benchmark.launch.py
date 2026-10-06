"""同一次 Gazebo 运动比较两套建图；SLAM Toolbox 仅输出独立地图。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = Path(get_package_share_directory('mini_nav_bringup'))
    slam = Path(get_package_share_directory('slam_toolbox'))
    fastlivo = Path(get_package_share_directory('mini_nav_fastlivo'))
    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='228'),
        DeclareLaunchArgument('gz_partition', default_value='mini_nav_slam_benchmark'),
        DeclareLaunchArgument('output_dir', default_value='./maps/slam_benchmark'),
        DeclareLaunchArgument('gui', default_value='false'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/fastlivo_mapping.launch.py')),
            launch_arguments={
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'gz_partition': LaunchConfiguration('gz_partition'),
                'output_dir': LaunchConfiguration('output_dir'),
                'gui': LaunchConfiguration('gui'), 'rviz': 'false',
                'teleop': 'false', 'external_control': 'true', 'record_session': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true',
                'slam_params_file': str(fastlivo/'config/slam_benchmark.yaml')}.items())
    ])
