import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("mini_nav_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                bringup_share, "maps", "turtlebot3_map.yaml"
            ),
            description="Map YAML file used by the official map_server.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_share, "config", "amcl_waffle.yaml"
            ),
            description="Nav2 parameters used by the official AMCL.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure and activate localization nodes.",
        ),
        DeclareLaunchArgument(
            "x_pose",
            default_value="-2.0",
            description="Initial Waffle x position in the official simulation.",
        ),
        DeclareLaunchArgument(
            "y_pose",
            default_value="-0.5",
            description="Initial Waffle y position in the official simulation.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "waffle_sim.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "x_pose": x_pose,
                "y_pose": y_pose,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, "launch", "localization_launch.py")
            ),
            launch_arguments={
                "map": map_file,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            }.items(),
        ),
        Node(
            package="mini_nav_nodes",
            executable="costmap_publisher_node",
            name="costmap_publisher",
            output="screen",
            parameters=[
                os.path.join(bringup_share, "config", "planning_costmap.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    "map_topic": "/map",
                    "map_file": "",
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                os.path.join(bringup_share, "rviz", "localization_astar.rviz"),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
