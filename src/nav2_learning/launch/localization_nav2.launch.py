import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_bringup = get_package_share_directory("nav2_bringup")
    nav2_learning = get_package_share_directory("nav2_learning")
    robot_sdf = os.path.join(nav2_learning, "urdf", "official_turtlebot3_waffle.sdf.xacro")
    params_file = os.path.join(nav2_learning, "config", "nav2_params.yaml")
    rviz_config = os.path.join(nav2_learning, "rviz", "nav2_default_view.rviz")
    model_resource_path = os.path.dirname(nav2_learning)
    official_launch = os.path.join(
        nav2_bringup, "launch", "tb3_simulation_launch.py"
    )

    map_file = LaunchConfiguration("map")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", model_resource_path
        ),
        DeclareLaunchArgument(
            "map",
            default_value="/home/a/ros2_ws/maps/tb3_learning.yaml",
            description="Saved map YAML file",
        ),
        DeclareLaunchArgument(
            "headless", default_value="False", description="Run Gazebo without GUI"
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="True", description="Start RViz2"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(official_launch),
            launch_arguments={
                "slam": "False",
                "robot_sdf": robot_sdf,
                "params_file": params_file,
                "rviz_config_file": rviz_config,
                "map": map_file,
                "headless": headless,
                "use_rviz": use_rviz,
                "autostart": "True",
            }.items(),
        ),
    ])
