import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_bringup = get_package_share_directory("nav2_bringup")
    nav2_learning = get_package_share_directory("nav2_learning")
    robot_sdf = os.path.join(nav2_learning, "urdf", "official_turtlebot3_waffle.sdf.xacro")
    model_resource_path = os.path.dirname(nav2_learning)
    params_file = os.path.join(nav2_learning, "config", "nav2_params.yaml")
    rviz_config = os.path.join(nav2_learning, "rviz", "nav2_default_view.rviz")
    launch_file = os.path.join(nav2_bringup, "launch", "tb3_simulation_launch.py")

    return LaunchDescription([
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", model_resource_path
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                "slam": "True",
                "headless": "True",
                "robot_sdf": robot_sdf,
                "params_file": params_file,
                "rviz_config_file": rviz_config,
                "use_rviz": "False",
                "autostart": "True",
            }.items(),
        )
    ])
