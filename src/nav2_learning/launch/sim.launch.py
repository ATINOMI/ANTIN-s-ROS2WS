import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("nav2_learning")
    gz_share = get_package_share_directory("ros_gz_sim")
    xacro_file = os.path.join(pkg, "urdf", "learning_robot.urdf.xacro")
    bridge_file = os.path.join(pkg, "config", "bridge.yaml")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )
    robot_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", xacro_file]), "use_sim_time": True}],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_file, "use_sim_time": True}],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "learning_robot", "-topic", "robot_description", "-z", "0.15"],
        output="screen",
    )
    return LaunchDescription([gazebo, robot_description, bridge, spawn])
