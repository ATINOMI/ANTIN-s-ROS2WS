"""
@file stvl_nav2.launch.py
@brief 启动独立的 Gazebo、定位、Nav2 和 STVL 学习案例。

本文件只属于 nav2_stvl_demo，不会修改 nav2_learning 的启动流程。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """构造 STVL 案例的启动描述。"""

    # 获取本案例和官方 Nav2 启动文件的安装目录。
    pkg = get_package_share_directory("nav2_stvl_demo")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    official_launch = os.path.join(nav2_bringup, "launch", "tb3_simulation_launch.py")
    robot_sdf = os.path.join(pkg, "urdf", "official_turtlebot3_waffle.sdf.xacro")
    params_file = os.path.join(pkg, "config", "nav2_params.yaml")
    # 模型中的网格通过 package://nav2_stvl_demo 查找，因此把本案例的
    # share 父目录加入 Gazebo 资源搜索路径。
    resource_path = os.path.dirname(pkg)

    # 这些参数可以在命令行覆盖，例如 headless:=True。
    map_file = LaunchConfiguration("map")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    # 将官方 TurtleBot3 的 /scan 转成 STVL 需要的 PointCloud2。
    converter = Node(
        package="nav2_stvl_demo",
        executable="scan_to_cloud",
        name="scan_to_cloud",
        output="screen",
    )

    # 复用官方 Nav2 TurtleBot3 仿真启动流程，只替换模型和参数文件。
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(official_launch),
        launch_arguments={
            "slam": "False",
            "map": map_file,
            "params_file": params_file,
            "robot_sdf": robot_sdf,
            "headless": headless,
            "use_rviz": use_rviz,
            "autostart": "True",
        }.items(),
    )

    return LaunchDescription([
        # 默认使用已经保存的地图进行 AMCL 定位，不启动 SLAM。
        DeclareLaunchArgument(
            "map",
            default_value="/home/a/ros2_ws/maps/tb3_learning.yaml",
        ),
        DeclareLaunchArgument("headless", default_value="False"),
        DeclareLaunchArgument("use_rviz", default_value="True"),
        # 使 Gazebo 能够找到本案例内的模型和网格资源。
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        # 先启动转换节点；它会等待 /scan 出现后自动发布点云。
        converter,
        # 启动官方 Gazebo、AMCL、Nav2 和 RViz 流程。
        nav2,
    ])
