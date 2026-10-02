"""在独立命名空间启动三套官方 Nav2 代价图用于参数与图形对照。

资源从 package-share 查找；运行时参数由 launch 声明并解析。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """建立启动描述。

    Returns:
        LaunchDescription: 节点、包含入口与参数声明组成的启动描述。

    Note:
        调用只构建动作描述；节点进程由 launch 执行动作时启动。
    """
    bringup_share = get_package_share_directory("mini_nav_bringup")
    params_file = os.path.join(
        bringup_share, "config", "nav2_reference_costmaps.yaml"
    )
    rviz_file = os.path.join(
        bringup_share, "rviz", "nav2_costmap_compare.rviz"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    sim_time_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use the clock of the already running mini_nav simulation.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Open a separate RViz view of the official costmaps.",
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            namespace="nav2_reference_global",
            name="costmap",
            output="screen",
            parameters=[params_file, sim_time_parameter],
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            namespace="nav2_reference_matched",
            name="costmap",
            output="screen",
            parameters=[params_file, sim_time_parameter],
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            namespace="nav2_reference_local",
            name="costmap",
            output="screen",
            parameters=[params_file, sim_time_parameter],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace="nav2_reference_global",
            name="lifecycle_manager_costmap",
            output="screen",
            parameters=[{
                **sim_time_parameter,
                "autostart": True,
                "node_names": ["costmap"],
                "bond_timeout": 0.0,
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace="nav2_reference_matched",
            name="lifecycle_manager_costmap",
            output="screen",
            parameters=[{
                **sim_time_parameter,
                "autostart": True,
                "node_names": ["costmap"],
                "bond_timeout": 0.0,
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            namespace="nav2_reference_local",
            name="lifecycle_manager_costmap",
            output="screen",
            parameters=[{
                **sim_time_parameter,
                "autostart": True,
                "node_names": ["costmap"],
                "bond_timeout": 0.0,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_nav2_costmap_compare",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_file],
            parameters=[sim_time_parameter],
        ),
    ])
