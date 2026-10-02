"""组合 Waffle 仿真、自研定位、规划、跟踪、任务管理及独立速度看门狗。

资源从 package-share 查找；运行时参数由 launch 声明并解析。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    ros_domain_id = LaunchConfiguration("ros_domain_id")
    gz_partition = LaunchConfiguration("gz_partition")
    rmw_implementation = LaunchConfiguration("rmw_implementation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    autostart = LaunchConfiguration("autostart")
    inflate_around_unknown = LaunchConfiguration("inflate_around_unknown")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="61",
            description="ROS domain shared by all nodes in this navigation example.",
        ),
        DeclareLaunchArgument(
            "gz_partition",
            default_value="mini_nav",
            description="Gazebo transport partition for this navigation example.",
        ),
        DeclareLaunchArgument(
            "rmw_implementation",
            default_value="rmw_cyclonedds_cpp",
            description="ROS middleware used by all nodes in this navigation example.",
        ),
        # 在包含仿真入口和启动节点之前设置，让所有子进程使用同一环境。
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle"),
        DeclareLaunchArgument("use_simulator", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                bringup_share, "maps", "tb3_learning.yaml"
            ),
            description="Map YAML file used by the official map_server.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_share, "config", "amcl_waffle.yaml"
            ),
            description="Parameters for the self-built AMCL node.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure and activate localization nodes.",
        ),
        DeclareLaunchArgument(
            "inflate_around_unknown",
            default_value="false",
            description="Add a hard safety band around unknown cells in both costmaps.",
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
            condition=IfCondition(LaunchConfiguration("use_simulator")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "x_pose": x_pose,
                "y_pose": y_pose,
            }.items(),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {
                    "yaml_filename": map_file,
                    "use_sim_time": use_sim_time,
                }
            ],
        ),
        Node(
            package="mini_nav_nodes",
            executable="mini_nav_amcl_node",
            name="amcl",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": ["map_server", "amcl"],
                }
            ],
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
                    "enable_topic_goals": False,
                    "fuse_local_obstacles": True,
                    "planning.inflate_around_unknown": ParameterValue(
                        inflate_around_unknown, value_type=bool
                    ),
                }
            ],
        ),
        Node(
            package="mini_nav_nodes",
            executable="local_costmap_node",
            name="local_costmap",
            output="screen",
            parameters=[
                os.path.join(bringup_share, "config", "local_costmap.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    "local_costmap.inflate_around_unknown": ParameterValue(
                        inflate_around_unknown, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="mini_nav_nodes",
            executable="path_follower_node",
            name="path_follower",
            output="screen",
            parameters=[
                os.path.join(bringup_share, "config", "path_follower.yaml"),
                {"use_sim_time": use_sim_time, "action_mode": True,
                 "require_localization_quality": True, "cmd_vel_topic": "/mini_nav/cmd_vel_raw"},
            ],
        ),
        Node(
            package="mini_nav_nodes", executable="navigation_manager_node",
            name="navigation_manager", output="screen",
            parameters=[os.path.join(bringup_share, "config", "navigation_manager.yaml"),
                        {"use_sim_time": use_sim_time, "enabled": ParameterValue(autostart, value_type=bool)}],
        ),
        Node(
            package="mini_nav_nodes", executable="velocity_guard_node",
            name="velocity_guard", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "command_timeout": 0.35}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
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
