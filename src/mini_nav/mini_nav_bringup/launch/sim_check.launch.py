"""只启动使用仿真时钟的 RViz，用于观察既有仿真与 TF。

资源从 package-share 查找；运行时参数由 launch 声明并解析。
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """建立启动描述。

    Returns:
        LaunchDescription: 节点、包含入口与参数声明组成的启动描述。

    Note:
        调用只构建动作描述；节点进程由 launch 执行动作时启动。
    """
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
