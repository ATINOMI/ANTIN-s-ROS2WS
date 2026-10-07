"""复用官方 TurtleBot3 Waffle 仿真入口，统一模型和初始位置参数。

资源从 package-share 查找；运行时参数由 launch 声明并解析。
"""
#这个模块提供了LaunchDescription类，用于描述ROS 2的启动过程。
from launch import LaunchDescription
# 这个模块提供了DeclareLaunchArgument、IncludeLaunchDescription和SetEnvironmentVariable类，用于在启动过程中声明参数、包含其他启动文件和设置环境变量。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
# 这个模块提供了PythonLaunchDescriptionSource类，用于从Python文件中加载启动描述。
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 这个模块提供了LaunchConfiguration类，用于在启动过程中获取参数的值。
from launch.substitutions import LaunchConfiguration
# 这个模块提供了FindPackageShare类，用于查找ROS 2包的共享目录。
from launch_ros.substitutions import FindPackageShare
# 这个模块提供了os模块，用于与操作系统进行交互，例如设置环境变量。
import os

def generate_launch_description():
    """建立启动描述。

    Returns:
        LaunchDescription: 节点、包含入口与参数声明组成的启动描述。

    Note:
        调用只构建动作描述；节点进程由 launch 执行动作时启动。
    """

    # 设置环境变量以指定TurtleBot3模型为Waffle
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # 声明Launch配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 声明TurtleBot3 Waffle的初始位置参数
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # 查找TurtleBot3 Gazebo包中的turtlebot3_world.launch.py文件
    turtlebot3_world_launch = PythonLaunchDescriptionSource([
        FindPackageShare('turtlebot3_gazebo'),
        '/launch/turtlebot3_world.launch.py',
    ])

    # 创建LaunchDescription对象并添加启动参数和包含的launch文件
    return LaunchDescription([
        # 声明使用仿真时间的参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock',
        ),
        # 声明TurtleBot3 Waffle的初始x位置参数
        DeclareLaunchArgument(
            'x_pose',
            default_value='-2.0',
            description='Initial Waffle x position in the world',
        ),
        # 声明TurtleBot3 Waffle的初始y位置参数
        DeclareLaunchArgument(
            'y_pose',
            default_value='-0.5',
            description='Initial Waffle y position in the world',
        ),
        # 包含turtlebot3_world
        IncludeLaunchDescription(
            turtlebot3_world_launch,
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items(),
        ),
    ])
