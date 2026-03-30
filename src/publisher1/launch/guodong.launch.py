
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 第一个节点：my_cpp_pkg功能包的my_node
        Node(
            package="my_cpp_pkg",  # 功能包名称（和你XML里的pkg对应）
            executable="my_node",  # 节点可执行文件名（和你XML里的type对应）
            name="my_node"         # 节点名称（和你XML里的name对应）
        ),
        # 第二个节点：anti功能包的sub（输出到屏幕，对应output="screen"）
        Node(
            package="anti",
            executable="sub",
            name="sub",
            output="screen"  # 终端打印节点输出（和你XML里的output="screen"一致）
        )
    ])