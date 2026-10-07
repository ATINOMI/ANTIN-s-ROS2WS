#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='true'
    )

    x_pose = LaunchConfiguration(
        'x_pose',
        default='-2.0'
    )

    y_pose = LaunchConfiguration(
        'y_pose',
        default='-0.5'
    )

    world = os.path.join(
        turtlebot3_gazebo_dir,
        'worlds',
        'turtlebot3_world.world'
    )

    map_file = '/home/a/ros2_ws/maps/turtlebot3_map.yaml'

    params_file = (
        '/home/a/ros2_ws/src/navigation2_tutorials/'
        'sam_bot_description/config/nav2_params.yaml'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_dir,
                'launch',
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/a/ros2_ws/launch/'
            'spawn_turtlebot3_custom.launch.py'
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            turtlebot3_gazebo_dir,
            'models'
        )
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(nav2_cmd)

    return ld
