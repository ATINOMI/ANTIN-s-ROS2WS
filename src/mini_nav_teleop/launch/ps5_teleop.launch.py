from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_controllers(context):
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    ros_context = Context()
    rclpy.init(context=ros_context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ps5_launch_preflight', context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        conflicts = [topic for topic in ['/cmd_vel', '/mini_nav/cmd_vel_raw', '/mini_nav/task_active']
                     if node.get_publishers_info_by_topic(topic)]
        if conflicts:
            raise RuntimeError(f'Another controller is active on {conflicts}')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=ros_context)
    return []


def generate_launch_description():
    linear = ParameterValue(LaunchConfiguration('linear_limit'), value_type=float)
    angular = ParameterValue(LaunchConfiguration('angular_limit'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('linear_limit', default_value='2.0'),
        DeclareLaunchArgument('angular_limit', default_value='2.0'),
        OpaqueFunction(function=check_controllers),
        Node(package='mini_nav_nodes', executable='velocity_guard_node', name='velocity_guard',
             parameters=[{'use_sim_time': True, 'command_timeout': 0.35,
                          'max_linear_speed': 2.0, 'max_angular_speed': 2.0}], output='screen'),
        Node(package='mini_nav_teleop', executable='ps5_teleop', name='ps5_teleop',
             parameters=[{'use_sim_time': True, 'linear_limit': linear, 'angular_limit': angular}], output='screen'),
    ])
