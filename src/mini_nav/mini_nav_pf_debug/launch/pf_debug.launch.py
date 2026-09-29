import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


FILTER_PARAMETERS = (
    "global_frame_id", "odom_frame_id", "base_frame_id", "map_topic", "scan_topic",
    "laser_model_type", "first_map_only", "update_min_d", "update_min_a",
    "laser_likelihood_max_dist", "laser_max_range", "laser_min_range",
    "alpha1", "alpha2", "alpha3", "alpha4", "alpha5",
    "z_hit", "z_short", "z_max", "z_rand", "sigma_hit", "lambda_short",
    "max_beams", "min_particles", "max_particles", "pf_err", "pf_z",
    "recovery_alpha_fast", "recovery_alpha_slow", "resample_interval",
)


def start_debug_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    with open(params_file, encoding="utf-8") as stream:
        amcl_params = yaml.safe_load(stream)["amcl"]["ros__parameters"]
    filter_params = {
        key: amcl_params[key] for key in FILTER_PARAMETERS if key in amcl_params
    }
    filter_params.update({
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        ),
        "output_file": ParameterValue(
            LaunchConfiguration("output_file"), value_type=str
        ),
        "max_updates": ParameterValue(
            LaunchConfiguration("max_updates"), value_type=int
        ),
        "max_sessions": ParameterValue(
            LaunchConfiguration("max_sessions"), value_type=int
        ),
    })
    return [Node(
        package="mini_nav_pf_debug",
        executable="pf_debug_node",
        name="pf_debug",
        output="screen",
        parameters=[filter_params],
    )]


def generate_launch_description():
    bringup_share = get_package_share_directory("mini_nav_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_share, "config", "amcl_waffle.yaml"),
        ),
        DeclareLaunchArgument("output_file", default_value="/tmp/mini_nav_pf_debug.html"),
        DeclareLaunchArgument("max_updates", default_value="15"),
        DeclareLaunchArgument("max_sessions", default_value="5"),
        OpaqueFunction(function=start_debug_node),
    ])
