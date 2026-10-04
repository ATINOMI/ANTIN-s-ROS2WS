#!/usr/bin/env python3
"""Generate an isolated simulator and Jazzy Nav2 configuration."""
import copy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parents[1]
SIM = ROOT / 'scurm_sim'


def main():
    source_sim = WS / 'src/fastlivo2_deploy/fastlivo_sim'
    (SIM / 'models').mkdir(exist_ok=True)
    shutil.copyfile(source_sim / 'models/tb3_livo.world', SIM / 'models/tb3_scurm.world')
    model = ET.parse(source_sim / 'models/waffle_livo.sdf')
    model.find('.//sensor[@name="livo_lidar"]/topic').text = '/scurm/lidar'
    model.find('.//sensor[@name="livo_lidar"]/gz_frame_id').text = 'scurm_lidar'
    # Each adapter owns its mode-specific TF; do not bridge the simulator's wheel TF.
    drive = model.find('.//plugin[@name="gz::sim::systems::DiffDrive"]')
    if drive is None:
        raise RuntimeError('No DiffDrive plugin in the sensor model')
    drive.find('odom_topic').text = '/scurm/wheel_odometry'
    tf = drive.find('tf_topic')
    if tf is None:
        tf = ET.SubElement(drive, 'tf_topic')
    tf.text = '/scurm/wheel_tf'
    model.find('.//plugin[@name="gz::sim::systems::PosePublisher"]')
    # The mapping demo has no image frontend; avoid rendering the unused camera.
    for link in model.findall('.//link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('type') in ['camera', 'depth_camera']:
                link.remove(sensor)
    model.write(SIM / 'models/waffle_scurm.sdf', encoding='unicode', xml_declaration=True)
    bridge = yaml.safe_load((source_sim / 'config/bridge.yaml').read_text())
    bridge = [entry for entry in bridge if entry['ros_topic_name'] not in
              ['odom', 'tf', 'camera/camera_info', '/fastlivo/ground_truth']]
    for entry in bridge:
        if entry['ros_topic_name'] == '/fastlivo/lidar/points':
            entry['ros_topic_name'] = '/scurm/lidar/points'
            entry['gz_topic_name'] = '/scurm/lidar/points'
    bridge.append({'ros_topic_name': '/scurm/wheel_odometry',
                   'gz_topic_name': '/scurm/wheel_odometry',
                   'ros_type_name': 'nav_msgs/msg/Odometry',
                   'gz_type_name': 'gz.msgs.Odometry', 'direction': 'GZ_TO_ROS'})
    (SIM / 'config/bridge.yaml').write_text(yaml.safe_dump(bridge, sort_keys=False))
    robot = ET.parse('/opt/ros/jazzy/share/turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf')
    ET.SubElement(robot.getroot(), 'link', name='scurm_lidar')
    joint = ET.SubElement(robot.getroot(), 'joint', name='scurm_lidar_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='scurm_lidar')
    ET.SubElement(joint, 'origin', xyz='0 0 0.30', rpy='0 0 0')
    robot.write(SIM / 'models/waffle_scurm.urdf', encoding='unicode', xml_declaration=True)

    baseline = yaml.safe_load(Path('/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml').read_text())
    config = {key: baseline[key] for key in [
        'bt_navigator', 'controller_server', 'local_costmap', 'global_costmap',
        'planner_server', 'smoother_server', 'behavior_server', 'velocity_smoother']}
    for params in config.values():
        if 'ros__parameters' in params:
            params['ros__parameters']['use_sim_time'] = True
            params['ros__parameters']['enable_stamped_cmd_vel'] = True
    navigator = config['bt_navigator']['ros__parameters']
    navigator.update(robot_base_frame='base_footprint', odom_topic='/state_estimation')
    controller = config['controller_server']['ros__parameters']
    controller.update(odom_topic='/state_estimation', min_y_velocity_threshold=0.0)
    controller['progress_checker'].update(required_movement_radius=0.1, movement_time_allowance=15.0)
    controller['general_goal_checker'].update(xy_goal_tolerance=0.15, yaw_goal_tolerance=0.3)
    mppi = controller['FollowPath']
    mppi.update(vx_max=0.2, vx_min=-0.15, vy_max=0.0, wz_max=0.7,
                ax_max=0.4, ax_min=-0.4, az_max=1.0, vx_std=0.1, wz_std=0.25,
                batch_size=1000, time_steps=50, prune_distance=1.2,
                visualize=False, motion_model='DiffDrive')
    planner = config['planner_server']['ros__parameters']
    planner['GridBased'] = {'plugin': 'nav2_theta_star_planner::ThetaStarPlanner',
                            'how_many_corners': 8, 'w_euc_cost': 1.0,
                            'w_traversal_cost': 2.0, 'allow_unknown': False}
    for name in ['local_costmap', 'global_costmap']:
        params = config[name][name]['ros__parameters']
        params.update(use_sim_time=True, robot_base_frame='base_footprint', robot_radius=0.24,
                      update_frequency=10.0, publish_frequency=5.0)
        params['inflation_layer'].update(inflation_radius=0.35, cost_scaling_factor=8.0)
    local = config['local_costmap']['local_costmap']['ros__parameters']
    local.update(width=4, height=4, plugins=['terrain_layer', 'inflation_layer'])
    local.pop('voxel_layer')
    local['terrain_layer'] = {
        'plugin': 'costmap_intensity::ObstacleLayerIntensity', 'enabled': True,
        'footprint_clearing_enabled': True, 'min_obstacle_intensity': 0.06,
        'max_obstacle_intensity': 2.0, 'observation_sources': 'pointcloud',
        'pointcloud': {'topic': '/terrain_map', 'sensor_frame': 'imu_link',
                      'data_type': 'PointCloud2', 'marking': True, 'clearing': True,
                      'min_obstacle_height': -2.0, 'max_obstacle_height': 2.0,
                      'obstacle_max_range': 3.0, 'obstacle_min_range': 0.2,
                      'raytrace_max_range': 3.5, 'raytrace_min_range': 0.1}}
    global_map = config['global_costmap']['global_costmap']['ros__parameters']
    global_map['obstacle_layer']['scan']['sensor_frame'] = 'base_scan'
    behaviors = config['behavior_server']['ros__parameters']
    behaviors.update(robot_base_frame='base_footprint', max_rotational_vel=0.7,
                     min_rotational_vel=0.1, rotational_acc_lim=1.0)
    smoother = config['velocity_smoother']['ros__parameters']
    smoother.update(odom_topic='/state_estimation', max_velocity=[0.2, 0.0, 0.7],
                    min_velocity=[-0.15, 0.0, -0.7], max_accel=[0.4, 0.0, 1.0],
                    max_decel=[-0.4, 0.0, -1.0], velocity_timeout=0.3)
    (SIM / 'config/nav2.yaml').write_text(yaml.safe_dump(config, sort_keys=False))


if __name__ == '__main__':
    main()
