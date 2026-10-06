#!/usr/bin/env python3
"""同次真实 Gazebo 输入采集 FAST-LIVO2 与 SLAM Toolbox，保存原始基准。"""
import json
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from slam_toolbox.srv import SerializePoseGraph
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from mini_nav_fastlivo.runtime import cloud_xyz, message_pose, message_tf, ns


if os.environ.get('ROS_DOMAIN_ID') != '228':
    raise RuntimeError('Requires isolated ROS_DOMAIN_ID=228')
parser = argparse.ArgumentParser()
parser.add_argument('--rotation-only', action='store_true')
args = parser.parse_args()
root = Path('/home/a/ros2_ws')
evidence = root / 'src/mini_nav/logs/26-10-4/trajectory_benchmark'
evidence.mkdir(exist_ok=True)
out = evidence / time.strftime('run_%Y%m%d_%H%M%S')
out.mkdir()
os.environ['ROS_LOG_DIR'] = str(out / 'ros_logs')
rclpy.init()
node = Node('wall_thickness_reproduction', parameter_overrides=[Parameter('use_sim_time', value=True)])
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
state, live, odom, collision, truth = {}, [], [], [], []
pose_trace, scan_trace, map_trace, cloud_trace = [], [], [], []
slam_live, slam_poses = [], []


def on_map(message):
    live[:] = [message]


def on_odom(message):
    odom[:] = [message]


def on_pose(message):
    pose_trace.append([ns(message.header.stamp), *message_pose(message.pose.pose).ravel().tolist()])


def on_truth(message):
    for tf in message.transforms:
        if 'waffle' in tf.child_frame_id:
            truth.append([ns(tf.header.stamp), *message_tf(tf).ravel().tolist()])


def on_collision(message):
    points = cloud_xyz(message)
    collision[:] = [points]
    if not cloud_trace or ns(message.header.stamp) - cloud_trace[-1][0] >= 200000000:
        cloud_trace.append((ns(message.header.stamp), points.astype(np.float32)))


def on_scan(message):
    scan_trace.append((ns(message.header.stamp), np.asarray(message.ranges, np.float32)))


node.create_subscription(String, '/fastlivo/mapping_status', lambda m: state.update(json.loads(m.data)), latched)
node.create_subscription(OccupancyGrid, '/fastlivo/map_2d_live', on_map, latched)
node.create_subscription(OccupancyGrid, '/slam_benchmark/map', lambda m: slam_live.__setitem__(slice(None), [m]), latched)
node.create_subscription(PoseWithCovarianceStamped, '/pose', lambda m: slam_poses.append([ns(m.header.stamp), *message_pose(m.pose.pose).ravel().tolist()]), 10)
node.create_subscription(Odometry, '/odom', on_odom, sensor)
node.create_subscription(Odometry, '/aft_mapped_to_init', on_pose, sensor)
node.create_subscription(TFMessage, '/fastlivo/ground_truth', on_truth, sensor)
node.create_subscription(PointCloud2, '/fastlivo/collision_cloud', on_collision, sensor)
node.create_subscription(LaserScan, '/scan', on_scan, sensor)


def spin(seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.01)


def wait(predicate, timeout=45):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.01)
        if predicate():
            return
    raise RuntimeError('Timeout: ' + json.dumps(state))


def line_from_first_cloud(points):
    xy = points[:, :2]
    rng = np.random.default_rng(17)
    best, count = None, 0
    for _ in range(600):
        pair = xy[rng.choice(len(xy), 2, replace=False)]
        direction = pair[1] - pair[0]
        length = np.linalg.norm(direction)
        if length < 0.8:
            continue
        direction /= length
        normal = np.array([-direction[1], direction[0]])
        offset = float(pair[0] @ normal)
        selected = abs(xy @ normal - offset) < 0.015
        along = xy[selected] @ direction
        if len(along) < 50 or np.ptp(along) < 1.2:
            continue
        if selected.sum() > count:
            count = int(selected.sum())
            best = (normal, direction, offset, np.quantile(along, [0.10, 0.90]))
    if best is None:
        raise RuntimeError('No long straight wall found in initial collision cloud')
    return best


def widths(message, line):
    normal, direction, offset, limits = line
    data = np.asarray(message.data).reshape(message.info.height, message.info.width)
    y, x = np.nonzero(data == 100)
    xy = np.column_stack([x + 0.5, y + 0.5]) * message.info.resolution
    xy += [message.info.origin.position.x, message.info.origin.position.y]
    perpendicular, along = xy @ normal - offset, xy @ direction
    chosen = (abs(perpendicular) < 0.4) & (along > limits[0]) & (along < limits[1])
    bins = np.floor((along[chosen] - limits[0]) / 0.10).astype(int)
    spans = []
    for index in np.unique(bins):
        values = perpendicular[chosen][bins == index]
        spans.append(float(np.ptp(values) + message.info.resolution))
    return {'occupied_wall_cells': int(chosen.sum()), 'median_width_m': float(np.median(spans)),
            'p90_width_m': float(np.quantile(spans, 0.90)),
            'occupied_total': int((data == 100).sum()), 'resolution_m': message.info.resolution}


spin(0.5)
if any(node.count_publishers(topic) for topic in ['/clock', '/cmd_vel', '/fastlivo/map_2d_live']):
    raise RuntimeError('Domain 228 is in use; refusing to connect')
log = (out / 'launch.log').open('w')
process = subprocess.Popen(['ros2', 'launch', 'mini_nav_bringup', 'fastlivo_slam_benchmark.launch.py',
    'ros_domain_id:=228', 'gz_partition:=mini_nav_wall_thickness_reproduction',
    'gui:=false',
    'output_dir:=' + str(out / 'display_maps')], stdout=log, stderr=log, start_new_session=True)
bag_log = (out / 'bag.log').open('w')
bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out/'sensors'), '/scan', '/odom', '/tf', '/tf_static', '/clock', '/pose', '/aft_mapped_to_init', '/fastlivo/ground_truth'], stdout=bag_log, stderr=bag_log, start_new_session=True)
raw = lease = None
velocity = [0.0, 0.0]


def command():
    if not odom or raw is None:
        return
    message = TwistStamped()
    message.header.stamp = odom[0].header.stamp
    message.header.frame_id = 'base_footprint'
    message.twist.linear.x, message.twist.angular.z = velocity
    raw.publish(message)
    lease.publish(Bool(data=any(velocity)))


timer = node.create_timer(0.05, command)
snapshots = []


def snapshot(name, line):
    wait(lambda: bool(live))
    message = live[0]
    if slam_live:
        sm = slam_live[0]
        np.savez_compressed(out / (name + '_slam.npz'), grid=np.asarray(sm.data, np.int8).reshape(sm.info.height, sm.info.width), resolution=sm.info.resolution, origin=[sm.info.origin.position.x, sm.info.origin.position.y], stamp_ns=ns(sm.header.stamp))
    data = np.asarray(message.data, np.int8).reshape(message.info.height, message.info.width)
    np.savez_compressed(out / (name + '.npz'), grid=data,
        resolution=message.info.resolution,
        origin=[message.info.origin.position.x, message.info.origin.position.y], collision=collision[0])
    result = {'name': name, 'stamp_ns': ns(message.header.stamp), 'status': dict(state),
              **widths(message, line)}
    snapshots.append(result)
    print(json.dumps(result), flush=True)


try:
    wait(lambda: state.get('scan_observations', 0) >= 20 and collision and live and odom and slam_live and slam_poses)
    raw = node.create_publisher(TwistStamped, '/mini_nav/cmd_vel_raw', 1)
    lease = node.create_publisher(Bool, '/mini_nav/task_active', 1)
    line = line_from_first_cloud(collision[0])
    snapshot('initial', line)
    spin(20.0)
    snapshot('stationary_20s', line)
    velocity[:] = [0.0, 0.5]
    spin(4 * math.pi)
    velocity[:] = [0.0, 0.0]
    spin(3.0)
    snapshot('after_ccw_rotation', line)
    velocity[:] = [0.0, -0.5]
    spin(4 * math.pi)
    velocity[:] = [0.0, 0.0]
    spin(3.0)
    snapshot('after_cw_rotation', line)
    if not args.rotation_only:
        for _ in range(3):
            velocity[:] = [0.1, 0.0]
            spin(6.0)
            velocity[:] = [-0.1, 0.0]
            spin(6.0)
        velocity[:] = [0.0, 0.0]
        spin(3.0)
        snapshot('after_repeated_translation', line)
    wait(lambda: bool(slam_live))
    spin(2.0)
    snapshot('final', line)
    serialize = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
    wait(serialize.service_is_ready, 5)
    request = SerializePoseGraph.Request()
    request.filename = str(out/'slam_graph')
    saved = serialize.call_async(request)
    wait(saved.done, 10)
    print('SLAM graph save result:', saved.result().result, flush=True)
    (out / 'measurement.json').write_text(json.dumps({'line': [np.asarray(v).tolist() for v in line],
        'snapshots': snapshots, 'cmd_vel_publishers': node.count_publishers('/cmd_vel')}, indent=2))
    baseline = next(item for item in snapshots if item['name'] == 'stationary_20s')
    rotated = next(item for item in snapshots if item['name'] == 'after_cw_rotation')
    (out / 'baseline_check.json').write_text(json.dumps({'growth_m': rotated['median_width_m'] - baseline['median_width_m'], 'phase': 'unmodified online preview with full raw recording'}, indent=2))
finally:
    velocity[:] = [0.0, 0.0]
    spin(0.4)
    timer.cancel()
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=8)
    np.savez_compressed(out / 'trace.npz', poses=np.asarray(pose_trace), truth=np.asarray(truth), slam_poses=np.asarray(slam_poses),
        scan_stamps=np.asarray([item[0] for item in scan_trace]),
        scans=np.asarray([item[1] for item in scan_trace]),
        cloud_stamps=np.asarray([item[0] for item in cloud_trace]),
        cloud_sizes=np.asarray([len(item[1]) for item in cloud_trace]),
        clouds=np.concatenate([item[1] for item in cloud_trace]) if cloud_trace else np.empty((0, 3)))
    (out / 'last_status.json').write_text(json.dumps(state, indent=2))
    if bag.poll() is None:
        os.killpg(bag.pid, signal.SIGINT)
        bag.wait(timeout=10)
    bag_log.close()
    log.close()
    node.destroy_node()
    rclpy.shutdown()
    print('Artifacts:', out, flush=True)
