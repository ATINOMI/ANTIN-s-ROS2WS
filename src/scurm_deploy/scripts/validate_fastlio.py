#!/usr/bin/env python3
"""Check real FAST-LIO2 streams with optional short, guarded simulator motion."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, LaserScan, PointCloud2


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def planar(pose, imu=False):
    angle = yaw(pose)
    return np.array([pose.position.x + (0.032 * math.cos(angle) if imu else 0.0),
                     pose.position.y + (0.032 * math.sin(angle) if imu else 0.0), angle])


def relative(first, last):
    c, s = math.cos(first[2]), math.sin(first[2])
    delta = last[:2] - first[:2]
    return np.array([[c, s], [-s, c]]) @ delta


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--motion', action='store_true')
    parser.add_argument('--controls-pid', type=int)
    args = parser.parse_args()
    controls_paused = False
    if args.motion:
        if args.controls_pid is None:
            parser.error('--motion requires the PID of this demo mapping_controls process')
        command = Path(f'/proc/{args.controls_pid}/cmdline').read_bytes()
        if b'/install_scurm/scurm_sim/lib/scurm_sim/mapping_controls' not in command:
            raise RuntimeError('Refusing to pause a process outside the demo controls')
        os.kill(args.controls_pid, signal.SIGSTOP)
        controls_paused = True

    node = None
    publisher = None
    result = {}
    try:
        rclpy.init()
        node = Node('scurm_fastlio_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        publisher = node.create_publisher(TwistStamped, '/scurm/cmd_vel_smoothed', 1)
        streams = {}
        poses = {'lio': [], 'wheel': []}
        latest_scan = [None, -math.inf]
        subscriptions = []

        def record(name, msg):
            stream = streams.setdefault(name, {'count': 0, 'first_stamp': None})
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            stream['count'] += 1
            if stream['first_stamp'] is None:
                stream['first_stamp'] = stamp
            stream['last_stamp'] = stamp
            stream['frame'] = msg.header.frame_id
            if isinstance(msg, PointCloud2):
                stream['points'] = msg.width * msg.height
            elif isinstance(msg, RosPath):
                stream['poses'] = len(msg.poses)
            if name in poses:
                position = planar(msg.pose.pose, imu=name == 'lio')
                if not np.isfinite(position).all():
                    raise RuntimeError(f'Non-finite {name} pose')
                poses[name].append(position)
            if name == 'scan':
                latest_scan[:] = [msg, time.monotonic()]

        for name, topic, message in [
            ('lidar', '/scurm/lidar/points', PointCloud2), ('imu', '/imu', Imu),
            ('map', '/ikd_tree', PointCloud2), ('registered', '/cloud_registered', PointCloud2),
            ('lio', '/scurm/imu_odometry', Odometry), ('wheel', '/scurm/wheel_odometry', Odometry),
            ('scan', '/scan', LaserScan), ('path', '/path', RosPath)]:
            subscriptions.append(node.create_subscription(message, topic,
                lambda msg, name=name: record(name, msg), qos_profile_sensor_data))
        begin = time.monotonic()
        finish = begin + (16.0 if args.motion else 8.0)
        translation_commands = 0
        drive_direction = None

        def clearance(scan, center):
            values = [value for index, value in enumerate(scan.ranges)
                      if abs(math.remainder(scan.angle_min + index * scan.angle_increment - center,
                                            2 * math.pi)) < 0.5
                      and value >= scan.range_min and not math.isnan(value)]
            return min(values, default=0.0)

        while time.monotonic() < finish:
            rclpy.spin_once(node, timeout_sec=0.01)
            elapsed = time.monotonic() - begin
            out = TwistStamped()
            out.header.stamp = node.get_clock().now().to_msg()
            out.header.frame_id = 'base_footprint'
            if args.motion:
                if 3.0 <= elapsed < 7.0:
                    scan, received = latest_scan
                    if scan is not None and time.monotonic() - received < 0.4:
                        if drive_direction is None:
                            drive_direction = 1 if clearance(scan, 0.0) >= clearance(scan, math.pi) else -1
                        if clearance(scan, 0.0 if drive_direction > 0 else math.pi) > 0.6:
                            out.twist.linear.x = drive_direction * 0.12
                            translation_commands += 1
                elif 7.0 <= elapsed < 12.0:
                    out.twist.angular.z = 0.25
                publisher.publish(out)
        for name, stream in streams.items():
            span = stream['last_stamp'] - stream['first_stamp']
            stream['rate_hz_sim'] = (stream['count'] - 1) / span if span > 0 else 0.0
        result = {'motion': args.motion, 'duration_wall_s': time.monotonic() - begin,
                  'streams': streams, 'cmd_vel_publishers': node.count_publishers('/cmd_vel'),
                  'translation_commands': translation_commands, 'drive_direction': drive_direction}
        if poses['lio'] and poses['wheel']:
            lio = relative(poses['lio'][0], poses['lio'][-1])
            wheel = relative(poses['wheel'][0], poses['wheel'][-1])
            result.update(lio_relative_xy=lio.tolist(), wheel_relative_xy=wheel.tolist(),
                relative_position_difference_m=float(np.linalg.norm(lio - wheel)),
                relative_yaw_difference_rad=abs(math.remainder(
                    (poses['lio'][-1][2] - poses['lio'][0][2]) -
                    (poses['wheel'][-1][2] - poses['wheel'][0][2]), 2 * math.pi)))
        result['passed'] = all(streams.get(name, {}).get('count', 0) > 5
                               for name in ['lidar', 'imu', 'map', 'registered', 'lio', 'wheel', 'path'])
        result['passed'] &= result['cmd_vel_publishers'] == 1
        if args.motion:
            result['passed'] &= (translation_commands > 0
                and np.linalg.norm(lio) > 0.2 and result['relative_position_difference_m'] < 0.1
                and result['relative_yaw_difference_rad'] < 0.1)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        if publisher is not None and rclpy.ok():
            publisher.publish(TwistStamped())
            # Allow the zero command to reach the adapter before resuming manual controls.
            rclpy.spin_once(node, timeout_sec=0.1)
        if controls_paused:
            os.kill(args.controls_pid, signal.SIGCONT)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not result.get('passed', False):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
