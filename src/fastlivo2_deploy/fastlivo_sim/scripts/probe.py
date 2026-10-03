#!/usr/bin/env python3
"""Collect real Gazebo inputs and FAST-LIVO outputs; optional isolated motion."""
import argparse
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TwistStamped
from sensor_msgs_py import point_cloud2


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class Probe(Node):
    def __init__(self, output):
        super().__init__('fastlivo_acceptance', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.output = output
        self.counts = {}
        self.last = {}
        self.backwards = {}
        self.details = {}
        self.poses = {'livo': [], 'odom': [], 'truth': []}
        self.subscriptions_list = []
        for topic, msg, key in [('/imu', Imu, 'imu'), ('/camera/image_raw', Image, 'image'),
                               ('/camera/camera_info', CameraInfo, 'camera_info'),
                               ('/fastlivo/lidar/points', PointCloud2, 'lidar'),
                               ('/aft_mapped_to_init', Odometry, 'livo'), ('/odom', Odometry, 'odom'),
                               ('/cloud_registered', PointCloud2, 'registered')]:
            self.subscriptions_list.append(self.create_subscription(msg, topic, lambda m, k=key: self.receive(k, m), qos_profile_sensor_data))
        self.subscriptions_list.append(self.create_subscription(TFMessage, '/fastlivo/ground_truth', self.truth, qos_profile_sensor_data))
        self.command = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def receive(self, key, msg):
        self.counts[key] = self.counts.get(key, 0) + 1
        t = stamp(msg)
        if t < self.last.get(key, -1):
            self.backwards[key] = self.backwards.get(key, 0) + 1
        self.last[key] = t
        if key in self.poses:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.poses[key].append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        if key == 'lidar' and key not in self.details:
            xyz = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            self.details[key] = {'width': msg.width, 'height': msg.height, 'fields': [f.name for f in msg.fields],
                                 'frame': msg.header.frame_id, 'finite_points': len(xyz),
                                 'z_min': float(np.min(xyz[:, 2])), 'z_max': float(np.max(xyz[:, 2]))}
        elif key == 'camera_info':
            self.details[key] = {'width': msg.width, 'height': msg.height, 'k': list(msg.k)}
        elif key == 'imu':
            self.details[key] = {'acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]}
        elif key == 'image' and not (self.output / 'camera.png').exists():
            from cv_bridge import CvBridge
            import cv2
            cv2.imwrite(str(self.output / 'camera.png'), CvBridge().imgmsg_to_cv2(msg, 'bgr8'))

    def truth(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id == 'waffle':
                p, q = tr.transform.translation, tr.transform.rotation
                self.poses['truth'].append([stamp(tr), p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def velocity(self, linear=0., angular=0.):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.angular.z = linear, angular
        self.command.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--drive', action='store_true')
    args = parser.parse_args()
    if args.drive and (os.environ.get('ROS_DOMAIN_ID') != '219' or os.environ.get('GZ_PARTITION') != 'mini_nav_fastlivo_deploy'):
        parser.error('Motion is restricted to domain 219 and mini_nav_fastlivo_deploy')
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    rclpy.init(); node = Probe(output)
    start = time.monotonic(); sim_start = None
    try:
        while time.monotonic() - start < max(60., args.seconds * 5):
            rclpy.spin_once(node, timeout_sec=0.05)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now > 0 and sim_start is None: sim_start = now
            elapsed = now - sim_start if sim_start is not None else 0.
            if args.drive:
                # Warm up, translate 0.5 m, rotate, then stop; finite simulation-time run.
                linear = 0.05 if 5. <= elapsed < 15. else 0.
                angular = 0.15 if 17. <= elapsed < 21. else 0.
                node.velocity(linear, angular)
            if elapsed >= args.seconds: break
    finally:
        if args.drive:
            for _ in range(5):
                node.velocity(); rclpy.spin_once(node, timeout_sec=0.05)
        result = {'counts': node.counts, 'last_stamps': node.last, 'backwards': node.backwards,
                  'details': node.details, 'poses': node.poses, 'elapsed_sim': elapsed,
                  'elapsed_wall': time.monotonic() - start, 'drive': args.drive}
        (output / 'result.json').write_text(json.dumps(result, indent=2))
        summary = {k: v for k, v in result.items() if k != 'poses'}
        for key, rows in node.poses.items():
            summary[key] = {'count': len(rows), 'first': rows[0] if rows else None, 'last': rows[-1] if rows else None,
                            'finite': bool(rows) and all(math.isfinite(v) for row in rows for v in row)}
        (output / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
