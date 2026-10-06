#!/usr/bin/env python3
"""Connect SCURM's IMU odometry to the simulated differential-drive robot."""
import copy
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

IMU_BASE = np.eye(4)
IMU_BASE[:3, 3] = [0.032, 0.0, -0.078]
IMU_LIDAR = np.array([0.032, 0.0, 0.232])


def pose_matrix(pose):
    q = pose.orientation
    quaternion = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 0.5:
        raise ValueError('Invalid orientation')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Non-finite pose')
    return matrix


def write_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


def command_allowed(seed, receipt_age, odometry_age, command_age, clock_age):
    return (seed and 0.0 <= receipt_age <= 0.5 and 0.0 <= odometry_age <= 0.5
            and 0.0 <= command_age <= 0.3 and 0.0 <= clock_age <= 0.5)


class Adapter(Node):
    def __init__(self):
        super().__init__('scurm_adapter')
        self.mapping_mode = self.declare_parameter('mapping_mode', False).value
        self.seed = np.eye(4) if self.mapping_mode else None
        self.odom_stamp = None
        self.odom_receipt = -math.inf
        self.command_receipt = -math.inf
        self.command = TwistStamped()
        self.last_clock = 0
        self.clock_receipt = -math.inf
        self.reset_detected = False
        self.ready = False
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/state_estimation', 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.cloud_pub = self.create_publisher(PointCloud2, '/scurm/icp_cloud', 2)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/scurm/initial_imu', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, '/scurm/localization_ready', latched)
        self.status_pub = self.create_publisher(String, '/scurm/status', latched)
        self.create_subscription(PoseWithCovarianceStamped, '/icp_result', self.initial, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.manual_initial, 1)
        self.create_subscription(PointCloud2, '/scurm/lidar/points', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/scurm/imu_odometry', self.odometry, 10)
        self.create_subscription(TwistStamped, '/scurm/cmd_vel_smoothed', self.velocity, 1)
        self.create_timer(0.05, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status_pub.publish(String(data='Waiting for FAST-LIO2' if self.mapping_mode
                                       else 'Waiting for ICP and FAST-LIO2'))

    def initial(self, msg):
        if self.seed is not None or self.reset_detected:
            return
        try:
            self.seed = pose_matrix(msg.pose.pose)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.get_logger().info('ICP aligned the 3D prior; waiting for FAST-LIO2 odometry')

    def manual_initial(self, msg):
        if self.seed is not None:
            self.get_logger().warn('Initial alignment is locked; restart the launch to relocalize')
            return
        if msg.header.frame_id != 'map':
            return
        try:
            # RViz gives map -> base_footprint; ICP consumes points in imu_link.
            matrix = pose_matrix(msg.pose.pose) @ np.linalg.inv(IMU_BASE)
            out = copy.deepcopy(msg)
            write_pose(out.pose.pose, matrix)
            self.initial_pub.publish(out)
        except ValueError as exc:
            self.get_logger().error(str(exc))

    def cloud(self, msg):
        if self.seed is not None:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = points.reshape(-1, 3)[::2]
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.linalg.norm(points, axis=1) > 0.3] + IMU_LIDAR
        header = copy.deepcopy(msg.header)
        header.frame_id = 'imu_link'
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points.astype(np.float32)))

    def odometry(self, msg):
        if self.seed is None or self.reset_detected:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.odom_stamp is not None and stamp < self.odom_stamp:
            self.reset_detected = True
            self.status_pub.publish(String(data='Clock/estimator reset; restart the complete launch'))
            return
        try:
            imu = pose_matrix(msg.pose.pose)
        except ValueError:
            return
        body = imu @ IMU_BASE
        out = copy.deepcopy(msg)
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        write_pose(out.pose.pose, body)
        # Shift linear velocity from IMU origin to the base origin.
        angular = out.twist.twist.angular
        velocity = out.twist.twist.linear
        shifted = np.array([velocity.x, velocity.y, velocity.z]) + np.cross(
            [angular.x, angular.y, angular.z], IMU_BASE[:3, 3])
        velocity.x, velocity.y, velocity.z = map(float, shifted)
        self.odom_pub.publish(out)
        transforms = []
        frames = [('odom', 'base_footprint', body)]
        if not self.mapping_mode:
            frames.insert(0, ('map', 'odom', self.seed))
        for parent, child, matrix in frames:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, matrix[:3, 3])
            q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
            transforms.append(tf)
        self.tf.sendTransform(transforms)
        self.odom_stamp = stamp
        self.odom_receipt = time.monotonic()

    def velocity(self, msg):
        values = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]
        if not all(math.isfinite(value) for value in values):
            self.command = TwistStamped()
            return
        self.command = msg
        self.command_receipt = time.monotonic()

    def watchdog(self):
        monotonic = time.monotonic()
        now = self.get_clock().now()
        if now.nanoseconds != self.last_clock:
            if now.nanoseconds < self.last_clock:
                self.reset_detected = True
            self.last_clock = now.nanoseconds
            self.clock_receipt = monotonic
        age = math.inf if self.odom_stamp is None else now.nanoseconds * 1e-9 - self.odom_stamp
        ready = command_allowed(self.seed is not None and not self.reset_detected,
                                monotonic - self.odom_receipt, age, 0.0, monotonic - self.clock_receipt)
        if ready != self.ready:
            self.ready = ready
            self.ready_pub.publish(Bool(data=ready))
            available = 'FAST-LIO2 mapping ready' if self.mapping_mode else 'FAST-LIO2 localization ready'
            self.status_pub.publish(String(data=available if ready else 'Estimator unavailable; velocity held at zero'))
        out = TwistStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'base_footprint'
        if ready and monotonic - self.command_receipt <= 0.3:
            out.twist.linear.x = max(-0.2, min(0.2, self.command.twist.linear.x))
            out.twist.angular.z = max(-0.7, min(0.7, self.command.twist.angular.z))
        self.cmd_pub.publish(out)


def main():
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(TwistStamped())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
